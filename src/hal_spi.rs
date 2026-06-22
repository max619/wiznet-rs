//! Platform SPI transport for the W6100 on stm32f1xx-hal.
//!
//! Implements the blocking `embedded-hal` `SpiDevice` (register ops) and the
//! crate's `w6100::SpiDma` (bulk socket-buffer transfers via DMA1). All the
//! HAL/DMA concretions live here so `w6100` stays platform-independent.
//!
//! NOTE: the manual SPI+DMA register programming below is **not yet
//! hardware-tested**; the channel config follows stm32f1xx-hal's own
//! `Spi::read_write`, but flag handling (overrun/BSY) may need on-target tuning.
//! Phase 1: the DMA transfer is started and immediately waited (blocking).

use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use stm32f1xx_hal::{
    dma::dma1,
    gpio::{Output, Pin},
    pac,
    spi::Spi,
};

use crate::w6100::{Error, SpiDma};

const HEADER: usize = 3;
const PAYLOAD: usize = 512;
/// One DMA transfer carries the 3-byte command header followed by the payload.
pub const SCRATCH: usize = HEADER + PAYLOAD;

pub struct HalSpi {
    spi: Spi<pac::SPI1, u8>,
    rx: dma1::C2, // SPI1_RX
    tx: dma1::C3, // SPI1_TX
    cs: Pin<'A', 9, Output>,
    rx_scratch: &'static mut [u8; SCRATCH],
    tx_scratch: &'static mut [u8; SCRATCH],
    data_len: usize,
    sysclk_hz: u32,
}

impl HalSpi {
    pub fn new(
        spi: Spi<pac::SPI1, u8>,
        rx: dma1::C2,
        tx: dma1::C3,
        cs: Pin<'A', 9, Output>,
        rx_scratch: &'static mut [u8; SCRATCH],
        tx_scratch: &'static mut [u8; SCRATCH],
        sysclk_hz: u32,
    ) -> Self {
        Self {
            spi,
            rx,
            tx,
            cs,
            rx_scratch,
            tx_scratch,
            data_len: 0,
            sysclk_hz,
        }
    }

    fn delay_ns(&self, ns: u32) {
        let cycles = ((ns as u64 * self.sysclk_hz as u64) / 1_000_000_000) as u32;
        cortex_m::asm::delay(cycles.max(1));
    }

    /// Address of the SPI1 data register, the DMA peripheral endpoint.
    fn spi_dr() -> u32 {
        #[allow(unsafe_code)]
        // SAFETY: read-only use of a fixed peripheral register address.
        unsafe {
            (*pac::SPI1::ptr()).dr().as_ptr() as u32
        }
    }

    fn spi_dma_enable(enable: bool) {
        #[allow(unsafe_code)]
        // SAFETY: toggling the SPI1 DMA-request bits; the bus is otherwise idle.
        unsafe {
            (*pac::SPI1::ptr())
                .cr2()
                .modify(|_, w| w.rxdmaen().bit(enable).txdmaen().bit(enable));
        }
    }

    /// Program both channels for an `n`-byte full-duplex transfer
    /// (tx_scratch → MOSI, MISO → rx_scratch) and start it.
    fn start_dma(&mut self, n: usize) {
        let dr = Self::spi_dr();

        // Ensure both channels are stopped and their flags cleared.
        self.rx.stop();
        self.tx.stop();

        self.rx.set_peripheral_address(dr, false);
        self.rx
            .set_memory_address(self.rx_scratch.as_mut_ptr() as u32, true);
        self.rx.set_transfer_length(n);

        self.tx.set_peripheral_address(dr, false);
        self.tx
            .set_memory_address(self.tx_scratch.as_ptr() as u32, true);
        self.tx.set_transfer_length(n);

        // 8-bit, no mem2mem/circular; RX writes to memory, TX reads from memory.
        self.rx.ch().cr().modify(|_, w| {
            w.mem2mem().clear_bit();
            w.pl().medium();
            w.msize().bits8();
            w.psize().bits8();
            w.circ().clear_bit();
            w.dir().clear_bit()
        });
        self.tx.ch().cr().modify(|_, w| {
            w.mem2mem().clear_bit();
            w.pl().medium();
            w.msize().bits8();
            w.psize().bits8();
            w.circ().clear_bit();
            w.dir().set_bit()
        });

        Self::spi_dma_enable(true);
        // RX armed before TX so the first received byte is captured.
        self.rx.start();
        self.tx.start();
    }
}

impl ErrorType for HalSpi {
    type Error = stm32f1xx_hal::spi::Error;
}

impl SpiDevice<u8> for HalSpi {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.cs.set_low();

        let mut result = Ok(());
        for op in operations {
            result = match op {
                Operation::Read(buf) => self.spi.read(buf),
                Operation::Write(buf) => self.spi.write(buf),
                Operation::Transfer(read, write) => self.spi.transfer(read, write),
                Operation::TransferInPlace(buf) => self.spi.transfer_in_place(buf),
                Operation::DelayNs(ns) => {
                    self.delay_ns(*ns);
                    Ok(())
                }
            };
            if result.is_err() {
                break;
            }
        }

        let flush = self.spi.flush();
        self.cs.set_high();
        result.and(flush)
    }
}

impl SpiDma for HalSpi {
    fn start_read(&mut self, header: &[u8], len: usize) -> Result<(), Error> {
        let n = header.len() + len;
        self.tx_scratch[..header.len()].copy_from_slice(header);
        self.tx_scratch[header.len()..n].fill(0); // dummy bytes to clock the read
        self.data_len = len;

        self.cs.set_low();
        self.start_dma(n);
        Ok(())
    }

    fn start_write(&mut self, header: &[u8], data: &[u8]) -> Result<(), Error> {
        let n = header.len() + data.len();
        self.tx_scratch[..header.len()].copy_from_slice(header);
        self.tx_scratch[header.len()..n].copy_from_slice(data);
        self.data_len = data.len();

        self.cs.set_low();
        self.start_dma(n);
        Ok(())
    }

    fn finish(&mut self) -> Result<(), Error> {
        // Phase 1: block until the RX channel signals transfer-complete.
        while self.rx.in_progress() {}

        Self::spi_dma_enable(false);
        self.rx.stop();
        self.tx.stop();
        let _ = self.spi.flush();
        self.cs.set_high();
        Ok(())
    }

    fn read_buffer(&self) -> &[u8] {
        &self.rx_scratch[HEADER..HEADER + self.data_len]
    }
}
