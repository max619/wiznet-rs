//! Platform SPI transport for the W6100 on stm32f1xx-hal.
//!
//! Implements the blocking `embedded-hal` `SpiDevice` (register ops) and the
//! crate's `w6100::SpiDma` (bulk socket-buffer transfers). Both are driven by
//! the HAL's own `Spi1RxTxDma::read_write` full-duplex DMA, so the SPI+DMA
//! sequencing is the library's tested path rather than hand-rolled register
//! pokes. `read_write` is awaited inline (blocking) for now; Phase 2 will move
//! the wait to a DMA-complete interrupt.

use embedded_hal::spi::{ErrorType, Operation, SpiDevice};
use stm32f1xx_hal::{
    dma::{ReadWriteDma, dma1},
    gpio::{Output, Pin},
    pac,
    spi::{Spi, Spi1RxTxDma},
};

use crate::w6100::{Error, SpiDma};

const HEADER: usize = 3;
const PAYLOAD: usize = 512;
/// One bulk DMA transfer carries the 3-byte header followed by the payload.
pub const SCRATCH: usize = HEADER + PAYLOAD;

pub struct HalSpi {
    /// `Option` only so it can be moved through `read_write`/`wait`; always
    /// `Some` between transfers.
    dma: Option<Spi1RxTxDma>,
    cs: Pin<'A', 9, Output>,
    rx_scratch: Option<&'static mut [u8; SCRATCH]>,
    tx_scratch: Option<&'static mut [u8; SCRATCH]>,
    data_len: usize,
    sysclk_hz: u32,
}

impl HalSpi {
    pub fn new(
        spi: Spi<pac::SPI1, u8>,
        rx_ch: dma1::C2, // SPI1_RX
        tx_ch: dma1::C3, // SPI1_TX
        cs: Pin<'A', 9, Output>,
        rx_scratch: &'static mut [u8; SCRATCH],
        tx_scratch: &'static mut [u8; SCRATCH],
        sysclk_hz: u32,
    ) -> Self {
        Self {
            dma: Some(spi.with_rx_tx_dma(rx_ch, tx_ch)),
            cs,
            rx_scratch: Some(rx_scratch),
            tx_scratch: Some(tx_scratch),
            data_len: 0,
            sysclk_hz,
        }
    }

    fn delay_ns(&self, ns: u32) {
        let cycles = ((ns as u64 * self.sysclk_hz as u64) / 1_000_000_000) as u32;
        cortex_m::asm::delay(cycles.max(1));
    }

    /// Blocking full-duplex transfer of `n` bytes: `tx_scratch[..n]` out,
    /// captured into `rx_scratch[..n]`.
    fn run(&mut self, n: usize) {
        let dma = self.dma.take().expect("dma present between transfers");

        #[allow(unsafe_code)]
        // SAFETY: `rx_scratch`/`tx_scratch` are distinct `'static` buffers. The
        // transfer is awaited before this function returns, so these length-`n`
        // views are the sole users of that memory for the transfer's duration.
        let rx = self.rx_scratch.take().unwrap();
        let tx: &mut [u8; SCRATCH] = self.tx_scratch.take().unwrap();

        let (buffers, dma) = dma.read_write(rx, tx).wait();
        self.dma = Some(dma);
        self.rx_scratch = Some(buffers.0);
        self.tx_scratch = Some(buffers.1);
    }
}

impl ErrorType for HalSpi {
    type Error = stm32f1xx_hal::spi::Error;
}

impl SpiDevice<u8> for HalSpi {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.cs.set_low();

        for op in operations {
            match op {
                Operation::Read(buf) => {
                    let n = buf.len();
                    self.tx_scratch.as_mut().unwrap()[..n].fill(0);
                    self.run(n);
                    buf.copy_from_slice(&self.rx_scratch.as_ref().unwrap()[..n]);
                }
                Operation::Write(buf) => {
                    let n = buf.len();
                    self.tx_scratch.as_mut().unwrap()[..n].copy_from_slice(buf);
                    self.run(n);
                }
                Operation::Transfer(read, write) => {
                    let n = read.len().max(write.len());
                    self.tx_scratch.unwrap()[..write.len()].copy_from_slice(write);
                    self.tx_scratch.unwrap()[write.len()..n].fill(0);
                    self.run(n);
                    let r = read.len();
                    read.copy_from_slice(&self.rx_scratch.unwrap()[..r]);
                }
                Operation::TransferInPlace(buf) => {
                    let n = buf.len();
                    self.tx_scratch.unwrap()[..n].copy_from_slice(buf);
                    self.run(n);
                    buf.copy_from_slice(&self.rx_scratch.unwrap()[..n]);
                }
                Operation::DelayNs(ns) => self.delay_ns(*ns),
            }
        }

        self.cs.set_high();
        Ok(())
    }
}

impl SpiDma for HalSpi {
    fn start_read(&mut self, header: &[u8], len: usize) -> Result<(), Error> {
        let h = header.len();
        self.tx_scratch[..h].copy_from_slice(header);
        self.tx_scratch[h..h + len].fill(0); // dummy bytes to clock the read in
        self.data_len = len;

        self.cs.set_low();
        self.run(h + len);
        Ok(())
    }

    fn start_write(&mut self, header: &[u8], data: &[u8]) -> Result<(), Error> {
        let h = header.len();
        self.tx_scratch[..h].copy_from_slice(header);
        self.tx_scratch[h..h + data.len()].copy_from_slice(data);
        self.data_len = data.len();

        self.cs.set_low();
        self.run(h + data.len());
        Ok(())
    }

    fn finish(&mut self) -> Result<(), Error> {
        // `run` already awaited the transfer (blocking); just release CS.
        self.cs.set_high();
        Ok(())
    }

    fn read_buffer(&self) -> &[u8] {
        // The 3 header bytes are clocked first, so payload sits at offset HEADER.
        &self.rx_scratch[HEADER..HEADER + self.data_len]
    }
}
