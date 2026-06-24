//! Platform SPI transport for the W6100 on stm32f1xx-hal.
//!
//! Implements the blocking `embedded-hal` `SpiDevice` (register ops) and the
//! crate's `w6100::SpiDma` (bulk socket-buffer transfers). Both are driven by
//! the HAL's own `Spi1RxTxDma::read_write` full-duplex DMA, so the SPI+DMA
//! sequencing is the library's tested path rather than hand-rolled register
//! pokes. `read_write` is awaited inline (blocking) for now; Phase 2 will move
//! the wait to a DMA-complete interrupt.
//!
//! Two clone-MCU quirks shape this (see TODO "Clone hardware quirks"):
//!   - The HAL's `wait()` keys completion off the RX channel's `TCIF`, and its
//!     `stop()` clears flags via the DMA *global*-clear bit, which doesn't
//!     reliably stick on this part — a stale `TCIF` makes the next `wait()`
//!     return before the transfer finishes. So we clear the channel-specific
//!     `CTCIF2` before every transfer (`clear_spi1_rx_tc`). stm32f1xx-hal 0.11
//!     has no per-channel `clear_event`, hence the one wrapped register write.
//!   - `read_write` transfers the whole buffer's array length, so to move
//!     exactly `n` bytes the `'static` scratch is wrapped in `DmaBuf`, which
//!     reports only its active `len` to the DMA.

use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};
use stm32f1xx_hal::{
    dma::{ReadWriteDma, dma1},
    gpio::{Output, Pin},
    pac,
    spi::{Spi, Spi1RxTxDma},
};

use wiznet_rs::{Error, SpiDma};

const HEADER: usize = 3;
const PAYLOAD: usize = 512;
/// One bulk DMA transfer carries the 3-byte header followed by the payload.
pub const SCRATCH: usize = HEADER + PAYLOAD;

pub struct HalSpi {
    /// `Option` only so it can be moved through `read_write`/`wait`; always
    /// `Some` between transfers.
    dma: Option<Spi1RxTxDma>,
    cs: Pin<'A', 9, Output>,
    /// Same `Option`-for-move rule as `dma`: taken during a transfer, restored
    /// after `wait()`.
    rx_scratch: Option<DmaBuf>,
    tx_scratch: Option<DmaBuf>,
    /// Payload length of the last bulk transfer (for `read_buffer`).
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
            rx_scratch: Some(DmaBuf::new(rx_scratch)),
            tx_scratch: Some(DmaBuf::new(tx_scratch)),
            data_len: 0,
            sysclk_hz,
        }
    }

    /// The TX scratch bytes, as a plain slice (always present between transfers).
    fn tx_buf(&mut self) -> &mut [u8] {
        &mut self.tx_scratch.as_mut().expect("tx scratch present").data[..]
    }

    /// The RX scratch bytes, as a plain slice (always present between transfers).
    fn rx_buf(&self) -> &[u8] {
        &self.rx_scratch.as_ref().expect("rx scratch present").data[..]
    }

    fn delay_ns(&self, ns: u32) {
        let cycles = ((ns as u64 * self.sysclk_hz as u64) / 1_000_000_000) as u32;
        cortex_m::asm::delay(cycles.max(1));
    }

    /// Blocking full-duplex transfer of exactly `n` bytes: `tx_scratch[..n]` out,
    /// captured into `rx_scratch[..n]`. CS must already be asserted by the caller.
    fn run(&mut self, n: usize) {
        // Clear any stale SPI1_RX transfer-complete flag before arming: `wait()`
        // treats "TCIF set" as done and the HAL's global-clear doesn't stick on
        // this part, so a leftover flag would make `wait()` return early.

        let dma = self.dma.take().expect("dma present between transfers");
        Self::clear_rx_tc(&dma);

        let mut rx = self
            .rx_scratch
            .take()
            .expect("rx scratch present between transfers");
        let mut tx = self
            .tx_scratch
            .take()
            .expect("tx scratch present between transfers");
        rx.len = n;
        tx.len = n;

        let (buffers, dma) = dma.read_write(rx, tx).wait();
        self.dma = Some(dma);
        self.rx_scratch = Some(buffers.0);
        self.tx_scratch = Some(buffers.1);
    }

    /// Clear the SPI1_RX (DMA1 channel 2) transfer-complete flag.
    ///
    /// stm32f1xx-hal 0.11 has no per-channel event-clear, and the channel is owned
    /// inside `Spi1RxTxDma`, so we write the channel-specific `CTCIF2` bit directly.
    /// This is deliberately the *channel*-specific clear, not the HAL's global one,
    /// which doesn't reliably stick on this clone MCU.
    fn clear_rx_tc(dma: &Spi1RxTxDma) {
        dma.rxchannel.ifcr().write(|w| w.ctcif2().set_bit());
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
                    self.tx_buf()[..n].fill(0);
                    self.run(n);
                    buf.copy_from_slice(&self.rx_buf()[..n]);
                }
                Operation::Write(buf) => {
                    let n = buf.len();
                    self.tx_buf()[..n].copy_from_slice(buf);
                    self.run(n);
                }
                Operation::Transfer(read, write) => {
                    let n = read.len().max(write.len());
                    self.tx_buf()[..write.len()].copy_from_slice(write);
                    self.tx_buf()[write.len()..n].fill(0);
                    self.run(n);
                    let r = read.len();
                    read.copy_from_slice(&self.rx_buf()[..r]);
                }
                Operation::TransferInPlace(buf) => {
                    let n = buf.len();
                    self.tx_buf()[..n].copy_from_slice(buf);
                    self.run(n);
                    buf.copy_from_slice(&self.rx_buf()[..n]);
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
        self.tx_buf()[..h].copy_from_slice(header);
        self.tx_buf()[h..h + len].fill(0); // dummy bytes to clock the read in
        self.data_len = len;

        self.cs.set_low();
        self.run(h + len);
        Ok(())
    }

    fn start_write(&mut self, header: &[u8], data: &[u8]) -> Result<(), Error> {
        let h = header.len();
        self.tx_buf()[..h].copy_from_slice(header);
        self.tx_buf()[h..h + data.len()].copy_from_slice(data);
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
        &self.rx_buf()[HEADER..HEADER + self.data_len]
    }
}

/// A `'static` scratch buffer that exposes only its first `len` bytes to the DMA.
///
/// `Spi1RxTxDma::read_write` transfers the buffer's whole array length, so to
/// move exactly `n` bytes we report `len = n` through the `embedded-dma` buffer
/// traits while keeping the full `SCRATCH`-sized backing storage.
struct DmaBuf {
    data: &'static mut [u8; SCRATCH],
    len: usize,
}

impl DmaBuf {
    fn new(data: &'static mut [u8; SCRATCH]) -> Self {
        Self { data, len: 0 }
    }
}

#[allow(unsafe_code)]
// SAFETY: `data` is a valid `'static` buffer and `len ≤ SCRATCH`, so the
// reported `(ptr, len)` window of `u8` words stays inside it. `read_write` owns
// the `DmaBuf` until `wait()` returns it, so nothing else touches the bytes
// during the transfer.
unsafe impl WriteBuffer for DmaBuf {
    type Word = u8;
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.data.as_mut_ptr(), self.len)
    }
}

#[allow(unsafe_code)]
// SAFETY: same window as above, read-only.
unsafe impl ReadBuffer for DmaBuf {
    type Word = u8;
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.data.as_ptr(), self.len)
    }
}
