//! Platform SPI transport for the W6100 on stm32f1xx-hal.
//!
//! Implements the driver's `wiznet_rs::SpiDmaDevice`: a byte-dumb full-duplex
//! DMA engine. The library's `Transceiver` owns the scratch buffers, packs the
//! W6100 command header + payload into them, hands them in via `transceive`, and
//! reads the captured bytes back out after `wait`. `HalSpi` itself knows nothing
//! of the W6100 protocol — it just clocks `len` bytes out of `tx` and into `rx`
//! over the HAL's `Spi1RxTxDma::read_write` full-duplex DMA.
//!
//! `transceive` consumes the device and returns an in-flight `HalTransaction`;
//! `wait` blocks for completion and hands the device + buffers back. (The wait is
//! blocking for now; Phase 2 moves it to a DMA-complete interrupt.)
//!
//! Two clone-MCU quirks shape this (see TODO "Clone hardware quirks"):
//!   - The HAL's `wait()` keys completion off the RX channel's `TCIF`, and its
//!     `stop()` clears flags via the DMA *global*-clear bit, which doesn't
//!     reliably stick on this part — a stale `TCIF` makes the next `wait()`
//!     return before the transfer finishes. So we clear the channel-specific
//!     `CTCIF2` before every transfer (`clear_rx_tc`). stm32f1xx-hal 0.11 has no
//!     per-channel `clear_event`, hence the one wrapped register write.
//!   - `read_write` transfers the whole buffer's array length, so to move exactly
//!     `len` bytes the scratch slice is wrapped in `RxWindow`/`TxWindow`, which
//!     report only the active `len` to the DMA.

use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_hal::{delay::DelayNs, spi::ErrorKind};
use stm32f1xx_hal::{
    dma::{ReadWriteDma, Transfer, W, dma1},
    gpio::{Output, Pin},
    pac,
    spi::{Spi, Spi1RxTxDma},
};

use wiznet_rs::{DmaBuffers, SpiDmaDevice, SpiDmaTransaction};

const HEADER: usize = 3;
const PAYLOAD: usize = 512;
/// One bulk DMA transfer carries the 3-byte header followed by the payload, so
/// the application sizes the driver's scratch buffers to fit both.
pub const SCRATCH: usize = HEADER + PAYLOAD;

/// The in-flight full-duplex transfer the HAL hands back from `read_write`.
type RxTxTransfer = Transfer<W, (RxWindow, TxWindow), Spi1RxTxDma>;

pub struct HalSpi {
    dma: Spi1RxTxDma,
    cs: Pin<'A', 9, Output>,
    sysclk_hz: u32,
}

impl HalSpi {
    pub fn new(
        spi: Spi<pac::SPI1, u8>,
        rx_ch: dma1::C2, // SPI1_RX
        tx_ch: dma1::C3, // SPI1_TX
        cs: Pin<'A', 9, Output>,
        sysclk_hz: u32,
    ) -> Self {
        Self {
            dma: spi.with_rx_tx_dma(rx_ch, tx_ch),
            cs,
            sysclk_hz,
        }
    }
}

impl DelayNs for HalSpi {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ((ns as u64 * self.sysclk_hz as u64) / 1_000_000_000) as u32;
        cortex_m::asm::delay(cycles.max(1));
    }
}

impl SpiDmaDevice for HalSpi {
    type Error = ErrorKind;
    type Transaction = HalTransaction;

    /// Start a full-duplex DMA of `buffers.len` bytes: `tx[..len]` out,
    /// captured into `rx[..len]`. Asserts CS for the duration (released by
    /// `wait`). The library guarantees one W6100 command per transfer, so the
    /// CS frame spans exactly one chip transaction.
    fn transceive(
        self,
        buffers: DmaBuffers,
    ) -> Result<Self::Transaction, (Self::Error, Self, DmaBuffers)> {
        let HalSpi {
            dma,
            mut cs,
            sysclk_hz,
        } = self;
        let DmaBuffers { rx, tx, len } = buffers;

        // Clear any stale SPI1_RX transfer-complete flag before arming: `wait()`
        // treats "TCIF set" as done and the HAL's global-clear doesn't stick on
        // this part, so a leftover flag would make `wait()` return early.
        dma.rxchannel.ifcr().write(|w| w.ctcif2().set_bit());

        cs.set_low();
        let transfer = dma.read_write(RxWindow { buf: rx, len }, TxWindow { buf: tx, len });

        Ok(HalTransaction {
            transfer,
            cs,
            sysclk_hz,
            len,
        })
    }
}

/// An in-flight transfer started by [`HalSpi::transceive`]. Holds the HAL
/// transfer plus the device's CS pin and clock so [`SpiDmaTransaction::wait`]
/// can reconstruct the `HalSpi` once the DMA completes.
pub struct HalTransaction {
    transfer: RxTxTransfer,
    cs: Pin<'A', 9, Output>,
    sysclk_hz: u32,
    len: usize,
}

impl SpiDmaTransaction<HalSpi> for HalTransaction {
    fn wait(self) -> (HalSpi, DmaBuffers) {
        let HalTransaction {
            transfer,
            mut cs,
            sysclk_hz,
            len,
        } = self;

        let ((rx_win, tx_win), dma) = transfer.wait();
        cs.set_high();

        (
            HalSpi {
                dma,
                cs,
                sysclk_hz,
            },
            DmaBuffers {
                rx: rx_win.buf,
                tx: tx_win.buf,
                len,
            },
        )
    }
}

/// The RX scratch slice exposed to the DMA as a write target of exactly `len`
/// bytes. `read_write` transfers the whole reported length, so the window caps
/// it at the active `len` while keeping the full backing slice to hand back.
struct RxWindow {
    buf: &'static mut [u8],
    len: usize,
}

/// The TX scratch slice exposed to the DMA as a read source of exactly `len`
/// bytes (same windowing rationale as [`RxWindow`]).
struct TxWindow {
    buf: &'static mut [u8],
    len: usize,
}

#[allow(unsafe_code)]
// SAFETY: `buf` is a valid `'static` slice and `len ≤ buf.len()`, so the
// reported `(ptr, len)` window of `u8` words stays inside it. `read_write` owns
// the window until `wait()` returns it, so nothing else touches the bytes during
// the transfer.
unsafe impl WriteBuffer for RxWindow {
    type Word = u8;
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.buf.as_mut_ptr(), self.len)
    }
}

#[allow(unsafe_code)]
// SAFETY: same window as above, read-only.
unsafe impl ReadBuffer for TxWindow {
    type Word = u8;
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.buf.as_ptr(), self.len)
    }
}
