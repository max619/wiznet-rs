//! Standalone SPI1 full-duplex DMA self-test for the blue pill —
//! **interrupt-driven**, mirroring the Phase 2 transport design.
//!
//! Purpose: prove the W6100 transport's bulk-DMA path (`src/hal_spi.rs`) moves
//! every byte correctly and in order — independent of the W6100 chip — and that
//! the *asynchronous* completion model works on this (likely cloned) MCU:
//! arm the transfer, `wfi` (CPU free), the RX DMA transfer-complete IRQ wakes us,
//! then we tear the transfer down and verify. Small enough to flash with full
//! debug info and step.
//!
//! Two clone gotchas this exercises together:
//!   1. The HAL's RxTx `wait()` keys completion off the RX channel's TCIF, and
//!      `start()` never clears it. A stale TCIF from the previous transfer makes
//!      `wait()` return early (truncated read) — and, with the IRQ enabled, fires
//!      the completion IRQ *immediately* on arm. We pre-clear TCIF before every
//!      transfer to prevent both.
//!   2. The TC IRQ must be quieted before returning or it storms. `stop()` only
//!      writes the DMA *global*-clear bit, which doesn't reliably stick on this
//!      part, so the handler tears the transfer down (disabling the channels)
//!      AND explicitly clears TCIF via the channel-specific `ctcif2` bit. With
//!      EN off and TCIF cleared, the IRQ line drops unconditionally.
//!
//! WIRING (required): jumper **MISO (PA6) ↔ MOSI (PA7)** so SPI loops back on
//! itself. Then whatever we DMA out must come straight back in: `rx == tx`.
//! Nothing else needs to be connected (the W6100 can stay attached; we don't
//! touch its CS/PA9, so it ignores the bus).
//!
//! RESULT (PC13 LED, active-low):
//!   - slow ~1 Hz blink  → all transfers verifying OK, test still running
//!   - fast ~10 Hz blink → a mismatch was latched; halts blinking the failure
//!
//! For detail, break in the debugger and inspect `report` (the `Report` struct):
//! `fails`, `last_fail_len`, `last_fail_idx`, `last_expected`, `last_got`,
//! `tx_snap`/`rx_snap`, plus `iterations`. With the loopback wire pulled you
//! should see it latch a fail — a quick sanity check that the comparison is real.
//!
//! Run: `cargo run --example spi_dma_loopback`  (or build + flash via the probe).

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::interrupt::{Mutex, free as interrupt_free};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use embedded_dma::{ReadBuffer, WriteBuffer};
use panic_halt as _;
use static_cell::StaticCell;
use stm32f1xx_hal::{
    dma::{ReadWriteDma, Transfer, W},
    gpio::{Output, Pin},
    pac::{self, interrupt},
    prelude::*,
    rcc, spi,
    spi::Spi1RxTxDma,
};

/// The in-flight full-duplex loopback transfer (RX + TX windows + the DMA).
type Loopback = Transfer<W, (SliceN, SliceN), Spi1RxTxDma>;
/// The torn-down resources the ISR hands back: the DMA and both buffers.
type Idle = (Spi1RxTxDma, &'static mut [u8; 512], &'static mut [u8; 512]);

/// Set by the `DMA1_CHANNEL2` (SPI1_RX transfer-complete) handler to wake the
/// thread that armed the transfer.
static DONE: AtomicBool = AtomicBool::new(false);
/// In-flight transfer, published by `main` for the ISR to tear down.
static INFLIGHT: Mutex<RefCell<Option<Loopback>>> = Mutex::new(RefCell::new(None));
/// Torn-down DMA + buffers, handed from the ISR back to `main`.
static RESULT: Mutex<RefCell<Option<Idle>>> = Mutex::new(RefCell::new(None));

/// Observable test state — inspect this in the debugger.
/// How many bytes of the failing transfer to snapshot for inspection.
const SNAP: usize = 32;

#[derive(Default)]
struct Report {
    iterations: u32,
    fails: u32,
    last_fail_len: usize,
    last_fail_idx: usize,
    last_expected: u8,
    last_got: u8,
    /// First `SNAP` bytes of the failing transfer: what we sent (`tx_snap`) vs
    /// what looped back (`rx_snap`). Compare side by side to read the failure
    /// shape — truncated tail (rx goes to 0xFF), a dropped byte (rx shifts by
    /// one from some index), or scattered corruption.
    tx_snap: [u8; SNAP],
    rx_snap: [u8; SNAP],
}

/// Transfer sizes exercised each round: single byte, the W6100 3-byte command
/// header size, an odd small size, and the full 512 B ring payload.
const SIZES: [usize; 4] = [1, 3, 17, 512];

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    // Same clock tree as the real firmware so timing matches.
    let mut rcc = dp.RCC.freeze(
        rcc::Config::hse(8.MHz()).sysclk(16.MHz()).pclk1(24.MHz()),
        &mut flash.acr,
    );
    let sysclk_hz = rcc.clocks.sysclk().raw();

    let gpio_a = dp.GPIOA.split(&mut rcc);
    let mut gpio_c = dp.GPIOC.split(&mut rcc);

    // PC13 status LED (active low), starts off.
    let mut led: Pin<'C', 13, Output> = gpio_c.pc13.into_push_pull_output(&mut gpio_c.crh);
    led.set_high();

    // SPI1: PA5 SCK, PA6 MISO, PA7 MOSI. Mode 0, 1 MHz — identical to main.rs.
    let spi_mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };
    let spi = dp.SPI1.spi(
        (Some(gpio_a.pa5), Some(gpio_a.pa6), Some(gpio_a.pa7)),
        spi_mode,
        16.MHz(),
        &mut rcc,
    );

    // DMA1: ch2 = SPI1_RX, ch3 = SPI1_TX.
    let dma1 = dp.DMA1.split(&mut rcc);
    let mut dma: Option<Spi1RxTxDma> = Some(spi.with_rx_tx_dma(dma1.2, dma1.3));

    // `'static` DMA buffers (read_write moves ownership in and hands them back).
    static RX: StaticCell<[u8; 512]> = StaticCell::new();
    static TX: StaticCell<[u8; 512]> = StaticCell::new();
    let mut rx: Option<&'static mut [u8; 512]> = Some(RX.init([0u8; 512]));
    let mut tx: Option<&'static mut [u8; 512]> = Some(TX.init([0u8; 512]));

    // Enable the SPI1_RX DMA transfer-complete interrupt at the NVIC. (`#[entry]`
    // already runs with interrupts globally unmasked; the channel's TCIE is
    // enabled once just below and stays on for the whole run.)
    #[allow(unsafe_code)]
    // SAFETY: just enabling the DMA-complete IRQ; nothing relies on it masked.
    unsafe {
        NVIC::unmask(pac::Interrupt::DMA1_CHANNEL2);
    }
    // Enable the RX channel's transfer-complete interrupt for the whole run.
    // The ISR tears each transfer down and clears TCIF, so this stays on.
    set_rx_tcie(true);

    let mut report = Report::default();

    loop {
        report.iterations = report.iterations.wrapping_add(1);
        let seed = report.iterations as u8;

        for &len in SIZES.iter() {
            let txb = tx.as_mut().unwrap();
            let rxb = rx.as_mut().unwrap();

            // Fresh, length/iteration-dependent pattern so a stale buffer or a
            // shifted/short transfer can't accidentally pass.
            for i in 0..len {
                txb[i] = (i as u8).wrapping_mul(0x1b).wrapping_add(seed);
            }
            rxb[..len].fill(0xFF); // sentinel: must be overwritten by loopback

            // Arm an interrupt-driven full-duplex DMA of exactly `len` bytes.
            // Clear any stale RX TCIF first (a leftover flag would fire the
            // completion IRQ immediately on arm); RX channel TCIE is already on.
            DONE.store(false, Ordering::Release);
            clear_rx_tcif();

            let d = dma.take().unwrap();
            let rx_owned = rx.take().unwrap();
            let tx_owned = tx.take().unwrap();
            // Start the DMA and publish it to the ISR atomically: `read_write`
            // starts both channels, so a short transfer could complete and fire
            // the IRQ before `INFLIGHT` is set. Masking interrupts across both
            // defers that IRQ until the transfer is published. CPU is free after.
            interrupt_free(|cs| {
                let transfer = d.read_write(SliceN(rx_owned, len), SliceN(tx_owned, len));
                INFLIGHT.borrow(cs).replace(Some(transfer));
            });

            // Sleep until the DMA-complete IRQ fires. This is the whole point of
            // the async path: `main` does nothing while the transfer runs.
            while !DONE.load(Ordering::Acquire) {
                cortex_m::asm::wfi();
            }

            // The ISR ran `wait()` on real completion (buffer fully written) and
            // handed back the DMA + buffers. Reclaim them for verify + reuse.
            let (d, rx_back, tx_back) =
                interrupt_free(|cs| RESULT.borrow(cs).borrow_mut().take()).unwrap();
            dma = Some(d);
            rx = Some(rx_back);
            tx = Some(tx_back);

            // Verify loopback: every byte out must have come back in order.
            let txb = tx.as_ref().unwrap();
            let rxb = rx.as_ref().unwrap();
            for i in 0..len {
                if rxb[i] != txb[i] {
                    report.fails = report.fails.wrapping_add(1);
                    report.last_fail_len = len;
                    report.last_fail_idx = i;
                    report.last_expected = txb[i];
                    report.last_got = rxb[i];
                    // Snapshot both buffers (first SNAP bytes) so the whole
                    // failure shape is visible at one breakpoint.
                    let n = len.min(SNAP);
                    report.tx_snap[..n].copy_from_slice(&txb[..n]);
                    report.rx_snap[..n].copy_from_slice(&rxb[..n]);
                    break;
                }
            }
        }

        // Status blink: fast & forever once anything failed, else slow heartbeat.
        if report.fails > 0 {
            blink(&mut led, sysclk_hz, 50);
        } else {
            blink(&mut led, sysclk_hz, 500);
        }
    }
}

/// Clear the DMA1 channel-2 (SPI1_RX) transfer-complete flag.
fn clear_rx_tcif() {
    #[allow(unsafe_code)]
    // SAFETY: DMA1 is a singleton; CTCIF2 is a write-1-to-clear status bit only,
    // no aliasing/ownership effect on memory.
    unsafe {
        (*pac::DMA1::ptr()).ifcr().write(|w| w.ctcif2().set_bit());
    }
}

/// Enable/disable the DMA1 channel-2 (SPI1_RX) transfer-complete interrupt.
///
/// NOTE the index mismatch: the pac's `ch(n)` accessor is 0-based, so SPI1_RX
/// (DMA1 *channel 2*) is `ch(1)`. The IFCR's named `ctcif2` field, by contrast,
/// is 1-based and really is channel 2 — hence `clear_rx_tcif` uses `ctcif2()`
/// but this uses `ch(1)`. (The HAL's own `Ch<DMA1, 1>` == its `C2`.)
fn set_rx_tcie(on: bool) {
    #[allow(unsafe_code)]
    // SAFETY: DMA1 singleton; only the SPI1_RX channel's TCIE control bit is
    // touched.
    unsafe {
        (*pac::DMA1::ptr())
            .ch(1)
            .cr()
            .modify(|_, w| w.tcie().bit(on));
    }
}

/// SPI1_RX DMA transfer-complete handler — the Phase 2 `DMA1_CHANNEL2` analog.
/// Tear the transfer down here (where the real firmware would run
/// `chip.dma_complete()` → `finish()`): TCIF is set on entry, so `wait()` is
/// instant and its `stop()` disables both channels. Then explicitly clear TCIF —
/// `stop()` only writes the global-clear bit, which on this clone doesn't
/// reliably stick, and a TCIF left set with TCIE enabled storms the IRQ.
#[interrupt]
fn DMA1_CHANNEL2() {
    interrupt_free(|cs| {
        if let Some(transfer) = INFLIGHT.borrow(cs).borrow_mut().take() {
            let ((rx_sl, tx_sl), dma) = transfer.wait();
            RESULT
                .borrow(cs)
                .borrow_mut()
                .replace((dma, rx_sl.0, tx_sl.0));
        }
    });
    clear_rx_tcif();
    DONE.store(true, Ordering::Release);
}

/// One LED on/off cycle, `half_period_ms` per half (busy-wait, examples only).
fn blink(led: &mut Pin<'C', 13, Output>, sysclk_hz: u32, half_period_ms: u32) {
    let cycles = (sysclk_hz / 1000) * half_period_ms;
    led.set_low(); // on
    cortex_m::asm::delay(cycles);
    led.set_high(); // off
    cortex_m::asm::delay(cycles);
}

/// Wraps a `&'static mut [u8; 512]` but exposes only its first `n` bytes to DMA,
/// so a single pair of 512 B buffers can drive transfers of any length ≤ 512.
struct SliceN(&'static mut [u8; 512], usize);

#[allow(unsafe_code)]
// SAFETY: `0` is a valid `'static` buffer and `self.1 ≤ 512`, so the reported
// (ptr, len) window of `u8` words stays inside the array.
unsafe impl WriteBuffer for SliceN {
    type Word = u8;
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.0.as_mut_ptr(), self.1)
    }
}

#[allow(unsafe_code)]
// SAFETY: same window as above, read-only.
unsafe impl ReadBuffer for SliceN {
    type Word = u8;
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.0.as_ptr(), self.1)
    }
}
