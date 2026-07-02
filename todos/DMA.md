# TODO — Async DMA for bulk SPI transfers (platform-independent)

## Status

- [x] **Restructure into a workspace.** The driver moved out of the firmware's
      `src/w6100/` into its own crate **`wiznet-rs/`**; the firmware is now the
      **`examples/echo/`** member (`main.rs` + `hal_spi.rs`). The boundary is
      enforced the same way: `grep -rn stm32f1xx_hal wiznet-rs/src/` is empty.
- [x] **Transport trait redesign.** The old `SpiDevice` + `SpiDma`
      (`start_read`/`start_write`/`finish`/`read_buffer`) pair is **gone**.
      Replaced by two small traits in `wiznet-rs/src/spi_dma.rs`:
      ```rust
      pub struct DmaBuffers { pub rx: &'static mut [u8], pub tx: &'static mut [u8], pub len: usize }

      pub trait SpiDmaTransaction<Device> {
          fn wait(self) -> (Device, DmaBuffers);          // block for completion, return device + buffers
      }

      pub trait SpiDmaDevice: DelayNs + Sized {
          type Error: embedded_hal::spi::Error + From<ErrorKind>;
          type Transaction: SpiDmaTransaction<Self>;
          fn transceive(self, buffers: DmaBuffers)        // start one full-duplex DMA of `len` bytes
              -> Result<Self::Transaction, (Self::Error, Self, DmaBuffers)>;
      }
      ```
      The **scratch buffers now live in the driver** (`Transceiver` owns
      `DmaBuffers` inside an `AtomicCell<DmaState>`), not in the platform impl.
      The transport is byte-dumb: it clocks `len` bytes `tx → rx` and knows
      nothing of the W6100.
- [x] **`Transceiver` is now a concrete struct** (`wiznet-rs/src/transiver.rs`),
      not a trait. It builds the 3-byte header (`create_header`), batches a
      `&mut [Operation]` into the scratch buffers, runs one `transceive` per chunk
      that fits, and copies results back. `DmaState { Idle, InFlight, Pending }`
      is the seam for async (see Phase 2).
- [x] **Phase 1 correctness** — bulk transfers go through the HAL's
      `Spi1RxTxDma::read_write` full-duplex DMA (`examples/echo/src/hal_spi.rs`),
      all SPI (register ops included) routed through it. **Root-caused & fixed:**
      the early-return / truncated-read bug was a **stale RX transfer-complete
      flag**. `Transfer::wait()` for `RxTxDma` keys "done" off the RX channel's
      `TCIF` (`is_done() = !rxchannel.in_progress()`, `in_progress() = TCIF
      clear`), and `start()` never clears it. The HAL clears it only in `stop()`
      via the DMA **global**-clear bit (`CGIF`) — which does **not reliably stick
      on this (clone) MCU**. So a leftover `TCIF` from the previous transfer made
      the next `wait()` return immediately, reading a partially-filled buffer.
      **Fix:** clear the channel-specific `CTCIF2` before each transfer (see Clone
      hardware quirks). Validated in `examples/spi_dma_loopback.rs` (MISO↔MOSI
      loopback): all sizes intact, blocking and interrupt-driven, up to 16 MHz.
      **Confirmed on hardware** — echo server works over the blocking DMA path.
- [x] **`hal_spi.rs` ported to the new traits.** `HalSpi` holds only the DMA SPI
      + CS + clock; `transceive` clears `CTCIF2`, asserts CS, starts `read_write`,
      returns a `HalTransaction`; `wait()` blocks on the transfer, releases CS,
      and reconstructs `(HalSpi, DmaBuffers)`. `RxWindow`/`TxWindow` bound the DMA
      to the active `len`.
- [x] **Phase 2** — the two bulk payload transfers are now genuinely async
      (start → return → DMA-complete IRQ → finish), one-in-flight. Shape:
      - `SpiDmaDevice::transceive` gained a `Completion::{Poll, Interrupt}` flag.
        For `Interrupt` the app (`hal_spi.rs`) arms the SPI1_RX **TC interrupt**
        (`rxchannel.listen`) *before* enabling the channel — no window where the
        transfer could complete before the IRQ is armed — and disarms in `wait`
        (clears `CTCIF2` + `unlisten`) so a stale flag can't storm the IRQ.
      - `Transceiver` got a non-blocking bulk pair: `start_read`/`finish_read`
        (closure delivers the captured window) and `start_write`/`finish_write`
        (closure stages the payload), plus `abort` for teardown. `DmaState`
        `Idle ↔ InFlight` is the seam.
      - **Concurrency:** the rx/tx ring buffers were rewritten as a **lock-free
        SPSC** (`spsc_ring.rs`) and moved *out* of the `AtomicCell`-guarded
        socket state into a sibling `SocketRings`. `W6100::dma_complete` (called
        from the app's `DMA1_CHANNEL2` ISR) delivers captured bytes into the rx
        ring and commits the chip pointers **without ever taking a socket cell**,
        so it cannot livelock against `main`'s handle ops.
      - `W6100::run` records the in-flight transfer in `bulk: Option<BulkOp>`
        (socket idx + kind + block + pointer + len) and stops; `service` defers
        all SPI while `bulk` is `Some`; `reset` aborts an in-flight transfer.
        `handle_close`'s flush stays synchronous so completion never mutates
        `status`. **Not yet exercised on hardware** (build + host tests pass).

## Context

All chip I/O runs in `service()` on the TIM2/EXTI interrupts. The two bulk
transfers — `read_rx_buffer` / `write_tx_buffer` (up to the 512 B ring size) —
take ~4 ms each at 1 MHz SPI, with the transfer blocking inside the ISR the whole
time (so `main` is preempted). Goal: move the payload phase off the blocking
`wait()` and make it **asynchronous** — start it, return from the ISR (CPU free),
finish on a DMA-complete interrupt. Small register/command/pointer ops (1–4 B)
can keep completing synchronously; only the large payload bursts need to go async.

Two hard requirements shape the design:
- **`wiznet-rs` must stay platform-independent.** It already is, via the
  `SpiDmaDevice`/`SpiDmaTransaction` traits (`wiznet-rs/src/spi_dma.rs`); express
  the async-DMA capability through those, not by importing HAL/DMA types into the
  crate. (`embedded-hal-async::SpiBus` was considered but needs an async executor;
  our start/IRQ/finish model is a better fit for a small custom trait.)
- **The SPI bus is owned for the whole DMA transfer**, so while one is in flight
  *no* SPI of any kind (register polls, link checks, commands) may happen —
  `service()` must fully defer until completion. Only one transfer in flight
  (single bus).

The HAL safe DMA path (`Spi1RxTxDma::read_write` → `dma::Transfer`) moves
ownership of bus + buffers and needs `'static` buffers, so the driver keeps
dedicated `'static` scratch (`DmaBuffers`) it loans to the transport per transfer;
the driver `memcpy`s scratch↔ring (cheap, ~µs).

## Design — layering (current)

`wiznet-rs` stays generic and HAL-free; the app supplies a concrete device
implementing `SpiDmaDevice`. Reset stays a plain `OutputPin` (it is not SPI).

```
app (hal_spi.rs):  HalSpi ── impl ──► SpiDmaDevice + DelayNs ;  HalTransaction ── impl ──► SpiDmaTransaction
wiznet-rs:         Transceiver  [owns DmaBuffers, frames Address, batches Operations]
wiznet-rs:         W6100<Spi: SpiDmaDevice, RstPin: OutputPin>  [rst unchanged]
```

`W6100::new(spi, rst, scratch_buffers, mac)` takes the scratch `DmaBuffers` and
hands them to `Transceiver::new` (which asserts rx.len() == tx.len()).

## Phase 2 — async split + one-in-flight coordination (all platform-independent)

The `transceive`/`wait` shape already models in-flight work; today
`Transceiver::transaction` calls `transceive` and then `wait()`s immediately
within the same lock. Phase 2 splits that across calls using `DmaState`:

- **Start:** on a bulk op, `transceive` → store the returned `Transaction` in
  `DmaState::InFlight`, enable the RX-channel TC interrupt, and return
  `WouldBlock`. While `InFlight`, `transaction` returns `WouldBlock` and
  `service()` does **no SPI at all**.
- **Complete:** `W6100::dma_complete(&self)` (called from the platform DMA IRQ via
  the app) locks the device, `wait()`s the `Transaction` (instant — TCIF already
  set), copies scratch→ring for the recorded op, returns to `DmaState::Idle`, and
  re-drives `service()`.
- **Bookkeeping:** record which socket/op/len is pending so the completion step
  knows what to finish (`receive`/`transmit` in `tcp_socket.rs` split into
  start/finish; consider `SocketStatus::Receiving`/`Transmitting` treated as
  no-ops by `run`).
- **Abort on link-down:** `reset` must tear down an in-flight transfer (`wait()`
  + drop the pending op) so nothing hangs.
- **Interrupt + init (`examples/echo/src/main.rs`):** add `#[interrupt] fn
  DMA1_CHANNEL2()` → cached chip → `chip.dma_complete()`; enable the RX-channel
  TC interrupt + `NVIC::unmask`. Chip alias is
  `W6100<'static, HalSpi, Pin<'A', 8, Output>>`.

## Clone hardware quirks (STM32F103 "blue pill", likely a clone)

These cost real debugging time — keep them in mind for all DMA work:

- **DMA global-clear (`CGIF`) is unreliable.** The HAL's `dma::Ch::stop()` clears
  flags by writing `IFCR.CGIFx`. On this part the `TCIF` often stays set after
  that. **Always clear the channel-specific flag** (`IFCR.CTCIF<n>`) instead.
  stm32f1xx-hal 0.11 exposes **no** per-channel event-clear (no `clear_event`;
  that's stm32f4xx-hal), and the channel is hidden inside `Spi1RxTxDma`, so this
  is a direct `CTCIF2` register write — inlined at the top of `HalSpi::transceive`
  (`dma.rxchannel.ifcr().write(|w| w.ctcif2().set_bit())`). A stale `TCIF` bites
  twice: blocking `wait()` returns early, and (Phase 2) with `TCIE` enabled it
  fires the completion IRQ instantly on arm. Clear it before every transfer.
- **Index numbering split (easy to get wrong).** SPI1_RX is DMA1 **channel 2**:
  NVIC line `DMA1_CHANNEL2`, flag field `CTCIF2`/`TCIF2` (1-based), but the pac
  register accessor is `DMA1.ch(1)` (0-based). The HAL's `dma1::C2` == `Ch<_, 1>`.
- **`read_write` transfers the whole buffer.** `dma.read_write(&mut [u8; N], …)`
  always moves `N` bytes (the buffer's array length), not a sub-length. To send
  exactly `len` bytes, wrap the `'static` slice in a type whose `ReadBuffer`/
  `WriteBuffer` reports `len` (the `RxWindow`/`TxWindow` wrappers in
  `examples/echo/src/hal_spi.rs`). Getting this wrong clocks garbage past the
  intended frame (corrupts W6100 writes).
- **SPI clock:** loopback-verified clean at **16 MHz** on this board (not just
  the 1 MHz it is set to) — the size bump in `## Out of scope` is proven safe.

## Notes
- A few scoped `#[allow(unsafe_code)]` blocks are needed in the **app** impl
  (NVIC unmask, the `RxWindow`/`TxWindow` DMA views); `wiznet-rs` stays safe.
- Phasing rationale: Phase 1 (blocking DMA, no IRQ) de-risked the boundary move +
  HAL DMA wiring before introducing async.

## Verification
- `cargo build` clean; confirm `wiznet-rs` has no `stm32f1xx-hal` imports
  (`grep -rn stm32f1xx_hal wiznet-rs/src/`) — proves platform independence.
- Echo test (`nc 192.168.10.10 5555`) identical behavior after each phase.
- Async win (Phase 2): watch PC13 / a spare GPIO around `wfi` to confirm `main`
  runs during a large transfer; verify a 512 B echo returns intact (length /
  handoff correct, no truncation/shift).
- Pull cable mid-transfer: link-down `reset` aborts the in-flight DMA cleanly.

## Out of scope / complementary
- SPI clock bump (1 MHz → ~9–18 MHz) shrinks every burst ~10–18×; orthogonal to
  DMA, can be done anytime. Biggest practical latency win regardless of DMA.
