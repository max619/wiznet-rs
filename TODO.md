# TODO — Async DMA for bulk SPI transfers (platform-independent)

## Status

- [x] **Phase 1** — `SpiDma` trait + move the SPI impl into the app + drive the
      bulk transfers over DMA (blocking). Builds clean; `w6100` stays HAL-free.
- [~] **Phase 1 correctness (in progress)** — bulk transfers now go through the
      HAL's `Spi1RxTxDma::read_write` full-duplex DMA (`src/hal_spi.rs`), all SPI
      (register ops included) routed through it. **Open bug:** SPI transactions
      appear not to wait for the DMA transfer to complete — investigating what
      `Transfer::wait()` keys off for `RxTxDma` (likely waits on the RX channel's
      transfer-complete; confirm it isn't returning before the SPI shift register
      drains / before TX has fully shifted out). Earlier symptom was echo garbage
      from byte misalignment (stale `RXNE`/`OVR` + enable ordering) — addressed by
      switching to the HAL's tested `read_write` path.
- [ ] **Phase 2** — make the bulk payload genuinely async (start → return →
      DMA-complete IRQ → finish), with one-in-flight coordination.

## Context

All chip I/O runs in `service()` on the TIM2/EXTI interrupts. The two bulk
transfers — `read_rx_buffer` / `write_tx_buffer` (up to the 512 B ring size) —
take ~4 ms each at 1 MHz SPI, with the CPU byte-banging the SPI data register
inside the ISR the whole time (so `main` is preempted). Goal: move the payload
phase to DMA and make it **asynchronous** — start it, return from the ISR (CPU
free), finish on a DMA-complete interrupt. Small register/command/pointer ops
(1–4 B) stay blocking; only the two payload bursts go async.

Two hard requirements shape the design:
- **`w6100` must stay platform-independent.** It already is, via the
  `Transceiver` trait (`src/w6100/transiver.rs`); express the async-DMA
  capability through a trait, not by importing HAL/DMA types into the module.
  (`embedded-hal-async::SpiBus` was considered but needs an async executor; our
  start/IRQ/finish model is a better fit for a small custom trait.)
- **The SPI bus is owned for the whole DMA transfer**, so while one is in flight
  *no* SPI of any kind (register polls, link checks, commands) may happen —
  `service()` must fully defer until completion. Only one transfer in flight
  (single bus).

The HAL safe DMA path (`SpiRxTxDma::read_write` → `dma::Transfer`) moves
ownership of bus + buffers and needs `'static` buffers, so the platform impl
keeps dedicated `'static` scratch buffers; DMA goes scratch↔chip and the driver
`memcpy`s scratch↔ring (cheap, ~µs).

## Design — layering

Reuse `embedded-hal` `SpiDevice` for the blocking path; add one small trait just
for DMA. Reset stays a plain `OutputPin` (it is not SPI). `w6100` stays generic
and HAL-free; the app supplies a concrete SPI type implementing both traits.

```
app (main.rs):  HalSpi  ── impl ──►  SpiDevice<u8> (blocking)  +  SpiDma (async)
w6100:          Transport<Spi: SpiDevice + SpiDma> ── impl ──► Transceiver  [frames Address]
w6100:          W6100<Spi: SpiDevice + SpiDma, RstPin: OutputPin>           [rst unchanged]
```

### 1. Trait `SpiDma` — async DMA only, byte-level (`src/w6100/transiver.rs`) — DONE
No W6100 concepts (no `Address`/block bits), no reset. Takes an opaque *header*
(short, clocked out blocking) plus the DMA payload; manages CS across the
transfer; owns scratch internally:
```rust
pub trait SpiDma {
    fn start_read(&mut self, header: &[u8], len: usize) -> Result<(), Error>;   // header then DMA-read `len`
    fn start_write(&mut self, header: &[u8], data: &[u8]) -> Result<(), Error>; // header then DMA-write (data copied in-call)
    fn finish(&mut self) -> Result<(), Error>;   // wait done (instant from IRQ), raise CS, free bus
    fn read_buffer(&self) -> &[u8];              // payload from last start_read
}
```
Blocking register ops keep using `embedded-hal` `SpiDevice::transaction`.

### 2. W6100 framing stays in `w6100`: `Transport<Spi: SpiDevice + SpiDma>` (`src/w6100/mod.rs`) — DONE
`Transport` builds the 3-byte header (addr BE + `(block<<3)|RWB|OM`, via
`header()`) and implements **`Transceiver`**: blocking `read`/`write` via
`SpiDevice`, plus `bulk_read`/`bulk_write` that frame the header and delegate to
`SpiDma`. `read_u8/u16/u32` helpers unchanged.

### 3. `W6100` keeps its generics; add the `SpiDma` bound (`src/w6100/mod.rs`) — DONE
`W6100<Spi: SpiDevice<u8> + SpiDma, RstPin: OutputPin>` — `Device { transport,
rst }` unchanged, `reset` still toggles the `RstPin`.

### 4. Platform impl `HalSpi` in the app (`src/hal_spi.rs`) — DONE (blocking)
A concrete type implementing **both** `SpiDevice<u8>` and `SpiDma`, replacing
`ExclusiveDevice`. Owns the DMA SPI (`with_rx_tx_dma`) + CS pin + `'static` rx/tx
scratch (`StaticCell`). A single `run(n)` helper does one blocking full-duplex
`read_write(...).wait()` of `n` bytes (length-`n` views of the scratch). For
Phase 2 this becomes an `Idle(SpiRxTxDma) | InFlight(Transfer<…>)` enum: `start_*`
kick off the DMA + enable the RX-channel TC interrupt and return; `finish`
`wait()`s, raises CS, returns to `Idle`. All HAL/DMA concretions live here only.

### 5. Async split + one-in-flight coordination (all platform-independent) — TODO (Phase 2)
- `receive`/`transmit` (`src/w6100/tcp_socket.rs`) split into start (regs →
  `trans.start_read/start_write` → record `PendingOp`) and finish (copy
  `trans.read_buffer()` → ring via `RingBuffer::writable`/`advance_write`, or it
  was already copied for tx → set pointer + `RECV`/`SEND`). New
  `SocketStatus::Receiving`/`Transmitting` (`src/w6100/socket.rs`); `run` treats
  them as no-ops.
- `W6100::service`: if a transfer is in flight, **return immediately, no SPI at
  all**; otherwise proceed and, the moment a socket starts a bulk op, stop the
  pass. In-flight bookkeeping (`Option<PendingOp{ socket, kind, len, ptr }>`)
  lives in the chip.
- `W6100::dma_complete(&self)` (called from the platform DMA IRQ via the app):
  lock the trans cell, `trans.finish()`, run the recorded finish step, clear
  in-flight, re-drive `service()`.
- Link-down `reset` must abort an in-flight transfer (`trans.finish()` + drop
  `PendingOp`) so nothing hangs.

### 6. Interrupt + init (`src/main.rs`) — TODO (Phase 2)
`let dma1 = dp.DMA1.split(&mut rcc)` [done]; build the SPI with `with_rx_tx_dma`
[done]; hand CS, rst, scratch to `HalSpi` [done]. Add `#[interrupt] fn
DMA1_CHANNEL2()` → cached chip → `chip.dma_complete()`; enable RX-channel TC +
`NVIC::unmask` [TODO]. Chip alias is `W6100<'static, HalSpi, Pin<'A',8,Output>>`.

## Notes
- A few scoped `#[allow(unsafe_code)]` blocks are needed in the **app** impl
  (matching the `NVIC::unmask` pattern); `w6100` stays safe.
- Phasing rationale: Phase 1 (blocking DMA, no IRQ) de-risks the boundary move +
  HAL DMA wiring before introducing async.

## Verification
- `cargo build` clean after each phase; confirm `w6100` has no `stm32f1xx-hal`
  imports (`grep -rn stm32f1xx_hal src/w6100/`) — proves platform independence.
- Echo test (`nc 192.168.10.10 5555`) identical behavior after each phase.
- Async win (Phase 2): watch PC13 / a spare GPIO around `wfi` to confirm `main`
  runs during a large transfer; verify a 512 B echo returns intact (length /
  handoff correct, no truncation/shift).
- Pull cable mid-transfer: link-down `reset` aborts the in-flight DMA cleanly.

## Out of scope / complementary
- SPI clock bump (1 MHz → ~9–18 MHz) shrinks every burst ~10–18×; orthogonal to
  DMA, can be done anytime. Biggest practical latency win regardless of DMA.
