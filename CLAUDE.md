# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A Cargo **workspace** with two members:

- **`wiznet-rs/`** — a platform-independent, `no_std` Rust driver for WIZnet
  hardwired TCP/IP + Ethernet controllers (W6100 today; see `README.MD`), with
  DMA-based SPI transport and (in progress) async transfers.
- **`examples/tcp_echo/`** — bare-metal firmware (`no_std`/`no_main`) for an
  **STM32F103C8** ("blue pill", 64K flash / 20K RAM) that uses the driver to run
  a TCP echo server on port 5555 over a W6100 attached via SPI. This is the
  app/BSP layer.

`examples/spi_dma_loopback.rs` is a standalone (non-member) SPI+DMA self-test;
it isn't built by the workspace.

## Commands

The target (`thumbv7m-none-eabi`) and linker are set in `.cargo/config.toml`, so:

- **Build everything:** `cargo build` (debug) / `cargo build --release`
- **Build just the driver:** `cargo build -p wiznet-rs`
- **Flash & run on hardware:** `cargo run -p tcp_echo` — uses the `probe-rs`
  runner (`--connect-under-reset --chip STM32F103C8Tx`); needs an attached SVD
  probe (e.g. ST-Link). VS Code launch config: `.vscode/launch.json`.
- **Verify the driver stays platform-independent:**
  `grep -rn stm32f1xx_hal wiznet-rs/src/` must return nothing (see below).

There are **no automated tests** (it's `no_std` firmware). "Testing" means
flashing to the board and exercising the echo server: `nc 192.168.10.10 5555`,
type bytes, expect them echoed back. PC13 LED lights while a client is connected.

Dev profile note (root `Cargo.toml`): your crates build at `opt-level = 1` with
debug info; all dependencies build at `opt-level = "z"` with debug stripped.
Don't "fix" this — it's a deliberate size/debuggability split.

## Architecture

### Two layers, hard boundary

- **`wiznet-rs/src/` — the driver crate. Platform-independent; MUST NOT import
  `stm32f1xx-hal`.** It depends only on `embedded-hal` traits plus its own
  `SpiDmaDevice`/`SpiDmaTransaction` traits (`spi_dma.rs`). Keep it that way —
  the boundary is the whole point.
- **`examples/tcp_echo/src/hal_spi.rs` + `main.rs` — the app/BSP.** All
  `stm32f1xx-hal`, DMA, GPIO, and interrupt wiring lives here. `HalSpi` is the
  concrete SPI/DMA transport handed to the driver.

### Ownership is inverted (this surprises people)

`W6100` **owns** the 8 hardware socket slots as `[AtomicCell<SocketBackend>; 8]`.
A `SocketBackend` holds the real state: the protocol state machine and the rx/tx
ring buffers. User code receives a thin **handle** (`TcpSocket`) that only holds
`&'a AtomicCell<SocketBackend>`. Handle methods (`read`/`write`/`status`/`close`/
`reconnect`) touch **only the local ring buffers — never SPI**. Dropping a handle
marks the slot for release; `run` closes it on the chip and frees the slot.

### Interior mutability + the singleton + WouldBlock-retry

`atomic_cell.rs` is a custom try-lock cell (an `AtomicBool` "busy" flag, not a
critical section). Every `W6100` method takes `&self`, so the chip can live as a
`&'static` singleton (via `StaticCell` in `main`) shared between `main` and the
interrupt handlers. Contention (e.g. a handle `read()` racing the ISR servicing
that socket) surfaces as **`Err(nb::Error::WouldBlock)`** (an `AtomicError` maps
to it) that the caller retries — chosen over `cortex_m::interrupt::Mutex`
precisely so interrupts are *not* disabled during the long SPI transfers. `main`
treats `WouldBlock` as "try again next tick". The driver's error type is
`Error = nb::Error<DriverError>` (`error.rs`).

### All SPI runs in the background; `main` is application-only

`W6100::service()` does every SPI operation: it polls the PHY link, (re)applies
the runtime network config, and advances all socket state machines one step. It
is called from two interrupt handlers in `main.rs`:
- **`TIM2`** — 1 ms periodic tick (drives non-interrupt transitions + TX flush +
  missed-edge backstop).
- **`EXTI15_10`** — the W6100 INT line (PA10), low-latency wake on chip events.

`main`'s loop only calls handle methods (`status`/`read`/`write`/`reconnect`) and
`wfi()`s. It uses the *cached* link state `chip.link_up()` (no SPI) to gate setup.

### Socket state machines (`tcp_socket.rs`)

Non-blocking: each `run` tick advances one step
(`Init → Opening → Connecting`/`Listening → Established → Closing → Closed`).
`receive`/`transmit` move bytes between the chip's RX/TX buffers and the local
ring buffers (`ring_buffer.rs`) with back-pressure; the local ring is drained
before a graceful close so no received data is lost. Protocol variety is an
**enum** (`BackendState { Free, Tcp(TcpSocketState) }` in `socket.rs`) — add UDP
etc. as a new variant, no trait objects.

### SPI/DMA transport stack (bottom to top)

- `SpiDmaDevice` + `SpiDmaTransaction` (`wiznet-rs/src/spi_dma.rs`) — the
  app-side contract. `SpiDmaDevice` is a `DelayNs` device that **consumes
  itself** and runs one full-duplex DMA over `DmaBuffers` (the rx/tx scratch
  slices plus an active `len`), returning a `SpiDmaTransaction`. `wait()` blocks
  for completion (Phase 2: a DMA-complete IRQ) and hands the device **and**
  buffers back. Deliberately knows nothing of W6100 (`Address`, block bits,
  reset are all driver/GPIO concepts that stay above it).
- `HalSpi` (`examples/tcp_echo/src/hal_spi.rs`) — concrete impl over the HAL's
  `Spi1RxTxDma::read_write`. Holds only the DMA SPI, the CS pin, and the clock;
  it manages CS across a transfer (assert in `transceive`, release in `wait`).
  The `'static` scratch lives in the driver, not here. `RxWindow`/`TxWindow`
  cap the DMA to the active `len` (see quirks).
- `Transceiver` (`wiznet-rs/src/transiver.rs`) — a **concrete struct** (not a
  trait) that **owns** the `DmaBuffers` scratch inside an `AtomicCell<DmaState>`.
  It exposes the driver's `Address`-level API (`read`/`write`/`read_u8/16/32`/
  `write_u8/16/32`) plus a generic `transaction(&mut [Operation])`. It builds the
  W6100 3-byte command header (`create_header`), batches operations into the
  scratch buffers, and runs **one full-duplex `transceive` per chunk that fits**,
  then copies the captured bytes back into the read operands.

> The bulk-DMA transport is under active development (moving from a blocking
> `transceive(...)` + immediate `wait()` toward a true async DMA-complete
> interrupt; the `DmaState::{Idle, InFlight, Pending}` machinery in
> `transiver.rs` is the seam for it). While a DMA transfer is in flight the SPI
> bus is fully owned — **no other SPI of any kind** (register polls, commands)
> may run until it completes; `service()` must defer. See `TODO.md`.

## Conventions / gotchas

- **Buffers are `&'static mut [u8]` handed to a socket at open and can only be
  handed out once** (`StaticCell`). So a socket is opened **once** and *re-armed*
  via `reconnect()` across link bounces — not reopened with fresh buffers.
- **DMA scratch lives in the driver.** The app builds a `DmaBuffers { rx, tx,
  len }` (rx/tx are equal-length `&'static mut [u8]`) and passes it to
  `W6100::new(spi, rst, scratch_buffers, mac)`; the `Transceiver` owns it and
  loans it to the transport per transfer. `HalSpi` keeps no buffers of its own.
- **Network addressing is set at runtime** via `set_network_config(...)` (built to
  later be driven by DHCP), not at `W6100::new`. `service()` applies it on link-up.
- The W6100 SPI write control byte must set **RWB = bit 2** (`(block << 3) | 0b100`);
  getting this wrong silently turns writes into reads (see `create_header`).
- `#![deny(unsafe_code)]` is crate-wide in **both** crates; the few necessary
  `unsafe` blocks (NVIC unmask, the `RxWindow`/`TxWindow` DMA buffer views) are
  locally `#[allow(unsafe_code)]` in the **app** layer only.
- `main` declares the socket rx/tx buffers, the DMA scratch, and the chip in
  `StaticCell`s; the chip is shared with ISRs through
  `cortex_m::interrupt::Mutex<Cell<Option<&'static …>>>` globals.
