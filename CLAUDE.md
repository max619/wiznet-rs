# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Bare-metal Rust firmware (`no_std`/`no_main`) for an **STM32F103C8** ("blue pill",
64K flash / 20K RAM) driving a **WIZnet W6100** hardwired TCP/IP + Ethernet
controller over SPI. The app is a TCP echo server on port 5555.

## Commands

The target (`thumbv7m-none-eabi`) and linker are set in `.cargo/config.toml`, so:

- **Build:** `cargo build` (debug) / `cargo build --release`
- **Flash & run on hardware:** `cargo run` — uses the `probe-rs` runner
  (`--connect-under-reset --chip STM32F103C8Tx`); needs an attached SVD probe
  (e.g. ST-Link). VS Code launch config: `.vscode/launch.json` (probe-rs-debug).
- **Verify the driver stays platform-independent:** `grep -rn stm32f1xx_hal src/w6100/`
  must return nothing (see below).

There are **no automated tests** (it's `no_std` firmware). "Testing" means
flashing to the board and exercising the echo server: `nc 192.168.10.10 5555`,
type bytes, expect them echoed back. PC13 LED lights while a client is connected.

Dev profile note (`Cargo.toml`): your crate builds at `opt-level = 1` with debug
info; all dependencies build at `opt-level = "z"` with debug stripped. Don't
"fix" this — it's a deliberate size/debuggability split.

## Architecture

### Two layers, hard boundary

- **`src/w6100/` — the driver. Platform-independent; MUST NOT import `stm32f1xx-hal`.**
  It depends only on `embedded-hal` traits plus its own `Transceiver`/`SpiDma`
  traits. Keep it that way — the boundary is the whole point.
- **`src/hal_spi.rs` + `src/main.rs` — the app/BSP.** All `stm32f1xx-hal`, DMA,
  GPIO, and interrupt wiring lives here. `HalSpi` is the concrete SPI transport.

### Ownership is inverted (this surprises people)

`W6100` **owns** the 8 hardware socket slots as `[AtomicCell<SocketBackend>; 8]`.
A `SocketBackend` holds the real state: the protocol state machine and the rx/tx
ring buffers. User code receives a thin **handle** (`TcpSocket`) that only holds
`&'a AtomicCell<SocketBackend>`. Handle methods (`read`/`write`/`status`/`close`/
`reconnect`) touch **only the local ring buffers — never SPI**. Dropping a handle
marks the slot for release; `run` closes it on the chip and frees the slot.

### Interior mutability + the singleton + Busy-retry

`atomic_cell.rs` is a custom try-lock cell (an `AtomicBool` "busy" flag, not a
critical section). Every `W6100` method takes `&self`, so the chip can live as a
`&'static` singleton (via `StaticCell` in `main`) shared between `main` and the
interrupt handlers. Contention (e.g. a handle `read()` racing the ISR servicing
that socket) surfaces as **`Err(Busy)`** that the caller retries — chosen over
`cortex_m::interrupt::Mutex` precisely so interrupts are *not* disabled during the
long SPI transfers. `main` treats `Busy` as "try again next tick".

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

- `embedded-hal::SpiDevice` — blocking register ops (CS-managed transactions).
- `SpiDma` (`transiver.rs`) — byte-level async DMA path: `start_read`/`start_write`/
  `finish`/`read_buffer`. Deliberately knows nothing of W6100 (`Address`, block
  bits, reset are all W6100/GPIO concepts that stay above it).
- `HalSpi` (`hal_spi.rs`) — implements **both** of the above over the HAL's
  `Spi1RxTxDma::read_write` full-duplex DMA. The `'static` scratch buffers exist
  because the HAL's DMA API needs owned `'static` buffers; data is `memcpy`d
  scratch↔ring.
- `Transceiver` (`transiver.rs`) — the driver's `Address`-level API used by the
  socket code. `Transport<Spi>` implements it, building the W6100 3-byte command
  header (`header()`) and delegating bytes to `SpiDevice`/`SpiDma`.

> The bulk-DMA transport is under active development (moving from blocking
> `read_write().wait()` toward an async DMA-complete interrupt). While a DMA
> transfer is in flight the SPI bus is fully owned — **no other SPI of any kind**
> (register polls, commands) may run until it completes; `service()` must defer.

## Conventions / gotchas

- **Buffers are `&'static mut [u8]` handed to a socket at open and can only be
  handed out once** (`StaticCell`). So a socket is opened **once** and *re-armed*
  via `reconnect()` across link bounces — not reopened with fresh buffers.
- **Network addressing is set at runtime** via `set_network_config(...)` (built to
  later be driven by DHCP), not at `W6100::new`. `service()` applies it on link-up.
- The W6100 SPI write control byte must set **RWB = bit 2** (`(block << 3) | 0b100`);
  getting this wrong silently turns writes into reads.
- `#![deny(unsafe_code)]` is crate-wide; the few necessary `unsafe` blocks (NVIC
  unmask, DMA buffer views) are locally `#[allow(unsafe_code)]` in the **app**
  layer only.
- `main` declares the rx/tx buffers and chip in `StaticCell`s; the chip is shared
  with ISRs through `cortex_m::interrupt::Mutex<Cell<Option<&'static …>>>` globals.
