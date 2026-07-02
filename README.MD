# wiznet-rs

A platform-independent, `no_std` Rust driver for **WIZnet** hard-wired TCP/IP +
Ethernet controllers, built around a **DMA-based SPI transport with asynchronous
bulk transfers** and a **lock-free data path**. Ships with bare-metal example
firmware for an STM32F103 "blue pill" that runs TCP and UDP echo servers over a
W6100.

The driver does all the on-chip protocol work from interrupt handlers, so the
application thread only ever touches lightweight socket handles and sleeps — no
SPI, no busy-waiting, and it keeps running *during* a large DMA transfer.

## Workspace layout

This is a Cargo workspace with two members plus tooling:

| Path | What it is |
| --- | --- |
| [`wiznet-rs/`](wiznet-rs/) | The driver crate. Platform-independent, `no_std`. Depends only on `embedded-hal` traits + its own SPI/DMA transport traits — **never** on a specific HAL. |
| [`examples/echo/`](examples/echo/) | Bare-metal firmware (`no_std`/`no_main`) for an **STM32F103C8** (64K flash / 20K RAM) that uses the driver to run a **TCP echo server on port 5555** and a **UDP echo server on port 5556** over a W6100 on SPI1. This is the app / BSP layer. |
| [`examples/spi_dma_loopback.rs`](examples/spi_dma_loopback.rs) | Standalone SPI+DMA self-test (MISO↔MOSI loopback). Not a workspace member. |
| [`examples/echo/tools/echo_test.py`](examples/echo/tools/echo_test.py) | Host-side integrity + throughput tester for both echo servers (TCP stream + UDP datagrams). |

## Features

- **Platform-independent driver.** The `wiznet-rs` crate depends only on
  `embedded-hal` traits and its own small `SpiDmaDevice` / `SpiDmaTransaction`
  transport contract. All HAL, DMA, GPIO, and interrupt wiring lives in the app
  layer. The boundary is enforced in CI-style: `grep -rn stm32f1xx_hal
  wiznet-rs/src/` must return nothing.
- **Asynchronous DMA transport.** Bulk rx/tx payloads move over a full-duplex
  DMA that owns the bus for the transfer; the driver starts it, returns, and
  finishes it from a DMA-complete interrupt — so the CPU is free while bytes
  clock. Small register/command ops (1–4 bytes) still complete synchronously.
- **Lock-free data path.** Each socket's rx/tx buffers are lock-free SPSC rings
  split across the interrupt boundary, so the servicing ISR delivers received
  data without taking any lock and can never livelock against the application
  thread. `read`/`write` (and UDP `recv_from`/`send_to`) never block.
- **All 8 hardware sockets**, handed out as thin user-facing handles.
- **TCP and UDP.** TCP active-connect and passive-listen byte streams; UDP
  datagrams with boundaries and peer addressing preserved
  (`recv_from`/`send_to`), oversized-datagram dropping, and empty datagrams.
- **Runtime network configuration** via `set_network_config` (built to later be
  driven by DHCP), applied on link-up.
- **`#![deny(unsafe_code)]` crate-wide** in both crates; the few necessary
  `unsafe` blocks (NVIC unmask, DMA buffer views) are local `#[allow]`s in the
  **app** layer only.

## Supported chips

- [ ] W6300
- [x] W6100
- [ ] W5500
- [ ] W5300
- [ ] W5100S
- [ ] W5100

The driver's protocol state is an enum (`BackendState { Free, Tcp(..), Udp(..) }`),
so new protocols are added as variants rather than trait objects. Additional
chips fit the same transport abstraction.

## Building

The target (`thumbv7m-none-eabi`) and linker are configured in
`.cargo/config.toml`, so a plain build cross-compiles for the STM32:

```sh
cargo build                 # debug, whole workspace
cargo build --release       # release
cargo build -p wiznet-rs    # just the driver
```

> The dev profile deliberately builds *your* crates with debug info and
> dependencies at `opt-level = "z"` (size) — a debuggability/size split. See the
> root `Cargo.toml`.

## Flashing & running

```sh
cargo run -p echo
```

Uses the `probe-rs` runner (`--connect-under-reset --chip STM32F103C8Tx`) and
needs an attached debug probe (e.g. ST-Link). A VS Code launch config is in
[`.vscode/launch.json`](.vscode/launch.json). The onboard LED (PC13) lights while
a TCP client is connected.

## Testing

**Host unit + integration tests** (the platform-independent logic — SPSC rings,
the `Transceiver` framing/batching and async start/finish, and a full TCP/UDP
echo harness that drives the real driver against a software model of the chip):

```sh
cargo test -p wiznet-rs --target aarch64-apple-darwin   # or your host triple
```

The `--target` override is required because the workspace default target is
`thumbv7m-none-eabi`.

**On hardware**, exercise the flashed firmware directly:

```sh
nc 192.168.10.10 5555            # TCP echo — type, see it echoed back
nc -u 192.168.10.10 5556         # UDP echo — each datagram bounced back

# or the bundled integrity + throughput tester (both protocols):
./examples/echo/tools/echo_test.py
./examples/echo/tools/echo_test.py --proto udp --seconds 30
```

## Architecture

A short tour; the full design notes (ownership model, the WouldBlock-retry
try-lock cell, the transport stack, and hardware quirks) live in
[`CLAUDE.md`](CLAUDE.md).

- **Two layers, hard boundary.** `wiznet-rs/src/` is the driver (no HAL);
  `examples/echo/src/{hal_spi.rs,main.rs}` is the concrete SPI/DMA transport and
  interrupt wiring.
- **Inverted ownership.** The `W6100` owns the 8 socket slots; user code gets a
  thin handle whose data methods touch only the lock-free rings.
- **Everything on interrupts.** `W6100::service()` does all SPI (PHY link, config,
  socket state machines) and is called from a periodic timer tick, the W6100 INT
  line, and the DMA-complete IRQ. `main` is application-only and `wfi()`s.

## Status

The DMA async transport (start → return → DMA-complete IRQ → finish, one transfer
in flight) is implemented and exercised on hardware. TCP and UDP echo both run on
the STM32F103 + W6100 target.
