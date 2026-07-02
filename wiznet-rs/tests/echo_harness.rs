//! Host harness: end-to-end TCP-echo test for the DMA transport.
//!
//! This drives the **real** `W6100` driver through its public API against a
//! software model of the W6100 chip that sits behind the `SpiDmaDevice` trait —
//! the exact seam the firmware's `HalSpi` plugs into. It reproduces, on the host,
//! the full echo data path that `examples/tcp_echo` runs on hardware:
//!
//!   client bytes -> chip RX buffer -> [async DMA] -> rx ring -> echo ->
//!   tx ring -> [async DMA] -> chip TX buffer -> "sent" output
//!
//! and asserts the bytes that come back out equal the bytes that went in, in
//! order. A "mixed" stream (wrong offsets / wrap) or a "missing" stream
//! (dropped / mis-committed pointers) fails the comparison and reports the first
//! divergence — which is the symptom seen in `out.txt`.
//!
//! ## How the hardware maps onto this single-threaded loop
//!
//! On the target three interrupts cooperate:
//!   - `TIM2`/`EXTI15_10` -> `W6100::service()`  (starts one bulk DMA)
//!   - `DMA1_CHANNEL2`     -> `W6100::dma_complete()` (finishes it)
//!   - `main`             -> reads the rx ring, writes the tx ring (the echo)
//!
//! We invoke the same three entry points in a deterministic order. The chip
//! model performs each `transceive` synchronously (the captured payload is ready
//! the instant the "DMA" is started, just like the in-tree `transiver` mock), so
//! `dma_complete()` always has data waiting — `Completion::Interrupt` collapses
//! to an immediate completion, which is the worst case for ordering bugs.

use std::cell::RefCell;
use std::rc::Rc;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal::spi::ErrorKind;

use wiznet_rs::{
    Completion, DmaBuffers, NetworkConfig, SocketStatus, SpiDmaDevice, SpiDmaTransaction, W6100,
};

// ---------------------------------------------------------------------------
// Hardware constants (the chip contract the driver speaks). These are *not*
// importable from the driver crate (they are `pub(crate)`), so they are
// restated here exactly as the W6100 datasheet / `socket_common.rs` define them.
// Restating them is the point: the harness is an independent model of the same
// hardware the driver targets.
// ---------------------------------------------------------------------------

const HEADER: usize = 3;

/// Per-socket on-chip buffer size. Small enough that a multi-KB stream wraps the
/// circular buffer many times — the regime where pointer/offset bugs surface.
const BUF_SIZE: u16 = 2048;

// Common-block registers (full 16-bit address).
const CIDR: u16 = 0x0000; // chip id    -> 0x6100
const VER: u16 = 0x0002; // version     -> 0x4661
const PHYSR: u16 = 0x3000; // phy status -> link up

// Socket-register-block offsets.
const SN_MR: u16 = 0x0000;
const SN_CR: u16 = 0x0010;
const SN_IR: u16 = 0x0020;
const SN_IRCLR: u16 = 0x0028;
const SN_SR: u16 = 0x0030;
const SN_TX_FSR: u16 = 0x0204;
const SN_TX_WR: u16 = 0x020C;
const SN_RX_RSR: u16 = 0x0224;
const SN_RX_RD: u16 = 0x0228;
const SN_RX_WR: u16 = 0x022C;

// Socket status-register values.
const SR_CLOSED: u8 = 0x00;
const SR_INIT: u8 = 0x13;
const SR_LISTEN: u8 = 0x14;
const SR_ESTABLISHED: u8 = 0x17;

// Socket commands.
const CMD_OPEN: u8 = 0x01;
const CMD_LISTEN: u8 = 0x02;
const CMD_DISCONNECT: u8 = 0x08;
const CMD_CLOSE: u8 = 0x10;
const CMD_SEND: u8 = 0x20;
const CMD_RECV: u8 = 0x40;

// Socket-interrupt bits.
const IR_RECV: u8 = 1 << 2;
const IR_CON: u8 = 1 << 0;

// Block-kind nibble (low 2 bits of the 5-bit block selector).
const KIND_COMMON: u8 = 0b00;
const KIND_REG: u8 = 0b01;
const KIND_TX: u8 = 0b10;
const KIND_RX: u8 = 0b11;

// ---------------------------------------------------------------------------
// Software model of one hardware socket.
// ---------------------------------------------------------------------------

/// A circular byte buffer matching the W6100's free-running 16-bit pointers: the
/// live span is `write_ptr - read_ptr` (wrapping), and a pointer indexes the
/// backing store modulo its size. This is exactly how the chip's RX/TX FIFOs
/// behave, so any off-by-one or wrap mistake in the driver's pointer math shows
/// up here.
struct ChipBuffer {
    data: Vec<u8>,
    read_ptr: u16,
    write_ptr: u16,
}

impl ChipBuffer {
    fn new() -> Self {
        Self {
            data: vec![0u8; BUF_SIZE as usize],
            read_ptr: 0,
            write_ptr: 0,
        }
    }

    /// Bytes currently stored (RX: received size; TX: occupied).
    fn used(&self) -> u16 {
        self.write_ptr.wrapping_sub(self.read_ptr)
    }

    fn free(&self) -> u16 {
        BUF_SIZE - self.used()
    }

    /// Store `src` at `ptr` (wrapping within the FIFO), as the chip does when the
    /// host bursts data into a buffer or a peer's bytes land in RX.
    fn poke(&mut self, ptr: u16, src: &[u8]) {
        let cap = self.data.len();
        for (i, &b) in src.iter().enumerate() {
            self.data[(ptr as usize + i) % cap] = b;
        }
    }

    /// Fetch `dst.len()` bytes from `ptr` (wrapping), as the chip drives onto MISO
    /// during a burst read.
    fn peek(&self, ptr: u16, dst: &mut [u8]) {
        let cap = self.data.len();
        for (i, b) in dst.iter_mut().enumerate() {
            *b = self.data[(ptr as usize + i) % cap];
        }
    }
}

struct SocketModel {
    sr: u8,
    ir: u8,
    rx: ChipBuffer,
    tx: ChipBuffer,
}

impl SocketModel {
    fn new() -> Self {
        Self {
            sr: SR_CLOSED,
            ir: 0,
            rx: ChipBuffer::new(),
            tx: ChipBuffer::new(),
        }
    }
}

/// The whole chip: eight sockets plus the bytes the chip has "transmitted" onto
/// the wire (what a `nc` peer would receive).
struct ChipState {
    sockets: Vec<SocketModel>,
    /// Bytes the chip has SENT, per socket — the echo output under test.
    sent: Vec<Vec<u8>>,
}

impl ChipState {
    fn new() -> Self {
        ChipState {
            sockets: (0..8).map(|_| SocketModel::new()).collect(),
            sent: (0..8).map(|_| Vec::new()).collect(),
        }
    }

    /// Process a socket command register write, mutating chip state the way the
    /// W6100 would on `OPEN`/`LISTEN`/`SEND`/etc. The command register self-clears
    /// (reads back 0), so the driver's `is_command_pending` sees it as done.
    fn run_command(&mut self, sock: usize, cmd: u8) {
        match cmd {
            CMD_OPEN => self.sockets[sock].sr = SR_INIT,
            CMD_LISTEN => self.sockets[sock].sr = SR_LISTEN,
            CMD_CLOSE | CMD_DISCONNECT => self.sockets[sock].sr = SR_CLOSED,
            CMD_SEND => {
                // Transmit everything between TX_RD and TX_WR, then advance
                // TX_RD up to TX_WR (freeing the space), as a real SEND does.
                let tx = &mut self.sockets[sock].tx;
                let n = tx.used();
                let mut out = vec![0u8; n as usize];
                tx.peek(tx.read_ptr, &mut out);
                tx.read_ptr = tx.write_ptr;
                self.sent[sock].extend_from_slice(&out);
            }
            CMD_RECV => { /* RX_RD already written; RSR is derived live. */ }
            _ => {}
        }
    }

    /// A register read: return the (big-endian) value of width `out.len()`.
    fn read_reg(&self, sock: usize, addr: u16, out: &mut [u8]) {
        let s = &self.sockets[sock];
        let v: u32 = match addr {
            SN_CR => 0, // command register self-clears -> never pending
            SN_IR => s.ir as u32,
            SN_SR => s.sr as u32,
            SN_TX_FSR => s.tx.free() as u32,
            SN_TX_WR => s.tx.write_ptr as u32,
            SN_RX_RSR => s.rx.used() as u32,
            SN_RX_RD => s.rx.read_ptr as u32,
            SN_RX_WR => s.rx.write_ptr as u32,
            _ => 0,
        };
        let be = v.to_be_bytes();
        let w = out.len();
        out.copy_from_slice(&be[4 - w..]);
    }

    /// A register write: parse the big-endian value and apply it.
    fn write_reg(&mut self, sock: usize, addr: u16, payload: &[u8]) {
        let mut v: u32 = 0;
        for &b in payload {
            v = (v << 8) | b as u32;
        }
        match addr {
            SN_MR => {}
            SN_CR => self.run_command(sock, v as u8),
            SN_IRCLR => self.sockets[sock].ir &= !(v as u8),
            SN_TX_WR => self.sockets[sock].tx.write_ptr = v as u16,
            SN_RX_RD => self.sockets[sock].rx.read_ptr = v as u16,
            _ => {}
        }
    }

    /// A common-block register read (chip id / version / phy).
    fn read_common(&self, addr: u16, out: &mut [u8]) {
        let v: u32 = match addr {
            CIDR => 0x6100,
            VER => 0x4661,
            PHYSR => 0x01, // cable on, link up
            _ => 0,
        };
        let be = v.to_be_bytes();
        let w = out.len();
        out.copy_from_slice(&be[4 - w..]);
    }

    /// Execute one full-duplex SPI transaction the driver issued: parse the 3-byte
    /// W6100 header, then service the register/buffer access it encodes. `tx` is
    /// MOSI (header + any write payload), `rx` is MISO we must fill for reads.
    fn transact(&mut self, tx: &[u8], rx: &mut [u8], len: usize) {
        if len < HEADER {
            return;
        }
        let addr = u16::from_be_bytes([tx[0], tx[1]]);
        let control = tx[2];
        let is_write = control & 0b100 != 0;
        let block = control >> 3;
        let kind = block & 0b11;
        let sock = (block >> 2) as usize & 0b111;
        let plen = len - HEADER;

        match (kind, is_write) {
            (KIND_COMMON, false) => self.read_common(addr, &mut rx[HEADER..len]),
            (KIND_COMMON, true) => {} // network/lock/etc. — accepted, no model needed
            (KIND_REG, false) => self.read_reg(sock, addr, &mut rx[HEADER..len]),
            (KIND_REG, true) => self.write_reg(sock, addr, &tx[HEADER..len]),
            (KIND_TX, true) => self.sockets[sock].tx.poke(addr, &tx[HEADER..len]),
            (KIND_RX, false) => {
                let mut tmp = vec![0u8; plen];
                self.sockets[sock].rx.peek(addr, &mut tmp);
                rx[HEADER..len].copy_from_slice(&tmp);
            }
            // The driver never reads the TX FIFO or writes the RX FIFO over SPI.
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// `SpiDmaDevice` bridging the driver to the chip model.
// ---------------------------------------------------------------------------

struct MockSpi {
    chip: Rc<RefCell<ChipState>>,
}

struct MockTransaction {
    dev: MockSpi,
    buffers: DmaBuffers,
}

impl DelayNs for MockSpi {
    fn delay_ns(&mut self, _ns: u32) {}
}

impl SpiDmaTransaction<MockSpi> for MockTransaction {
    fn wait(self) -> (MockSpi, DmaBuffers) {
        (self.dev, self.buffers)
    }
}

impl SpiDmaDevice for MockSpi {
    type Error = ErrorKind;
    type Transaction = MockTransaction;

    fn transceive(
        self,
        buffers: DmaBuffers,
        _completion: Completion,
    ) -> Result<Self::Transaction, (Self::Error, Self, DmaBuffers)> {
        let DmaBuffers { rx, tx, len } = buffers;
        self.chip.borrow_mut().transact(tx, rx, len);
        Ok(MockTransaction {
            dev: self,
            buffers: DmaBuffers { rx, tx, len },
        })
    }
}

/// A reset pin that does nothing (the chip model is always present and ready).
struct MockRstPin;
impl ErrorType for MockRstPin {
    type Error = core::convert::Infallible;
}
impl OutputPin for MockRstPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Harness backdoors: the chip side a TCP peer would drive.
// ---------------------------------------------------------------------------

/// Leak a zeroed buffer so it satisfies the driver's `&'static mut [u8]`.
fn leak(n: usize) -> &'static mut [u8] {
    vec![0u8; n].leak()
}

/// Simulate a connected peer: push the socket from LISTEN to ESTABLISHED.
fn peer_connect(chip: &Rc<RefCell<ChipState>>, sock: usize) {
    let mut c = chip.borrow_mut();
    c.sockets[sock].sr = SR_ESTABLISHED;
    c.sockets[sock].ir |= IR_CON;
}

/// Simulate the peer sending `data` into the socket's RX FIFO, bounded by the
/// chip's free space (TCP flow control). Returns how many bytes were accepted.
fn peer_send(chip: &Rc<RefCell<ChipState>>, sock: usize, data: &[u8]) -> usize {
    let mut c = chip.borrow_mut();
    let free = c.sockets[sock].rx.free() as usize;
    let n = data.len().min(free);
    if n == 0 {
        return 0;
    }
    let wr = c.sockets[sock].rx.write_ptr;
    c.sockets[sock].rx.poke(wr, &data[..n]);
    c.sockets[sock].rx.write_ptr = wr.wrapping_add(n as u16);
    c.sockets[sock].ir |= IR_RECV;
    n
}

fn echoed_len(chip: &Rc<RefCell<ChipState>>, sock: usize) -> usize {
    chip.borrow().sent[sock].len()
}

// ---------------------------------------------------------------------------
// The harness driver.
// ---------------------------------------------------------------------------

/// Run a full echo of `input` through the driver and return the bytes the chip
/// transmitted back. `chunk` is how many bytes the simulated peer offers per
/// iteration (varying it exercises different RX-FIFO fill levels and wrap
/// alignments).
fn run_echo(input: &[u8], cfg: EchoCfg) -> Vec<u8> {
    let chip = Rc::new(RefCell::new(ChipState::new()));
    let spi = MockSpi { chip: chip.clone() };

    let scratch = DmaBuffers {
        rx: leak(HEADER + 512),
        tx: leak(HEADER + 512),
        len: 0,
    };
    let mac = [0xfc, 0xd7, 0xfd, 0xab, 0x8b, 0xe4];

    let w6100 = W6100::new(spi, MockRstPin, scratch, mac).expect("chip init");

    // Mirror `main`: provide addressing and open the listener (rings: 512 each,
    // same as the firmware).
    w6100
        .set_network_config(NetworkConfig {
            ip: u32::from_be_bytes([192, 168, 10, 10]),
            gateway: u32::from_be_bytes([192, 168, 10, 1]),
            subnet: u32::from_be_bytes([255, 255, 255, 0]),
        })
        .unwrap();
    let sock = w6100
        .open_tcp_listen(5555, leak(512), leak(512))
        .expect("open listener");

    // Bring the listener up and let the (simulated) peer connect. A bring-up step
    // is one servicing interrupt + its (no-op) DMA completion.
    let mut connected = false;
    for _ in 0..50 {
        let _ = w6100.service();
        w6100.dma_complete();
        match sock.status() {
            Ok(SocketStatus::Listening) if !connected => {
                peer_connect(&chip, 0);
                connected = true;
            }
            Ok(SocketStatus::Established) => break,
            _ => {}
        }
    }
    assert!(
        matches!(sock.status(), Ok(SocketStatus::Established)),
        "socket never reached Established"
    );

    // The `main`-thread echo step: read up to 16 bytes (exactly as the firmware
    // does) and write them back. Two modes:
    //   - faithful: `let _ = write(..)`, dropping whatever the tx ring rejects —
    //     bit-for-bit the firmware's `examples/tcp_echo` loop.
    //   - lossless: carry the remainder in `pending` so an app-level short write
    //     never drops data, isolating *transport* correctness from app backpressure.
    // Per-run echo state. `Lossless` uses `pending`; `FirmwareCarry` uses the
    // bounded `buf`/`off`/`len` triple (the exact firmware shape); `DropOnFull`
    // needs neither.
    let mut pending: Vec<u8> = Vec::new();
    let mut buf = [0u8; 16];
    let mut off = 0usize;
    let mut len = 0usize;
    let main_step = |pending: &mut Vec<u8>, buf: &mut [u8; 16], off: &mut usize, len: &mut usize| {
        if !matches!(sock.status(), Ok(SocketStatus::Established)) {
            return;
        }
        match cfg.app {
            AppLoop::DropOnFull => {
                let n = sock.read(buf).unwrap_or(0);
                if n > 0 {
                    let _ = sock.write(&buf[..n]);
                }
            }
            AppLoop::FirmwareCarry => {
                // Exactly the fixed firmware loop: refill only when drained, and
                // retry the remainder of a short write.
                if *off == *len {
                    *len = sock.read(buf).unwrap_or(0);
                    *off = 0;
                }
                if *off < *len {
                    *off += sock.write(&buf[*off..*len]).unwrap_or(0);
                }
            }
            AppLoop::Lossless => {
                let n = sock.read(buf).unwrap_or(0);
                pending.extend_from_slice(&buf[..n]);
                if !pending.is_empty() {
                    let accepted = sock.write(pending).unwrap_or(0);
                    pending.drain(..accepted);
                }
            }
        }
    };

    let total = input.len();
    let mut fed = 0usize;

    // Stall-based termination: once everything has been fed, keep ticking until
    // the output stops growing for a long stretch (rings + chip FIFOs fully
    // drained). This works whether or not bytes were dropped along the way.
    let mut last_echoed = 0usize;
    let mut stall = 0usize;
    let max_iters = total * 64 + 100_000;
    let mut iters = 0;

    loop {
        iters += 1;
        assert!(
            iters < max_iters,
            "echo did not converge: fed {fed}/{total}, echoed {}/{total}",
            echoed_len(&chip, 0)
        );

        // Peer offers more bytes.
        if fed < total {
            let end = (fed + cfg.chunk).min(total);
            fed += peer_send(&chip, 0, &input[fed..end]);
        }

        if cfg.interleave {
            // Faithful concurrency: `service` starts a bulk DMA, then `main` runs
            // *while that DMA is in flight* (touching only the lock-free rings),
            // and finally the DMA-complete interrupt finishes it. This is the
            // "main genuinely runs during a large transfer" path from CLAUDE.md.
            let _ = w6100.service();
            main_step(&mut pending, &mut buf, &mut off, &mut len);
            w6100.dma_complete();
        } else {
            // Sequential: the application acts, then a full background step runs
            // to completion before the next one.
            main_step(&mut pending, &mut buf, &mut off, &mut len);
            let _ = w6100.service();
            w6100.dma_complete();
        }

        let echoed = echoed_len(&chip, 0);
        if echoed == last_echoed {
            stall += 1;
        } else {
            stall = 0;
            last_echoed = echoed;
        }

        // No app-side bytes still waiting to be echoed (unbounded carry for
        // `Lossless`, the bounded `buf[off..len]` for `FirmwareCarry`).
        let carry_empty = pending.is_empty() && off == len;
        let drained = fed >= total && carry_empty;
        if drained && (echoed >= total || stall > 4096) {
            break;
        }
    }

    chip.borrow().sent[0].clone()
}

/// Which `main`-thread echo loop to drive.
#[derive(Clone, Copy, PartialEq)]
enum AppLoop {
    /// Read up to 16 bytes, carry any remainder the tx ring rejects in an
    /// unbounded buffer. Drops nothing; isolates *transport* correctness.
    Lossless,
    /// The current firmware loop: a bounded 16-byte staging buffer, refilled only
    /// once fully echoed, retrying short writes (`examples/tcp_echo/src/main.rs`).
    /// Also lossless, but with the exact bounded-carry logic the firmware uses.
    FirmwareCarry,
    /// The old firmware bug: read up to 16 bytes and `let _ = write(..)`, dropping
    /// whatever a full tx ring rejects.
    DropOnFull,
}

/// Knobs for one echo run.
#[derive(Clone, Copy)]
struct EchoCfg {
    /// Bytes the simulated peer offers per iteration.
    chunk: usize,
    /// Run `main` concurrently with an in-flight bulk DMA (vs. fully sequential).
    interleave: bool,
    app: AppLoop,
}

impl EchoCfg {
    /// Lossless, sequential — isolates transport framing/pointer correctness.
    fn transport(chunk: usize) -> Self {
        Self {
            chunk,
            interleave: false,
            app: AppLoop::Lossless,
        }
    }

    /// Lossless, but `main` runs during the in-flight DMA — exercises the SPSC
    /// ring handshake across the (simulated) interrupt boundary.
    fn concurrent(chunk: usize) -> Self {
        Self {
            chunk,
            interleave: true,
            app: AppLoop::Lossless,
        }
    }

    /// The current (fixed) firmware echo loop, concurrent.
    fn firmware(chunk: usize) -> Self {
        Self {
            chunk,
            interleave: true,
            app: AppLoop::FirmwareCarry,
        }
    }

    /// The old drop-on-full firmware loop, concurrent — the `out.txt` bug.
    fn dropping(chunk: usize) -> Self {
        Self {
            chunk,
            interleave: true,
            app: AppLoop::DropOnFull,
        }
    }
}

/// Report the first index where `got` diverges from `want`, with context.
fn assert_echo(want: &[u8], got: &[u8]) {
    if want == got {
        return;
    }
    let first = (0..want.len().min(got.len()))
        .find(|&i| want[i] != got[i])
        .unwrap_or(want.len().min(got.len()));
    let lo = first.saturating_sub(8);
    panic!(
        "echo mismatch: want {} bytes, got {} bytes; first diff at index {first}\n  want[{lo}..]: {:?}\n  got [{lo}..]: {:?}",
        want.len(),
        got.len(),
        &want[lo..(first + 8).min(want.len())],
        &got[lo..(first + 8).min(got.len())],
    );
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn echo_short_message() {
    let msg = b"hello, w6100";
    assert_echo(msg, &run_echo(msg, EchoCfg::transport(64)));
}

/// A counting pattern long enough to wrap the 2 KB chip FIFOs several times. The
/// value of each byte encodes its stream position (mod 256), so a *mixed* stream
/// is caught even when the length happens to match.
#[test]
fn echo_large_counting_stream() {
    let input: Vec<u8> = (0..8192).map(|i| (i % 251) as u8).collect();
    assert_echo(&input, &run_echo(&input, EchoCfg::transport(64)));
}

/// Same payload, but offered in awkward chunk sizes that land mid-FIFO and
/// mid-ring, stressing wrap alignment between the chip FIFO and the SPSC rings.
#[test]
fn echo_large_stream_odd_chunks() {
    let input: Vec<u8> = (0..6000).map(|i| (i % 251) as u8).collect();
    for &chunk in &[1usize, 7, 13, 17, 100, 333, 512] {
        assert_echo(&input, &run_echo(&input, EchoCfg::transport(chunk)));
    }
}

/// The real-world reproduction: echo the project's own `CLAUDE.md` (the file the
/// user piped through `nc`) and require it back byte-for-byte.
#[test]
fn echo_claude_md_file() {
    let input = include_bytes!("../../CLAUDE.md");
    assert_echo(input, &run_echo(input, EchoCfg::transport(128)));
}

// --- Concurrency: `main` runs *during* an in-flight bulk DMA --------------

/// The transport must survive `main` touching the SPSC rings while a bulk DMA is
/// in flight — the firmware's headline claim. Across many chunk alignments.
#[test]
fn echo_concurrent_with_inflight_dma() {
    let input: Vec<u8> = (0..6000).map(|i| (i % 251) as u8).collect();
    for &chunk in &[1usize, 7, 13, 16, 17, 64, 333, 512] {
        assert_echo(&input, &run_echo(&input, EchoCfg::concurrent(chunk)));
    }
}

/// The `CLAUDE.md` file again, but echoed concurrently (the `out.txt` scenario).
#[test]
fn echo_claude_md_concurrent() {
    let input = include_bytes!("../../CLAUDE.md");
    assert_echo(input, &run_echo(input, EchoCfg::concurrent(128)));
}

/// The fixed firmware echo loop (bounded 16-byte carry, retrying short writes)
/// must round-trip `CLAUDE.md` byte-for-byte under concurrency — the direct
/// regression test for the `out.txt` corruption.
#[test]
fn echo_firmware_loop_roundtrips() {
    let input = include_bytes!("../../CLAUDE.md");
    assert_echo(input, &run_echo(input, EchoCfg::firmware(128)));
}

/// Root-cause guard. The *old* firmware loop did `let _ = sock.write(..)`,
/// ignoring how many bytes the tx ring accepted and dropping the rest. Under a
/// fast sender the driver keeps *receiving* (it prioritizes RX over TX in
/// `handle_established`), so the tx ring fills and those dropped bytes vanish —
/// producing the "missing + mixed" stream in `out.txt`.
///
/// Every lossless/fixed variant above passes byte-for-byte over the same
/// transport and concurrency, which pins the loss on that dropped return value,
/// not the DMA path. This test guards the diagnosis: drop-on-full *does* corrupt.
#[test]
fn drop_on_full_corrupts_stream() {
    let input = include_bytes!("../../CLAUDE.md");
    let got = run_echo(input, EchoCfg::dropping(128));
    assert_ne!(
        input.as_slice(),
        got.as_slice(),
        "dropping on a full tx ring must lose data — this is the out.txt bug the \
         bounded-carry fix in examples/tcp_echo/src/main.rs prevents"
    );
}
