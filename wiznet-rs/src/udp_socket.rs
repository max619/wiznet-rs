//! The UDP (datagram) protocol state machine and its user-facing handle.
//!
//! Structurally a sibling of [`tcp_socket`](crate::tcp_socket): a
//! [`UdpSocketState`] lives inside a [`SocketBackend`] slot owned by the
//! `W6100`, is driven by `run` (which has chip access), and is observed through
//! a [`UdpSocket`] handle. It shares the same socket [`SocketRings`] and the same
//! async bulk-DMA plumbing.
//!
//! **Datagram framing.** UDP is message-oriented, but the rings are byte SPSC
//! queues. To preserve datagram boundaries and the peer address without adding a
//! second data structure, every datagram is stored in the ring as a
//! length-prefixed **frame**:
//!
//! ```text
//! [ len_hi len_lo | ip0 ip1 ip2 ip3 | port_hi port_lo | payload (len bytes) ]
//!  \___________________ FRAME_HEADER (8 bytes) __________________/
//! ```
//!
//! `len` is the payload length; `ip`/`port` are the peer (source on rx, dest on
//! tx). The [`SpscRing`](crate::spsc_ring)'s frame counter publishes a frame only
//! once all its bytes are committed, so a reader/consumer that gates on
//! `frames() > 0` never sees a torn datagram.
//!
//! **The chip's per-packet header** (RX): in UDP mode the W6100 prepends a
//! PACKINFO header to each received datagram in its RX buffer. For a UDP4 socket
//! it is **8 bytes**:
//!
//! ```text
//!   bytes 0..2 : u16 big-endian — top 5 bits = packet-info flags
//!                (IPv6 / BRD-ALL / MUL / LLA), low 11 bits = UDP data length
//!   bytes 2..6 : peer IPv4 address   (datasheet calls it "Destination")
//!   bytes 6..8 : peer port           (datasheet calls it "Destination Port")
//! ```
//!
//! We read it synchronously to learn the peer and length, mask off the flag bits
//! (a UDP4 socket only receives IPv4), then DMA just the payload. (A UDP6/dual
//! socket would carry a 16-byte address here — out of scope for now.)

use crate::{
    DriverError, Error, SpiDmaDevice,
    atomic_cell::AtomicMutLock,
    socket::{BulkAction, BulkKind, Socket, SocketProtocolMode, SocketRings, SocketStatus},
    socket_common::{
        SocketCommand, SocketInterrupt, SocketStatusRegister, clear_interrupts, get_interrupts,
        get_rx_read_pointer, get_rx_received_size, get_tx_free_size, get_tx_write_pointer,
        init_socket, is_command_pending, read_rx_buffer, read_status, send_sock_command,
        set_dst_port, set_ipv4_dst_addr, set_rx_read_pointer, set_src_port, start_read_rx_buffer,
        start_write_tx_buffer,
    },
    transiver::{BlockAddress, Transceiver},
};

/// An IPv4 socket address: the peer of a datagram. Returned by
/// [`UdpSocket::recv_from`] and supplied to [`UdpSocket::send_to`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SocketAddr {
    /// IPv4 address in host order (as passed to [`crate::NetworkConfig`], i.e.
    /// `u32::from_be_bytes([a, b, c, d])`).
    pub ip: u32,
    pub port: u16,
}

/// Local frame header prepended to every datagram stored in a ring:
/// `[len(2) | ip(4) | port(2)]`. Distinct from the chip's PACKINFO header.
const FRAME_HEADER: usize = 8;

/// The W6100's per-packet PACKINFO header for a received UDP4 datagram:
/// `[flags:5|len:11 (2 bytes) | peer_ip(4) | peer_port(2)]` — see the module docs.
const PACKINFO: usize = 8;

/// Mask selecting the 11-bit UDP data length from the first PACKINFO word (the
/// top 5 bits are packet-info flags).
const PACKINFO_LEN_MASK: u16 = 0x07FF;

/// Build the local ring frame header for a datagram of `len` payload bytes
/// to/from `addr`.
fn frame_header(len: u16, addr: SocketAddr) -> [u8; FRAME_HEADER] {
    let mut h = [0u8; FRAME_HEADER];
    h[0..2].copy_from_slice(&len.to_be_bytes());
    h[2..6].copy_from_slice(&addr.ip.to_be_bytes());
    h[6..8].copy_from_slice(&addr.port.to_be_bytes());
    h
}

/// Parse a local ring frame header back into `(payload_len, peer)`.
fn parse_frame_header(h: &[u8; FRAME_HEADER]) -> (usize, SocketAddr) {
    let len = u16::from_be_bytes([h[0], h[1]]) as usize;
    let ip = u32::from_be_bytes([h[2], h[3], h[4], h[5]]);
    let port = u16::from_be_bytes([h[6], h[7]]);
    (len, SocketAddr { ip, port })
}

/// The UDP protocol state machine. Unlike TCP there is no connection: once
/// `OPEN`ed the socket sits in `Established` (chip status `SOCK_UDP`) and moves
/// datagrams in both directions until closed. Holds only protocol state — the
/// datagrams themselves live in the socket's [`SocketRings`].
pub(crate) struct UdpSocketState {
    src_port: u16,
    status: SocketStatus,

    pending_error: Option<Error>,

    /// Set by the handle's `close`; acted on by the next `run` tick.
    close_requested: bool,
}

impl UdpSocketState {
    pub(crate) fn bind(src_port: u16) -> Self {
        Self {
            src_port,
            status: SocketStatus::Init,
            pending_error: None,
            close_requested: false,
        }
    }

    /// The socket's status. For UDP, `Established` means "open and usable"
    /// (bound); there is no peer-connection to report.
    pub(crate) fn status(&self) -> SocketStatus {
        if self.pending_error.is_some() {
            SocketStatus::Error
        } else {
            self.status
        }
    }

    /// Re-arm so the next `run` re-opens the socket on the chip (after a
    /// link-loss reset).
    pub(crate) fn rearm(&mut self) {
        self.status = SocketStatus::Init;
        self.pending_error = None;
        self.close_requested = false;
    }

    /// Request a close on the next `run` tick.
    pub(crate) fn request_close(&mut self) {
        self.close_requested = true;
    }

    pub(crate) fn is_closed(&self) -> bool {
        matches!(
            self.status,
            SocketStatus::Closed | SocketStatus::Error | SocketStatus::Timeout
        )
    }

    /// Act on a pending close: UDP has no graceful shutdown, so just `CLOSE` the
    /// hardware socket and let the teardown machinery finish.
    fn handle_close<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
    ) -> Result<(), Error> {
        self.close_requested = false;

        match self.status {
            SocketStatus::Closed
            | SocketStatus::Error
            | SocketStatus::Timeout
            | SocketStatus::ClosingDueToError
            | SocketStatus::ClosingDueToTimeout
            | SocketStatus::Closing => Ok(()),

            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::Closing;

                Ok(())
            }
        }
    }

    /// Configure the hardware socket for UDP and issue `OPEN`. The source port
    /// must be set before `OPEN`.
    fn handle_init<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
    ) -> Result<(), Error> {
        init_socket(block, trans, SocketProtocolMode::UDP4)?;
        set_src_port(block, trans, self.src_port)?;

        send_sock_command(block, trans, SocketCommand::Open)?;
        self.status = SocketStatus::Opening;

        Ok(())
    }

    /// Wait for `OPEN` to move the socket into `SOCK_UDP`; then it is ready.
    fn handle_opening<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
    ) -> Result<(), Error> {
        if is_command_pending(block, trans)? {
            return Ok(());
        }

        match read_status(block, trans)? {
            SocketStatusRegister::Udp => {
                self.status = SocketStatus::Established;
                Ok(())
            }

            // Status register may lag a tick behind Sn_CR clearing; keep waiting.
            SocketStatusRegister::Closed => Ok(()),

            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::ClosingDueToError;

                Err(Error::Other(DriverError::UnexpectedResponse))
            }
        }
    }

    /// If a datagram is waiting on the chip and the rx ring has room for the
    /// whole frame, read its PACKINFO header synchronously and **start** an
    /// asynchronous DMA of just the payload. The frame (header + payload) is
    /// committed into the rx ring by `W6100::dma_complete`. One datagram per
    /// call. Non-blocking.
    fn receive<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        let available = get_rx_received_size(block, trans)? as usize;
        if available < PACKINFO {
            // No complete packet header yet.
            return Ok(BulkAction::None);
        }

        // Peek the chip's per-packet header (does not advance Sn_RX_RD).
        let start = get_rx_read_pointer(block, trans)?;
        let mut head = [0u8; PACKINFO];
        read_rx_buffer(block, trans, start, &mut head)?;
        // The first word packs the packet-info flags (top 5 bits) with the UDP
        // data length (low 11 bits); a UDP4 socket only receives IPv4, so we
        // just mask out the flags and read the peer that follows.
        let payload_len = (u16::from_be_bytes([head[0], head[1]]) & PACKINFO_LEN_MASK) as usize;
        let src_ip = u32::from_be_bytes([head[2], head[3], head[4], head[5]]);
        let src_port = u16::from_be_bytes([head[6], head[7]]);

        // The whole datagram must be present on the chip before we consume it.
        if available < PACKINFO + payload_len {
            return Ok(BulkAction::None);
        }

        let framed_len = FRAME_HEADER + payload_len;

        // A datagram whose frame is larger than the entire ring can never be
        // delivered, so back-pressure would wedge the chip's RX buffer forever
        // (blocking every datagram behind it). Drop it instead: skip the whole
        // packet on the chip and RECV, delivering nothing.
        if framed_len > rings.rx.capacity() {
            let end = start.wrapping_add((PACKINFO + payload_len) as u16);
            set_rx_read_pointer(block, trans, end)?;
            send_sock_command(block, trans, SocketCommand::Receive)?;
            clear_interrupts(block, trans, SocketInterrupt::RECV)?;
            return Ok(BulkAction::None);
        }

        // The frame fits the ring in principle but not right now: leave it on the
        // chip (back-pressure) and retry once the consumer has drained some.
        if rings.rx.free() < framed_len {
            return Ok(BulkAction::None);
        }

        // Empty datagrams are valid but carry no payload to DMA: stage the
        // frame synchronously and advance past the header.
        if payload_len == 0 {
            rings.rx.write(&frame_header(0, SocketAddr {
                ip: src_ip,
                port: src_port,
            }));
            rings.rx.commit_frame();

            set_rx_read_pointer(block, trans, start.wrapping_add(PACKINFO as u16))?;
            send_sock_command(block, trans, SocketCommand::Receive)?;
            clear_interrupts(block, trans, SocketInterrupt::RECV)?;

            return Ok(BulkAction::None);
        }

        // Payload starts right after the chip's PACKINFO header.
        let payload_ptr = start.wrapping_add(PACKINFO as u16);
        start_read_rx_buffer(block, trans, payload_ptr, payload_len)?;

        Ok(BulkAction::Started {
            kind: BulkKind::ReceiveDatagram { src_ip, src_port },
            pointer: payload_ptr,
            len: payload_len,
        })
    }

    /// If a datagram is staged in the tx ring and the chip has room for it,
    /// program the destination and **start** an asynchronous DMA of the payload
    /// out of the ring. The write pointer is committed and `SEND` issued by
    /// `W6100::dma_complete`. One datagram per call. Non-blocking.
    fn transmit<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        if rings.tx.frames() == 0 {
            return Ok(BulkAction::None);
        }

        // Inspect the frame header without consuming it: if the chip can't take
        // the whole datagram this tick we leave it staged and retry later.
        let mut head = [0u8; FRAME_HEADER];
        rings.tx.peek(&mut head);
        let (payload_len, dst) = parse_frame_header(&head);

        let free = get_tx_free_size(block, trans)? as usize;
        if free < payload_len {
            // Chip TX buffer can't fit the datagram yet — retry, nothing consumed.
            return Ok(BulkAction::None);
        }

        // Committed to sending: drain the frame header out of the ring.
        let mut sink = [0u8; FRAME_HEADER];
        rings.tx.read(&mut sink);

        set_ipv4_dst_addr(block, trans, dst.ip)?;
        set_dst_port(block, trans, dst.port)?;

        // Empty datagram: nothing to DMA, just SEND (write pointer unchanged).
        if payload_len == 0 {
            send_sock_command(block, trans, SocketCommand::Send)?;
            rings.tx.release_frame();
            return Ok(BulkAction::None);
        }

        let pointer = get_tx_write_pointer(block, trans)?;
        start_write_tx_buffer(block, trans, pointer, payload_len, |buf| {
            rings.tx.read(buf);
        })?;
        rings.tx.release_frame();

        Ok(BulkAction::Started {
            kind: BulkKind::Transmit,
            pointer,
            len: payload_len,
        })
    }

    /// Idle step for an open UDP socket: move a datagram in either direction. A
    /// bulk receive/transmit returns `Started` (bus now owned, finished from the
    /// DMA-complete interrupt).
    fn handle_established<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        // An ARP/send timeout drops the offending datagram but does not kill a
        // connectionless socket; just acknowledge it and carry on.
        if get_interrupts(block, trans)?.contains(SocketInterrupt::TIMEOUT) {
            clear_interrupts(block, trans, SocketInterrupt::TIMEOUT)?;
        }

        // Drain the chip first; only if nothing started do we push a datagram.
        match self.receive(block, trans, rings)? {
            action @ BulkAction::Started { .. } => Ok(action),
            BulkAction::None => self.transmit(block, trans, rings),
        }
    }

    fn handle_closing<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
    ) -> Result<(), Error> {
        if read_status(block, trans)? == SocketStatusRegister::Closed {
            self.status = match &self.status {
                SocketStatus::ClosingDueToError => SocketStatus::Error,
                SocketStatus::ClosingDueToTimeout => SocketStatus::Timeout,
                SocketStatus::Closing => SocketStatus::Closed,
                _ => panic!("Unexpected status"),
            }
        }

        Ok(())
    }

    /// Advance the state machine by one non-blocking tick. A swallowed (non
    /// `WouldBlock`) error is stored and surfaced through `status`.
    pub(crate) fn run<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        let result = if self.close_requested {
            self.handle_close(block, trans).map(|()| BulkAction::None)
        } else {
            match self.status {
                SocketStatus::Init => self.handle_init(block, trans).map(|()| BulkAction::None),
                SocketStatus::Opening => {
                    self.handle_opening(block, trans).map(|()| BulkAction::None)
                }
                SocketStatus::Established => self.handle_established(block, trans, rings),
                SocketStatus::ClosingDueToError
                | SocketStatus::ClosingDueToTimeout
                | SocketStatus::Closing => {
                    self.handle_closing(block, trans).map(|()| BulkAction::None)
                }

                // UDP never enters these TCP-only states.
                SocketStatus::Connecting
                | SocketStatus::Listening
                | SocketStatus::Timeout
                | SocketStatus::Closed
                | SocketStatus::Error => Ok(BulkAction::None),
            }
        };

        match result {
            Ok(action) => Ok(action),
            Err(Error::WouldBlock) => Err(Error::WouldBlock),
            e @ Err(_) => {
                self.pending_error = e.err();
                Ok(BulkAction::None)
            }
        }
    }
}

/// A lightweight, user-facing handle to a UDP socket. Holds only a reference to
/// the [`Socket`] slot owned by the `W6100`. `recv_from`/`send_to` touch the
/// lock-free rings directly (no cell); control ops go through the protocol cell.
pub struct UdpSocket<'a> {
    socket: &'a Socket<'a>,
}

impl<'a> UdpSocket<'a> {
    pub(crate) fn new(socket: &'a Socket<'a>) -> Self {
        Self { socket }
    }

    /// Receive the next buffered datagram into `dst`, returning the number of
    /// payload bytes copied and the sender's address, or `None` if none is
    /// waiting. If `dst` is smaller than the datagram the excess is discarded
    /// (UDP semantics). Lock-free, non-blocking.
    pub fn recv_from(&self, dst: &mut [u8]) -> Result<Option<(usize, SocketAddr)>, Error> {
        let rx = &self.socket.rings.rx;
        if rx.frames() == 0 {
            return Ok(None);
        }

        // A published frame is fully committed, so every read below is satisfied.
        let mut head = [0u8; FRAME_HEADER];
        rx.read(&mut head);
        let (payload_len, peer) = parse_frame_header(&head);

        let n = core::cmp::min(payload_len, dst.len());
        let mut copied = 0;
        while copied < n {
            copied += rx.read(&mut dst[copied..n]);
        }

        // Drop any payload bytes that didn't fit `dst`, keeping the ring aligned
        // on the next frame boundary.
        let mut discard = payload_len - n;
        let mut scratch = [0u8; 64];
        while discard > 0 {
            let take = core::cmp::min(discard, scratch.len());
            discard -= rx.read(&mut scratch[..take]);
        }

        rx.release_frame();
        Ok(Some((n, peer)))
    }

    /// Queue one datagram of `src` for transmission to `dst`, returning the
    /// number of payload bytes accepted (`src.len()` on success, `0` if the tx
    /// ring can't currently hold the whole datagram — retry later). The datagram
    /// is sent by `W6100::run`. A datagram larger than the tx ring capacity can
    /// never be staged and always returns `0`. Lock-free, non-blocking.
    pub fn send_to(&self, src: &[u8], dst: SocketAddr) -> Result<usize, Error> {
        let tx = &self.socket.rings.tx;
        let need = FRAME_HEADER + src.len();

        // Won't fit now (or ever, if larger than the ring): accept nothing.
        if tx.free() < need {
            return Ok(0);
        }

        // Room checked above, so both writes take everything.
        tx.write(&frame_header(src.len() as u16, dst));
        tx.write(src);
        tx.commit_frame();

        Ok(src.len())
    }

    pub fn status(&self) -> Result<SocketStatus, Error> {
        let mut guard = self.socket.backend.lock_mut()?;

        Ok(match guard.as_mut().as_udp_mut() {
            Some(udp) => udp.status(),
            None => SocketStatus::Closed,
        })
    }

    /// Request a close of the socket. Non-blocking: the `CLOSE` is issued by the
    /// next `W6100::run` tick; poll [`status`](Self::status) for `Closed`.
    pub fn close(&self) -> Result<(), Error> {
        let mut guard = self.socket.backend.lock_mut()?;

        if let Some(udp) = guard.as_mut().as_udp_mut() {
            udp.request_close();
        }

        Ok(())
    }

    /// Re-arm the socket so `W6100::run` re-opens it. Use after a link-loss
    /// `reset` to bring it back up.
    pub fn reconnect(&self) -> Result<(), Error> {
        let mut guard = self.socket.backend.lock_mut()?;

        if let Some(udp) = guard.as_mut().as_udp_mut() {
            udp.rearm();
            // Drop any datagrams left over from before the reset. Safe here: the
            // socket is being re-armed (not open), so neither the ISR nor `main`
            // is moving bytes through the rings.
            self.socket.rings.clear();
        }

        Ok(())
    }
}

impl<'a> Drop for UdpSocket<'a> {
    /// Releasing the handle hands the hardware socket back to the pool: the
    /// backend is marked for release so the next `W6100::run` ticks close it on
    /// the chip and return the slot to `Free`.
    fn drop(&mut self) {
        if let Ok(mut guard) = self.socket.backend.lock_mut() {
            guard.as_mut().request_release();
        }
    }
}
