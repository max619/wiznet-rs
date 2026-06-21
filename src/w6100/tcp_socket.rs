use crate::w6100::{
    Error,
    atomic_cell::{AtomicCell, AtomicMutLock},
    ring_buffer::RingBuffer,
    socket::{SocketBackend, SocketProtocolMode, SocketStatus},
    socket_common::{
        SocketCommand, SocketInterrupt, SocketStatusRegister, clear_interrupts, get_interrupts,
        get_rx_read_pointer, get_rx_received_size, get_tx_free_size, get_tx_write_pointer,
        init_socket, is_command_pending, read_rx_buffer, read_status, send_sock_command,
        set_dst_port, set_ipv4_dst_addr, set_rx_read_pointer, set_src_port, set_tx_write_pointer,
        write_tx_buffer,
    },
    tcp_socket::TcpMode::{Connect, Listen},
    transiver::{BlockAddress, Transceiver},
};

enum TcpMode {
    Connect {
        dst_addr: u32,
        dst_port: u16,
        src_port: u16,
    },
    Listen {
        src_port: u16,
    },
}

/// The TCP protocol state machine and its buffers. Lives inside a
/// [`SocketBackend`] slot owned by the `W6100`; it is driven by `run` (which
/// has chip access) and read/written through a [`TcpSocket`] handle (which does
/// not — it only touches the local rings).
pub(crate) struct TcpSocketState<'a> {
    mode: TcpMode,
    status: SocketStatus,

    rx_buffer: RingBuffer<'a>,
    tx_buffer: RingBuffer<'a>,

    pending_error: Option<Error>,

    /// Set by the handle's `close`; acted on by the next `run` tick (which has
    /// chip access).
    close_requested: bool,
}

impl<'a> TcpSocketState<'a> {
    pub(crate) fn connect(
        addr: u32,
        port: u16,
        src_port: u16,
        rx_buffer: &'a mut [u8],
        tx_buffer: &'a mut [u8],
    ) -> Self {
        Self {
            mode: Connect {
                dst_addr: addr,
                dst_port: port,
                src_port,
            },
            status: SocketStatus::Init,

            rx_buffer: RingBuffer::new(rx_buffer),
            tx_buffer: RingBuffer::new(tx_buffer),

            pending_error: None,

            close_requested: false,
        }
    }

    pub(crate) fn listen(port: u16, rx_buffer: &'a mut [u8], tx_buffer: &'a mut [u8]) -> Self {
        Self {
            mode: Listen { src_port: port },
            status: SocketStatus::Init,

            rx_buffer: RingBuffer::new(rx_buffer),
            tx_buffer: RingBuffer::new(tx_buffer),

            pending_error: None,

            close_requested: false,
        }
    }

    /// Drain up to `dst.len()` already-received bytes from the local rx ring.
    pub(crate) fn read(&mut self, dst: &mut [u8]) -> usize {
        self.rx_buffer.read(dst)
    }

    /// Stage up to `src.len()` bytes into the local tx ring for transmission.
    pub(crate) fn write(&mut self, src: &[u8]) -> usize {
        self.tx_buffer.write(src)
    }

    pub(crate) fn status(&self) -> SocketStatus {
        if self.pending_error.is_some() {
            SocketStatus::Error
        } else {
            self.status
        }
    }

    /// Re-arm the socket so the next `run` re-opens it on the chip. Used after a
    /// link-loss reset to reconnect without dropping the handle or buffers.
    pub(crate) fn rearm(&mut self) {
        self.status = SocketStatus::Init;
        self.pending_error = None;
        self.close_requested = false;
    }

    /// Request a graceful close on the next `run` tick.
    pub(crate) fn request_close(&mut self) {
        self.close_requested = true;
    }

    /// Whether the connection has reached a terminal (closed) state.
    pub(crate) fn is_closed(&self) -> bool {
        matches!(
            self.status,
            SocketStatus::Closed | SocketStatus::Error | SocketStatus::Timeout
        )
    }

    fn store_error<F: FnMut(&mut Self) -> Result<(), Error>>(
        &mut self,
        mut f: F,
    ) -> Result<(), Error> {
        match f(self) {
            Ok(_) => Ok(()),
            Err(e) => match e {
                Error::Busy => Err(Error::Busy),
                e => {
                    self.pending_error = Some(e);
                    Ok(())
                }
            },
        }
    }

    /// Act on a pending close request. For a live connection this sends a
    /// graceful FIN (`DISCON`) after one best-effort flush of staged tx data;
    /// for a socket still coming up it aborts with `CLOSE`. Either way we land
    /// in a closing state and the existing teardown machinery finishes the job.
    fn handle_close<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        self.close_requested = false;

        match self.status {
            // Nothing live to tear down.
            SocketStatus::Closed
            | SocketStatus::Error
            | SocketStatus::Timeout
            | SocketStatus::ClosingDueToError
            | SocketStatus::ClosingDueToTimeout
            | SocketStatus::Closing => Ok(()),

            // Graceful shutdown of an established connection. Best-effort flush
            // (bounded: a single pass) so already-staged bytes go out ahead of
            // the FIN; anything that doesn't fit on the chip is dropped.
            SocketStatus::Established => {
                self.transmit(block, trans)?;

                send_sock_command(block, trans, SocketCommand::Disconnect)?;
                self.status = SocketStatus::Closing;

                Ok(())
            }

            // Abort a socket that is still opening/connecting/listening.
            SocketStatus::Init
            | SocketStatus::Opening
            | SocketStatus::Connecting
            | SocketStatus::Listening => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::Closing;

                Ok(())
            }
        }
    }

    /// Configure the hardware socket and issue `OPEN`. This is the first tick of
    /// a freshly opened (or re-armed) socket; the source port must be set before
    /// `OPEN`, and `CONNECT`/`LISTEN` are only valid once it reaches `SOCK_INIT`.
    fn handle_init<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        init_socket(block, trans, SocketProtocolMode::TCP4)?;

        let src_port = match &self.mode {
            TcpMode::Connect { src_port, .. } => *src_port,
            TcpMode::Listen { src_port } => *src_port,
        };
        set_src_port(block, trans, src_port)?;

        send_sock_command(block, trans, SocketCommand::Open)?;
        self.status = SocketStatus::Opening;

        Ok(())
    }

    /// Waiting for the `OPEN` command to move the socket into `SOCK_INIT`,
    /// then issuing the protocol-specific command (`CONNECT`/`LISTEN`).
    fn handle_opening<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        // `OPEN` not processed yet.
        if is_command_pending(block, trans)? {
            return Ok(());
        }

        match read_status(block, trans)? {
            SocketStatusRegister::Init => match &self.mode {
                TcpMode::Connect {
                    dst_addr, dst_port, ..
                } => {
                    set_ipv4_dst_addr(block, trans, *dst_addr)?;
                    set_dst_port(block, trans, *dst_port)?;

                    send_sock_command(block, trans, SocketCommand::Connect)?;
                    self.status = SocketStatus::Connecting;

                    Ok(())
                }
                TcpMode::Listen { .. } => {
                    send_sock_command(block, trans, SocketCommand::Listen)?;
                    self.status = SocketStatus::Listening;

                    Ok(())
                }
            },

            // Status register may lag a tick behind Sn_CR clearing; keep waiting.
            SocketStatusRegister::Closed => Ok(()),

            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::ClosingDueToError;

                Err(Error::UnexpectedResponse)
            }
        }
    }

    fn handle_connecting<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        if is_command_pending(block, trans)? {
            return Ok(());
        }

        // A connect timeout (ARP or SYN retries exhausted) is reported via the
        // TIMEOUT interrupt; the socket then falls back to SOCK_CLOSED.
        if get_interrupts(block, trans)?.contains(SocketInterrupt::TIMEOUT) {
            clear_interrupts(block, trans, SocketInterrupt::TIMEOUT)?;

            send_sock_command(block, trans, SocketCommand::Close)?;
            self.status = SocketStatus::ClosingDueToTimeout;

            return Ok(());
        }

        match read_status(block, trans)? {
            SocketStatusRegister::Established => {
                self.status = SocketStatus::Established;
                clear_interrupts(block, trans, SocketInterrupt::CON)?;

                Ok(())
            }

            // Handshake still in flight.
            SocketStatusRegister::Synsent | SocketStatusRegister::Init => Ok(()),

            // Reset/refused before the timeout fired.
            SocketStatusRegister::Closed => {
                self.status = SocketStatus::Closed;

                Ok(())
            }

            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::ClosingDueToError;

                Err(Error::UnexpectedResponse)
            }
        }
    }

    /// Passively waiting on a `LISTEN`ing socket for a client to connect. Once
    /// the handshake completes the socket is `SOCK_ESTABLISHED` and is handed
    /// off to the same established machinery as an outbound connection.
    fn handle_listening<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        if is_command_pending(block, trans)? {
            return Ok(());
        }

        match read_status(block, trans)? {
            // Still waiting for a client, or the SYN handshake is in progress.
            SocketStatusRegister::Listen | SocketStatusRegister::Synrecv => Ok(()),

            // A client connected. (Treat an immediate FIN the same — let the
            // established handler drain and tear it down next tick.)
            SocketStatusRegister::Established | SocketStatusRegister::CloseWait => {
                self.status = SocketStatus::Established;
                clear_interrupts(block, trans, SocketInterrupt::CON)?;

                Ok(())
            }

            // A half-open attempt was reset/timed out and the chip closed the
            // socket; re-arm to listen again.
            SocketStatusRegister::Closed => {
                clear_interrupts(block, trans, SocketInterrupt::TIMEOUT)?;
                self.status = SocketStatus::Closed;

                Ok(())
            }

            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::ClosingDueToError;

                Err(Error::UnexpectedResponse)
            }
        }
    }

    /// Move any data waiting in the chip's RX buffer into the local rx ring,
    /// limited by the room available there. Anything that doesn't fit is left
    /// on the chip and picked up on a later tick. Non-blocking.
    ///
    /// Returns the number of bytes still sitting in the chip's RX buffer after
    /// this pass (non-zero only when the local ring filled up). A return of `0`
    /// means the chip buffer is fully drained.
    fn receive<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<usize, Error> {
        let available = get_rx_received_size(block, trans)? as usize;
        if available == 0 {
            return Ok(0);
        }

        let to_read = core::cmp::min(available, self.rx_buffer.free());
        if to_read == 0 {
            // No local room; leave the data on the chip and try again later.
            return Ok(available);
        }

        // The local ring may wrap, so copy in up to two contiguous chunks. The
        // chip auto-advances/wraps its own buffer, so we just keep stepping the
        // read pointer by the amount consumed.
        let mut pointer = get_rx_read_pointer(block, trans)?;
        let mut remaining = to_read;
        while remaining > 0 {
            let region = self.rx_buffer.writable();
            let n = core::cmp::min(region.len(), remaining);

            read_rx_buffer(block, trans, pointer, &mut region[..n])?;
            self.rx_buffer.advance_write(n);

            pointer = pointer.wrapping_add(n as u16);
            remaining -= n;
        }

        // Commit the consumed bytes back to the chip and free its buffer.
        set_rx_read_pointer(block, trans, pointer)?;
        send_sock_command(block, trans, SocketCommand::Receive)?;
        clear_interrupts(block, trans, SocketInterrupt::RECV)?;

        Ok(available - to_read)
    }

    /// Push data staged in the local tx ring into the chip's TX buffer and
    /// kick off transmission, limited by the room available on the chip.
    /// Anything that doesn't fit stays in the ring for a later tick. Non-blocking.
    ///
    /// Returns the number of bytes still queued in the local tx ring after this
    /// pass (non-zero only when the chip's TX buffer filled up). A return of `0`
    /// means everything staged has been handed to the chip.
    fn transmit<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<usize, Error> {
        let pending = self.tx_buffer.len();
        if pending == 0 {
            return Ok(0);
        }

        let free = get_tx_free_size(block, trans)? as usize;
        let to_send = core::cmp::min(pending, free);
        if to_send == 0 {
            // Chip TX buffer is full; leave the data staged and retry later.
            return Ok(pending);
        }

        // The local ring may wrap, so copy out in up to two contiguous chunks.
        // The chip auto-advances/wraps its own buffer, so we just keep stepping
        // the write pointer by the amount staged.
        let mut pointer = get_tx_write_pointer(block, trans)?;
        let mut remaining = to_send;
        while remaining > 0 {
            let region = self.tx_buffer.readable();
            let n = core::cmp::min(region.len(), remaining);

            write_tx_buffer(block, trans, pointer, &region[..n])?;
            self.tx_buffer.advance_read(n);

            pointer = pointer.wrapping_add(n as u16);
            remaining -= n;
        }

        // Commit the new write pointer and tell the chip to send.
        set_tx_write_pointer(block, trans, pointer)?;
        send_sock_command(block, trans, SocketCommand::Send)?;

        Ok(pending - to_send)
    }

    /// Idle health-check for an established connection: watch for the failure
    /// and teardown conditions that can occur while we are not actively
    /// sending or receiving.
    fn handle_established<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        let interrupts = get_interrupts(block, trans)?;

        // Retransmission / keep-alive timeout: the peer is unreachable, so the
        // connection is dead. Tear it down and report it as a timeout.
        if interrupts.contains(SocketInterrupt::TIMEOUT) {
            clear_interrupts(block, trans, SocketInterrupt::TIMEOUT)?;

            send_sock_command(block, trans, SocketCommand::Close)?;
            self.status = SocketStatus::ClosingDueToTimeout;

            return Ok(());
        }

        // FIN received from the peer. Acknowledge the interrupt; the status
        // register below tells us how far the teardown has progressed.
        if interrupts.contains(SocketInterrupt::DISCON) {
            clear_interrupts(block, trans, SocketInterrupt::DISCON)?;
        }

        match read_status(block, trans)? {
            // Still connected and healthy — pull down any pending data and push
            // out anything we have staged to send.
            SocketStatusRegister::Established => {
                self.receive(block, trans)?;
                self.transmit(block, trans)?;

                Ok(())
            }

            // Peer initiated a graceful close (sent FIN). Keep draining the
            // chip's RX buffer into our local ring; only close our side once
            // every byte is off the chip (the consumer may still drain the
            // local ring afterwards). If the ring is full we stay here and
            // retry next tick once `read` frees space.
            SocketStatusRegister::CloseWait => {
                let pending = self.receive(block, trans)?;

                if pending == 0 {
                    send_sock_command(block, trans, SocketCommand::Disconnect)?;
                    self.status = SocketStatus::Closing;
                }

                Ok(())
            }

            // Connection already fully torn down.
            SocketStatusRegister::Closed => {
                self.status = SocketStatus::Closed;

                Ok(())
            }

            // Any other state is unexpected for an established socket.
            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::ClosingDueToError;

                Err(Error::UnexpectedResponse)
            }
        }
    }

    fn handle_closing<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
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

    /// Advance the state machine by one non-blocking tick.
    pub(crate) fn run<T: Transceiver>(
        &mut self,
        block: &BlockAddress,
        trans: &mut T,
    ) -> Result<(), Error> {
        self.store_error(|me| {
            if me.close_requested {
                return me.handle_close(block, trans);
            }

            match me.status {
                SocketStatus::Init => me.handle_init(block, trans),
                SocketStatus::Opening => me.handle_opening(block, trans),
                SocketStatus::Connecting => me.handle_connecting(block, trans),
                SocketStatus::Established => me.handle_established(block, trans),
                SocketStatus::Listening => me.handle_listening(block, trans),
                SocketStatus::ClosingDueToError
                | SocketStatus::ClosingDueToTimeout
                | SocketStatus::Closing => me.handle_closing(block, trans),

                SocketStatus::Timeout | SocketStatus::Closed | SocketStatus::Error => Ok(()),
            }
        })
    }
}

/// A lightweight, user-facing handle to a TCP socket. Holds only an atomic
/// reference to the backend slot owned by the `W6100`; all chip I/O happens in
/// `W6100::run`, so the handle's operations touch only the local ring buffers.
pub struct TcpSocket<'a> {
    backend: &'a AtomicCell<SocketBackend<'a>>,
}

impl<'a> TcpSocket<'a> {
    pub(crate) fn new(backend: &'a AtomicCell<SocketBackend<'a>>) -> Self {
        Self { backend }
    }

    /// Drain up to `dst.len()` bytes already received and buffered locally.
    /// Returns the number copied (0 if nothing is buffered). Non-blocking.
    pub fn read(&self, dst: &mut [u8]) -> Result<usize, Error> {
        let mut guard = self.backend.lock_mut()?;

        Ok(match guard.as_mut().as_tcp_mut() {
            Some(tcp) => tcp.read(dst),
            None => 0,
        })
    }

    /// Queue up to `src.len()` bytes for transmission, returning the number
    /// accepted into the local tx ring. Non-blocking: the staged data is pushed
    /// onto the chip and sent by `W6100::run`.
    pub fn write(&self, src: &[u8]) -> Result<usize, Error> {
        let mut guard = self.backend.lock_mut()?;

        Ok(match guard.as_mut().as_tcp_mut() {
            Some(tcp) => tcp.write(src),
            None => 0,
        })
    }

    pub fn status(&self) -> Result<SocketStatus, Error> {
        let mut guard = self.backend.lock_mut()?;

        Ok(match guard.as_mut().as_tcp_mut() {
            Some(tcp) => tcp.status(),
            None => SocketStatus::Closed,
        })
    }

    /// Request a graceful close of the connection. Non-blocking: the FIN is sent
    /// by the next `W6100::run` tick and the teardown completes asynchronously;
    /// poll [`status`](Self::status) for `Closed` to confirm.
    pub fn close(&self) -> Result<(), Error> {
        let mut guard = self.backend.lock_mut()?;

        if let Some(tcp) = guard.as_mut().as_tcp_mut() {
            tcp.request_close();
        }

        Ok(())
    }

    /// Re-arm the socket so `W6100::run` re-opens and reconnects it. Use after a
    /// link-loss `reset` to bring the connection back up.
    pub fn reconnect(&self) -> Result<(), Error> {
        let mut guard = self.backend.lock_mut()?;

        if let Some(tcp) = guard.as_mut().as_tcp_mut() {
            tcp.rearm();
        }

        Ok(())
    }
}

impl<'a> Drop for TcpSocket<'a> {
    /// Releasing the handle hands the hardware socket back to the pool: the
    /// backend is marked for release so the next `W6100::run` ticks gracefully
    /// close it on the chip and then return the slot to `Free` for reuse.
    fn drop(&mut self) {
        // Single-threaded cooperative use means the cell is not held elsewhere
        // at drop time; if it somehow is, the slot is reclaimed on the next
        // `reset` instead.
        if let Ok(mut guard) = self.backend.lock_mut() {
            guard.as_mut().request_release();
        }
    }
}
