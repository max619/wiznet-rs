use crate::{
    DriverError, Error, SpiDmaDevice,
    atomic_cell::AtomicMutLock,
    socket::{BulkAction, BulkKind, Socket, SocketProtocolMode, SocketRings, SocketStatus},
    socket_common::{
        SocketCommand, SocketInterrupt, SocketStatusRegister, clear_interrupts, get_interrupts,
        get_rx_read_pointer, get_rx_received_size, get_tx_free_size, get_tx_write_pointer,
        init_socket, is_command_pending, read_status, send_sock_command, set_dst_port,
        set_ipv4_dst_addr, set_src_port, set_tx_write_pointer, start_read_rx_buffer,
        start_write_tx_buffer, write_tx_buffer,
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

/// The TCP protocol state machine. Lives inside a [`SocketBackend`] slot owned by
/// the `W6100`; it is driven by `run` (which has chip access) and observed
/// through a [`TcpSocket`] handle. The handle's data path (`read`/`write`) no
/// longer goes through here — it touches the socket's lock-free
/// [`SocketRings`](crate::socket::SocketRings) directly — so this struct holds
/// only protocol state, not the rings.
pub(crate) struct TcpSocketState {
    mode: TcpMode,
    status: SocketStatus,

    pending_error: Option<Error>,

    /// Set by the handle's `close`; acted on by the next `run` tick (which has
    /// chip access).
    close_requested: bool,
}

impl TcpSocketState {
    pub(crate) fn connect(addr: u32, port: u16, src_port: u16) -> Self {
        Self {
            mode: Connect {
                dst_addr: addr,
                dst_port: port,
                src_port,
            },
            status: SocketStatus::Init,
            pending_error: None,
            close_requested: false,
        }
    }

    pub(crate) fn listen(port: u16) -> Self {
        Self {
            mode: Listen { src_port: port },
            status: SocketStatus::Init,
            pending_error: None,
            close_requested: false,
        }
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

    /// Act on a pending close request. For a live connection this sends a
    /// graceful FIN (`DISCON`) after one best-effort flush of staged tx data;
    /// for a socket still coming up it aborts with `CLOSE`. Either way we land
    /// in a closing state and the existing teardown machinery finishes the job.
    ///
    /// The flush is **synchronous** (the rare close path), so completion never
    /// has to mutate `status` from the DMA-complete interrupt.
    fn handle_close<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
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
                self.flush_sync(block, trans, rings)?;

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
    fn handle_init<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
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
    fn handle_opening<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
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

                Err(Error::Other(DriverError::UnexpectedResponse))
            }
        }
    }

    fn handle_connecting<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
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

                Err(Error::Other(DriverError::UnexpectedResponse))
            }
        }
    }

    /// Passively waiting on a `LISTEN`ing socket for a client to connect. Once
    /// the handshake completes the socket is `SOCK_ESTABLISHED` and is handed
    /// off to the same established machinery as an outbound connection.
    fn handle_listening<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
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

                Err(Error::Other(DriverError::UnexpectedResponse))
            }
        }
    }

    /// If the chip has received data and the local rx ring has room, **start** an
    /// asynchronous DMA of `min(available, ring free)` bytes from the chip into
    /// scratch. The bytes are delivered into the ring and the read pointer is
    /// committed by `W6100::dma_complete` once the DMA finishes. One transfer per
    /// call; the chip's RX buffer is re-checked on the next tick. Non-blocking.
    fn receive<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        let available = get_rx_received_size(block, trans)? as usize;
        if available == 0 {
            return Ok(BulkAction::None);
        }

        let to_read = core::cmp::min(available, rings.rx.free());
        if to_read == 0 {
            // No local room; leave the data on the chip and try again later.
            return Ok(BulkAction::None);
        }

        let pointer = get_rx_read_pointer(block, trans)?;
        start_read_rx_buffer(block, trans, pointer, to_read)?;

        Ok(BulkAction::Started {
            kind: BulkKind::Receive,
            pointer,
            len: to_read,
        })
    }

    /// If the local tx ring has staged data and the chip has room, **start** an
    /// asynchronous DMA of `min(pending, chip free)` bytes out of the ring (the
    /// `fill` closure drains it straight into scratch) to the chip. The write
    /// pointer is committed and `SEND` issued by `W6100::dma_complete` once the
    /// DMA finishes. Non-blocking.
    fn transmit<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        let pending = rings.tx.len();
        if pending == 0 {
            return Ok(BulkAction::None);
        }

        let free = get_tx_free_size(block, trans)? as usize;
        let to_send = core::cmp::min(pending, free);
        if to_send == 0 {
            // Chip TX buffer is full; leave the data staged and retry later.
            return Ok(BulkAction::None);
        }

        let pointer = get_tx_write_pointer(block, trans)?;
        start_write_tx_buffer(block, trans, pointer, to_send, |buf| {
            rings.tx.read(buf);
        })?;

        Ok(BulkAction::Started {
            kind: BulkKind::Transmit,
            pointer,
            len: to_send,
        })
    }

    /// Synchronous best-effort flush used by the graceful-close path: push as
    /// much staged tx data as fits onto the chip in one pass and `SEND`. Bounded
    /// and blocking — close is rare, and keeping it synchronous means the
    /// completion interrupt never has to change `status`.
    fn flush_sync<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<(), Error> {
        let pending = rings.tx.len();
        if pending == 0 {
            return Ok(());
        }

        let free = get_tx_free_size(block, trans)? as usize;
        let to_send = core::cmp::min(pending, free);
        if to_send == 0 {
            return Ok(());
        }

        let mut pointer = get_tx_write_pointer(block, trans)?;
        let mut staged = 0;
        let mut tmp = [0u8; 64];
        while staged < to_send {
            let n = core::cmp::min(tmp.len(), to_send - staged);
            let got = rings.tx.read(&mut tmp[..n]);
            if got == 0 {
                break;
            }

            write_tx_buffer(block, trans, pointer, &tmp[..got])?;
            pointer = pointer.wrapping_add(got as u16);
            staged += got;
        }

        set_tx_write_pointer(block, trans, pointer)?;
        send_sock_command(block, trans, SocketCommand::Send)?;

        Ok(())
    }

    /// Idle health-check for an established connection: watch for the failure
    /// and teardown conditions, then move data. A bulk receive/transmit returns
    /// `Started` (bus now owned, finished from the DMA-complete interrupt).
    fn handle_established<D: SpiDmaDevice>(
        &mut self,
        block: &BlockAddress,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        let interrupts = get_interrupts(block, trans)?;

        // Retransmission / keep-alive timeout: the peer is unreachable, so the
        // connection is dead. Tear it down and report it as a timeout.
        if interrupts.contains(SocketInterrupt::TIMEOUT) {
            clear_interrupts(block, trans, SocketInterrupt::TIMEOUT)?;

            send_sock_command(block, trans, SocketCommand::Close)?;
            self.status = SocketStatus::ClosingDueToTimeout;

            return Ok(BulkAction::None);
        }

        // FIN received from the peer. Acknowledge the interrupt; the status
        // register below tells us how far the teardown has progressed.
        if interrupts.contains(SocketInterrupt::DISCON) {
            clear_interrupts(block, trans, SocketInterrupt::DISCON)?;
        }

        match read_status(block, trans)? {
            // Still connected and healthy — pull down any pending data first;
            // only if nothing started asynchronously do we push staged tx data.
            SocketStatusRegister::Established => match self.receive(block, trans, rings)? {
                action @ BulkAction::Started { .. } => Ok(action),
                BulkAction::None => self.transmit(block, trans, rings),
            },

            // Peer initiated a graceful close (sent FIN). Keep draining the
            // chip's RX buffer; only close our side once every byte is off the
            // chip (the consumer may still drain the local ring afterwards).
            SocketStatusRegister::CloseWait => match self.receive(block, trans, rings)? {
                action @ BulkAction::Started { .. } => Ok(action),

                // In case we cant transmit on close wait, meybe there is something to recieve
                BulkAction::None => match self.transmit(block, trans, rings)? {
                    action @ BulkAction::Started { .. } => Ok(action),
                    BulkAction::None => {
                        if rings.tx.len() == 0
                            && rings.rx.len() == 0
                            && get_rx_received_size(block, trans)? == 0
                        {
                            send_sock_command(block, trans, SocketCommand::Disconnect)?;
                            self.status = SocketStatus::Closing;
                        }

                        Ok(BulkAction::None)
                    }
                },
            },

            // Connection already fully torn down.
            SocketStatusRegister::Closed => {
                self.status = SocketStatus::Closed;

                Ok(BulkAction::None)
            }

            // Any other state is unexpected for an established socket.
            _ => {
                send_sock_command(block, trans, SocketCommand::Close)?;
                self.status = SocketStatus::ClosingDueToError;

                Err(Error::Other(DriverError::UnexpectedResponse))
            }
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
            self.handle_close(block, trans, rings)
                .map(|()| BulkAction::None)
        } else {
            match self.status {
                SocketStatus::Init => self.handle_init(block, trans).map(|()| BulkAction::None),
                SocketStatus::Opening => {
                    self.handle_opening(block, trans).map(|()| BulkAction::None)
                }
                SocketStatus::Connecting => self
                    .handle_connecting(block, trans)
                    .map(|()| BulkAction::None),
                SocketStatus::Established => self.handle_established(block, trans, rings),
                SocketStatus::Listening => self
                    .handle_listening(block, trans)
                    .map(|()| BulkAction::None),
                SocketStatus::ClosingDueToError
                | SocketStatus::ClosingDueToTimeout
                | SocketStatus::Closing => {
                    self.handle_closing(block, trans).map(|()| BulkAction::None)
                }

                SocketStatus::Timeout | SocketStatus::Closed | SocketStatus::Error => {
                    Ok(BulkAction::None)
                }
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

/// A lightweight, user-facing handle to a TCP socket. Holds only a reference to
/// the [`Socket`] slot owned by the `W6100`. `read`/`write` touch the lock-free
/// rings directly (no cell); control ops (`status`/`close`/`reconnect`/`drop`)
/// go through the protocol cell.
pub struct TcpSocket<'a> {
    socket: &'a Socket<'a>,
}

impl<'a> TcpSocket<'a> {
    pub(crate) fn new(socket: &'a Socket<'a>) -> Self {
        Self { socket }
    }

    /// Drain up to `dst.len()` bytes already received and buffered locally.
    /// Returns the number copied (0 if nothing is buffered). Lock-free,
    /// non-blocking.
    pub fn read(&self, dst: &mut [u8]) -> Result<usize, Error> {
        Ok(self.socket.rings.rx.read(dst))
    }

    /// Queue up to `src.len()` bytes for transmission, returning the number
    /// accepted into the local tx ring. Lock-free, non-blocking: the staged data
    /// is pushed onto the chip and sent by `W6100::run`.
    pub fn write(&self, src: &[u8]) -> Result<usize, Error> {
        Ok(self.socket.rings.tx.write(src))
    }

    pub fn status(&self) -> Result<SocketStatus, Error> {
        let mut guard = self.socket.backend.lock_mut()?;

        Ok(match guard.as_mut().as_tcp_mut() {
            Some(tcp) => tcp.status(),
            None => SocketStatus::Closed,
        })
    }

    /// Request a graceful close of the connection. Non-blocking: the FIN is sent
    /// by the next `W6100::run` tick and the teardown completes asynchronously;
    /// poll [`status`](Self::status) for `Closed` to confirm.
    pub fn close(&self) -> Result<(), Error> {
        let mut guard = self.socket.backend.lock_mut()?;

        if let Some(tcp) = guard.as_mut().as_tcp_mut() {
            tcp.request_close();
        }

        Ok(())
    }

    /// Re-arm the socket so `W6100::run` re-opens and reconnects it. Use after a
    /// link-loss `reset` to bring the connection back up.
    pub fn reconnect(&self) -> Result<(), Error> {
        let mut guard = self.socket.backend.lock_mut()?;

        if let Some(tcp) = guard.as_mut().as_tcp_mut() {
            tcp.rearm();
            // Drop any bytes left over from the previous connection so the
            // reconnected session starts with empty rx/tx rings. Safe here: the
            // socket is being re-armed (not established), so neither the ISR nor
            // `main` is moving bytes through the rings.
            self.socket.rings.clear();
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
        if let Ok(mut guard) = self.socket.backend.lock_mut() {
            guard.as_mut().request_release();
        }
    }
}
