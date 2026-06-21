use core::marker::PhantomData;

use crate::w6100::{
    Error,
    socket::{SocketInternal, SocketProtocolMode, SocketStatus, UserSocket},
    ring_buffer::RingBuffer,
    socket_common::{
        SocketCommand, SocketInterrupt, SocketStatusRegister, clear_interrupts, get_interrupts,
        get_rx_read_pointer, get_rx_received_size, get_tx_free_size, get_tx_write_pointer,
        init_socket, is_command_pending, read_rx_buffer, read_status, send_sock_command,
        set_dst_port, set_ipv4_dst_addr, set_rx_read_pointer, set_src_port, set_tx_write_pointer,
        write_tx_buffer,
    },
    tcp_socket::TcpMode::{Connect, Listen},
    transiver::{Address, BlockAddress, Transceiver},
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

pub struct TcpSocket<'a, Trans: Transceiver> {
    mode: TcpMode,
    status: SocketStatus,

    rx_buffer: RingBuffer<'a>,
    tx_buffer: RingBuffer<'a>,

    pending_error: Option<Error>,

    _ph: PhantomData<Trans>,
}

impl<'a, Trans: Transceiver> TcpSocket<'a, Trans> {
    pub fn connect(
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

            _ph: PhantomData::<Trans>,
        }
    }

    pub fn listen(port: u16, rx_buffer: &'a mut [u8], tx_buffer: &'a mut [u8]) -> Self {
        Self {
            mode: Listen { src_port: port },
            status: SocketStatus::Init,

            rx_buffer: RingBuffer::new(rx_buffer),
            tx_buffer: RingBuffer::new(tx_buffer),

            pending_error: None,

            _ph: PhantomData::<Trans>,
        }
    }

    /// Drain up to `dst.len()` bytes that have been received and buffered
    /// locally. Returns the number of bytes copied (0 if nothing is buffered).
    /// Non-blocking: it only reads what [`run`](SocketInternal::run) has already
    /// pulled off the chip.
    pub fn read(&mut self, dst: &mut [u8]) -> usize {
        self.rx_buffer.read(dst)
    }

    /// Queue up to `src.len()` bytes for transmission, returning the number
    /// accepted into the local tx ring (limited by its free space). Non-blocking:
    /// the staged data is pushed onto the chip and sent by
    /// [`run`](SocketInternal::run).
    pub fn write(&mut self, src: &[u8]) -> usize {
        self.tx_buffer.write(src)
    }

    fn raise_pending_error(&mut self) -> Result<(), Error> {
        match self.pending_error {
            Some(e) => {
                let err = e;
                self.pending_error = None;

                Err(err)
            }
            None => Ok(()),
        }
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

    /// Waiting for the `OPEN` command to move the socket into `SOCK_INIT`,
    /// then issuing the protocol-specific command (`CONNECT`/`LISTEN`).
    fn handle_opening(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
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

    fn handle_connecting(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
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

    /// Move any data waiting in the chip's RX buffer into the local rx ring,
    /// limited by the room available there. Anything that doesn't fit is left
    /// on the chip and picked up on a later tick. Non-blocking.
    ///
    /// Returns the number of bytes still sitting in the chip's RX buffer after
    /// this pass (non-zero only when the local ring filled up). A return of `0`
    /// means the chip buffer is fully drained.
    fn receive(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<usize, Error> {
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
    fn transmit(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<usize, Error> {
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
    fn handle_established(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
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

    fn handle_closing(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
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
}

impl<'a, Trans: Transceiver> UserSocket<Trans> for TcpSocket<'a, Trans> {
    fn get_status(&self) -> SocketStatus {
        if self.pending_error.is_some() {
            SocketStatus::Error
        } else {
            self.status
        }
    }
}

impl<'a, Trans: Transceiver> SocketInternal<'a, Trans> for TcpSocket<'a, Trans> {
    fn init(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
        self.store_error(|me| {
            init_socket(block, trans, SocketProtocolMode::TCP4)?;

            // The source port must be set before OPEN; CONNECT/LISTEN are only
            // valid once the socket has reached SOCK_INIT.
            let src_port = match &me.mode {
                TcpMode::Connect { src_port, .. } => *src_port,
                TcpMode::Listen { src_port } => *src_port,
            };
            set_src_port(block, trans, src_port)?;

            send_sock_command(block, trans, SocketCommand::Open)?;
            me.status = SocketStatus::Opening;

            Ok(())
        })
    }

    fn run(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
        self.store_error(|me| match &me.status {
            SocketStatus::Opening => me.handle_opening(block, trans),
            SocketStatus::Connecting => me.handle_connecting(block, trans),
            SocketStatus::Established => me.handle_established(block, trans),
            SocketStatus::Listening => todo!(),
            SocketStatus::ClosingDueToError
            | SocketStatus::ClosingDueToTimeout
            | SocketStatus::Closing => me.handle_closing(block, trans),

            SocketStatus::Timeout
            | SocketStatus::Closed
            | SocketStatus::Init
            | SocketStatus::Error => Ok(()),
        })
    }
}
