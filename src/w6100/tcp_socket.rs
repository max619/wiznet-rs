use core::marker::PhantomData;

use crate::w6100::{
    Error,
    socket::{SocketInternal, SocketProtocolMode, SocketStatus, UserSocket},
    socket_common::{
        SocketCommand, SocketInterrupt, SocketStatusRegister, clear_interrupts, get_interrupts,
        init_socket, is_command_pending, read_status, send_sock_command, set_dst_port,
        set_ipv4_dst_addr,
    },
    tcp_socket::TcpMode::{Connect, Listen},
    transiver::{Address, BlockAddress, Transceiver},
};

enum TcpMode {
    Connect(u32, u16),
    Listen(u16),
}

pub struct TcpSocket<'a, Trans: Transceiver> {
    mode: TcpMode,
    status: SocketStatus,

    rx_buffer: &'a mut [u8],
    tx_buffer: &'a mut [u8],

    pending_error: Option<Error>,

    _ph: PhantomData<Trans>,
}

impl<'a, Trans: Transceiver> TcpSocket<'a, Trans> {
    pub fn connect(addr: u32, port: u16, rx_buffer: &'a mut [u8], tx_buffer: &'a mut [u8]) -> Self {
        Self {
            mode: Connect(addr, port),
            status: SocketStatus::Init,

            rx_buffer,
            tx_buffer,

            pending_error: None,

            _ph: PhantomData::<Trans>,
        }
    }

    pub fn listen(port: u16, rx_buffer: &'a mut [u8], tx_buffer: &'a mut [u8]) -> Self {
        Self {
            mode: Listen(port),
            status: SocketStatus::Init,

            rx_buffer,
            tx_buffer,

            pending_error: None,

            _ph: PhantomData::<Trans>,
        }
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

    fn handle_connecting(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
        if is_command_pending(block, trans)? {
            return Ok(());
        }

        let status = read_status(block, trans)?;
        if &status == &SocketStatusRegister::Established {
            self.status = SocketStatus::Established;
            clear_interrupts(block, trans, SocketInterrupt::CON)?;

            return Ok(());
        }

        if &status == &SocketStatusRegister::TimeWait {
            self.status = SocketStatus::Timeout;
            clear_interrupts(block, trans, SocketInterrupt::TIMEOUT)?;

            self.status = SocketStatus::ClosingDueToTimeout;

            return Ok(());
        }

        send_sock_command(block, trans, SocketCommand::Close)?;
        self.status = SocketStatus::ClosingDueToError;

        Err(Error::UnexpectedResponse)
    }

    fn handle_established(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
        Ok(())
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

            match &me.mode {
                TcpMode::Connect(addr, port) => {
                    set_ipv4_dst_addr(block, trans, *addr)?;
                    set_dst_port(block, trans, *port)?;

                    send_sock_command(block, trans, SocketCommand::Connect)?;

                    me.status = SocketStatus::Connecting;
                }

                TcpMode::Listen(port) => {
                    set_dst_port(block, trans, *port)?;

                    send_sock_command(block, trans, SocketCommand::Listen)?;

                    me.status = SocketStatus::Listening;
                }
            }

            Ok(())
        })
    }

    fn run(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error> {
        self.store_error(|me| match &me.status {
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
