use bitflags::bitflags;

use crate::w6100::{
    Error,
    socket_common::init_socket,
    tcp_socket::TcpSocketState,
    transiver::{BlockAddress, Transceiver},
};

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct SocketProtocolMode: u8 {
        const CLOSED = 0b0000;
        const TCP4 = 0b0001;
        const UDP4 = 0b0010;
        const IPRAW4 = 0b0011;
        const MACRAW = 0b0111;
        const TCP6 = 0b1001;
        const UDP6 = 0b1010;
        const IPRAW6 = 0b1011;
        const TCP_DUAL = 0b1101;
        const UDP_DUAL = 0b1110;
        const MASK = 0b1111;
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SocketStatus {
    Init,

    Opening,
    Connecting,
    Established,

    Listening,

    ClosingDueToError,
    ClosingDueToTimeout,
    Closing,

    Timeout,
    Error,
    Closed,
}

/// The protocol-specific state machine living inside a [`SocketBackend`] slot.
///
/// Adding a new protocol is a matter of introducing a `*SocketState` module and
/// a variant here; [`SocketBackend::run`] (and the chip driver above it) keep
/// the same shape.
pub(crate) enum BackendState<'a> {
    Free,
    Tcp(TcpSocketState<'a>),
}

/// One of the chip's eight hardware sockets. Owned by the `W6100`; user-facing
/// handles only hold an atomic reference to the enclosing cell.
pub(crate) struct SocketBackend<'a> {
    block: BlockAddress,
    state: BackendState<'a>,
}

impl<'a> SocketBackend<'a> {
    pub(crate) fn new(block: BlockAddress) -> Self {
        Self {
            block,
            state: BackendState::Free,
        }
    }

    pub(crate) fn is_free(&self) -> bool {
        matches!(self.state, BackendState::Free)
    }

    /// Claim a free slot for a TCP socket. Returns `false` if already in use.
    pub(crate) fn claim_tcp(&mut self, tcp: TcpSocketState<'a>) -> bool {
        if !self.is_free() {
            return false;
        }

        self.state = BackendState::Tcp(tcp);
        true
    }

    pub(crate) fn as_tcp_mut(&mut self) -> Option<&mut TcpSocketState<'a>> {
        match &mut self.state {
            BackendState::Tcp(tcp) => Some(tcp),
            _ => None,
        }
    }

    /// Drive whichever protocol state machine occupies this slot for one tick.
    pub(crate) fn run<T: Transceiver>(&mut self, trans: &mut T) -> Result<(), Error> {
        match &mut self.state {
            BackendState::Free => Ok(()),
            BackendState::Tcp(tcp) => tcp.run(&self.block, trans),
        }
    }

    /// Force the hardware socket back to CLOSED. The slot itself is kept (any
    /// outstanding handle stays valid); an occupied slot is re-armed so it
    /// re-opens on the next `run`.
    pub(crate) fn reset<T: Transceiver>(&mut self, trans: &mut T) -> Result<(), Error> {
        init_socket(&self.block, trans, SocketProtocolMode::CLOSED)?;

        if let BackendState::Tcp(tcp) = &mut self.state {
            tcp.rearm();
        }

        Ok(())
    }
}
