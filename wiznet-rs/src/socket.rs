use bitflags::bitflags;

use crate::{
    Error, SpiDmaDevice,
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

    /// Set when the owning handle is dropped: the slot is closed on the chip and
    /// then returned to `Free` so it can be reused.
    release_requested: bool,
}

impl<'a> SocketBackend<'a> {
    pub(crate) fn new(block: BlockAddress) -> Self {
        Self {
            block,
            state: BackendState::Free,
            release_requested: false,
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
        self.release_requested = false;
        true
    }

    pub(crate) fn as_tcp_mut(&mut self) -> Option<&mut TcpSocketState<'a>> {
        match &mut self.state {
            BackendState::Tcp(tcp) => Some(tcp),
            _ => None,
        }
    }

    /// Mark the slot for release (handle dropped): gracefully close whatever is
    /// running so the next `run` ticks can finish teardown and free the slot.
    pub(crate) fn request_release(&mut self) {
        self.release_requested = true;

        if let BackendState::Tcp(tcp) = &mut self.state {
            tcp.request_close();
        }
    }

    /// Drive whichever protocol state machine occupies this slot for one tick,
    /// then free the slot if a release was requested and teardown has finished.
    pub(crate) fn run<D: SpiDmaDevice>(&mut self, trans: &Transceiver<D>) -> Result<(), Error> {
        let result = match &mut self.state {
            BackendState::Free => Ok(()),
            BackendState::Tcp(tcp) => tcp.run(&self.block, trans),
        };

        if self.release_requested && self.is_terminal() {
            self.state = BackendState::Free;
            self.release_requested = false;
        }

        result
    }

    /// Whether the occupying protocol state machine has reached a closed state.
    fn is_terminal(&self) -> bool {
        match &self.state {
            BackendState::Free => true,
            BackendState::Tcp(tcp) => tcp.is_closed(),
        }
    }

    /// Force the hardware socket back to CLOSED. A slot pending release is freed
    /// immediately (it is already closed on the chip); otherwise an occupied
    /// slot is re-armed so it re-opens on the next `run`.
    pub(crate) fn reset<D: SpiDmaDevice>(&mut self, trans: &Transceiver<D>) -> Result<(), Error> {
        init_socket(&self.block, trans, SocketProtocolMode::CLOSED)?;

        if self.release_requested {
            self.state = BackendState::Free;
            self.release_requested = false;
        } else if let BackendState::Tcp(tcp) = &mut self.state {
            tcp.rearm();
        }

        Ok(())
    }
}
