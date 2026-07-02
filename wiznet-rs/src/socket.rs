use bitflags::bitflags;

use crate::{
    Error, SpiDmaDevice,
    atomic_cell::AtomicCell,
    socket_common::init_socket,
    spsc_ring::SpscRing,
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

/// Which bulk payload transfer a socket kicked off this tick. Carried up to the
/// `W6100` so it can record the in-flight context and finish it from the
/// DMA-complete interrupt.
#[derive(Clone, Copy)]
pub(crate) enum BulkKind {
    Receive,
    Transmit,
}

/// Result of advancing a socket one tick: either nothing crossed the bus
/// asynchronously, or a bulk DMA was started and now owns it.
pub(crate) enum BulkAction {
    None,
    Started {
        kind: BulkKind,
        /// Chip-side buffer pointer the transfer started at.
        pointer: u16,
        /// Payload length in bytes.
        len: usize,
    },
}

/// The local rx/tx ring pair for a hardware socket. Lives **outside** the
/// `AtomicCell` that guards the protocol state so the lock-free SPSC ends can be
/// reached from both `main` (handle) and the servicing interrupt without
/// contending — see [`SpscRing`].
pub(crate) struct SocketRings<'a> {
    pub(crate) rx: SpscRing<'a>,
    pub(crate) tx: SpscRing<'a>,
}

impl<'a> SocketRings<'a> {
    pub(crate) const fn new() -> Self {
        Self {
            rx: SpscRing::new(),
            tx: SpscRing::new(),
        }
    }

    /// Install the backing buffers (once, at socket open).
    pub(crate) fn install(&self, rx: &'a mut [u8], tx: &'a mut [u8]) {
        self.rx.install(rx);
        self.tx.install(tx);
    }

    /// Drop any buffered rx/tx bytes, returning both rings to empty. Called on
    /// socket re-arm so a reconnected session starts with clean buffers rather
    /// than data left over from the previous connection.
    pub(crate) fn clear(&self) {
        self.rx.clear();
        self.tx.clear();
    }
}

/// One of the chip's eight hardware sockets: the protocol state machine behind a
/// try-lock cell, plus its lock-free rings as a sibling field. The split is what
/// lets `dma_complete` deliver into `rings` without ever taking the cell.
pub(crate) struct Socket<'a> {
    pub(crate) backend: AtomicCell<SocketBackend>,
    pub(crate) rings: SocketRings<'a>,
}

/// The protocol-specific state machine living inside a [`SocketBackend`] slot.
///
/// Adding a new protocol is a matter of introducing a `*SocketState` module and
/// a variant here; [`SocketBackend::run`] (and the chip driver above it) keep
/// the same shape.
pub(crate) enum BackendState {
    Free,
    Tcp(TcpSocketState),
}

/// The protocol state for one hardware socket. Owned by the `W6100`; user-facing
/// handles only hold an atomic reference to the enclosing cell.
pub(crate) struct SocketBackend {
    block: BlockAddress,
    state: BackendState,

    /// Set when the owning handle is dropped: the slot is closed on the chip and
    /// then returned to `Free` so it can be reused.
    release_requested: bool,
}

impl SocketBackend {
    pub(crate) fn new(block: BlockAddress) -> Self {
        Self {
            block,
            state: BackendState::Free,
            release_requested: false,
        }
    }

    /// The chip-side block selectors for this socket (used by the `W6100` to
    /// record an in-flight bulk transfer).
    pub(crate) fn block(&self) -> BlockAddress {
        self.block
    }

    pub(crate) fn is_free(&self) -> bool {
        matches!(self.state, BackendState::Free)
    }

    /// Claim a free slot for a TCP socket. Returns `false` if already in use.
    pub(crate) fn claim_tcp(&mut self, tcp: TcpSocketState) -> bool {
        if !self.is_free() {
            return false;
        }

        self.state = BackendState::Tcp(tcp);
        self.release_requested = false;
        true
    }

    pub(crate) fn as_tcp_mut(&mut self) -> Option<&mut TcpSocketState> {
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
    pub(crate) fn run<D: SpiDmaDevice>(
        &mut self,
        trans: &Transceiver<D>,
        rings: &SocketRings,
    ) -> Result<BulkAction, Error> {
        let result = match &mut self.state {
            BackendState::Free => Ok(BulkAction::None),
            BackendState::Tcp(tcp) => tcp.run(&self.block, trans, rings),
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
