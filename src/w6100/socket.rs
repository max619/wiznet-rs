use bitflags::bitflags;

use crate::w6100::{Error, atomic_cell::AtomicCell, transiver::Transceiver};

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

pub struct SocketState<'b> {
    pub(crate) rx: &'b mut [u8],
    pub(crate) tx: &'b mut [u8],
}

pub(crate) trait SocketInternal<'a, Trans: Transceiver> {
    const MODE: SocketProtocolMode;

    fn get_state(&'a self) -> &'a AtomicCell<SocketState<'a>>;
}
