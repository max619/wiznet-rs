use core::marker::PhantomData;

use crate::w6100::{
    atomic_cell::AtomicCell,
    socket::{SocketInternal, SocketProtocolMode, SocketState},
    transiver::Transceiver,
};

pub struct TcpSocket<'a, Trans: Transceiver> {
    state: AtomicCell<SocketState<'a>>,
    port: u16,

    _ph: PhantomData<Trans>,
}

impl<'a, Trans: Transceiver> TcpSocket<'a, Trans> {
    pub fn new(port: u16, rx: &'a mut [u8], tx: &'a mut [u8]) -> Self {
        Self {
            state: AtomicCell::new(SocketState { rx, tx }),
            port,
            _ph: PhantomData::<Trans>,
        }
    }
}

impl<'a, Trans: Transceiver> SocketInternal<'a, Trans> for TcpSocket<'a, Trans> {
    const MODE: SocketProtocolMode = SocketProtocolMode::TCP4;

    fn get_state(&'a self) -> &'a AtomicCell<SocketState<'a>> {
        &self.state
    }
}
