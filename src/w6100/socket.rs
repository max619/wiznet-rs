use core::{marker::PhantomData, pin::Pin, sync::atomic::AtomicBool};

use bitflags::bitflags;

use crate::w6100::{
    Error,
    atomic_cell::{AtomicCellGuard, AtomicLock, AtomicMutLock, AtomicRefCell, MutAtomicCellGuard},
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

#[derive(Clone, Copy)]
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

pub trait UserSocket<Trans: Transceiver> {
    fn get_status(&self) -> SocketStatus;
}

pub(crate) trait SocketInternal<'a, Trans: Transceiver>: UserSocket<Trans> {
    fn init(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error>;

    fn run(&mut self, block: &BlockAddress, trans: &mut Trans) -> Result<(), Error>;
}

pub(crate) trait SocketAccess<'a, Trans: Transceiver> {
    fn lock_inner(
        &self,
    ) -> Result<MutAtomicCellGuard<'_, dyn SocketInternal<'a, Trans> + 'a>, Error>;
}

pub struct PinnedSocket<'a, Trans: Transceiver, Sock: SocketInternal<'a, Trans>> {
    inner: AtomicRefCell<&'a mut Sock>,

    _ph: PhantomData<Trans>,
}

impl<'a, Trans: Transceiver, Sock: SocketInternal<'a, Trans>> PinnedSocket<'a, Trans, Sock> {
    pub fn pin(inner: &'a mut Sock) -> Self {
        Self {
            inner: AtomicRefCell::new(inner),
            _ph: PhantomData::<Trans>,
        }
    }

    pub fn lock(&self) -> Result<AtomicCellGuard<'_, Sock>, Error> {
        let res = self.inner.lock()?;
        Ok(res)
    }

    pub fn lock_mut(&self) -> Result<MutAtomicCellGuard<'_, Sock>, Error> {
        let res = self.inner.lock_mut()?;
        Ok(res)
    }
}

impl<'a, Trans: Transceiver, Sock: SocketInternal<'a, Trans>> SocketAccess<'a, Trans>
    for PinnedSocket<'a, Trans, Sock>
{
    fn lock_inner(
        &self,
    ) -> Result<MutAtomicCellGuard<'_, dyn SocketInternal<'a, Trans> + 'a>, Error> {
        Ok(self
            .lock_mut()?
            .map(|s| s as &mut (dyn SocketInternal<'a, Trans> + 'a)))
    }
}
