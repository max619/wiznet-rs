use core::cell::UnsafeCell;
use core::sync::atomic::AtomicBool;

pub struct AtomicCell<T> {
    cell: UnsafeCell<T>,
    busy: AtomicBool,
}

pub struct AtomicCellGuard<'a, T: ?Sized> {
    value: &'a T,
    busy: &'a AtomicBool,
}

pub struct MutAtomicCellGuard<'a, T: ?Sized> {
    value: &'a mut T,
    busy: &'a AtomicBool,
}

#[derive(Debug, Clone, Copy)]
pub enum AtomicError {
    Busy,
}

pub trait AtomicLock<T> {
    fn lock<'a>(&'a self) -> Result<AtomicCellGuard<'a, T>, AtomicError>;
}

pub trait AtomicMutLock<T> {
    fn lock_mut<'a>(&'a self) -> Result<MutAtomicCellGuard<'a, T>, AtomicError>;
}

impl<T> AtomicCell<T> {
    /// Create a new `AtomicCell`
    pub fn new(value: T) -> Self {
        Self {
            cell: UnsafeCell::new(value),
            busy: AtomicBool::from(false),
        }
    }

    fn try_lock_internal(&self) -> Result<(), AtomicError> {
        self.busy
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .map_err(|_| AtomicError::Busy)?;

        Ok(())
    }
}

#[allow(unsafe_code)]
unsafe impl<T: Send> Send for AtomicCell<T> {}

#[allow(unsafe_code)]
unsafe impl<T: Send> Sync for AtomicCell<T> {}

impl<'a, T: ?Sized> Drop for AtomicCellGuard<'a, T> {
    fn drop(&mut self) {
        self.busy.store(false, core::sync::atomic::Ordering::SeqCst);
    }
}

impl<'a, T: ?Sized> Drop for MutAtomicCellGuard<'a, T> {
    fn drop(&mut self) {
        self.busy.store(false, core::sync::atomic::Ordering::SeqCst);
    }
}

impl<'a, T: ?Sized> AsRef<T> for AtomicCellGuard<'a, T> {
    fn as_ref(&self) -> &T {
        self.value
    }
}

impl<'a, T: ?Sized> AsRef<T> for MutAtomicCellGuard<'a, T> {
    fn as_ref(&self) -> &T {
        self.value
    }
}

impl<'a, T: ?Sized> AsMut<T> for MutAtomicCellGuard<'a, T> {
    fn as_mut(&mut self) -> &mut T {
        self.value
    }
}

impl<T> AtomicLock<T> for AtomicCell<T> {
    fn lock<'a>(&'a self) -> Result<AtomicCellGuard<'a, T>, AtomicError> {
        self.try_lock_internal()?;

        #[allow(unsafe_code)]
        let value = unsafe { &*self.cell.get() };

        Ok(AtomicCellGuard::<'a, T> {
            value,
            busy: &self.busy,
        })
    }
}

impl<T> AtomicMutLock<T> for AtomicCell<T> {
    fn lock_mut<'a>(&'a self) -> Result<MutAtomicCellGuard<'a, T>, AtomicError> {
        self.try_lock_internal()?;

        #[allow(unsafe_code)]
        let value = unsafe { &mut *self.cell.get() };

        Ok(MutAtomicCellGuard::<'a, T> {
            value,
            busy: &self.busy,
        })
    }
}
