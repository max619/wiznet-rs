use core::cell::UnsafeCell;
use core::sync::atomic::AtomicBool;

pub struct AtomicCell<T> {
    cell: UnsafeCell<T>,
    busy: AtomicBool,
}

pub struct AtomicRefCell<T> {
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

impl<T> AtomicRefCell<T> {
    /// Create a new `AtomicCell`
    pub fn new(value: T) -> Self {
        Self {
            cell: UnsafeCell::new(value),
            busy: AtomicBool::new(false),
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

impl<'a, T: ?Sized> AtomicCellGuard<'a, T> {
    /// Transform the guarded reference while keeping the lock held.
    ///
    /// Typically used to unsize a concrete guard into a trait-object guard,
    /// e.g. `AtomicCellGuard<Sock>` -> `AtomicCellGuard<dyn Trait>` via
    /// `guard.map(|s| s as &dyn Trait)`.
    pub fn map<U: ?Sized>(
        self,
        f: impl FnOnce(&'a T) -> &'a U,
    ) -> AtomicCellGuard<'a, U> {
        let this = core::mem::ManuallyDrop::new(self);

        #[allow(unsafe_code)]
        // SAFETY: `this` is wrapped in `ManuallyDrop`, so the original guard's
        // `Drop` (which clears the busy flag) never runs. Each field is moved
        // out exactly once into the new guard, which takes over clearing the
        // flag on its own drop -- so the lock is released exactly once.
        let (value, busy) = unsafe { (core::ptr::read(&this.value), core::ptr::read(&this.busy)) };

        AtomicCellGuard {
            value: f(value),
            busy,
        }
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

impl<'a, T: ?Sized> MutAtomicCellGuard<'a, T> {
    /// Transform the guarded reference while keeping the lock held.
    ///
    /// Typically used to unsize a concrete guard into a trait-object guard,
    /// e.g. `MutAtomicCellGuard<Sock>` -> `MutAtomicCellGuard<dyn Trait>` via
    /// `guard.map(|s| s as &mut dyn Trait)`.
    pub fn map<U: ?Sized>(
        self,
        f: impl FnOnce(&'a mut T) -> &'a mut U,
    ) -> MutAtomicCellGuard<'a, U> {
        let this = core::mem::ManuallyDrop::new(self);

        #[allow(unsafe_code)]
        // SAFETY: `this` is wrapped in `ManuallyDrop`, so the original guard's
        // `Drop` (which clears the busy flag) never runs. Each field is moved
        // out exactly once into the new guard, which takes over clearing the
        // flag on its own drop -- so the lock is released exactly once.
        let (value, busy) = unsafe { (core::ptr::read(&this.value), core::ptr::read(&this.busy)) };

        MutAtomicCellGuard {
            value: f(value),
            busy,
        }
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

impl<T> AtomicLock<T> for AtomicRefCell<&T> {
    fn lock<'a>(&'a self) -> Result<AtomicCellGuard<'a, T>, AtomicError> {
        self.try_lock_internal()?;

        #[allow(unsafe_code)]
        let value = unsafe { *self.cell.get() };

        Ok(AtomicCellGuard::<'a, T> {
            value,
            busy: &self.busy,
        })
    }
}

impl<T> AtomicLock<T> for AtomicRefCell<&mut T> {
    fn lock<'a>(&'a self) -> Result<AtomicCellGuard<'a, T>, AtomicError> {
        self.try_lock_internal()?;

        #[allow(unsafe_code)]
        let value = unsafe { &**self.cell.get() };

        Ok(AtomicCellGuard::<'a, T> {
            value,
            busy: &self.busy,
        })
    }
}

impl<T> AtomicMutLock<T> for AtomicRefCell<&mut T> {
    fn lock_mut<'a>(&'a self) -> Result<MutAtomicCellGuard<'a, T>, AtomicError> {
        self.try_lock_internal()?;

        #[allow(unsafe_code)]
        let value = unsafe { &mut **self.cell.get() };

        Ok(MutAtomicCellGuard::<'a, T> {
            value,
            busy: &self.busy,
        })
    }
}
