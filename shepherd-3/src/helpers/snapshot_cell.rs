use core::cell::RefCell;
use embassy_sync::blocking_mutex::ThreadModeMutex;

/// Cell capable of holding mutable cross-task data
/// by allowing readers to copy out owned "snapshots"
/// of the data at any instant.
pub struct SnapshotCell<T: Copy> {
    inner: ThreadModeMutex<RefCell<T>>,
}
impl<T: Copy> SnapshotCell<T> {
    /// Creates a new `SnapshotCell` with an inner value of `value`.
    pub const fn new(value: T) -> Self {
        Self { inner: ThreadModeMutex::new(RefCell::new(value)) }
    }

    /// Copies out a snapshot of the inner data.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt).
    pub fn read(&self) -> T {
        self.partial_read(|inner| *inner)
    }

    /// Calls `f` with a reference to the inner data and returns whatever `f` copies out of it.
    ///
    /// You can use this instead of [`Self::read`] when only part of `T` is needed.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt), or if `f`
    /// calls [`Self::set`] on this same cell.
    pub fn partial_read<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        self.inner.lock(|inner| f(&inner.borrow()))
    }

    /// Replaces the current inner data with `value`.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt), or from
    /// inside a [`Self::read`] closure on this same cell.
    pub fn set(&self, value: T) {
        self.inner.lock(|inner| *inner.borrow_mut() = value)
    }

    /// Converts this to an unwriteable.
    pub const fn to_unwriteable(&self) -> &UnwriteableSnapshotCell<T> {
        // SAFETY: `UnwriteableSnapshotCell` uses `#[repr(transparent)]` with `SnapshotCell`.
        // This guarantees that they have an identical layout.
        unsafe { &*(self as *const Self as *const UnwriteableSnapshotCell<T>) }
    }
}

/// SnapshotCell that doesn't allow you to mutate the inner data.
///
/// This is useful if your API ever needs to let readers read data without
/// letting them mutate it.
#[repr(transparent)]
pub struct UnwriteableSnapshotCell<T: Copy> {
    inner: SnapshotCell<T>,
}
impl<T: Copy> UnwriteableSnapshotCell<T> {
    /// Copies out a snapshot of the inner data.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt).
    pub fn read(&self) -> T {
        self.inner.read()
    }

    /// Calls `f` with a reference to the inner data and returns whatever `f` copies out of it.
    ///
    /// You can use this instead of [`Self::read`] when only part of `T` is needed.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt).
    pub fn partial_read<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        self.inner.partial_read(f)
    }
}
