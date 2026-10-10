use embassy_time::{Instant, Duration};
use core::cell::Cell;
use embassy_sync::blocking_mutex::ThreadModeMutex;

/// Cell capable of holding mutable cross-task data
/// by allowing readers to copy out owned "snapshots"
/// of the data at any instant.
pub struct SnapshotCell<T: Copy> {
    inner: ThreadModeMutex<Cell<T>>,
}
impl<T: Copy> SnapshotCell<T> {
    /// Creates a new `SnapshotCell` with an inner value of `value`.
    pub const fn new(value: T) -> Self {
        Self { inner: ThreadModeMutex::new(Cell::new(value)) }
    }

    /// Copies out a snapshot of the inner data.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt).
    pub fn take_snapshot(&self) -> T {
        self.inner.lock(|inner| inner.get())
    }

    /// Replaces the current inner data with `value`.
    ///
    /// ### Panics
    /// If this is called in a non-thread-mode context (e.g., in an interrupt).
    pub fn set(&self, value: T) {
        self.inner.lock(|inner| inner.set(value))
    }
}