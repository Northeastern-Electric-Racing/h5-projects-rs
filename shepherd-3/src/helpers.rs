/// Various helper types that can be used throughout this project.
use embassy_time::{Instant, Duration};
use core::cell::Cell;
use embassy_sync::blocking_mutex::ThreadModeMutex;

#[derive(Copy, Clone)]
pub struct Deadline {
    inner: Instant,
}
impl Deadline {
    /// Creates a `Deadline` that expires at earliest instant that can possibly be represented
    /// by an `Instant`.
    pub const fn expire_at_beginning_of_time() -> Self {
        Self { inner: Instant::MIN }
    }

    /// Creates a `Deadline` that expires in `duration` seconds.
    pub fn expire_in(duration: Duration) -> Self {
        Self { inner: Instant::now() + duration }
    }

    /// Creates a `Deadline` that expires at `instant`.
    pub fn expire_at(instant: Instant) -> Self {
        Self { inner: instant }
    }

    /// Creates a `Deadline` that expires immediately.
    ///
    /// This could be useful if you need something to run immediately on the first
    /// iteration, but then run at a later deadline after that.
    pub fn expire_now() -> Self {
        Self { inner: Instant::now() }
    }

    /// Checks if we are currently based the scheduled deadline.
    pub fn past(&self) -> bool {
        self.inner < Instant::now()
    }
}

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
