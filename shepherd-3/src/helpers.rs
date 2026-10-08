/// Various helper types that can be used throughout this project.
use embassy_time::{Instant, Duration};

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
