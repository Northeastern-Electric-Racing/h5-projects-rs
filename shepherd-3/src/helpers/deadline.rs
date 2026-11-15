use embassy_time::{Instant, Duration};

use crate::time::PlantTime;

/// A point in time something is due.
///
/// Defaults to the MCU's clock, so `Deadline` on its own means what it always did. A deadline on
/// the plant's clock is a `Deadline<PlantTime>`, and the two cannot be tested against each other.
/// See [`crate::time`].
#[derive(Copy, Clone, defmt::Format, PartialEq)]
pub struct Deadline<T = Instant> {
    inner: T,
}

impl Deadline<Instant> {
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

    /// Checks if the provided `Instant` is past the scheduled deadline.
    pub fn past_at(&self, i: Instant) -> bool {
        self.inner < i
    }
}

/// The plant's clock may not be readable at all, so a plant deadline is always built from, and
/// tested against, a reading the caller already has.
impl Deadline<PlantTime> {
    /// Already expired, for a first pass that should act rather than wait.
    pub const fn expired() -> Self {
        Self { inner: PlantTime::MIN }
    }

    /// Never expires. For a field that must hold something before the plant clock has been read.
    pub const fn never() -> Self {
        Self { inner: PlantTime::MAX }
    }

    /// Expires once the plant has experienced `duration` more.
    pub fn expire_after(now: PlantTime, duration: Duration) -> Self {
        Self { inner: now + duration }
    }

    /// Checks if the plant has reached the scheduled deadline.
    pub fn past_at(&self, now: PlantTime) -> bool {
        self.inner < now
    }
}
