use embassy_time::{Duration, Instant};

/// A point in time something is due, on the MCU's clock.
///
/// A [`crate::helpers::time::PlantTime`] converts in, so a deadline measured on the plant's clock is still
/// just an `Instant` here -- build it and test it against readings from the same clock.
#[derive(Copy, Clone, defmt::Format, PartialEq)]
pub struct Deadline {
    inner: Instant,
}

impl Deadline {
    /// Already expired, for a first pass that should act rather than wait.
    pub const fn expired() -> Self {
        Self { inner: Instant::MIN }
    }

    /// Never expires. For a field that must hold something before anything has been scheduled.
    pub const fn never() -> Self {
        Self { inner: Instant::MAX }
    }

    /// Expires at `at`.
    pub fn expire_at(at: impl Into<Instant>) -> Self {
        Self { inner: at.into() }
    }

    /// Expires once `duration` has passed from `now`, on whichever clock `now` was read off.
    pub fn expire_after(now: impl Into<Instant>, duration: Duration) -> Self {
        Self { inner: now.into() + duration }
    }

    /// Creates a `Deadline` that expires in `duration` seconds.
    pub fn expire_in(duration: Duration) -> Self {
        Self::expire_after(Instant::now(), duration)
    }

    /// Creates a `Deadline` that expires immediately.
    ///
    /// This could be useful if you need something to run immediately on the first
    /// iteration, but then run at a later deadline after that.
    pub fn expire_now() -> Self {
        Self::expire_at(Instant::now())
    }

    /// Checks if we are currently past the scheduled deadline, on the MCU's clock.
    pub fn past(&self) -> bool {
        self.past_at(Instant::now())
    }

    /// Checks the deadline against a reading the caller already has.
    pub fn past_at(&self, now: impl Into<Instant>) -> bool {
        self.inner < now.into()
    }
}
