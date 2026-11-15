//! A measurement paired with the instant it was taken.

use embassy_time::{Duration, Instant};

/// A value and when it was measured.
#[derive(Copy, Clone)]
pub struct Stamped<T> {
    value: T,
    at: Instant,
}

impl<T: Copy> Stamped<T> {
    /// `at` should be taken before the measurement, so it bounds the value's age.
    pub const fn new(value: T, at: Instant) -> Self {
        Self { value, at }
    }

    /// The value, however old. For display and telemetry.
    pub const fn get(&self) -> T {
        self.value
    }

    /// The value, but only while it is no older than `within`. For anything used as evidence.
    pub fn fresh(&self, within: Duration) -> Option<T> {
        (self.age() <= within).then_some(self.value)
    }

    /// Whether the value is no older than `within`, without taking it.
    pub fn is_fresh(&self, within: Duration) -> bool {
        self.age() <= within
    }

    /// Age against the MCU clock. For another timebase use [`Self::age_since`].
    pub fn age(&self) -> Duration {
        self.age_since(Instant::now())
    }

    /// When the measurement was taken, on whichever clock stamped it.
    pub const fn at(&self) -> Instant {
        self.at
    }

    /// Age against a caller-supplied `now`, for values stamped on a clock other than the MCU's.
    pub fn age_since(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.at)
    }
}
