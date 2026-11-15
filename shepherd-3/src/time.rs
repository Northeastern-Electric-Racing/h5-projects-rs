//! The clock the measured world runs on, which is not always the MCU's.
//!
//! The MCU's clock is `Instant::now()`, used as normal everywhere. It is right for scheduling,
//! for task timing, and for link watchdogs.
//!
//! [`PlantTime`] is the other one. On a rig the Simulink model runs about five times slower than
//! real time, so an interval the *pack* lived through is not the interval the MCU measured. Use it
//! wherever correctness depends on how much time the pack experienced.

use core::ops::Add;
use embassy_time::{Duration, Instant};

/// An instant on the plant's clock.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, defmt::Format)]
pub struct PlantTime(Instant);

/// Carrying the distinction costs nothing at runtime.
const _: () = assert!(size_of::<PlantTime>() == size_of::<Instant>());

impl PlantTime {
    /// Earlier than any reading, for a deadline that has already passed.
    pub const MIN: Self = Self(Instant::MIN);
    /// Later than any reading, for a deadline that never passes.
    pub const MAX: Self = Self(Instant::MAX);

    pub const fn from_micros(micros: u64) -> Self {
        Self(Instant::from_micros(micros))
    }

    pub const fn as_micros(self) -> u64 {
        self.0.as_micros()
    }

    /// Zero rather than a panic when `earlier` is in the future, which a rig's clock can be after
    /// the model restarts under a firmware that did not.
    pub fn saturating_duration_since(self, earlier: Self) -> Duration {
        self.0.saturating_duration_since(earlier.0)
    }
}

impl Add<Duration> for PlantTime {
    type Output = Self;

    fn add(self, duration: Duration) -> Self {
        Self(self.0 + duration)
    }
}

/// The plant's clock. When not in HIL we just get MCU's clock time.
pub fn plant_now() -> Option<PlantTime> {
    #[cfg(feature = "hil")]
    return crate::hv_plate::hil::sim_time::now();

    #[cfg(not(feature = "hil"))]
    return Some(PlantTime(Instant::now()));
}
