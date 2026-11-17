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
#[cfg(feature = "hil")]
use core::cell::Cell;
#[cfg(feature = "hil")]
use embassy_sync::blocking_mutex::ThreadModeMutex;

/// An instant on the plant's clock.
/// `repr(transparent)` so carrying the distinction is guaranteed to cost nothing at runtime.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, defmt::Format)]
#[repr(transparent)]
pub struct PlantTime(Instant);

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

    /// Zero rather than a panic when `earlier` is in the future, which happens when the model restarts under a firmware that did not.
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

/// Converts into an [`Instant`] for the helpers that only speak the MCU's clock, such as
/// [`crate::helpers::deadline::Deadline`]. The conversion drops the distinction, so only hand one
/// to something that will measure it against another reading off this same clock.
impl From<PlantTime> for Instant {
    fn from(plant: PlantTime) -> Self {
        plant.0
    }
}

/// The plant's clock. Off a rig that is just the MCU's; on one it is whatever the model last reported.
pub fn plant_now() -> Option<PlantTime> {
    #[cfg(feature = "hil")]
    return PLANT_MICROS.lock(|micros| micros.get()).map(PlantTime::from_micros);

    #[cfg(not(feature = "hil"))]
    return Some(PlantTime(Instant::now()));
}

/// Simulation time the rig last reported, in microseconds. `None`, never zero, until it has answered.
#[cfg(feature = "hil")]
static PLANT_MICROS: ThreadModeMutex<Cell<Option<u64>>> = ThreadModeMutex::new(Cell::new(None));

/// Hands the clock a fresh reading and returns the one it replaced, so whoever owns the link can judge a jump.
#[cfg(feature = "hil")]
pub(crate) fn report_plant_micros(micros: u64) -> Option<u64> {
    PLANT_MICROS.lock(|cell| cell.replace(Some(micros)))
}
