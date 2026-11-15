use embassy_time::Duration;

use crate::time::PlantTime;

/// Level-triggered, not edge-triggered: once held for `period` it reports true on every subsequent call.
///
/// Runs on the plant's clock, because both things this debounces -- the bus charging and the
/// contacts bleeding down -- are the plant's doing. See [`crate::time`]. (`shutdown` debounces
/// real GPIO pins against the MCU's clock, and has its own.)
#[derive(Default)]
pub struct Debouncer {
    armed_at: Option<PlantTime>,
}

impl Debouncer {
    pub const fn new() -> Self {
        Self { armed_at: None }
    }

    /// Clears any arm in progress, so the next hold starts counting from scratch.
    pub const fn reset(&mut self) {
        self.armed_at = None;
    }

    pub fn calculate(&mut self, input: bool, period: Duration, now: PlantTime) -> bool {
        match (input, self.armed_at) {
            (false, _) => {
                self.armed_at = None;
                false
            },
            (true, None) => {
                self.armed_at = Some(now);
                false
            },
            (true, Some(start)) => now.saturating_duration_since(start) >= period,
        }
    }
}
