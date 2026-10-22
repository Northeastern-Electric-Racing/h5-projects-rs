use embassy_time::{Duration, Instant};

/// Level-triggered, not edge-triggered: once held for `period` it reports true on every subsequent call.
///
/// This is the Rust equivalent of C's `debounce()` + `nertimer_t` pair, minus the callback.
pub struct Debouncer {
    armed_at: Option<Instant>,
}

impl Debouncer {
    pub const fn new() -> Self {
        Self { armed_at: None }
    }

    /// Feeds one sample. Returns whether `input` has been continuously true for `period`.
    ///
    /// A single false resets the timer, so the caller gets the full `period` back on the next rise.
    pub fn poll(&mut self, input: bool, period: Duration, now: Instant) -> bool {
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

impl Default for Debouncer {
    fn default() -> Self {
        Self::new()
    }
}
