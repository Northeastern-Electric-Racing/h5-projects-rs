use embassy_time::{Duration, Instant};

/// Level-triggered, not edge-triggered: once held for `period` it reports true on every subsequent call.
#[derive(Default)]
pub struct Debouncer {
    armed_at: Option<Instant>,
}

impl Debouncer {
    pub const fn new() -> Self {
        Self { armed_at: None }
    }

    pub fn calculate(&mut self, input: bool, period: Duration, now: Instant) -> bool {
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
