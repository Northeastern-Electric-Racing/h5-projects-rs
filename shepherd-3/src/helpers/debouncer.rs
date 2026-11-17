use embassy_time::{Duration, Instant};

/// Level-triggered, not edge-triggered: once held for `period` it reports true on every subsequent call.
/// Times against whichever clock the caller passes in -- `shutdown` counts raw samples instead and has its own.
#[derive(Default)]
pub struct Debouncer {
    armed_at: Option<Instant>,
}

impl Debouncer {
    pub const fn new() -> Self {
        Self { armed_at: None }
    }

    /// Clears any arm in progress, so the next hold starts counting from scratch.
    pub const fn reset(&mut self) {
        self.armed_at = None;
    }

    pub fn calculate(&mut self, input: bool, period: Duration, now: impl Into<Instant>) -> bool {
        match (input, self.armed_at) {
            (false, _) => {
                self.armed_at = None;
                false
            },
            (true, None) => {
                self.armed_at = Some(now.into());
                false
            },
            (true, Some(start)) => now.into().saturating_duration_since(start) >= period,
        }
    }
}
