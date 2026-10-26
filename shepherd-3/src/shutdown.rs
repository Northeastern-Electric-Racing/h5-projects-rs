use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::Level;
use embassy_time::{Instant, Duration};

static CIRCUIT_CLOSED: AtomicBool = AtomicBool::new(false);

const DEBOUNCE_TIME: Duration = Duration::from_millis(200);

// struct for holding the memory between ticks: stable = boolean indicating the debounced output, disagreed_at = the time the disagreement between ticks started.
struct Debouncer {
    stable: bool,
    disagreed_at: Option<Instant>,
}

impl Debouncer {
    const fn new(stable: bool) -> Self {
        Self { stable, disagreed_at: None }
    }

    fn update(&mut self, raw: bool, now: Instant) -> Option<bool> {
        if raw == self.stable {
            self.disagreed_at = None;
            return None;
        }

        match self.disagreed_at {
            None => {
                self.disagreed_at = Some(now);
                return None;
            },
            Some(started) => {
                if now.duration_since(started) < DEBOUNCE_TIME {
                    return None;
                } else {
                    self.stable = raw;
                    self.disagreed_at = None;
                    return Some(self.stable);
                }
            },
        }
    }
}

pub fn set_fault(faulted: bool) {
    let level = if faulted {
        Level::Low // should always be asserted at level low! otherwise loop won't always shut down when a node is opened!
    } else {
        Level::High
    };
    crate::gpios::pins().get().mcu_fault_pin.set_level(level);
}
