use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::{Output, Level};
use embassy_sync::blocking_mutex::ThreadModeMutex;
use embassy_time::{Instant, Duration};

static CIRCUIT_CLOSED: AtomicBool = AtomicBool::new(false);

// variable representing the BMS's contact in the shutdown loop. Low = asserted = loop open.
static FAULT_PIN: ThreadModeMutex<RefCell<Option<Output<'static>>>> = ThreadModeMutex::new(RefCell::new(None));
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

    FAULT_PIN.lock(|cell| match cell.borrow_mut().as_mut() {
        Some(pin) => pin.set_level(level),
        None => defmt::error!("set_fault called before shutdown::init(); PG15 is not claimed!"),
    })
}
