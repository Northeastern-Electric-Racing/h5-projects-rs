use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::Level;
use embassy_time::{Instant, Duration, Ticker};
use crate::can;
use crate::helpers::deadline::Deadline;
use crate::can::types::ShutdownAsReadByBms;

static CIRCUIT_CLOSED: AtomicBool = AtomicBool::new(false);

/// `true` when all four sense pins have read high for 200ms straight. The BMS only
/// taps four points, so the loop could still be cut somewhere it can't see.
/// One atomic read, safe to call from an interrupt.
pub fn circuit_closed() -> bool {
    CIRCUIT_CLOSED.load(Ordering::Relaxed)
}

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

    const fn stable(&self) -> bool {
        self.stable
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

/// Breaks or restores the BMS's link in the shutdown loop.
///
/// `true` pulls PG15 low, which breaks the loop and cuts the pack off the car.
/// Low is the active state
pub fn set_fault(faulted: bool) {
    let level = if faulted {
        Level::Low // should always be asserted at level low! otherwise loop won't always shut down when a node is opened!
    } else {
        Level::High
    };
    crate::gpios::pins().get().mcu_fault_pin.set_level(level);
}

/// Reads the four shutdown sense pins every 50ms, debounces them, and sends the
/// result on CAN when it changes or every 500ms.
#[embassy_executor::task]
pub async fn shutdown_task() -> ! {
    const POLL_PERIOD: Duration = Duration::from_millis(50);
    const TELEM_PERIOD: Duration = Duration::from_millis(500);

    let mut ts_minus = Debouncer::new(false);
    let mut ts_plus = Debouncer::new(false);
    let mut acc_sense = Debouncer::new(false);
    let mut tsip_sense = Debouncer::new(false);
    let mut and_for_all_reads = Debouncer::new(false);

    let mut ticker = Ticker::every(POLL_PERIOD);
    let mut telem = Deadline::expire_now();

    let p = crate::gpios::pins().get();

    loop {
        let now = Instant::now();

        let raw_ts_minus = p.ts_minus_sense_pin.is_high();
        let raw_ts_plus = p.ts_plus_sense_pin.is_high();
        let raw_acc_sense = p.acc_sense_pin.is_high();
        let raw_tsip_sense = p.tsip_sense_pin.is_high();
        let combined = raw_ts_minus && raw_ts_plus && raw_acc_sense && raw_tsip_sense;

        let updated_ts_minus = ts_minus.update(raw_ts_minus, now);
        let updated_ts_plus = ts_plus.update(raw_ts_plus, now);
        let updated_acc_sense = acc_sense.update(raw_acc_sense, now);
        let updated_tsip_sense = tsip_sense.update(raw_tsip_sense, now);
        let updated_and_for_all_reads = and_for_all_reads.update(combined, now);

        let changed: bool = updated_ts_minus.is_some() || updated_ts_plus.is_some() || updated_acc_sense.is_some() || updated_tsip_sense.is_some() || updated_and_for_all_reads.is_some();

        CIRCUIT_CLOSED.store(and_for_all_reads.stable(), Ordering::Relaxed);

        if changed || telem.past() {
            let shutdown_state = ShutdownAsReadByBms {
                shutdown_state: and_for_all_reads.stable(),
                shutdown_ts_minus_sense: ts_minus.stable(),
                shutdown_ts_plus_sense: ts_plus.stable(),
                shutdown_acc_sense: acc_sense.stable(),
                shutdown_tsip_sense: tsip_sense.stable(),
            }
            .as_frame();

            can::send(shutdown_state).await;
            telem = Deadline::expire_in(TELEM_PERIOD);
        }

        ticker.next().await
    }
}
