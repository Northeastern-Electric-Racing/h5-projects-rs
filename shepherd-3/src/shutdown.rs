use core::sync::atomic::{AtomicBool, Ordering};
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

struct Debouncer {
    /// a boolean indicating the debounced output
    stable: bool,
    /// stores the time the disagreement between ticks started
    disagreed_at: Option<Instant>,
}

impl Debouncer {
    const fn new(stable: bool) -> Self {
        Self { stable, disagreed_at: None }
    }

    const fn stable(&self) -> bool {
        self.stable
    }

    /// Whether the raw reading currently disagrees with the debounced one.
    ///
    /// The whole point of the debounce is that [`Self::stable`] does not move until the
    /// disagreement has lasted [`DEBOUNCE_TIME`], so a line that is chattering shows up here and
    /// nowhere else.
    const fn pending(&self) -> bool {
        self.disagreed_at.is_some()
    }

    /// given one raw reading, returns `true` only on the tick the debounced value changes
    /// call `stable()` for the value itself
    fn update(&mut self, raw: bool, now: Instant) -> bool {
        if raw == self.stable {
            self.disagreed_at = None;
            return false;
        }

        match self.disagreed_at {
            None => {
                self.disagreed_at = Some(now);
                return false;
            },
            Some(started) => {
                if now.duration_since(started) < DEBOUNCE_TIME {
                    return false;
                } else {
                    self.stable = raw;
                    self.disagreed_at = None;
                    return true;
                }
            },
        }
    }
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

        let changed: bool = updated_ts_minus || updated_ts_plus || updated_acc_sense || updated_tsip_sense || updated_and_for_all_reads;

        CIRCUIT_CLOSED.store(and_for_all_reads.stable(), Ordering::Relaxed);

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            defmt_monitor::monitor!("Shutdown/TsMinus/raw", desc = "TS- shutdown sense pin as read this tick, before debouncing.", "{=bool}", raw_ts_minus);
            defmt_monitor::monitor!("Shutdown/TsMinus/debounced", desc = "TS- shutdown sense after 200 ms of agreement.", "{=bool}", ts_minus.stable());
            defmt_monitor::monitor!("Shutdown/TsMinus/pending", desc = "TS- raw reading currently disagrees with its debounced value, so the line is bouncing.", "{=bool}", ts_minus.pending());

            defmt_monitor::monitor!("Shutdown/TsPlus/raw", desc = "TS+ shutdown sense pin as read this tick, before debouncing.", "{=bool}", raw_ts_plus);
            defmt_monitor::monitor!("Shutdown/TsPlus/debounced", desc = "TS+ shutdown sense after 200 ms of agreement.", "{=bool}", ts_plus.stable());
            defmt_monitor::monitor!("Shutdown/TsPlus/pending", desc = "TS+ raw reading currently disagrees with its debounced value, so the line is bouncing.", "{=bool}", ts_plus.pending());

            defmt_monitor::monitor!("Shutdown/AccSense/raw", desc = "Accumulator shutdown sense pin as read this tick, before debouncing.", "{=bool}", raw_acc_sense);
            defmt_monitor::monitor!("Shutdown/AccSense/debounced", desc = "Accumulator shutdown sense after 200 ms of agreement.", "{=bool}", acc_sense.stable());
            defmt_monitor::monitor!("Shutdown/AccSense/pending", desc = "Accumulator raw reading currently disagrees with its debounced value, so the line is bouncing.", "{=bool}", acc_sense.pending());

            defmt_monitor::monitor!("Shutdown/TsipSense/raw", desc = "TSIP shutdown sense pin as read this tick, before debouncing.", "{=bool}", raw_tsip_sense);
            defmt_monitor::monitor!("Shutdown/TsipSense/debounced", desc = "TSIP shutdown sense after 200 ms of agreement.", "{=bool}", tsip_sense.stable());
            defmt_monitor::monitor!("Shutdown/TsipSense/pending", desc = "TSIP raw reading currently disagrees with its debounced value, so the line is bouncing.", "{=bool}", tsip_sense.pending());

            defmt_monitor::monitor!("Shutdown/Joined/raw", desc = "All four sense pins high on this tick, before debouncing.", "{=bool}", combined);
            defmt_monitor::monitor!("Shutdown/Joined/debounced", desc = "All four sense pins high for 200 ms straight. This is what is stored for `circuit_closed`. The BMS only taps four points, so the loop could still be cut somewhere it cannot see.", "{=bool}", and_for_all_reads.stable());
            defmt_monitor::monitor!("Shutdown/Joined/pending", desc = "The joined raw reading currently disagrees with its debounced value.", "{=bool}", and_for_all_reads.pending());

            // What callers actually get.
            defmt_monitor::monitor!("Shutdown/1CircuitClosed", desc = "What `circuit_closed()` returns to callers, which is what gates HV. Compare against Shutdown/Joined/debounced: a disagreement means the function is overriding the measurement.", "{=bool}", circuit_closed());
        }

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
