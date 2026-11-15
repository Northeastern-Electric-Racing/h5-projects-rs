use crate::helpers::moving_average::MovingAverage;
use crate::helpers::debouncer::Debouncer;
use crate::segments::NUM_CELLS_TOTAL;
use crate::time::PlantTime;
use crate::units::{Voltage, volt};
use embassy_time::Duration;

// Exported functions
pub use task::precharge_task;

/// Per cell, so the gate tracks the cell count under `four-segments`. The undervoltage limit: below it there is no pack worth judging.
const MINIMUM_CELL_VOLTAGE: Voltage = Voltage::from_volts(2.5);
const MINIMUM_PACK_VOLTAGE: Voltage = MINIMUM_CELL_VOLTAGE.scale(NUM_CELLS_TOTAL as f32);
const BATT_VOLT_BUFFER: Voltage = Voltage::from_volts(10.0);

/// Whether `batt` is plausible enough to compare TS against.
fn pack_present(batt: Voltage) -> bool {
    batt >= MINIMUM_PACK_VOLTAGE - BATT_VOLT_BUFFER
}

/// Inputs for the precharge logic
#[derive(Copy, Clone)]
struct Inputs {
    /// `None` when nothing is currently reporting TS voltage.
    ts: Option<crate::pack::Sourced<Voltage>>,
    batt: Voltage,
}

/// TS as a fraction of pack, at or above which the bus counts as charged.
const TRIGGER_RATIO: f32 = 0.90;
/// Depth of the averaging window on both bus voltages.
const NUM_SAMPLES: usize = 5;
/// Minimum dwell between relay transitions, in either direction. Plant time: it is the bus that
/// has to hold, so a rig running at half speed gets half as much wall clock to hold it for.
const TOGGLE_TIME: Duration = Duration::from_millis(200);

use crate::hv_plate::RelayRequest;

struct Precharge {
    requested_state: RelayRequest,
    ts: MovingAverage<NUM_SAMPLES>,
    batt: MovingAverage<NUM_SAMPLES>,
    to_closed: Debouncer,
}

impl Precharge {
    const fn new() -> Self {
        Self {
            requested_state: RelayRequest::Open,
            ts: MovingAverage::new(),
            batt: MovingAverage::new(),
            to_closed: Debouncer::new(),
        }
    }

    /// Ticks the precharge logic and commands the plate with the result.
    ///
    /// `now` is plant time: [`TOGGLE_TIME`] is a dwell on the bus, not on the link. See
    /// [`crate::time`].
    fn tick(&mut self, now: PlantTime, inputs: Option<Inputs>, hv_permitted: bool) -> RelayRequest {
        // Unpack and evaluate ts and batt voltage readings
        let state_from_ts_batt_volts = match inputs {
            Some(Inputs { ts: Some(ts), batt }) => {
                self.ts.push(ts.get().get::<volt>());
                self.batt.push(batt.get::<volt>());

                // Evaluate ratio and decide next state
                let (ts_avg, batt_avg) = (Voltage::from_volts(self.ts.get()), Voltage::from_volts(self.batt.get()));
                if pack_present(batt_avg) && ts_avg >= batt_avg * TRIGGER_RATIO { RelayRequest::Closed } else { RelayRequest::Open }
            },

            // The pack is measurable but the bus is not, so it cannot be shown to be charged.
            // Still average BATT: it is the reference the ratio is taken against,
            // and the diagnostics below are the only way to see which input is missing.
            Some(Inputs { ts: None, batt }) => {
                self.batt.push(batt.get::<volt>());
                RelayRequest::Open
            },

            // Default to open if we can't read inputs (gets debounced anyway)
            None => RelayRequest::Open,
        };

        // Match on hv_permitted and current requested state
        match (hv_permitted, self.requested_state) {
            // The acc shutdown loop outranks everything below it, the latch included.
            (false, _) => {
                self.requested_state = RelayRequest::Open;
                self.to_closed.reset();
            },

            // LATCHED. Once closed the relay stays closed; only an unpermitted loop reopens it.
            (true, RelayRequest::Closed) => {},

            // Not latched, so a charged bus held for TOGGLE_TIME closes the relay.
            (true, RelayRequest::Open) => {
                if self.to_closed.calculate(state_from_ts_batt_volts == RelayRequest::Closed, TOGGLE_TIME, now) {
                    self.transition_to(RelayRequest::Closed);
                }
            },
        }

        self.requested_state
    }

    /// What to request with no plant clock to time [`TOGGLE_TIME`] on.
    ///
    /// Nothing can be shown to have been held, so the dwell is cleared and the relay is left where
    /// it is: open, or latched closed until an unpermitted loop reopens it.
    fn hold(&mut self, hv_permitted: bool) -> RelayRequest {
        self.to_closed.reset();
        if !hv_permitted {
            self.requested_state = RelayRequest::Open;
        }
        self.requested_state
    }

    /// Moves to `next`, clearing every debouncer, so transitions can only happen in [`TOGGLE_TIME`].
    fn transition_to(&mut self, next: RelayRequest) {
        if self.requested_state == next {
            return;
        }

        self.requested_state = next;
        self.to_closed.reset();
    }

    /// Returns the relay state the precharge logic is currently requesting.
    const fn requested_state(&self) -> RelayRequest {
        self.requested_state
    }

    /// Averaged TS voltage, in volts, or `None` if no TS reading has ever arrived.
    const fn ts_average(&self) -> Option<f32> {
        if self.ts.seeded() { Some(self.ts.get()) } else { None }
    }

    /// Averaged BATT-side voltage, in volts, or `None` if the plate has never read BATT.
    const fn batt_average(&self) -> Option<f32> {
        if self.batt.seeded() { Some(self.batt.get()) } else { None }
    }
}

/// Welded-contact detection.
mod weld {
    use super::*;

    /// How long the bus gets to bleed down before a live TS counts as welded contacts. Plant time:
    /// the bleed-down is the model's, so it is simulated seconds that have to pass.
    const SETTLE: Duration = Duration::from_secs(5);
    /// TS as a fraction of pack, above which the bus counts as still connected.
    const RATIO: f32 = 0.85;

    #[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
    pub enum WeldState {
        Welded,
        Clear,
        /// The AIRs should be closed, there is no bus reading, the pack is too low to judge, or the bleed window has not elapsed.
        Unknown,
    }

    pub struct Detector {
        still_contacting: Debouncer,
    }

    impl Detector {
        pub const fn new() -> Self {
            Self { still_contacting: Debouncer::new() }
        }

        /// `now` is plant time: [`SETTLE`] is how long the bus takes to bleed down, which is the
        /// plant's doing.
        /// No plant clock, so the bleed window cannot be timed. That is not the same as the
        /// contacts being fine, which is why it reports [`WeldState::Unknown`] rather than Clear.
        pub fn hold(&mut self) -> WeldState {
            self.still_contacting.reset();
            WeldState::Unknown
        }

        pub fn tick(&mut self, now: PlantTime, inputs: Option<Inputs>, relay_open: bool) -> WeldState {
            // Not a test we can run. Clear the timer so the next real window starts from zero.
            let Some(inputs) = inputs.filter(|inputs| relay_open && pack_present(inputs.batt)) else {
                self.still_contacting.calculate(false, SETTLE, now);
                return WeldState::Unknown;
            };

            // No TS reading means no verdict: a quiet bus and a disconnected sensor look the same.
            let Some(ts) = inputs.ts else {
                self.still_contacting.calculate(false, SETTLE, now);
                return WeldState::Unknown;
            };

            let ts_still_up = ts.get() >= inputs.batt * RATIO;

            if self.still_contacting.calculate(ts_still_up, SETTLE, now) {
                WeldState::Welded
            } else if ts_still_up {
                WeldState::Unknown
            } else {
                WeldState::Clear
            }
        }
    }
}

mod task {
    use super::*;
    use crate::faults::{self, FaultCommand, PassFailAction};
    use crate::helpers::deadline::Deadline;
    use crate::hv_plate;
    use crate::shutdown;
    use weld::{Detector, WeldState};
    use embassy_time::Ticker;

    const PERIOD: Duration = Duration::from_millis(20);
    const DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(1);

    #[embassy_executor::task]
    pub async fn precharge_task() {
        let mut precharge = Precharge::new();
        let mut weld_detector = Detector::new();
        let mut ticker = Ticker::every(PERIOD);
        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();

        // Only queued on a change, so a welded bus does not spam the fault queue at 50 Hz.
        let mut weld_state = WeldState::Unknown;

        loop {
            ticker.next().await;

            // `None` until the plate has read BATT at least once.
            let inputs = crate::pack::batt_voltage().map(|batt| Inputs { ts: crate::pack::ts_voltage(), batt: batt.get() });

            let shutdown_closed = shutdown::circuit_closed();
            let relay_detected_state = hv_plate::relay_state();
            let relay_open = !shutdown_closed || matches!(relay_detected_state, hv_plate::RelayState::Open);

            // The bus charging and the contacts bleeding down are both the plant's doing, so both
            // dwells below are measured on its clock. Without one, neither machine has evidence
            // that anything has been held, and both hold where they are.
            let plant = crate::time::plant_now();
            let (requested, detected_weld_state) = match plant {
                Some(now) => (precharge.tick(now, inputs, shutdown_closed), weld_detector.tick(now, inputs, relay_open)),
                None => (precharge.hold(shutdown_closed), weld_detector.hold()),
            };

            crate::hv_plate::request_relay(requested);

            // Queue fault on edge
            if detected_weld_state != weld_state {
                match detected_weld_state {
                    WeldState::Unknown => weld_state = detected_weld_state,
                    WeldState::Welded | WeldState::Clear => {
                        let action = if detected_weld_state == WeldState::Welded { PassFailAction::NotifyBad } else { PassFailAction::NotifyOkay };
                        if faults::try_queue(FaultCommand::ContactorWeldedFault(action)).is_ok() {
                            if detected_weld_state == WeldState::Welded {
                                defmt::error!("HvPlate: precharge: AIR contacts appear welded -- the coil should be released but TS is still at pack voltage.");
                            }
                            weld_state = detected_weld_state;
                        }
                    },
                }
            }

            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt::trace!("HvPlate: precharge: {}", precharge.requested_state());

                // The AIRs may close on a reported TS, so which instrument supplied it is a
                // safety-relevant fact rather than a curiosity.
                defmt_monitor::monitor!("HvPlate/Precharge/TsSource", desc = "Which instrument supplied the TS voltage the AIRs are decided on. Anything but Plate means the bus is being judged from across the contactor; None means no source has one.", "{}", inputs.and_then(|reading| reading.ts).map(|ts| ts.source()));

                let (ts, batt) = (precharge.ts_average(), precharge.batt_average());
                let ratio = match (ts, batt) {
                    (Some(ts), Some(batt)) if batt > 0.0 => ts / batt,
                    _ => f32::NAN,
                };
                defmt_monitor::monitor!("HvPlate/Precharge/RequestedRelayState", desc = "Precharge supervisor state", "{}", precharge.requested_state());
                defmt_monitor::monitor!("HvPlate/Precharge/TsVolts", desc = "Averaged TS voltage, in volts. NaN means no source has ever reported one, which is not the same as a bus at 0 V.", "{=f32}", ts.unwrap_or(f32::NAN));
                defmt_monitor::monitor!("HvPlate/Precharge/BattVolts", desc = "Averaged pack voltage, in volts, off the 2950's BATT divider -- not the segments' cell sum. NaN until the plate has read BATT.", "{=f32}", batt.unwrap_or(f32::NAN));
                defmt_monitor::monitor!("HvPlate/Precharge/Ratio", desc = "Averaged TS voltage as a fraction of averaged BATT-side voltage. NaN when either is unknown.", "{=f32}", ratio);
                defmt_monitor::monitor!("HvPlate/Precharge/Weld", desc = "Welded-contact verdict. Unknown means the test could not run, not that the contacts are fine.", "{}", detected_weld_state);
                defmt_monitor::monitor!("HvPlate/Precharge/PlantClock", desc = "Whether the plant clock can be read. False on a rig whose model has not answered RDHILTIME, and while it is false the relay cannot close however charged the bus looks.", "{=bool}", plant.is_some());
                defmt_monitor::monitor!("HvPlate/Precharge/RelayState", desc = "What the 2950 last acknowledged about GPO4, which is not necessarily what the contacts are doing.", "{}", relay_detected_state);
            }
        }
    }
}
