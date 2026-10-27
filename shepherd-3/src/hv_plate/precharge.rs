use crate::helpers::moving_average::MovingAverage;
use crate::helpers::debouncer::Debouncer;
use crate::units::{Voltage, volt};
use embassy_time::{Duration, Instant};

// Exported functions
pub(in crate::hv_plate) use close::relay_should_be_closed;
pub use task::precharge_task;

const MINIMUM_PACK_VOLTAGE: Voltage = Voltage::from_volts(200.0);
const BATT_VOLT_BUFFER: Voltage = Voltage::from_volts(10.0);

/// Whether `batt` is plausible enough to compare TS against.
fn pack_present(batt: Voltage) -> bool {
    batt >= MINIMUM_PACK_VOLTAGE - BATT_VOLT_BUFFER
}

// u_TODO - use 2950 here probably
/// The bus as the analyzer last saw it.
#[derive(Copy, Clone)]
struct Bus {
    ts: Voltage,
    batt: Voltage,
}

impl Bus {
    /// `None` means there is no TS reading; `batt` is always available once the analyzer is.
    fn read() -> Option<Self> {
        let (ts, batt) = crate::pack::analyzer::analyzer().try_partial_get(|inner| (inner.data.ts_voltage, inner.data.pack_voltage))?;

        Some(Self { ts: ts?, batt })
    }
}

/// Deciding whether the AIRs may close.
mod close {
    use super::*;
    use atomic_enum::atomic_enum;
    use core::sync::atomic::Ordering;

    /// TS as a fraction of pack, at or above which the bus counts as charged.
    const TRIGGER_RATIO: f32 = 0.90;
    /// Depth of the averaging window on both bus voltages.
    const NUM_SAMPLES: usize = 5;
    /// Minimum dwell between relay transitions, in either direction.
    const TOGGLE_TIME: Duration = Duration::from_millis(200);

    /// What the bus looks like, which is the only thing the AIRs are decided on.
    #[atomic_enum]
    #[derive(PartialEq, Eq, defmt::Format)]
    pub enum PrechargeRelayState {
        Open = 0,
        Closed,
    }

    static STATE: AtomicPrechargeRelayState = AtomicPrechargeRelayState::new(PrechargeRelayState::Open);

    /// Whether the HV relay should be energized. [`Precharge::tick`] is the only writer.
    pub fn relay_should_be_closed() -> bool {
        matches!(STATE.load(Ordering::Relaxed), PrechargeRelayState::Closed)
    }

    pub struct Precharge {
        relay_state: PrechargeRelayState,
        ts: MovingAverage<NUM_SAMPLES>,
        batt: MovingAverage<NUM_SAMPLES>,
        to_closed: Debouncer,
        to_open: Debouncer,
    }

    impl Precharge {
        pub const fn new() -> Self {
            Self {
                relay_state: PrechargeRelayState::Open,
                ts: MovingAverage::new(),
                batt: MovingAverage::new(),
                to_closed: Debouncer::new(),
                to_open: Debouncer::new(),
            }
        }

        /// Ticks the precharge logic, publishing the result for [`relay_should_be_closed`].
        pub fn tick(&mut self, now: Instant, inputs: Option<Bus>, hv_permitted: bool) -> PrechargeRelayState {
            let state = self.decide(now, inputs, hv_permitted);
            STATE.store(state, Ordering::Relaxed);
            state
        }

        fn decide(&mut self, now: Instant, inputs: Option<Bus>, hv_permitted: bool) -> PrechargeRelayState {
            // Unpack and evaluate inputs
            let detected_state = match inputs {
                Some(inputs) => {
                    self.ts.push(inputs.ts.get::<volt>());
                    self.batt.push(inputs.batt.get::<volt>());
                    Self::evaluate(Voltage::from_volts(self.ts.get()), Voltage::from_volts(self.batt.get()))
                },
                // Default to open if we can't read inputs (gets debounced anyway)
                None => PrechargeRelayState::Open,
            };

            // Latch on closed relay state and only open if HV is not permitted
            if self.relay_state == PrechargeRelayState::Closed {
                if !hv_permitted {
                    self.transition_to(PrechargeRelayState::Open);
                }
                return self.relay_state;
            }

            if !hv_permitted {
                self.transition_to(PrechargeRelayState::Open);
                return self.relay_state;
            }

            if self.to_closed.calculate(detected_state == PrechargeRelayState::Closed, TOGGLE_TIME, now) {
                self.transition_to(PrechargeRelayState::Closed);
            }
            if self.to_open.calculate(detected_state == PrechargeRelayState::Open, TOGGLE_TIME, now) {
                self.transition_to(PrechargeRelayState::Open);
            }

            self.relay_state
        }

        fn evaluate(ts: Voltage, batt: Voltage) -> PrechargeRelayState {
            let ts_up = ts >= batt * TRIGGER_RATIO;
            if pack_present(batt) && ts_up { PrechargeRelayState::Closed } else { PrechargeRelayState::Open }
        }

        /// Moves to `next`, clearing every debouncer, so transitions can only happen in [`TOGGLE_TIME`].
        fn transition_to(&mut self, next: PrechargeRelayState) {
            if self.relay_state == next {
                return;
            }

            self.relay_state = next;
            self.to_closed = Debouncer::new();
            self.to_open = Debouncer::new();
        }

        /// Averaged TS voltage, in volts.
        pub const fn ts_average(&self) -> f32 {
            self.ts.get()
        }

        /// Averaged pack voltage, in volts.
        pub const fn batt_average(&self) -> f32 {
            self.batt.get()
        }
    }
}

/// Welded-contact detection.
mod weld {
    use super::*;

    /// How long the bus gets to bleed down before a live TS counts as welded contacts.
    const SETTLE: Duration = Duration::from_millis(400);
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

        pub fn tick(&mut self, now: Instant, inputs: Option<Bus>, should_be_open: bool) -> WeldState {
            // Not a test we can run. Clear the timer so the next real window starts from zero.
            let Some(inputs) = inputs.filter(|inputs| should_be_open && pack_present(inputs.batt)) else {
                self.still_contacting.calculate(false, SETTLE, now);
                return WeldState::Unknown;
            };

            let ts_still_up = inputs.ts >= inputs.batt * RATIO;

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
    use crate::hv_plate::core::relay;
    use close::Precharge;
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

            let now = Instant::now();
            let inputs = Bus::read();
            // u_TODO - should read this signal from shutdown sense
            let hv_permitted = true;

            let state = precharge.tick(now, inputs, hv_permitted);

            let relay_state = relay::state();
            let should_be_open = !hv_permitted || matches!(relay_state, relay::RelayState::Open);
            let detected_weld_state = weld_detector.tick(now, inputs, should_be_open);

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
                defmt::trace!("HvPlate: precharge: {}", state);

                let (ts, batt) = (precharge.ts_average(), precharge.batt_average());
                defmt_monitor::monitor!("HvPlate/Precharge/State", desc = "Precharge supervisor state", "{}", state);
                defmt_monitor::monitor!("HvPlate/Precharge/TsVolts", desc = "Averaged TS Voltage.", "{=f32}", ts);
                defmt_monitor::monitor!("HvPlate/Precharge/BattVolts", desc = "Averaged pack Voltage.", "{=f32}", batt);
                defmt_monitor::monitor!("HvPlate/Precharge/Ratio", desc = "Averaged TS voltage as a fraction of averaged pack voltage.", "{=f32}", if batt > 0.0 { ts / batt } else { f32::NAN });
                defmt_monitor::monitor!("HvPlate/Precharge/Weld", desc = "Welded-contact verdict. Unknown means the test could not run, not that the contacts are fine.", "{}", detected_weld_state);
                defmt_monitor::monitor!("HvPlate/Precharge/RelayState", desc = "What the 2950 last acknowledged about GPO4, which is not necessarily what the contacts are doing.", "{}", relay_state);
            }
        }
    }
}
