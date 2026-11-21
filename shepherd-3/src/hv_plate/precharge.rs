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

/// The bus as the analyzer last saw it.
#[derive(Copy, Clone)]
struct BusReading {
    /// `None` when nothing is currently reporting TS voltage.
    ///
    /// Separate from the reading as a whole on purpose: `batt` is available as soon as the
    /// analyzer has published, so an unknown TS must not take it down too. It used to, and the
    /// averages below were never fed, which read out as a pack sitting at 0 V forever.
    ts: Option<Voltage>,
    batt: Voltage,
}

impl BusReading {
    /// `None` only before the analyzer's first publish.
    fn read() -> Option<Self> {
        let (ts, batt) = crate::pack::analyzer::analyzer().try_partial_get(|inner| (inner.data.ts_voltage, inner.data.pack_voltage))?;

        Some(Self { ts, batt })
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

    /// What the precharge supervisor is asking the relay to do.
    #[atomic_enum]
    #[derive(PartialEq, Eq, defmt::Format)]
    pub enum RelayRequest {
        Open = 0,
        Closed,
    }

    static STATE: AtomicRelayRequest = AtomicRelayRequest::new(RelayRequest::Open);

    /// Whether the HV relay should be energized. [`Precharge::tick`] is the only writer.
    pub fn relay_should_be_closed() -> bool {
        matches!(STATE.load(Ordering::Relaxed), RelayRequest::Closed)
    }

    pub struct Precharge {
        requested_state: RelayRequest,
        ts: MovingAverage<NUM_SAMPLES>,
        batt: MovingAverage<NUM_SAMPLES>,
        to_closed: Debouncer,
        to_open: Debouncer,
    }

    impl Precharge {
        pub const fn new() -> Self {
            Self {
                requested_state: RelayRequest::Open,
                ts: MovingAverage::new(),
                batt: MovingAverage::new(),
                to_closed: Debouncer::new(),
                to_open: Debouncer::new(),
            }
        }

        /// Ticks the precharge logic, publishing the result for [`relay_should_be_closed`].
        pub fn tick(&mut self, now: Instant, bus_reading: Option<BusReading>, hv_permitted: bool) -> RelayRequest {
            let state = self.decide(now, bus_reading, hv_permitted);
            STATE.store(state, Ordering::Relaxed);
            state
        }

        fn decide(&mut self, now: Instant, bus_reading: Option<BusReading>, hv_permitted: bool) -> RelayRequest {
            // Unpack and evaluate ts and batt voltage readings
            let state_from_ts_batt_volts = match bus_reading {
                Some(BusReading { ts: Some(ts), batt }) => {
                    self.ts.push(ts.get::<volt>());
                    self.batt.push(batt.get::<volt>());
                    Self::evaluate(Voltage::from_volts(self.ts.get()), Voltage::from_volts(self.batt.get()))
                },

                // The pack is measurable but the bus is not, so it cannot be shown to be charged.
                // Still average the pack voltage: it is the reference the ratio is taken against,
                // and the diagnostics below are the only way to see which input is missing.
                Some(BusReading { ts: None, batt }) => {
                    self.batt.push(batt.get::<volt>());
                    RelayRequest::Open
                },

                // Default to open if we can't read inputs (gets debounced anyway)
                None => RelayRequest::Open,
            };

            // The acc shutdown loop outranks everything below it including the latch.
            if hv_permitted == false {
                self.transition_to(RelayRequest::Open);
                self.disarm();
                return self.requested_state;
            }

            // Latched on closed relay
            if self.requested_state == RelayRequest::Closed {
                return self.requested_state;
            }

            if self.to_closed.calculate(state_from_ts_batt_volts == RelayRequest::Closed, TOGGLE_TIME, now) {
                self.transition_to(RelayRequest::Closed);
            }
            if self.to_open.calculate(state_from_ts_batt_volts == RelayRequest::Open, TOGGLE_TIME, now) {
                self.transition_to(RelayRequest::Open);
            }

            self.requested_state
        }

        fn evaluate(ts: Voltage, batt: Voltage) -> RelayRequest {
            let ts_up = ts >= batt * TRIGGER_RATIO;
            if pack_present(batt) && ts_up { RelayRequest::Closed } else { RelayRequest::Open }
        }

        /// Moves to `next`, clearing every debouncer, so transitions can only happen in [`TOGGLE_TIME`].
        fn transition_to(&mut self, next: RelayRequest) {
            if self.requested_state == next {
                return;
            }

            self.requested_state = next;
            self.disarm();
        }

        /// Clears both transition timers, so nothing armed on an earlier tick can fire on the next one.
        fn disarm(&mut self) {
            self.to_closed = Debouncer::new();
            self.to_open = Debouncer::new();
        }

        /// Averaged TS voltage, in volts, or `None` if no TS reading has ever arrived.
        pub const fn ts_average(&self) -> Option<f32> {
            if self.ts.seeded() { Some(self.ts.get()) } else { None }
        }

        /// Averaged pack voltage, in volts, or `None` if the analyzer has never published.
        pub const fn batt_average(&self) -> Option<f32> {
            if self.batt.seeded() { Some(self.batt.get()) } else { None }
        }
    }
}

/// Welded-contact detection.
mod weld {
    use super::*;

    /// How long the bus gets to bleed down before a live TS counts as welded contacts.
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

        pub fn tick(&mut self, now: Instant, inputs: Option<BusReading>, should_be_open: bool) -> WeldState {
            // Not a test we can run. Clear the timer so the next real window starts from zero.
            let Some(inputs) = inputs.filter(|inputs| should_be_open && pack_present(inputs.batt)) else {
                self.still_contacting.calculate(false, SETTLE, now);
                return WeldState::Unknown;
            };

            // No TS reading means no verdict: a quiet bus and a disconnected sensor look the same.
            let Some(ts) = inputs.ts else {
                self.still_contacting.calculate(false, SETTLE, now);
                return WeldState::Unknown;
            };

            let ts_still_up = ts >= inputs.batt * RATIO;

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
            let inputs = BusReading::read();

            // Read shutdown sense onbaord acc to see if hv should be permitted
            let hv_permitted = shutdown::circuit_closed();
            let requested_state = precharge.tick(now, inputs, hv_permitted);

            // Detect contactors welded state
            let acknowledged_state = hv_plate::core::relay::state();
            let should_be_open = !hv_permitted || matches!(acknowledged_state, hv_plate::core::relay::RelayState::Open);
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
                defmt::trace!("HvPlate: precharge: {}", requested_state);

                let (ts, batt) = (precharge.ts_average(), precharge.batt_average());
                let ratio = match (ts, batt) {
                    (Some(ts), Some(batt)) if batt > 0.0 => ts / batt,
                    _ => f32::NAN,
                };
                defmt_monitor::monitor!("HvPlate/Precharge/State", desc = "Precharge supervisor state", "{}", requested_state);
                defmt_monitor::monitor!("HvPlate/Precharge/TsVolts", desc = "Averaged TS voltage, in volts. NaN means no source has ever reported one, which is not the same as a bus at 0 V.", "{=f32}", ts.unwrap_or(f32::NAN));
                defmt_monitor::monitor!("HvPlate/Precharge/BattVolts", desc = "Averaged pack voltage, in volts, summed from the segment cell readings. NaN until the analyzer has published.", "{=f32}", batt.unwrap_or(f32::NAN));
                defmt_monitor::monitor!("HvPlate/Precharge/Ratio", desc = "Averaged TS voltage as a fraction of averaged pack voltage. NaN when either is unknown.", "{=f32}", ratio);
                defmt_monitor::monitor!("HvPlate/Precharge/Weld", desc = "Welded-contact verdict. Unknown means the test could not run, not that the contacts are fine.", "{}", detected_weld_state);
                defmt_monitor::monitor!("HvPlate/Precharge/RelayState", desc = "What the 2950 last acknowledged about GPO4, which is not necessarily what the contacts are doing.", "{}", acknowledged_state);
            }
        }
    }
}
