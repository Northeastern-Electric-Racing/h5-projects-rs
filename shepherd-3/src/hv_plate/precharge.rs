use crate::units::{Current, Voltage, volt};
use uom::si::electric_current::ampere;
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_time::{Duration, Instant};
use strum::FromRepr;

const MINIMUM_PACK_VOLTAGE: Voltage = Voltage::from_volts(200.0);
const BATT_VOLT_BUFFER: Voltage = Voltage::from_volts(10.0);
/// Below this the bus is down, not floating.
const TS_VOLT_BUFFER: Voltage = Voltage::from_volts(5.0);
const TRIGGER_RATIO: f32 = 0.90;
const FLOATING_CURRENT_MAX_A: f32 = 1.0;
const NUM_SAMPLES: usize = 5;
const TOGGLE_TIME: Duration = Duration::from_millis(200);
const FLOATING_TIME: Duration = Duration::from_millis(10_000);

/// Floating sits between Open and Closed.
#[derive(Copy, Clone, PartialEq, Eq, FromRepr, defmt::Format)]
#[repr(u8)]
pub enum State {
    Open = 0,
    Floating,
    Closed,
}

static STATE: AtomicU8 = AtomicU8::new(State::Open as u8);

/// Whether the HV relay should be energized.
pub(in crate::hv_plate) fn relay_should_be_closed() -> bool {
    matches!(State::from_repr(STATE.load(Ordering::Relaxed)).unwrap_or(State::Open), State::Closed)
}

pub struct Precharge {
    relay_state: State,
    ts: Averager,
    batt: Averager,
    to_closed: Debouncer,
    to_open: Debouncer,
    floating: Debouncer,
}

impl Precharge {
    pub const fn new() -> Self {
        Self {
            relay_state: State::Open,
            ts: Averager::new(),
            batt: Averager::new(),
            to_closed: Debouncer::new(),
            to_open: Debouncer::new(),
            floating: Debouncer::new(),
        }
    }

    /// Ticks the precharge logic.
    pub fn tick(&mut self, now: Instant, inputs: Option<task::Inputs>, hv_permitted: bool) -> State {
        // Unpack and evaluate inputs
        let detected_state = match inputs {
            Some(inputs) => {
                self.ts.push(inputs.ts.get::<volt>());
                self.batt.push(inputs.batt.get::<volt>());
                Self::evaluate(Voltage::from_volts(self.ts.average()), Voltage::from_volts(self.batt.average()), inputs.current.get::<ampere>())
            },
            // Default to open if we can't read inputs (gets debounced anyway)
            None => State::Open,
        };

        // Latch on closed relay state and only open if HV is not permitted
        if self.relay_state == State::Closed {
            if !hv_permitted {
                self.transition_to(State::Open);
            }
            return self.relay_state;
        }

        if !hv_permitted {
            self.transition_to(State::Open);
            return self.relay_state;
        }

        if self.to_closed.poll(detected_state == State::Closed, TOGGLE_TIME, now) {
            self.transition_to(State::Closed);
        }
        if self.to_open.poll(detected_state == State::Open, TOGGLE_TIME, now) {
            self.transition_to(State::Open);
        }

        // A bus that sits partly charged without ever reaching the ratio has failed to
        // precharge. `FLOATING_TIME` is a precharge timeout, not a debounce.
        let drifting = matches!(self.relay_state, State::Open | State::Floating) && detected_state == State::Floating;
        if self.floating.poll(drifting, FLOATING_TIME, now) {
            self.transition_to(State::Floating);
        }

        self.relay_state
    }

    fn evaluate(ts: Voltage, batt: Voltage, amps: f32) -> State {
        if batt < MINIMUM_PACK_VOLTAGE - BATT_VOLT_BUFFER {
            State::Open
        } else if ts >= batt * TRIGGER_RATIO {
            State::Closed
        } else if amps.abs() <= FLOATING_CURRENT_MAX_A && ts > TS_VOLT_BUFFER {
            // The current check separates a stuck bus from one that is simply loaded.
            State::Floating
        } else {
            State::Open
        }
    }

    /// Moves to `next`, clearing every debouncer, so transitions can only happen in [`TOGGLE_TIME`].
    fn transition_to(&mut self, next: State) {
        if self.relay_state == next {
            return;
        }

        self.relay_state = next;
        self.to_closed = Debouncer::new();
        self.to_open = Debouncer::new();
        self.floating = Debouncer::new();
    }

    /// Averaged TS voltage, in volts.
    pub const fn ts_average(&self) -> f32 {
        self.ts.average()
    }

    /// Averaged pack voltage, in volts.
    pub const fn batt_average(&self) -> f32 {
        self.batt.average()
    }
}

/// Level-triggered, not edge-triggered: once held for `period` it reports true on every subsequent call.
struct Debouncer {
    armed_at: Option<Instant>,
}

impl Debouncer {
    const fn new() -> Self {
        Self { armed_at: None }
    }

    fn poll(&mut self, input: bool, period: Duration, now: Instant) -> bool {
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

/// Fixed-depth averager [`NUM_SAMPLES`]
struct Averager {
    samples: [f32; NUM_SAMPLES],
    index: usize,
    average: f32,
    seeded: bool,
}

impl Averager {
    const fn new() -> Self {
        Self { samples: [0.0; NUM_SAMPLES], index: 0, average: 0.0, seeded: false }
    }

    /// Refills the entire window with `value`, so the average is `value` immediately.
    const fn reset(&mut self, value: f32) {
        self.samples = [value; NUM_SAMPLES];
        self.index = 0;
        self.average = value;
        self.seeded = true;
    }

    fn push(&mut self, sample: f32) {
        if !self.seeded {
            self.reset(sample);
            return;
        }

        self.samples[self.index] = sample;
        self.index = (self.index + 1) % NUM_SAMPLES;

        self.average = self.samples.iter().sum::<f32>() / NUM_SAMPLES as f32;
    }

    const fn average(&self) -> f32 {
        self.average
    }
}

pub mod task {
    use super::*;
    use crate::helpers::deadline::Deadline;
    use core::sync::atomic::Ordering;
    use embassy_time::{Duration, Instant, Ticker};

    /// What a tick needs to see of the bus.
    #[derive(Copy, Clone)]
    pub struct Inputs {
        pub ts: Voltage,
        pub batt: Voltage,
        pub current: Current,
    }

    impl Inputs {
        fn read() -> Option<Self> {
            let analyzer = crate::pack::analyzer::analyzer()?;

            Some(Self {
                ts: analyzer.data.ts_voltage?,
                batt: analyzer.data.pack_voltage,
                current: analyzer.data.pack_current?,
            })
        }
    }

    const PERIOD: Duration = Duration::from_millis(20);

    const DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(1);

    #[embassy_executor::task]
    pub async fn precharge_task() {
        let mut precharge = Precharge::new();
        let mut ticker = Ticker::every(PERIOD);
        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();

        loop {
            ticker.next().await;

            let now = Instant::now();
            let inputs = Inputs::read();
            // u_TODO - should read from the shutdown loop to decide whether HV is permitted or not.
            let hv_permitted = true;

            let state = precharge.tick(now, inputs, hv_permitted);
            STATE.store(state as u8, Ordering::Relaxed);
            defmt::trace!("HvPlate: precharge: {}", state);

            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);

                let (ts, batt) = (precharge.ts_average(), precharge.batt_average());
                defmt_monitor::monitor!("HvPlate/Precharge/State", desc = "Precharge supervisor state", "{}", state);
                defmt_monitor::monitor!("HvPlate/Precharge/TsVolts", desc = "Averaged TS Voltage.", "{=f32}", ts);
                defmt_monitor::monitor!("HvPlate/Precharge/BattVolts", desc = "Averaged pack Voltage.", "{=f32}", batt);
                defmt_monitor::monitor!("HvPlate/Precharge/Amps", desc = "Pack current, in amps.", "{}", inputs.map(|inputs| inputs.current.get::<ampere>()));
                defmt_monitor::monitor!("HvPlate/Precharge/Ratio", desc = "Averaged TS voltage as a fraction of averaged pack voltage.", "{=f32}", if batt > 0.0 { ts / batt } else { f32::NAN });
            }
        }
    }
}
pub use task::precharge_task;
