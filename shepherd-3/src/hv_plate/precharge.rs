use super::{cache, core as hv_core};
use crate::units::{Current, Voltage, volt};
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

pub fn state() -> State {
    State::from_repr(STATE.load(Ordering::Relaxed)).unwrap_or(State::Open)
}

/// `None` means the reading is missing, which resolves to [`State::Open`].
struct Inputs {
    ts: Option<Voltage>,
    batt: Option<Voltage>,
    current: Option<Current>,
}

pub struct Action {
    pub relay_closed: bool,
    pub state: State,
}

pub struct Precharge {
    relay_state: State,
    ts: VoltageAverager,
    batt: VoltageAverager,
    to_closed: Debouncer,
    to_open: Debouncer,
    floating: Debouncer,
    closed_lost: Debouncer,
}

impl Precharge {
    pub const fn new() -> Self {
        Self {
            relay_state: State::Open,
            ts: VoltageAverager::new(),
            batt: VoltageAverager::new(),
            to_closed: Debouncer::new(),
            to_open: Debouncer::new(),
            floating: Debouncer::new(),
            closed_lost: Debouncer::new(),
        }
    }

    /// Ticks the precharge supervisor.
    ///
    /// Determine current state of precharge routine then send [`Action`] back to hv plate task to perform.
    pub fn tick(&mut self, now: Instant) -> Action {
        let detected = self.classify(inputs());

        if self.to_closed.poll(detected == State::Closed, TOGGLE_TIME, now) {
            self.relay_state = State::Closed;
        }

        if self.to_open.poll(detected == State::Open, TOGGLE_TIME, now) {
            self.relay_state = State::Open;
        }

        let drifting = matches!(self.relay_state, State::Open | State::Floating) && detected == State::Floating;
        if self.floating.poll(drifting, FLOATING_TIME, now) {
            self.relay_state = State::Floating;
        }

        let lost = self.relay_state == State::Closed && matches!(detected, State::Floating | State::Open);
        if self.closed_lost.poll(lost, TOGGLE_TIME, now) {
            self.relay_state = State::Open;
        }

        STATE.store(self.relay_state as u8, Ordering::Relaxed);

        Action {
            relay_closed: self.relay_state == State::Closed,
            state: self.relay_state,
        }
    }

    /// Port of `get_precharge_state`
    fn classify(&mut self, inputs: Inputs) -> State {
        use uom::si::electric_current::ampere;

        let Inputs { ts: Some(ts), batt: Some(batt), current: Some(current) } = inputs else {
            return State::Open;
        };

        self.ts.push(ts);
        self.batt.push(batt);

        let (Some(ts), Some(batt)) = (self.ts.average(), self.batt.average()) else {
            return State::Open;
        };
        let amps = current.get::<ampere>();

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
}

/// Level-triggered, not edge-triggered: once held for `period` it reports true on every
/// subsequent call.
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

/// Fixed-depth moving average, in volts.
///
/// One push per tick against data that refreshes every tick.
struct VoltageAverager {
    samples: [f32; NUM_SAMPLES],
    index: usize,
    len: usize,
}

impl VoltageAverager {
    const fn new() -> Self {
        Self { samples: [0.0; NUM_SAMPLES], index: 0, len: 0 }
    }

    fn push(&mut self, sample: Voltage) {
        self.samples[self.index] = sample.get::<volt>();
        self.index = (self.index + 1) % NUM_SAMPLES;
        if self.len < NUM_SAMPLES {
            self.len += 1;
        }
    }

    /// `None` until at least one sample has landed.
    ///
    /// `len` is tracked rather than averaging the whole array, so the zero-filled tail cannot
    /// drag the mean down before the window fills.
    fn average(&self) -> Option<Voltage> {
        if self.len == 0 {
            return None;
        }

        let sum: f32 = self.samples[..self.len].iter().sum();
        Some(Voltage::new::<volt>(sum / self.len as f32))
    }
}

/// This tick's inputs. Anything missing comes back `None`.
fn inputs() -> Inputs {
    Inputs {
        ts: hv_core::api::ts_voltage(),
        batt: cache::CACHE.get_current_voltage().try_nice().map(|r| r.batt_voltage).ok(),
        current: hv_core::api::pack_current(),
    }
}
