use super::cache;
use crate::units::{Voltage, volt};
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

/// The current precharge state.
pub fn state() -> State {
    State::from_repr(STATE.load(Ordering::Relaxed)).unwrap_or(State::Open)
}

/// Whether the HV relay should be energized.
pub(in crate::hv_plate) fn relay_should_be_closed() -> bool {
    matches!(state(), State::Closed)
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

    /// Ticks the precharge logic
    pub fn tick(&mut self, now: Instant, readings: Option<task::Inputs>) -> State {
        let detected = self.classify(readings);

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

        self.relay_state
    }

    fn classify(&mut self, readings: Option<task::Inputs>) -> State {
        use uom::si::electric_current::ampere;

        let Some(task::Inputs { ts, batt, current }) = readings else {
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

/// Fixed-depth moving average, in volts.
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

pub mod task {
    use super::*;
    use crate::helpers::deadline::Deadline;
    use core::sync::atomic::Ordering;
    use embassy_time::{Duration, Instant, Ticker};

    #[derive(Copy, Clone)]
    pub struct Inputs {
        pub ts: Voltage,
        pub batt: Voltage,
        pub current: crate::units::Current,
    }

    impl Inputs {
        fn read(batt: &cache::current_voltage::Nice) -> Option<Self> {
            let analyzer = crate::pack::analyzer::analyzer()?;

            Some(Self {
                ts: analyzer.data.ts_voltage?,
                batt: batt.data().batt_voltage,
                current: analyzer.data.pack_current?,
            })
        }
    }

    const PERIOD: Duration = Duration::from_millis(100);
    const DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(1);

    #[embassy_executor::task]
    pub async fn precharge_task() {
        let batt = cache().get_current_voltage().nice().ready().await;

        let mut precharge = Precharge::new();
        let mut ticker = Ticker::every(PERIOD);
        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();

        loop {
            ticker.next().await;

            let now = Instant::now();
            let readings = Inputs::read(&batt.get());

            let state = precharge.tick(now, readings);
            STATE.store(state as u8, Ordering::Relaxed);
            defmt::trace!("HvPlate: precharge: {}", state);

            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt_monitor::monitor!("HvPlate/Precharge/State", desc = "Precharge supervisor state: 0 Open, 1 Floating, 2 Closed.", "{}", state);
            }
        }
    }
}

pub use task::precharge_task;
