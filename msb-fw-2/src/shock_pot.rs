//! Shock potentiometer polling. Ported from `u_shock_pot.c` in the C firmware.

use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, SharedMux};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

const POLL_PERIOD: Duration = Duration::from_millis(20);

const MAX_VOLTS: f32 = 3.3;
const MAX_ADC_VAL_12B: f32 = 4095.0;

const ZERO_OFFSET: [f32; NUM_SHOCK_POTS] = [0.0, 0.0];
const SCALE_FACTOR: [f32; NUM_SHOCK_POTS] = [1.0, 1.0];
/// Voltage at zero travel. MEASURE AND REPLACE THIS.
const CALIBRATED_V: [f32; NUM_SHOCK_POTS] = [3.3, 3.3];
/// Per-pot trim from the C firmware (left / right).
const TRAVEL_TRIM_IN: [f32; NUM_SHOCK_POTS] = [0.195, 0.140];
/// Full stroke length. REPLACE THIS.
const SHOCK_POT_LENGTH_IN: f32 = 1.9685;

pub const NUM_SHOCK_POTS: usize = 2;

/// Max number of tasks that can subscribe to [`SHOCK_POT_DATA`].
const MAX_RECEIVERS: usize = 2;

/// Latest shock pot readings, indexed by [`ShockPot`].
pub static SHOCK_POT_DATA: Watch<ThreadModeRawMutex, [ShockPotReading; NUM_SHOCK_POTS], MAX_RECEIVERS> = Watch::new();

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ShockPot {
    /// `SHOCK_POT1` in the C firmware.
    Front = 0,
    /// `SHOCK_POT2` in the C firmware.
    Back = 1,
}

impl ShockPot {
    pub const ALL: [ShockPot; NUM_SHOCK_POTS] = [ShockPot::Front, ShockPot::Back];

    /// Fixed-width name so log rows line up.
    const fn name(self) -> &'static str {
        match self {
            ShockPot::Front => "front",
            ShockPot::Back => "back ",
        }
    }

    /// Where this pot is wired on the muxes (LPF1 / LPF2 on U18).
    const fn source(self) -> (MuxId, MuxChannel, MuxInput) {
        match self {
            ShockPot::Front => (MuxId::U18, MuxChannel::Ch1, MuxInput::B),
            ShockPot::Back => (MuxId::U18, MuxChannel::Ch2, MuxInput::B),
        }
    }
}

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct ShockPotReading {
    pub raw: u16,
    pub volts: f32,
    /// Calibrated voltage.
    pub position: f32,
    pub inch_travel: f32,
}

/// Converts a raw ADC reading into calibrated values.
pub fn convert(pot: ShockPot, raw: u16) -> ShockPotReading {
    let i = pot as usize;
    let volts = f32::from(raw) * MAX_VOLTS / MAX_ADC_VAL_12B;
    let position = (volts - ZERO_OFFSET[i]) * SCALE_FACTOR[i];
    let inch_travel = (CALIBRATED_V[i] - position) * (SHOCK_POT_LENGTH_IN / MAX_VOLTS) - TRAVEL_TRIM_IN[i];

    ShockPotReading { raw, volts, position, inch_travel }
}

#[embassy_executor::task]
pub async fn shock_pot_task(mux: &'static SharedMux) {
    let sender = SHOCK_POT_DATA.sender();
    let mut ticker = Ticker::every(POLL_PERIOD);

    loop {
        // Read every mux source; only the shock pots (LPF1 / LPF2) are used for now.
        let snapshot = mux.lock().await.read_all().await;

        let readings = ShockPot::ALL.map(|pot| {
            let (id, channel, input) = pot.source();
            convert(pot, snapshot.get(id, channel, input))
        });
        for pot in ShockPot::ALL {
            let r = readings[pot as usize];
            defmt::info!("Shock pot {=str}: raw={=u16} volts={=f32} in={=f32}", pot.name(), r.raw, r.volts, r.inch_travel);
        }
        sender.send(readings);

        ticker.next().await;
    }
}
