//! Shock potentiometer polling. Ported from `u_shock_pot.c` in the C firmware.

use crate::analog_sensor::{self, AnalogSensor, MAX_VOLTS, SensorData};
use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, MuxSource};
use embassy_sync::watch::Watch;
use embassy_time::Duration;

const ZERO_OFFSET: [f32; NUM_SHOCK_POTS] = [0.0, 0.0];
const SCALE_FACTOR: [f32; NUM_SHOCK_POTS] = [1.0, 1.0];
/// Voltage at zero travel. MEASURE AND REPLACE THIS.
const CALIBRATED_V: [f32; NUM_SHOCK_POTS] = [3.3, 3.3];
/// Per-pot trim from the C firmware, indexed by [`ShockPot`] (pot 1 = right, pot 2 = left).
const TRAVEL_TRIM_IN: [f32; NUM_SHOCK_POTS] = [0.195, 0.140];
/// Full stroke length. REPLACE THIS.
const SHOCK_POT_LENGTH_IN: f32 = 1.9685;

pub const NUM_SHOCK_POTS: usize = 2;

/// Latest shock pot readings, indexed by [`ShockPot`].
pub static SHOCK_POT_DATA: SensorData<ShockPotReading, NUM_SHOCK_POTS> = Watch::new();

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ShockPot {
    /// `SHOCK_POT1` in the C firmware.
    Right = 0,
    /// `SHOCK_POT2` in the C firmware.
    Left = 1,
}

impl ShockPot {
    pub const ALL: [ShockPot; NUM_SHOCK_POTS] = [ShockPot::Right, ShockPot::Left];

    /// Fixed-width name so log rows line up.
    const fn name(self) -> &'static str {
        match self {
            ShockPot::Right => "right",
            ShockPot::Left => "left ",
        }
    }

    /// Where this pot is wired on the muxes (LPF1 / LPF2 on U18).
    const fn source(self) -> MuxSource {
        match self {
            ShockPot::Right => (MuxId::U18, MuxChannel::Ch1, MuxInput::B),
            ShockPot::Left => (MuxId::U18, MuxChannel::Ch2, MuxInput::B),
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

/// Both shock pots, as one [`AnalogSensor`].
pub struct ShockPots;

impl AnalogSensor<NUM_SHOCK_POTS> for ShockPots {
    const POLL_PERIOD: Duration = Duration::from_millis(20);
    const SOURCES: [MuxSource; NUM_SHOCK_POTS] = [ShockPot::Right.source(), ShockPot::Left.source()];
    const NAMES: [&'static str; NUM_SHOCK_POTS] = [ShockPot::Right.name(), ShockPot::Left.name()];

    type Reading = ShockPotReading;

    fn convert(i: usize, raw: u16, volts: f32) -> ShockPotReading {
        let position = (volts - ZERO_OFFSET[i]) * SCALE_FACTOR[i];
        let inch_travel = (CALIBRATED_V[i] - position) * (SHOCK_POT_LENGTH_IN / MAX_VOLTS) - TRAVEL_TRIM_IN[i];

        ShockPotReading { raw, volts, position, inch_travel }
    }

    fn log(name: &'static str, r: &ShockPotReading) {
        defmt::info!("Shock pot {=str}: raw={=u16} volts={=f32} in={=f32}", name, r.raw, r.volts, r.inch_travel);
    }

    fn data() -> &'static SensorData<ShockPotReading, NUM_SHOCK_POTS> {
        &SHOCK_POT_DATA
    }
}

#[embassy_executor::task]
pub async fn shock_pot_task() {
    analog_sensor::run::<ShockPots, NUM_SHOCK_POTS>().await
}
