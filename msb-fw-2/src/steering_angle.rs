//! Steering angle polling. Ported from `u_steering_angle.c` in the C firmware.

use crate::analog_sensor::{self, AnalogSensor, SensorData};
use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, MuxSource};
use embassy_sync::watch::Watch;
use embassy_time::Duration;

/// Sensor voltage at full right lock.
const V_MIN: f32 = 1.0626;
/// Sensor voltage with the wheel centred.
const V_STRAIGHT: f32 = 1.80;
/// Sensor voltage at full left lock.
const V_MAX: f32 = 2.434;

/// Angle magnitude at full right lock (`V_MIN`), degrees.
const ANGLE_RIGHT_MAX: f32 = 87.0;
/// Angle magnitude at full left lock (`V_MAX`), degrees.
const ANGLE_LEFT_MAX: f32 = 75.0;

/// How far outside the sensor's physical range a voltage may go before it's flagged as a fault
/// (disconnected or shorted sensor).
const FAULT_MARGIN_V: f32 = 0.1;

pub const NUM_STEERING_ANGLES: usize = 1;

/// Latest steering angle reading.
pub static STEERING_ANGLE_DATA: SensorData<SteeringAngleReading, NUM_STEERING_ANGLES> = Watch::new();

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct SteeringAngleReading {
    pub raw: u16,
    pub volts: f32,
    /// Degrees; positive is right (up to +87), negative is left (down to -75).
    pub angle_deg: f32,
    /// `false` if the voltage is well outside the sensor's physical range. The angle is clamped to
    /// full lock in that case, so it shouldn't be trusted.
    pub in_range: bool,
}

/// The steering angle sensor, as an [`AnalogSensor`].
pub struct SteeringAngle;

impl AnalogSensor<NUM_STEERING_ANGLES> for SteeringAngle {
    const POLL_PERIOD: Duration = Duration::from_millis(20);
    /// LPF3 on U18 (PC3, ADC1 ch13), read with the same SEL state as the shock pots.
    const SOURCES: [MuxSource; NUM_STEERING_ANGLES] = [(MuxId::U18, MuxChannel::Ch3, MuxInput::B)];
    const NAMES: [&'static str; NUM_STEERING_ANGLES] = ["steering"];

    type Reading = SteeringAngleReading;

    fn convert(_i: usize, raw: u16, volts: f32) -> SteeringAngleReading {
        let in_range = (V_MIN - FAULT_MARGIN_V..=V_MAX + FAULT_MARGIN_V).contains(&volts);

        let v = volts.clamp(V_MIN, V_MAX);
        let angle_deg = if v <= V_STRAIGHT {
            ANGLE_RIGHT_MAX * (V_STRAIGHT - v) / (V_STRAIGHT - V_MIN)
        } else {
            -ANGLE_LEFT_MAX * (v - V_STRAIGHT) / (V_MAX - V_STRAIGHT)
        };

        SteeringAngleReading { raw, volts, angle_deg, in_range }
    }

    fn log(_name: &'static str, r: &SteeringAngleReading) {
        defmt::info!("Steering angle: raw={=u16} volts={=f32} angle={=f32}deg", r.raw, r.volts, r.angle_deg);
        if !r.in_range {
            defmt::warn!("Steering angle sensor out of range ({=f32} V); check wiring.", r.volts);
        }
    }

    fn data() -> &'static SensorData<SteeringAngleReading, NUM_STEERING_ANGLES> {
        &STEERING_ANGLE_DATA
    }
}

#[embassy_executor::task]
pub async fn steering_angle_task() {
    analog_sensor::run::<SteeringAngle, NUM_STEERING_ANGLES>().await
}
