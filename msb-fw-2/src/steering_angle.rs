//! Steering angle polling. Ported from `u_steering_angle.c` in the C firmware.

use crate::multiplexor_handler::{MUX_SNAPSHOT, MuxChannel, MuxId, MuxInput};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

const POLL_PERIOD: Duration = Duration::from_millis(20);

const MAX_VOLTS: f32 = 3.3;
const MAX_ADC_VAL_12B: f32 = 4095.0;

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

/// LPF3 on U18 (PC3, ADC1 ch13), read with the same SEL state as the shock pots.
const SOURCE: (MuxId, MuxChannel, MuxInput) = (MuxId::U18, MuxChannel::Ch3, MuxInput::B);

/// Max number of tasks that can subscribe to [`STEERING_ANGLE_DATA`].
const MAX_RECEIVERS: usize = 2;

/// Latest steering angle reading.
pub static STEERING_ANGLE_DATA: Watch<ThreadModeRawMutex, SteeringAngleReading, MAX_RECEIVERS> = Watch::new();

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

/// Converts a raw ADC reading into a steering angle.
pub fn convert(raw: u16) -> SteeringAngleReading {
    let volts = f32::from(raw) * MAX_VOLTS / MAX_ADC_VAL_12B;
    let in_range = (V_MIN - FAULT_MARGIN_V..=V_MAX + FAULT_MARGIN_V).contains(&volts);

    let v = volts.clamp(V_MIN, V_MAX);
    let angle_deg = if v <= V_STRAIGHT {
        ANGLE_RIGHT_MAX * (V_STRAIGHT - v) / (V_STRAIGHT - V_MIN)
    } else {
        -ANGLE_LEFT_MAX * (v - V_STRAIGHT) / (V_MAX - V_STRAIGHT)
    };

    SteeringAngleReading { raw, volts, angle_deg, in_range }
}

#[embassy_executor::task]
pub async fn steering_angle_task() {
    let mut rx = MUX_SNAPSHOT.receiver().expect("Too many MUX_SNAPSHOT receivers.");
    let sender = STEERING_ANGLE_DATA.sender();
    let mut ticker = Ticker::every(POLL_PERIOD);

    loop {
        let snapshot = rx.get().await;
        let (id, channel, input) = SOURCE;
        let reading = convert(snapshot.get(id, channel, input));

        defmt::info!("Steering angle: raw={=u16} volts={=f32} angle={=f32}deg", reading.raw, reading.volts, reading.angle_deg);
        if !reading.in_range {
            defmt::warn!("Steering angle sensor out of range ({=f32} V); check wiring.", reading.volts);
        }
        sender.send(reading);

        ticker.next().await;
    }
}
