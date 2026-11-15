//! Battery box fan duty logic (4 fans on PWM0).
//!
//! Port of `update_batt_box_fan` and `write_fan_duty_cycle_percentage`
//! (device_fan0 only) from TSECU-Shepherd `Core/Src/control.c`.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_time::Timer;
use uom::si::thermodynamic_temperature::degree_celsius;

use crate::can::types::FanDutyCyclePercentage;
use crate::{FanResources, can, pack};

/// TODO: set to the TIM3 frequency the C code configures (htim3 init).
const FAN_PWM_FREQ_HZ: u32 = 25_000;
/// How often the duty is recomputed, matching the C control thread (100 ms).
const FAN_UPDATE_PERIOD_MS: u64 = 100;

/// How often the final duty is reported on FanDutyCyclePercentage (0x700), matching the C code's 2000 ms telemetry timer.
const FAN_TELEMETRY_PERIOD_MS: u64 = 2000;
const FAN_TELEMETRY_EVERY_N_UPDATES: u64 = FAN_TELEMETRY_PERIOD_MS / FAN_UPDATE_PERIOD_MS;

/// Latest fan duty (percent) requested by Calypso over CAN (0x500).
/// Starts at 0, so with no request the thermal table alone decides.
static CALYPSO_REQUEST_PERCENT: AtomicU8 = AtomicU8::new(0);

/// Called from the CAN rx processor when a ShepherdBmsFanPercent frame arrives.
pub fn store_calypso_request(percent: u8) {
    CALYPSO_REQUEST_PERCENT.store(percent.min(100), Ordering::Relaxed);
}

/// Latest duty requested by Calypso, in percent.
pub fn calypso_request_percent() -> u8 {
    CALYPSO_REQUEST_PERCENT.load(Ordering::Relaxed)
}

/// TODO: set to the value of CONTROL_MIN_DUTY_FAN from the C code.
const MIN_DUTY_PERCENT: u8 = 20;

/// Map the pack's highest cell temperature (deg C) to a fan duty (percent).
/// Thresholds match the C code: <=30 -> min, <=50 -> 50, <=60 -> 75, else 100.
/// A NaN reading fails every comparison and falls through to 100, so a bad temperature reading gives full cooling.
pub fn thermal_duty_percent(max_temp_c: f32) -> u8 {
    if max_temp_c <= 30.0 {
        MIN_DUTY_PERCENT
    } else if max_temp_c <= 50.0 {
        50
    } else if max_temp_c <= 60.0 {
        75
    } else {
        100
    }
}

/// Final fan duty (percent): the larger of the thermal-table duty and the latest duty Calypso requested over CAN (ShepherdBmsFanPercent, 0x500)
/// Calypso can only ask for more cooling, never less.
pub fn fan_duty_percent(max_temp_c: f32) -> u8 {
    thermal_duty_percent(max_temp_c).max(calypso_request_percent())
}

/// Pack's highest cell temp in deg C, from the analyzer's `max_temp`.
/// Returns NaN (which `thermal_duty_percent` treats as full cooling) when there is no usable reading
fn max_cell_temp_c() -> f32 {
    let Some(holder) = pack::analyzer::analyzer().try_get() else {
        return f32::NAN;
    };

    let temp = holder.data.max_temp.value().get::<degree_celsius>();
    if temp <= f32::MIN { f32::NAN } else { temp }
}

/// Drives the battery box fans (PWM0, TIM3_CH2).
#[embassy_executor::task]
pub async fn fan_task(r: FanResources) -> ! {
    // Open-drain, not push-pull: the fan has an internal pull-up on its control wire, so we only ever pull the line low and let the fan make the high.
    let pin = PwmPin::new(r.fan_pin, OutputType::OpenDrain);
    let mut pwm = SimplePwm::new(r.fan_timer, None, Some(pin), None, None, Hertz(FAN_PWM_FREQ_HZ), CountingMode::EdgeAlignedUp);

    let mut ch = pwm.channel(Channel::Ch2);
    // Nothing comes out of the pin until the channel is enabled.
    ch.enable();

    let mut updates: u64 = 0;
    loop {
        let duty = fan_duty_percent(max_cell_temp_c());
        ch.set_duty_cycle_percent(duty);

        updates += 1;
        if updates % FAN_TELEMETRY_EVERY_N_UPDATES == 0 {
            // Reports the final duty (after the max with Calypso's request) as a 0-100 percent
            can::send(FanDutyCyclePercentage { duty_cycle_percentage: duty }.as_frame()).await;
        }

        Timer::after_millis(FAN_UPDATE_PERIOD_MS).await;
    }
}
