use core::sync::atomic::{AtomicBool, AtomicI16, Ordering};
use embassy_time::{Duration, Instant, Timer};
use uom::si::f32::ThermodynamicTemperature as Temperature;
use uom::si::thermodynamic_temperature::degree_celsius;

use crate::statemachine::{self, FuncState};

const MOTOR_TEMP_SENSOR_MAX: f32 = 125.0;
const MOTOR_TEMP_SENSOR_FAULT_TIMEOUT: Duration = Duration::from_secs(5);
const MOTOR_TEMP_SENSOR_PERIOD: Duration = Duration::from_millis(100);

static MOTOR_TEMP: AtomicI16 = AtomicI16::new(0);
static CONTROLLER_TEMP: AtomicI16 = AtomicI16::new(0);
static MOTOR_TEMP_SENSOR_FAULT: AtomicBool = AtomicBool::new(false);

pub fn get_motor_temp() -> Temperature {
    Temperature::new::<degree_celsius>(f32::from(MOTOR_TEMP.load(Ordering::Relaxed)))
}

pub fn get_controller_temp() -> Temperature {
    Temperature::new::<degree_celsius>(f32::from(CONTROLLER_TEMP.load(Ordering::Relaxed)))
}

pub fn motor_temp_sensor_faulted() -> bool {
    MOTOR_TEMP_SENSOR_FAULT.load(Ordering::Relaxed)
}

pub fn handle_temps_message(data: &[u8]) {
    if let [c1, c2, m1, m2, ..] = *data {
        CONTROLLER_TEMP.store(i16::from_be_bytes([c1, c2]) / 10, Ordering::Relaxed);
        MOTOR_TEMP.store(i16::from_be_bytes([m1, m2]) / 10, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn motor_temp_sensor_task() -> ! {
    let mut last_sensor_fault: Option<Instant> = None;

    loop {
        if get_motor_temp().get::<degree_celsius>() >= MOTOR_TEMP_SENSOR_MAX {
            MOTOR_TEMP_SENSOR_FAULT.store(true, Ordering::Relaxed);
            last_sensor_fault = Some(Instant::now());
            statemachine::set_fault_state().await;
        } else if let Some(last) = last_sensor_fault
            && last.elapsed() >= MOTOR_TEMP_SENSOR_FAULT_TIMEOUT
        {
            MOTOR_TEMP_SENSOR_FAULT.store(false, Ordering::Relaxed);
            last_sensor_fault = None;
            if statemachine::get_func_state().await == FuncState::Faulted {
                statemachine::set_ready_state().await;
            }
        }

        Timer::after(MOTOR_TEMP_SENSOR_PERIOD).await;
    }
}
