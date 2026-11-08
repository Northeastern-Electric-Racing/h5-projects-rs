use core::sync::atomic::{AtomicI16, Ordering};
use uom::si::f32::ThermodynamicTemperature as Temperature;
use uom::si::thermodynamic_temperature::degree_celsius;

static MOTOR_TEMP: AtomicI16 = AtomicI16::new(0);
static CONTROLLER_TEMP: AtomicI16 = AtomicI16::new(0);

pub fn get_motor_temp() -> Temperature {
    Temperature::new::<degree_celsius>(f32::from(MOTOR_TEMP.load(Ordering::Relaxed)))
}

pub fn get_controller_temp() -> Temperature {
    Temperature::new::<degree_celsius>(f32::from(CONTROLLER_TEMP.load(Ordering::Relaxed)))
}

pub fn handle_temps_message(data: &[u8]) {
    if let [c1, c2, m1, m2, ..] = *data {
        CONTROLLER_TEMP.store(i16::from_be_bytes([c1, c2]) / 10, Ordering::Relaxed);
        MOTOR_TEMP.store(i16::from_be_bytes([m1, m2]) / 10, Ordering::Relaxed);
    }
}
