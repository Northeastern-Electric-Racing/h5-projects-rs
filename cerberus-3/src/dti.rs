use core::cell::RefCell;
use core::f32::consts::PI;

use cangen::ToCanFrame;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use uom::ConstZero;
use uom::si::angular_velocity::revolution_per_minute;
use uom::si::electric_current::ampere;
use uom::si::f32::{AngularVelocity, Velocity};
use uom::si::f32::{ElectricCurrent, Length, ThermodynamicTemperature as Temperature};
use uom::si::length::{inch, meter};
use uom::si::thermodynamic_temperature::degree_celsius;
use uom::si::velocity::{meter_per_second, mile_per_hour};

use crate::can;

pub const SAMPLES: usize = 3;
pub const POLE_PAIRS: u32 = 10;
pub const GEAR_RATIO: f32 = 39.0 / 13.0;
pub const KT: f32 = 0.61;
pub const TIRE_DIAMETER_INCHES: f32 = 16.0;

pub struct DtiState {
    rpm: AngularVelocity,
    ac_current: ElectricCurrent,
    dc_current: ElectricCurrent,
    controller_temp: Temperature,
    motor_temp: Temperature,
}
impl DtiState {
    pub const fn new() -> Self {
        Self {
            rpm: AngularVelocity::ZERO,
            ac_current: ElectricCurrent::ZERO,
            dc_current: ElectricCurrent::ZERO,
            controller_temp: Temperature::ZERO,
            motor_temp: Temperature::ZERO,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FilterState {
    pub torque_buf: [f32; SAMPLES],
    pub torque_idx: usize,
    pub regen_buf: [f32; SAMPLES],
    pub regen_idx: usize,
}

impl FilterState {
    pub const fn new() -> Self {
        Self {
            torque_buf: [0.0; SAMPLES],
            torque_idx: 0,
            regen_buf: [0.0; SAMPLES],
            regen_idx: 0,
        }
    }

    /// Store a torque sample and return the average of the buffer.
    pub fn push_torque(&mut self, sample: f32) -> f32 {
        self.torque_buf[self.torque_idx] = sample;
        self.torque_idx = (self.torque_idx + 1) % SAMPLES;
        self.torque_buf.iter().sum::<f32>() / SAMPLES as f32
    }

    /// Store a regen sample and return the average of the buffer.
    pub fn push_regen(&mut self, sample: f32) -> f32 {
        self.regen_buf[self.regen_idx] = sample;
        self.regen_idx = (self.regen_idx + 1) % SAMPLES;
        self.regen_buf.iter().sum::<f32>() / SAMPLES as f32
    }
}

/// Shared filter state, initialised to zero.
pub static FILTER_STATE: Mutex<ThreadModeRawMutex, RefCell<FilterState>> =
    Mutex::new(RefCell::new(FilterState::new()));

/// Shared DTI state, initialised to zero.
pub static DTI_STATE: Mutex<ThreadModeRawMutex, RefCell<DtiState>> =
    Mutex::new(RefCell::new(DtiState::new()));

pub async fn set_torque(torque: f32) {
    let mut average = FILTER_STATE.lock(|t| t.borrow_mut().push_torque(torque));

    if torque == 0.0 {
        average = 0.0;
    }

    let ac_current = average / KT * 1.414; //1.414 is the sqrt of 2

    let de =
        cangen::DriveEnableCommand::new().with_drive_enable(!cfg!(feature = "tsms_overide") as u8);
    let ac = cangen::AcCurrentCommand::new().with_current_target_ac(ac_current);

    can::OUTGOING.send(de.to_can_frame()).await;
    can::OUTGOING.send(ac.to_can_frame()).await;
}

pub async fn set_regen(regen: f32) {
    let average = FILTER_STATE.lock(|t| t.borrow_mut().push_regen(regen));

    let de =
        cangen::DriveEnableCommand::new().with_drive_enable(!cfg!(feature = "tsms_overide") as u8);
    let ac = cangen::BrakeCurrentCommand::new().with_brake_ac_current(average);

    can::OUTGOING.send(de.to_can_frame()).await;
    can::OUTGOING.send(ac.to_can_frame()).await;
}

pub fn get_rpm() -> AngularVelocity {
    DTI_STATE.lock(|t| t.borrow().rpm)
}

pub fn get_ac_current() -> ElectricCurrent {
    DTI_STATE.lock(|t| t.borrow().ac_current)
}

pub fn get_controller_temp() -> Temperature {
    DTI_STATE.lock(|t| t.borrow().controller_temp)
}

pub fn get_motor_temp() -> Temperature {
    DTI_STATE.lock(|t| t.borrow().motor_temp)
}

pub fn get_velocity() -> Velocity {
    let tire_diameter = Length::new::<inch>(TIRE_DIAMETER_INCHES);

    // wheel revolutions per minute = motor rpm / gear ratio
    let wheel_rpm = get_rpm().get::<revolution_per_minute>() / GEAR_RATIO;

    // each wheel revolution travels one circumference (π × diameter)
    let circumference_m = tire_diameter.get::<meter>() * PI;

    Velocity::new::<meter_per_second>(circumference_m * wheel_rpm / 60.0)
}

pub fn get_mph() -> f32 {
    get_velocity().get::<mile_per_hour>()
}

pub fn set_rpm(rpm: AngularVelocity) {
    DTI_STATE.lock(|t| t.borrow_mut().rpm = rpm)
}

pub fn set_rpm_from_can(raw: f32) {
    let rpm = AngularVelocity::new::<revolution_per_minute>(raw);
    DTI_STATE.lock(|t| t.borrow_mut().rpm = rpm)
}

pub fn set_ac_current(ac_current: ElectricCurrent) {
    DTI_STATE.lock(|t| t.borrow_mut().ac_current = ac_current)
}

pub fn set_ac_current_from_can(raw: f32) {
    let ac_current = ElectricCurrent::new::<ampere>(raw);
    set_ac_current(ac_current);
}

pub fn set_dc_current(dc_current: ElectricCurrent) {
    DTI_STATE.lock(|t| t.borrow_mut().dc_current = dc_current)
}

pub fn set_dc_current_from_can(raw: f32) {
    let dc_current = ElectricCurrent::new::<ampere>(raw);
    set_dc_current(dc_current);
}

pub fn set_controller_temp(controller_temp: Temperature) {
    DTI_STATE.lock(|t| t.borrow_mut().controller_temp = controller_temp)
}

pub fn set_controller_temp_from_can(raw: f32) {
    let controller_temp = Temperature::new::<degree_celsius>(raw);
    set_controller_temp(controller_temp);
}

pub fn set_motor_temp(motor_temp: Temperature) {
    DTI_STATE.lock(|t| t.borrow_mut().motor_temp = motor_temp)
}

pub fn set_motor_temp_from_can(raw: f32) {
    let motor_temp = Temperature::new::<degree_celsius>(raw);
    set_motor_temp(motor_temp);
}
