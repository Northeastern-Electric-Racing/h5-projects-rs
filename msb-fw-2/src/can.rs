//! CAN: sensor telemetry over FDCAN2.
//!
//! The bus itself is handled by the shared `can-handler` crate from firmware-rs: [`NerCan`] sets up
//! FDCAN2 with the team's standard config (classic CAN, 500 kbit/s, reject-all filter), and
//! [`can_handler::can_tx`] drains [`OUTGOING`] onto the bus. This module adds the tasks that encode
//! and queue each sensor's message as soon as that sensor publishes new readings, so CAN updates at
//! each sensor's own rate. Every sensor has its own small task.
//!
//! Message layouts and IDs come from the team's Odyssey-Definitions (`msb.json`) through `cangen`,
//! at the `cangen` rev pinned in the workspace `Cargo.toml`. There is one MSB on the car, so it uses
//! the front message set:
//!
//! | ID    | Message                     | Sent from                                      |
//! |-------|-----------------------------|------------------------------------------------|
//! | 0x606 | Front Left Shockpot         | left shock pot: travel (in) + raw ADC count    |
//! | 0x626 | Front Right Shockpot        | right shock pot: travel (in) + raw ADC count   |
//! | 0x605 | Front MSB Strain One Two    | strain gauges 1, 2 (raw ADC counts)            |
//! | 0x612 | Front MSB Strain Three Four | strain gauges 3, 4 (raw ADC counts)            |
//! | 0x631 | Front Wheel RPM             | left/right wheel speed (rpm)                   |
//! | 0x632 | Steering Angle              | steering angle (deg, positive is right)        |
//! | 0x630 | Load Cell CAN Message       | load cells 1, 2 (force)                        |
//! | 0x633 | Thermocouple                | thermocouple (°C)                              |
//! | 0x603 | Front MSB Accel             | IMU acceleration (mg)                          |
//! | 0x604 | Front MSB Gyro              | IMU angular rate (mdps)                        |
//! | 0x602 | Front MSB Env               | HDC2021 temperature (°C) + humidity (%)        |
//! | 0x611 | Front MSB Mag               | magnetometer (mgauss)                          |
//!
//! The MSB only transmits: the global filter rejects every incoming frame.

use crate::bind_interrupts::Irqs;
//use crate::hdc2021::HDC2021_DATA;
use crate::imu::{IMU_DATA, ImuReading};
use crate::load_cell::{LOAD_CELL_DATA, LoadCell, LoadCellReading, NUM_LOAD_CELLS};
use crate::magnetometer::MAG_DATA;
use crate::shock_pot::{NUM_SHOCK_POTS, SHOCK_POT_DATA, ShockPot, ShockPotReading};
use crate::strain_gauge::{NUM_STRAIN_GAUGES, STRAIN_GAUGE_DATA, StrainGauge, StrainGaugeReading};
use crate::thermocouple::{NUM_THERMOCOUPLES, THERMOCOUPLE_DATA, ThermocoupleReading};
use crate::wheel_speed::{NUM_WHEELS, WHEEL_SPEED_DATA, Wheel, WheelSpeedReading};
use crate::steering_angle::{NUM_STEERING_ANGLES, STEERING_ANGLE_DATA, SteeringAngle, SteeringAngleReading};
use can_handler::{NerCan, StdFilter};
use cangen::{CanMessage, Msg, ToCanFrame};
use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::can::filter::StandardFilterSlot;
use embassy_stm32::can::{CanConfigurator, Frame};
use embassy_stm32::peripherals::{FDCAN2, PB5, PB6};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use hdc2021_ner::Measurement;
use lis2mdl_ner::MagneticField;

/// Frames waiting for a free FDCAN TX buffer.
const OUTGOING_CAPACITY: usize = 32;

static INCOMING: Channel<ThreadModeRawMutex, Frame, 32> = Channel::new();

static OUTGOING: Channel<ThreadModeRawMutex, Frame, OUTGOING_CAPACITY> = Channel::new();

/// Peripherals used by CAN.
pub struct CanResources {
    pub can: Peri<'static, FDCAN2>,
    pub rx: Peri<'static, PB5>,
    pub tx: Peri<'static, PB6>,
}

/// Starts FDCAN2 through [`NerCan`] and spawns the CAN tasks.
pub fn start(spawner: Spawner, r: CanResources, bootloader: boot_control::Bootloader<'static>) {
    const SLOT_0: StdFilter = StdFilter {
        slot: StandardFilterSlot::_0,
        id1: cangen::MsbFrontBootRequest::ID,
        id2: None,
    };
    let (tx, rx, _properties) = NerCan::init(CanConfigurator::new(r.can, r.rx, r.tx, Irqs))
        .with_standard_filters(&[SLOT_0])
        .start();
    spawner.spawn(
        can_handler::can_rx(rx, INCOMING.dyn_sender()).expect("Failed to spawn CAN receiver"),
    );
    spawner.spawn(can_inbox_task(bootloader).expect("Failed to spawn CAN processor"));

    spawner.spawn(
        can_handler::can_tx(tx, OUTGOING.dyn_receiver(), OUTGOING.dyn_sender())
            .expect("Failed to spawn can_handler::can_tx()."),
    );
    spawner.spawn(shockpot_can_task().expect("Failed to spawn can::shockpot_can_task()."));
    spawner.spawn(strain_gauge_can_task().expect("Failed to spawn can::strain_gauge_can_task()."));
    spawner.spawn(wheel_speed_can_task().expect("Failed to spawn can::wheel_speed_can_task()."));
    spawner.spawn(steering_angle_can_task().expect("Failed to spawn can::steering_angle_can_task()."));
    spawner.spawn(load_cell_can_task().expect("Failed to spawn can::load_cell_can_task()."));
    spawner.spawn(thermocouple_can_task().expect("Failed to spawn can::thermocouple_can_task()."));
    spawner.spawn(imu_can_task().expect("Failed to spawn can::imu_can_task()."));
    //spawner.spawn(hdc2021_can_task().expect("Failed to spawn can::hdc2021_can_task()."));
    spawner.spawn(magnetometer_can_task().expect("Failed to spawn can::magnetometer_can_task()."));
}

/// Queues a frame to send, waiting if [`OUTGOING`] is full.
pub async fn send(frame: Frame) {
    can_handler::send(OUTGOING.dyn_sender(), frame).await
}


/// All strain gauges, replacing telemetry_task by putting strain gauges 1-2 within the same function as 3-4
/// This is `STRAIN_GAUGE_DATA`'s sole receiver which fits its limit of 2. (leaves one open)
#[embassy_executor::task]
async fn strain_gauge_can_task() {
    let mut rx = STRAIN_GAUGE_DATA
        .receiver()
        .expect("Too many STRAIN_GAUGE_DATA receivers.");
    loop {
        let readings = rx.changed().await;
         send(strain_one_two_frame(&readings)).await;
         send(strain_three_four_frame(&readings)).await;
    }
}

// Shockpot can task, replacing telemetry_task by separating shockpots from strain gauges 1-2
#[embassy_executor::task]
async fn shockpot_can_task() {
    let mut rx = SHOCK_POT_DATA
        .receiver()
        .expect("Too many SHOCK_POT_DATA receivers.");
    loop {
        let readings = rx.changed().await;
        for frame in shock_pot_frames(&readings) {
            send(frame).await;
        }
    }
}


#[embassy_executor::task]
async fn wheel_speed_can_task() {
    let mut rx = WHEEL_SPEED_DATA
        .receiver()
        .expect("Too many WHEEL_SPEED_DATA receivers.");
    loop {
        let readings = rx.changed().await;
        send(wheel_speed_frame(&readings)).await;
    }
}

#[embassy_executor::task]
async fn steering_angle_can_task() {
    let mut rx = STEERING_ANGLE_DATA
        .receiver()
        .expect("Too many STEERING_ANGLE_DATA receivers.");
    loop {
        let readings = rx.changed().await;
        send(steering_angle_frame(&readings)).await;
    }
}

#[embassy_executor::task]
async fn load_cell_can_task() {
    let mut rx = LOAD_CELL_DATA
        .receiver()
        .expect("Too many LOAD_CELL_DATA receivers.");
    loop {
        let readings = rx.changed().await;
        send(load_cell_frame(&readings)).await;
    }
}


#[embassy_executor::task]
async fn thermocouple_can_task() {
    let mut rx = THERMOCOUPLE_DATA
        .receiver()
        .expect("Too many THERMOCOUPLE_DATA receivers.");
    loop {
        let readings = rx.changed().await;
        send(thermocouple_frame(&readings)).await;
    }
}

#[embassy_executor::task]
async fn imu_can_task() {
    let mut rx = IMU_DATA.receiver().expect("Too many IMU_DATA receivers.");
    loop {
        let reading = rx.changed().await;
        for frame in imu_frames(&reading) {
            send(frame).await;
        }
    }
}

//HDC commented out for time being
//#[embassy_executor::task]
//async fn hdc2021_can_task() {
//  let mut rx = HDC2021_DATA
//        .receiver()
//      .expect("Too many HDC2021_DATA receivers.");
//    loop {
//        let measurement = rx.changed().await;
//        send(hdc2021_frame(&measurement)).await;
//    }
//}

#[embassy_executor::task]
async fn magnetometer_can_task() {
    let mut rx = MAG_DATA.receiver().expect("Too many MAG_DATA receivers.");
    loop {
        let field = rx.changed().await;
        send(magnetometer_frame(&field)).await;
    }
}

/// `Front Left Shockpot` (0x606) and `Front Right Shockpot` (0x626): each pot's travel in inches
/// plus its raw ADC count. `cangen` scales the travel (x1000) and saturates it at 0, so negative
/// travel (from the trim) goes out as 0.
fn shock_pot_frames(readings: &[ShockPotReading; NUM_SHOCK_POTS]) -> [Frame; 2] {
    let left = readings[ShockPot::Left as usize];
    let right = readings[ShockPot::Right as usize];
    [
        cangen::FrontLeftShockpot::new()
            .with_shock_left(left.inch_travel)
            .with_shock_left_raw(u32::from(left.raw))
            .to_can_frame(),
        cangen::FrontRightShockpot::new()
            .with_shock_right(right.inch_travel)
            .with_shock_right_raw(u32::from(right.raw))
            .to_can_frame(),
    ]
}

/// `Front MSB Strain One Two` (0x605): strain gauges 1 and 2.
///
/// The fields are unscaled `u32`s and the gauges aren't calibrated yet, so this sends the raw ADC
/// counts. TODO: switch to calibrated values once the gauges and the definition's units are set.
fn strain_one_two_frame(readings: &[StrainGaugeReading; NUM_STRAIN_GAUGES]) -> Frame {
    let strain1 = u32::from(readings[StrainGauge::Sg1 as usize].raw);
    let strain2 = u32::from(readings[StrainGauge::Sg2 as usize].raw);
    cangen::FrontMsbStrainOneTwo::new()
        .with_strain1(strain1)
        .with_strain2(strain2)
        .to_can_frame()
}

/// `Front MSB Strain Three Four` (0x612): strain gauges 3 and 4, raw ADC counts (see
/// [`strain_gauge_frame`]).
fn strain_three_four_frame(readings: &[StrainGaugeReading; NUM_STRAIN_GAUGES]) -> Frame {
    cangen::FrontMsbStrainThreeFour::new()
        .with_strain3(u32::from(readings[StrainGauge::Sg3 as usize].raw))
        .with_strain4(u32::from(readings[StrainGauge::Sg4 as usize].raw))
        .to_can_frame()
}

/// `Front Wheel RPM` (0x631): left and right wheel speed in rpm, as `u16`s.
/// `f32 as u16` saturates (negative or NaN becomes 0, anything above 65535 becomes 65535).
fn wheel_speed_frame(readings: &[WheelSpeedReading; NUM_WHEELS]) -> Frame {
    let left = readings[Wheel::Left as usize].rpm;
    let right = readings[Wheel::Right as usize].rpm;
    cangen::FrontWheelRpm::new()
        .with_left_rpm(left as u16)
        .with_right_rpm(right as u16)
        .to_can_frame()
}

/// `Steering Angle` (0x632): degrees, positive is right. `cangen` scales it x10 into an `i32`.
///
/// Sent even when the sensor reads out of range; the angle is then clamped to full lock, so check
/// the MSB's log for the out-of-range warning before trusting it.
fn steering_angle_frame(readings: &[SteeringAngleReading; NUM_STEERING_ANGLES]) -> Frame {
    cangen::SteeringAngle::new()
        .with_steering_angle_1(readings[0].angle_deg)
        .to_can_frame()
}

/// `Load Cell CAN Message` (0x630): load cells 1 and 2. `cangen` scales each x100 into a `u16`, so
/// the representable range is 0 to 655.35; values outside it saturate.
///
/// TODO: `force` equals the cell's volts until the cells are calibrated (see `load_cell.rs`).
fn load_cell_frame(readings: &[LoadCellReading; NUM_LOAD_CELLS]) -> Frame {
    cangen::LoadCellCanMessage::new()
        .with_load_cell_1(readings[LoadCell::Lc1 as usize].force)
        .with_load_cell_2(readings[LoadCell::Lc2 as usize].force)
        .to_can_frame()
}

/// `Thermocouple` (0x633): temperature in °C. `cangen` scales it x100 into an `i32`.
fn thermocouple_frame(readings: &[ThermocoupleReading; NUM_THERMOCOUPLES]) -> Frame {
    cangen::Thermocouple::new()
        .with_thermocouple1(readings[0].temp_c)
        .to_can_frame()
}

/// `Front MSB Accel` (0x603, mg) and `Front MSB Gyro` (0x604, mdps): unscaled signed 16-bit
/// fields, which `cangen` saturates. In mdps an `i16` only reaches about ±32.7 °/s, so the gyro
/// will clip during real driving.
fn imu_frames(r: &ImuReading) -> [Frame; 2] {
    [
        cangen::FrontMsbAccel::new()
            .with_x_force(r.accel_mg.x)
            .with_y_force(r.accel_mg.y)
            .with_z_force(r.accel_mg.z)
            .to_can_frame(),
        cangen::FrontMsbGyro::new()
            .with_x_deg(r.gyro_mdps.x)
            .with_y_deg(r.gyro_mdps.y)
            .with_z_deg(r.gyro_mdps.z)
            .to_can_frame(),
    ]
}

/// `Front MSB Env` (0x602): temperature (°C) and relative humidity (%), each scaled x10 into an
/// unsigned 16-bit field, so temperatures below 0 °C go out as 0.
//fn hdc2021_frame(m: &Measurement) -> Frame {
//    cangen::FrontMsbEnv::new()
//        .with_temp(m.temperature)
//        .with_humidity(m.humidity)
//        .to_can_frame()
//}

/// `Front MSB Mag` (0x611): magnetic field on each axis in milligauss, as unscaled signed 16-bit
/// fields. `cangen` saturates anything outside ±32767 mG; Earth's field is roughly 250-650 mG.
fn magnetometer_frame(f: &MagneticField) -> Frame {
    cangen::FrontMsbMag::new()
        .with_x_mgauss(f.x_mgauss)
        .with_y_mgauss(f.y_mgauss)
        .with_z_mgauss(f.z_mgauss)
        .to_can_frame()
}

#[embassy_executor::task]
async fn can_inbox_task(mut bootloader: boot_control::Bootloader<'static>) -> ! {
    loop {
        let frame = INCOMING.receive().await;
        match Msg::decode(&frame) {
            Some(Msg::MsbFrontBootRequest(_)) => {
                if boot_control::is_valid_request(frame.data()) {
                    // Begin cleanup before reset: add application-specific shutdown here.

                    // End cleanup before reset.
                    bootloader.enter_bootloader();
                }
            }
            _ => {}
        }
    }
}