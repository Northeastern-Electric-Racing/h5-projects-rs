//! CAN: sensor telemetry over FDCAN2.
//!
//! The bus itself is handled by the shared `can-handler` crate from firmware-rs: [`NerCan`] sets up
//! FDCAN2 with the team's standard config (classic CAN, 500 kbit/s, reject-all filter), and
//! [`can_handler::can_tx`] drains [`OUTGOING`] onto the bus. This module only adds
//! [`telemetry_task`], which encodes and queues a sensor's message as soon as that sensor publishes
//! new readings, so CAN updates at the sensors' rate (50 Hz).
//!
//! Message layouts and IDs come from the team's Odyssey-Definitions (`msb.json`) through `cangen`,
//! at the `cangen` rev pinned in the workspace `Cargo.toml`. There is one MSB on the car, so it uses
//! the front message set:
//!
//! | ID    | Message              | Sent from                                      |
//! |-------|----------------------|------------------------------------------------|
//! | 0x606 | Front Left Shockpot  | left shock pot: travel (in) + raw ADC count    |
//! | 0x626 | Front Right Shockpot | right shock pot: travel (in) + raw ADC count   |
//! | 0x605 | Front MSB Strain     | strain gauges 1, 2 (raw ADC counts)            |
//! | 0x632 | Steering Angle       | steering angle (deg x 10), built by hand       |
//!
//! 0x632 only exists on Odyssey-Definitions' `msb-updates` branch, not in the pinned definitions,
//! so [`steering_angle_frame`] builds it by hand to the same layout. TODO: switch it to `cangen`
//! once the message is merged into Odyssey-Definitions `main` and `cangen` is bumped.
//!
//! TODO: the pinned definitions have no messages for strain gauges 3-4 or wheel speed. Add those
//! once the definitions include them.
//!
//! The MSB only transmits: the global filter rejects every incoming frame.

use crate::bind_interrupts::Irqs;
use crate::shock_pot::{NUM_SHOCK_POTS, SHOCK_POT_DATA, ShockPot, ShockPotReading};
use crate::steering_angle::{NUM_STEERING_ANGLES, STEERING_ANGLE_DATA, SteeringAngleReading};
use crate::strain_guage::{NUM_STRAIN_GAUGES, STRAIN_GAUGE_DATA, StrainGauge, StrainGaugeReading};
use can_handler::NerCan;
use cangen::ToCanFrame;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::Peri;
use embassy_stm32::can::{CanConfigurator, Frame};
use embassy_stm32::peripherals::{FDCAN2, PB5, PB6};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

/// `Steering Angle` ID, from Odyssey-Definitions' `msb-updates` branch.
const STEERING_ANGLE_ID: u16 = 0x632;

/// Frames waiting for a free FDCAN TX buffer.
const OUTGOING_CAPACITY: usize = 32;

static OUTGOING: Channel<ThreadModeRawMutex, Frame, OUTGOING_CAPACITY> = Channel::new();

/// Peripherals used by CAN.
pub struct CanResources {
    pub can: Peri<'static, FDCAN2>,
    pub rx: Peri<'static, PB5>,
    pub tx: Peri<'static, PB6>,
}

/// Starts FDCAN2 through [`NerCan`] and spawns the CAN tasks.
pub fn start(spawner: Spawner, r: CanResources) {
    let (tx, rx, _properties) = NerCan::init(CanConfigurator::new(r.can, r.rx, r.tx, Irqs)).start();
    // Nothing is received (reject-all filter), but the RX half must stay alive: dropping both
    // halves shuts the peripheral down.
    core::mem::forget(rx);

    spawner.spawn(
        can_handler::can_tx(tx, OUTGOING.dyn_receiver(), OUTGOING.dyn_sender())
            .expect("Failed to spawn can_handler::can_tx()."),
    );
    spawner.spawn(telemetry_task().expect("Failed to spawn can::telemetry_task()."));
}

/// Queues a frame to send, waiting if [`OUTGOING`] is full.
pub async fn send(frame: Frame) {
    can_handler::send(OUTGOING.dyn_sender(), frame).await
}

/// Sends each sensor's message as soon as that sensor publishes new readings.
#[embassy_executor::task]
async fn telemetry_task() {
    let mut shock_pot_rx = SHOCK_POT_DATA
        .receiver()
        .expect("Too many SHOCK_POT_DATA receivers.");
    let mut strain_gauge_rx = STRAIN_GAUGE_DATA
        .receiver()
        .expect("Too many STRAIN_GAUGE_DATA receivers.");
    let mut steering_angle_rx = STEERING_ANGLE_DATA
        .receiver()
        .expect("Too many STEERING_ANGLE_DATA receivers.");

    loop {
        // Whichever sensor published first; the others are picked up on the next pass.
        match select3(
            shock_pot_rx.changed(),
            strain_gauge_rx.changed(),
            steering_angle_rx.changed(),
        )
        .await
        {
            Either3::First(readings) => {
                for frame in shock_pot_frames(&readings) {
                    send(frame).await;
                }
            }
            Either3::Second(readings) => send(strain_gauge_frame(&readings)).await,
            Either3::Third(readings) => send(steering_angle_frame(&readings)).await,
        }
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

/// `Front MSB Strain` (0x605): strain gauges 1 and 2.
///
/// The fields are unscaled `u32`s and the gauges aren't calibrated yet, so this sends the raw ADC
/// counts. TODO: switch to calibrated values once the gauges and the definition's units are set.
fn strain_gauge_frame(readings: &[StrainGaugeReading; NUM_STRAIN_GAUGES]) -> Frame {
    let strain1 = u32::from(readings[StrainGauge::Sg1 as usize].raw);
    let strain2 = u32::from(readings[StrainGauge::Sg2 as usize].raw);
    cangen::FrontMsbStrain::new()
        .with_strain1(strain1)
        .with_strain2(strain2)
        .to_can_frame()
}

/// `Steering Angle` (0x632): one signed 32-bit big-endian value, degrees x 10 (positive is right),
/// matching the `msb-updates` definition (`steering_angle_1`, int32, divide by 10).
///
/// Sent even when the sensor reads out of range; the angle is then clamped to full lock, so check
/// the MSB's log for the out-of-range warning before trusting it.
fn steering_angle_frame(readings: &[SteeringAngleReading; NUM_STEERING_ANGLES]) -> Frame {
    // Round to the nearest tenth (`f32::round` needs std).
    let scaled = readings[0].angle_deg * 10.0_f32;
    let tenths = if scaled >= 0.0_f32 {
        scaled + 0.5_f32
    } else {
        scaled - 0.5_f32
    } as i32;
    Frame::new_standard(STEERING_ANGLE_ID, &tenths.to_be_bytes())
        .expect("0x632 with 4 data bytes is a valid standard frame.")
}
