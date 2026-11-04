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
//!
//! TODO: the pinned definitions have no messages for strain gauges 3-4, wheel speed or steering
//! angle. Add those once the definitions include them.
//!
//! Incoming frames are filtered to the application boot request ID.

use crate::bind_interrupts::Irqs;
use crate::shock_pot::{NUM_SHOCK_POTS, SHOCK_POT_DATA, ShockPot, ShockPotReading};
use crate::strain_guage::{NUM_STRAIN_GAUGES, STRAIN_GAUGE_DATA, StrainGauge, StrainGaugeReading};
use can_handler::{NerCan, StdFilter};
use cangen::{CanMessage, Msg, ToCanFrame};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use embassy_stm32::can::filter::StandardFilterSlot;
use embassy_stm32::can::{CanConfigurator, Frame};
use embassy_stm32::peripherals::{FDCAN2, PB5, PB6};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

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

    loop {
        // Whichever sensor published first; the other is picked up on the next pass.
        match select(shock_pot_rx.changed(), strain_gauge_rx.changed()).await {
            Either::First(readings) => {
                for frame in shock_pot_frames(&readings) {
                    send(frame).await;
                }
            }
            Either::Second(readings) => send(strain_gauge_frame(&readings)).await,
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

#[embassy_executor::task]
async fn can_inbox_task(mut bootloader: boot_control::Bootloader<'static>) -> ! {
    loop {
        let frame = INCOMING.receive().await;
        match Msg::decode(&frame) {
            Some(Msg::MsbFrontBootRequest(_)) => bootloader
                .handle_request(frame.data())
                .unwrap_or_else(|_| defmt::error!("Failed to persist bootloader request")),
            _ => {}
        }
    }
}
