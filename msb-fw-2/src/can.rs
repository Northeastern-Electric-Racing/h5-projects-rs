//! CAN: the FDCAN2 driver and sensor telemetry.
//!
//! [`start`] configures FDCAN2 for classic CAN at 500 kbit/s (same settings as shepherd-3) and
//! spawns:
//! - [`can_tx_task`], which drains [`OUTGOING`] onto the bus;
//! - [`telemetry_task`], which encodes and queues a sensor's message as soon as that sensor
//!   publishes new readings, so CAN updates at the sensors' rate (50 Hz) without adding its own
//!   polling delay;
//! - [`health_task`], which logs bus state, error counters and frame counts every
//!   [`HEALTH_PERIOD`].
//!
//! Message layouts and IDs come from the team's Odyssey-Definitions (`msb.json`) through `cangen`,
//! at the `cangen` rev pinned in the workspace `Cargo.toml`. There is one MSB on the car, so it uses
//! the front message set:
//!
//! | ID    | Message          | Sent from                                      |
//! |-------|------------------|------------------------------------------------|
//! | 0x606 | Front Shockpot   | right shock pot: travel (in, f32) + raw ADC count |
//! | 0x605 | Front MSB Strain | strain gauges 1, 2 (raw ADC counts)            |
//!
//! TODO: the pinned definitions have one shock pot slot and no messages for the left pot, strain
//! gauges 3-4, wheel speed or steering angle. Add those once the definitions include them.
//!
//! The MSB only transmits: the global filter rejects every incoming frame.

use crate::bind_interrupts::Irqs;
use crate::shock_pot::{NUM_SHOCK_POTS, SHOCK_POT_DATA, ShockPot, ShockPotReading};
use crate::strain_guage::{NUM_STRAIN_GAUGES, STRAIN_GAUGE_DATA, StrainGauge, StrainGaugeReading};
use cangen::ToCanFrame;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_stm32::can::enums::BusErrorMode;
use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Frame, Properties};
use embassy_stm32::peripherals::{FDCAN2, PB5, PB6};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use embassy_time::{Duration, Ticker, Timer};

const BITRATE: u32 = 500_000;

/// Frames waiting for a free FDCAN TX buffer.
const OUTGOING_CAPACITY: usize = 32;

/// Longest a frame may wait for a free TX buffer before it's dropped, so [`can_tx_task`] can never
/// sleep forever. shepherd-3 saw the embassy TX waker stall; this is the same guard it uses.
const TX_TIMEOUT: Duration = Duration::from_millis(50);

/// How often [`health_task`] logs.
const HEALTH_PERIOD: Duration = Duration::from_secs(1);

static OUTGOING: Channel<ThreadModeRawMutex, Frame, OUTGOING_CAPACITY> = Channel::new();

/// Frame counters kept by [`can_tx_task`], reported by [`health_task`].
mod stats {
    use super::{AtomicU32, Ordering};

    /// Frames handed to the FDCAN TX buffers.
    pub static SENT: AtomicU32 = AtomicU32::new(0);
    /// Lower-priority frames bumped out of the TX buffers and put back on OUTGOING.
    pub static REQUEUED: AtomicU32 = AtomicU32::new(0);
    /// Frames dropped because OUTGOING was full when requeueing.
    pub static DROPPED_FULL: AtomicU32 = AtomicU32::new(0);
    /// Frames dropped because no TX buffer freed up within `TX_TIMEOUT`.
    pub static DROPPED_STALLED: AtomicU32 = AtomicU32::new(0);

    pub fn bump(counter: &AtomicU32) {
        counter.fetch_add(1, Ordering::Relaxed);
    }

    pub fn get(counter: &AtomicU32) -> u32 {
        counter.load(Ordering::Relaxed)
    }
}

/// Peripherals used by CAN.
pub struct CanResources {
    pub can: Peri<'static, FDCAN2>,
    pub rx: Peri<'static, PB5>,
    pub tx: Peri<'static, PB6>,
}

/// Configures FDCAN2 and spawns the CAN tasks.
pub fn start(spawner: Spawner, r: CanResources) {
    use embassy_stm32::can::config::{FdCanConfig, FrameTransmissionConfig, GlobalFilter};

    let mut configurator = CanConfigurator::new(r.can, r.rx, r.tx, Irqs);
    configurator.set_config(
        FdCanConfig::default()
            .set_automatic_bus_off_recovery(true)
            .set_automatic_retransmit(false)
            .set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly)
            .set_transmit_pause(true)
            .set_global_filter(GlobalFilter::reject_all()),
    );
    configurator.set_bitrate(BITRATE);

    let (tx, rx, properties) = configurator.into_normal_mode().split();

    defmt::info!("CAN: started at {=u32} bit/s.", BITRATE);
    spawner.spawn(can_tx_task(tx, rx).expect("Failed to spawn can::can_tx_task()."));
    spawner.spawn(telemetry_task().expect("Failed to spawn can::telemetry_task()."));
    spawner.spawn(health_task(properties).expect("Failed to spawn can::health_task()."));
}

/// Queues a frame to send, waiting if [`OUTGOING`] is full.
pub async fn send(frame: Frame) {
    if let Err(TrySendError::Full(frame)) = OUTGOING.try_send(frame) {
        defmt::warn!("CAN: OUTGOING is full; waiting for space. Consider raising OUTGOING_CAPACITY.");
        OUTGOING.send(frame).await;
    }
}

/// Drains [`OUTGOING`] into the FDCAN TX buffers.
///
/// `_rx` is held only to keep the receive half alive; nothing is received.
#[embassy_executor::task]
async fn can_tx_task(mut tx: CanTx<'static>, _rx: CanRx<'static>) {
    loop {
        let frame = OUTGOING.receive().await;
        match select(tx.write(&frame), Timer::after(TX_TIMEOUT)).await {
            // Sent (or queued in a TX buffer).
            Either::First(None) => stats::bump(&stats::SENT),
            // A lower-priority frame was bumped out of the TX buffers to make room; requeue it.
            Either::First(Some(bumped)) => {
                stats::bump(&stats::SENT);
                if OUTGOING.try_send(bumped).is_ok() {
                    stats::bump(&stats::REQUEUED);
                } else {
                    stats::bump(&stats::DROPPED_FULL);
                    defmt::warn!("CAN: dropped a frame; OUTGOING was full when requeueing it.");
                }
            }
            Either::Second(()) => {
                stats::bump(&stats::DROPPED_STALLED);
                defmt::warn!("CAN: TX stalled for {=u64} ms; dropped a frame.", TX_TIMEOUT.as_millis());
            }
        }
    }
}

/// Logs CAN health every [`HEALTH_PERIOD`]: bus state, error counters, frames sent and dropped in
/// the last period, and how full OUTGOING is.
///
/// Warns instead of info when the bus is error-passive or bus-off, or when frames were dropped.
/// A rising TX error count with zero RX errors usually means no other node is acknowledging,
/// e.g. the MSB is alone on the bus or the termination/wiring is wrong.
#[embassy_executor::task]
async fn health_task(properties: Properties) {
    let mut ticker = Ticker::every(HEALTH_PERIOD);
    let mut last_sent = 0u32;
    let mut last_dropped = 0u32;

    loop {
        ticker.next().await;

        let sent = stats::get(&stats::SENT);
        let dropped = stats::get(&stats::DROPPED_FULL) + stats::get(&stats::DROPPED_STALLED);
        let sent_per_period = sent.wrapping_sub(last_sent);
        let dropped_per_period = dropped.wrapping_sub(last_dropped);
        last_sent = sent;
        last_dropped = dropped;

        let mode = properties.bus_error_mode();
        let tec = properties.tx_error_count();
        let rec = properties.rx_error_count();
        let queued = OUTGOING.len();

        let (mode_name, bus_ok) = match mode {
            BusErrorMode::ErrorActive => ("ok (error-active)", true),
            BusErrorMode::ErrorPassive => ("ERROR-PASSIVE", false),
            BusErrorMode::BusOff => ("BUS-OFF", false),
        };
        let healthy = bus_ok && dropped_per_period == 0;

        if healthy {
            defmt::info!(
                "CAN health: {=str} | tx_err={=u8} rx_err={=u8} | sent={=u32}/s dropped={=u32} | queued={=usize}/{=usize}",
                mode_name, tec, rec, sent_per_period, dropped_per_period, queued, OUTGOING_CAPACITY
            );
        } else {
            defmt::warn!(
                "CAN health: {=str} | tx_err={=u8} rx_err={=u8} | sent={=u32}/s dropped={=u32} | queued={=usize}/{=usize} | total requeued={=u32}",
                mode_name, tec, rec, sent_per_period, dropped_per_period, queued, OUTGOING_CAPACITY,
                stats::get(&stats::REQUEUED)
            );
        }
    }
}

/// The shock pot sent in the one-pot `Front Shockpot` message.
const CAN_SHOCK_POT: ShockPot = ShockPot::Right;

/// Sends each sensor's message as soon as that sensor publishes new readings.
#[embassy_executor::task]
async fn telemetry_task() {
    let mut shock_pot_rx = SHOCK_POT_DATA.receiver().expect("Too many SHOCK_POT_DATA receivers.");
    let mut strain_gauge_rx = STRAIN_GAUGE_DATA.receiver().expect("Too many STRAIN_GAUGE_DATA receivers.");

    loop {
        // Whichever sensor published first; the other is picked up on the next pass.
        let frame = match select(shock_pot_rx.changed(), strain_gauge_rx.changed()).await {
            Either::First(readings) => shock_pot_frame(&readings),
            Either::Second(readings) => strain_gauge_frame(&readings),
        };
        send(frame).await;
    }
}

/// `Front Shockpot` (0x606): the right shock pot's travel in inches, plus its raw ADC count.
fn shock_pot_frame(readings: &[ShockPotReading; NUM_SHOCK_POTS]) -> Frame {
    let r = readings[CAN_SHOCK_POT as usize];
    cangen::FrontShockpot::new().with_shock1(r.inch_travel).with_shock1_raw(r.raw).to_can_frame()
}

/// `Front MSB Strain` (0x605): strain gauges 1 and 2.
///
/// The fields are unscaled `u32`s and the gauges aren't calibrated yet, so this sends the raw ADC
/// counts. TODO: switch to calibrated values once the gauges and the definition's units are set.
fn strain_gauge_frame(readings: &[StrainGaugeReading; NUM_STRAIN_GAUGES]) -> Frame {
    let strain1 = u32::from(readings[StrainGauge::Sg1 as usize].raw);
    let strain2 = u32::from(readings[StrainGauge::Sg2 as usize].raw);
    cangen::FrontMsbStrain::new().with_strain1(strain1).with_strain2(strain2).to_can_frame()
}
