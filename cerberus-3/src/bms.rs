use core::sync::atomic::{AtomicBool, AtomicI16, Ordering};
use defmt::debug;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use num_traits::Float;
use uom::si::f32::ThermodynamicTemperature as Temperature;
use uom::si::thermodynamic_temperature::degree_celsius;

#[derive(Clone, Copy, defmt::Format)]
pub(crate) enum PrechargeState {
    Open,
    Floating,
    Closed,
}

impl TryFrom<u8> for PrechargeState {
    type Error = u8; // the unrecognized byte

    /// decode the raw precharge state byte from CAN
    fn try_from(byte: u8) -> Result<Self, Self::Error> {
        match byte {
            0 => Ok(PrechargeState::Open),
            1 => Ok(PrechargeState::Floating),
            2 => Ok(PrechargeState::Closed),
            other => Err(other),
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
enum Fault {
    PrechargeFloating,
    BmsCanMonitor,
}

// TODO once faults have been implemented
fn queue_fault_placeholder(fault: Fault) {
    debug!(
        "Fault queued (placeholder, faults not ported yet): {}",
        fault
    );
}

/// how long the BMS can go without a DCL message before BmsCanMonitor is queued
const BMS_CAN_TIMEOUT: Duration = Duration::from_secs(4);

/// temperature in centidegrees
static BATTERY_BOX_TEMP_CENTIDEGREES: AtomicI16 = AtomicI16::new(0);

/// default state is false (open/floating)
static PRECHARGE_CLOSED: AtomicBool = AtomicBool::new(false);

///gets the battery box temp as a Temperature value
pub fn get_battery_box_temp() -> Temperature {
    let centidegrees = BATTERY_BOX_TEMP_CENTIDEGREES.load(Ordering::Relaxed);
    Temperature::new::<degree_celsius>(f32::from(centidegrees) / 100.0) // convert from centidegrees to degrees
}

///sets the battery box temp from a Temperature value
pub fn set_battery_box_temp(temp: Temperature) {
    let scaled = temp.get::<degree_celsius>() * 100.0; // convert from degrees to centidegrees
    BATTERY_BOX_TEMP_CENTIDEGREES.store(scaled.round() as i16, Ordering::Relaxed);
}

pub fn get_precharge_state() -> bool {
    PRECHARGE_CLOSED.load(Ordering::Relaxed)
}

pub fn set_precharge_state(state: PrechargeState) {
    match state {
        PrechargeState::Open => PRECHARGE_CLOSED.store(false, Ordering::Relaxed),
        PrechargeState::Floating => {
            PRECHARGE_CLOSED.store(false, Ordering::Relaxed);
            queue_fault_placeholder(Fault::PrechargeFloating);
        }
        PrechargeState::Closed => PRECHARGE_CLOSED.store(true, Ordering::Relaxed),
    }
}

/// signaled by `bms_handle_dcl_message` to reset the monitor's ticker
static DCL_RECEIVED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// call whenever the BMS's DCL message arrives, resets the monitor
pub fn bms_handle_dcl_message() {
    DCL_RECEIVED.signal(());
}

/// queues BmsCanMonitor if no DCL message arrives for `BMS_CAN_TIMEOUT`
#[embassy_executor::task]
pub async fn bms_can_monitor_task() {
    let mut ticker = Ticker::every(BMS_CAN_TIMEOUT);

    loop {
        match select(ticker.next(), DCL_RECEIVED.wait()).await {
            // timed out with no DCL message, keep firing every period until messages resume
            Either::First(_) => queue_fault_placeholder(Fault::BmsCanMonitor),
            // got a message, restart the full period
            Either::Second(_) => ticker.reset(),
        }
    }
}
