//! Monitors the lightning CAN bus for pulses,
//! and queues a fault if no pulses are received for 4 seconds.

use defmt::debug;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};

/// enum to record the state of the lights
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum LightStatus {
    Off,
    Green,
    Red,
}

/// for converting from a lightstatus to a u8 byte
impl From<LightStatus> for u8 {
    fn from(status: LightStatus) -> u8 {
        match status {
            LightStatus::Off => 0,
            LightStatus::Green => 1,
            LightStatus::Red => 2,
        }
    }
}

static TICKER_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// records that a pulse just arrived by restarting the ticker's 4-second countdown
pub fn lightning_handle_pulse_message() {
    TICKER_RESET.signal(());
}

#[embassy_executor::task]
pub async fn lightning_can_monitor_task() {
    let mut ticker = Ticker::every(Duration::from_secs(4));

    loop {
        if let Either::First(_) = select(ticker.next(), TICKER_RESET.wait()).await {
            // TODO: queue Fault::LightningCanMonitor once cerberus-3 has faults
            debug!("LightningCanMonitor fault was queued");
        } else {
            ticker.reset();
        }
    }
}
