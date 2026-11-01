use cangen::ToCanFrame;
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

/// creates a lightningboardlightstatus object and sends it over can
pub async fn send_lightning_board_light_status(status: LightStatus) {
    let light_status = cangen::LightningBoardLightStatus::new().with_status(u8::from(status));
    crate::can::OUTGOING.send(light_status.to_can_frame()).await
}

/// set by lightning_handle_pulse_message when a pulse arrives; lightning_can_monitor_task waits on it to restart the ticker
static TICKER_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// records that a pulse just arrived by restarting the ticker's 4-second countdown
pub fn lightning_handle_pulse_message() {
    TICKER_RESET.signal(());
}

/// sends LightStatus::Off on startup, then queues the LightningCanMonitor fault if no pulse arrives from the lightning board for 4 seconds
#[embassy_executor::task]
pub async fn lightning_can_monitor_task() {
    let mut ticker = Ticker::every(Duration::from_secs(4));
    send_lightning_board_light_status(LightStatus::Off).await;

    loop {
        match select(ticker.next(), TICKER_RESET.wait()).await {
            Either::First(_) => {
                // TODO: queue Fault::LightningCanMonitor once cerberus-3 has faults
                debug!("LightningCanMonitor fault was queued");
            }
            Either::Second(_) => {
                ticker.reset();
            }
        }
    }
}
