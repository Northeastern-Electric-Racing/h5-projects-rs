use crate::can::CAN_TIMEOUT;
use crate::can::FaultframeState;
use crate::can::GRACE_PERIOD_DURATION;
use crate::lights;
use defmt::warn;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, TimeoutError, with_deadline};
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum State {
    Red,
    Green,
    Startup,
}

const QUEUE_SIZE: usize = 24;
pub static FAULT_QUEUE: Channel<ThreadModeRawMutex, FaultframeState, QUEUE_SIZE> = Channel::new();

fn next_state(
    state: State,
    event: Result<FaultframeState, TimeoutError>,
    any_timeout: bool,
    awaiting_contact: bool,
    any_fault: bool,
) -> State {
    use FaultframeState::*;

    match (state, event) {
        _ if any_timeout => State::Red,
        _ if awaiting_contact => State::Startup,
        _ if any_fault => State::Red,
        (State::Red, Ok(ResetRequested)) => State::Green,
        (State::Red, _) => State::Red,
        _ => State::Green,
    }
}

#[embassy_executor::task]
pub async fn statemachine_task(mut leds: lights::Leds) -> ! {
    let mut state: State = State::Startup;
    let mut bms_seen: bool = false;
    let mut imd_seen: bool = false;
    let mut bms_faulted: bool = false;
    let mut imd_faulted: bool = false;

    // Note that this always waits slightly longer than the grace period
    leds.set_all_off(); // TODO: Change this to set_all_on when lamp check hardware allows
    embassy_time::Timer::after(GRACE_PERIOD_DURATION).await; //Wait until the end of the grace period

    leds.set_all_off();

    // Setup for CAN Timeout
    let mut last_bms = Instant::now();
    let mut last_imd = Instant::now();
    let mut bms_timed_out = false;
    let mut imd_timed_out = false;

    loop {
        let bms_deadline = if bms_timed_out {
            Instant::MAX
        } else {
            last_bms + CAN_TIMEOUT
        };
        let imd_deadline = if imd_timed_out {
            Instant::MAX
        } else {
            last_imd + CAN_TIMEOUT
        };
        let deadline = bms_deadline.min(imd_deadline);
        let event = with_deadline(deadline, FAULT_QUEUE.receive()).await;
        match event {
            Ok(FaultframeState::BMSFault | FaultframeState::BMSOk) => {
                bms_seen = true;
                last_bms = Instant::now();
                bms_timed_out = false;
                bms_faulted = event == Ok(FaultframeState::BMSFault);
            }
            Ok(FaultframeState::IMDFault | FaultframeState::IMDOk) => {
                imd_seen = true;
                last_imd = Instant::now();
                imd_timed_out = false;
                imd_faulted = event == Ok(FaultframeState::IMDFault);
            }
            Ok(FaultframeState::ResetRequested) | Err(_) => {}
        }
        let now = Instant::now();
        if !bms_timed_out && now >= last_bms + CAN_TIMEOUT {
            bms_timed_out = true;
            warn!("BMS CAN timeout");
        } else if !imd_timed_out && now >= last_imd + CAN_TIMEOUT {
            imd_timed_out = true;
            warn!("IMD CAN timeout");
        }
        let new = next_state(
            state,
            event,
            bms_timed_out || imd_timed_out,
            !bms_seen || !imd_seen,
            bms_faulted || imd_faulted,
        );
        if new != state {
            if state == State::Red {
                warn!("Fault latch reset: {} -> {}", state, new);
            }
            state = new;
            match state {
                State::Red => leds.set_red_on(),
                State::Green => leds.set_green_on(),
                State::Startup => leds.set_all_off(),
            }
        }
    }
}
