use crate::can::FaultframeState;
use crate::can::GRACE_PERIOD_DURATION;
use crate::lights;
use defmt::warn;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

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
    event: FaultframeState,
    awaiting_contact: bool,
    any_fault: bool,
) -> State {
    use FaultframeState::*;

    match (state, event) {
        _ if awaiting_contact => State::Startup,
        _ if any_fault => State::Red,
        (State::Red, ResetRequested) => State::Green,
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
    loop {
        let event = FAULT_QUEUE.receive().await;
        match event {
            FaultframeState::BMSFault | FaultframeState::BMSOk => {
                bms_seen = true;
                bms_faulted = event == FaultframeState::BMSFault;
            }
            FaultframeState::IMDFault | FaultframeState::IMDOk => {
                imd_seen = true;
                imd_faulted = event == FaultframeState::IMDFault;
            }
            FaultframeState::ResetRequested => {}
        }

        let new = next_state(
            state,
            event,
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
