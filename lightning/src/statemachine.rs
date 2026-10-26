use crate::can::FaultframeState;
use crate::can::GRACE_PERIOD_DURATION;
use crate::lights;
use defmt::{error, warn};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

#[derive(Debug, PartialEq, Eq)]
enum State {
    Red,
    Green,
    Startup,
    _Invalid, // Never constructed. Kept for future use
}

const QUEUE_SIZE: usize = 24;
pub static FAULT_QUEUE: Channel<ThreadModeRawMutex, FaultframeState, QUEUE_SIZE> = Channel::new();

fn handle_transition(from: &mut State, to: State) {
    if *from == State::Red && to == State::Green {
        warn!("Transitioning from Red to Green, this should not happen!");
    } else {
        *from = to;
    }
}

#[embassy_executor::task]
pub async fn statemachine_task(mut leds: lights::Leds) -> ! {
    let mut state: State = State::Startup;
    let mut bms_seen: bool = false;
    let mut imd_seen: bool = false;

    // Note that this always waits slightly longer than the grace period
    leds.set_all_off(); // TODO: Change this to set_all_on when lamp check hardware allows
    embassy_time::Timer::after(GRACE_PERIOD_DURATION).await; //Wait until the end of the grace
    //period

    leds.set_all_off();
    loop {
        let grace_period = !(bms_seen || imd_seen); // Grace period is true if neither fault has been seen yet
        match FAULT_QUEUE.receive().await {
            FaultframeState::BMSFault => {
                if grace_period {
                    bms_seen = true;
                    state = State::Startup;
                } else {
                    handle_transition(&mut state, State::Red);
                }
            }
            FaultframeState::IMDFault => {
                if grace_period {
                    imd_seen = true;
                    state = State::Startup;
                } else {
                    handle_transition(&mut state, State::Red);
                }
            }
            FaultframeState::ResetRequested => {
                if !grace_period {
                    warn!("Resetting Fault due to latch");
                    state = State::Green;
                }
            }
            FaultframeState::BMSOk => {
                bms_seen = true;
                if grace_period {
                    state = State::Startup
                } else {
                    handle_transition(&mut state, State::Green);
                }
            }
            FaultframeState::IMDOk => {
                imd_seen = true;
                if grace_period {
                    state = State::Startup
                } else {
                    handle_transition(&mut state, State::Green);
                }
            }
        };
        match state {
            State::Red => leds.set_red_on(),
            State::Green => leds.set_green_on(),
            State::Startup => leds.set_all_off(),
            State::_Invalid => {
                error!("Invalid State Reached!");
                leds.set_all_off();
            }
        }
    }
}
