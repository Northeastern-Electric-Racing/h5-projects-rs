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

fn next_state(state: State, event: FaultframeState, in_grace: bool) -> State {
    use FaultframeState::*;

    if in_grace {
        return State::Startup;
    }
    match (state, event) {
        // Red is latched: only an explicit reset clears it
        (State::Red, ResetRequested) => State::Green,
        (State::Red, _) => State::Red,
        (_, BMSFault | IMDFault) => State::Red,
        (_, BMSOk | IMDOk | ResetRequested) => State::Green,
    }
}

#[embassy_executor::task]
pub async fn statemachine_task(mut leds: lights::Leds) -> ! {
    let mut state: State = State::Startup;
    let mut bms_seen: bool = false;
    let mut imd_seen: bool = false;

    // Note that this always waits slightly longer than the grace period
    leds.set_all_off(); // TODO: Change this to set_all_on when lamp check hardware allows
    embassy_time::Timer::after(GRACE_PERIOD_DURATION).await; //Wait until the end of the grace period

    leds.set_all_off();
    loop {
        let event = FAULT_QUEUE.receive().await;
        let in_grace = !(bms_seen && imd_seen);
        match event {
            FaultframeState::BMSFault | FaultframeState::BMSOk => bms_seen = true,
            FaultframeState::IMDFault | FaultframeState::IMDOk => imd_seen = true,
            FaultframeState::ResetRequested => {}
        }

        let new = next_state(state, event, in_grace);
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
