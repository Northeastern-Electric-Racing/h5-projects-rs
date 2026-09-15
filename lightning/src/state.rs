// TODO: Handle state machine logic here
mod state_machine {
    use crate::hardware::Leds;
    use crate::inbox::{
        self,
        FaultframeState::{self, BMSFault, IMDFault, LatchingFault},
    };
    #[derive(Debug, PartialEq, Eq)]
    enum State {
        Red,
        Green,
        Startup,
        Invalid,
    }
    use defmt::{error, warn};
    use embassy_executor::task;
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
    use embassy_time::Instant;
    use heapless::mpmc::Queue;
    #[embassy_executor::task]
    pub async fn state_machine(
        quetex: &'static Mutex<CriticalSectionRawMutex, Queue<Option<FaultframeState>, 32>>,
        mut leds: Leds,
    ) -> ! {
        let mut state: State = State::Startup;
        let boot_time = Instant::now();
        // QUESTION: Why not just sleep until the grace period is over?

        loop {
            match quetex.lock().await.dequeue() {
                Some(latest) => {
                    match latest {
                        Some(msg) => {
                            match msg {
                                BMSFault | IMDFault | LatchingFault => state = State::Red,
                                _ => {
                                    if state != State::Red {
                                        // Don't allow red -> green transitions
                                        state = State::Green
                                    }
                                }
                            };
                        }
                        None =>
                            // This means that the frame was unrelated or not found
                            {}
                    }
                }
                None =>
                    // This means that the queue is empty. Check the old code for what to do here
                    {}
            }
            match state {
                State::Red => leds.set_red_on(),
                State::Green => leds.set_green_on(),
                State::Startup => leds.set_all_off(),
                State::Invalid => {
                    error!("Invalid State Reached!");
                    leds.set_all_off();
                }
            }
        }
    }
}
