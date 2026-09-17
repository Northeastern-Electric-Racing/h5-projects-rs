pub mod state_machine {
    use crate::hardware::Leds;
    use crate::inbox::FaultframeState::{self, BMSFault, IMDFault, LatchingFault};
    #[derive(Debug, PartialEq, Eq)]
    enum State {
        Red,
        Green,
        Startup,
        _Invalid, // Never constructed. Kept for future use
    }

    const GRACE_PERIOD_DURATION: Duration = Duration::from_secs(3);
    use defmt::error;
    use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
    use embassy_sync::mutex::Mutex;
    use embassy_time::{Duration, Instant};
    use heapless::mpmc::Queue;
    #[embassy_executor::task]
    pub async fn state_machine(
        quetex: &'static Mutex<ThreadModeRawMutex, &'static Queue<Option<FaultframeState>, 32>>,
        mut leds: Leds,
    ) -> ! {
        let mut state: State = State::Startup;
        let boot_time = Instant::now(); // This might crash after a few hours; it is probably fine
        let mut grace_period: bool = true;
        // QUESTION: Why not just sleep until the grace period is over?

        loop {
            grace_period = (Instant::now() - boot_time) >= GRACE_PERIOD_DURATION;
            match quetex.lock().await.dequeue() {
                Some(latest) => {
                    match latest {
                        Some(msg) => {
                            match msg {
                                // TODO: Fix LatchingFault Logic.
                                BMSFault | IMDFault | LatchingFault => {
                                    if !grace_period {
                                        state = State::Red;
                                    } else {
                                        state = State::Startup;
                                    }
                                }
                                _ => {
                                    if grace_period {
                                        state = State::Startup
                                    }
                                    if state != State::Red && !grace_period {
                                        // Don't allow red -> green transitions
                                        state = State::Green;
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
                State::_Invalid => {
                    error!("Invalid State Reached!");
                    leds.set_all_off();
                }
            }
        }
    }
}
