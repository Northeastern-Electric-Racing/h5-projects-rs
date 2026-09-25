pub mod state_machine {
    use crate::hardware::Leds;
    use crate::inbox::FaultframeState::{self, BMSFault, BMSOk, IMDFault, IMDOk, ResetRequested};
    #[derive(Debug, PartialEq, Eq)]
    enum State {
        Red,
        Green,
        Startup,
        _Invalid, // Never constructed. Kept for future use
    }

    const GRACE_PERIOD_DURATION: Duration = Duration::from_secs(3);
    use defmt::{error, warn};
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
        let mut bms_seen: bool = false;
        let mut imd_seen: bool = false;
        // Note that this always waits slightly longer than the grace period
        leds.set_all_off();
        embassy_time::Timer::after(GRACE_PERIOD_DURATION).await; //Wait untill the end of the grace
        //period

        loop {
            grace_period = !bms_seen && !imd_seen;
            let item = quetex.lock().await.dequeue();
            match item {
                Some(latest) => {
                    match latest {
                        Some(msg) => {
                            match msg {
                                BMSFault | IMDFault => {
                                    if !grace_period {
                                        state = State::Red;
                                    } else {
                                        if msg == BMSFault {
                                            bms_seen = true;
                                        } else if msg == IMDFault {
                                            imd_seen = true;
                                        }
                                        state = State::Startup;
                                    }
                                }
                                ResetRequested => {
                                    if !grace_period {
                                        warn!("Resetting Fault due to latch");
                                        state = State::Green;
                                    }
                                }
                                BMSOk => {
                                    bms_seen = true;
                                    if grace_period {
                                        state = State::Startup
                                    }
                                    if state != State::Red && !grace_period {
                                        // Don't allow red -> green transitions
                                        state = State::Green;
                                    }
                                }
                                IMDOk => {
                                    imd_seen = true;
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
                        {
                            embassy_time::Timer::after_millis(50).await;
                        }
                    }
                }
                None =>
                // This means that the queue is empty. Check the old code for what to do here
                {
                    embassy_time::Timer::after_millis(50).await;
                }
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
