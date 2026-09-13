// TODO: Handle state machine logic here
mod state_machine {
    use crate::inbox::{
        self,
        FaultframeState::{self, BMSFault, IMDFault, LatchingFault},
    };
    use embassy_executor::task;
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
    use heapless::mpmc::Queue;
    #[embassy_executor::task]
    pub async fn state_machine(
        quetex: &'static Mutex<CriticalSectionRawMutex, Queue<Option<FaultframeState>, 32>>,
    ) -> ! {
        // TODO: Init; have both lights off here
        loop {
            match quetex.lock().await.dequeue() {
                Some(latest) => {
                    match latest {
                        Some(msg) => {
                            match msg {
                                BMSFault | IMDFault | LatchingFault => {
                                    todo!("set red")
                                }
                                _ => {
                                    todo!("set green")
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
        }
    }
}
