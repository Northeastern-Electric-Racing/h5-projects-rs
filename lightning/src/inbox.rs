pub use self::inbox::FaultframeState;
mod inbox {
    use can_handler::NerCan;
    use defmt::warn;
    use embassy_stm32::can::Frame;
    use embassy_sync::{
        blocking_mutex::{
            CriticalSectionMutex,
            raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
        },
        channel::Receiver,
        mutex::Mutex,
    };
    const CAN_RECV_TIMEOUT: Duration = Duration::from_millis(500);
    // Previously known as IMD_GENERAL_MSG_ID
    const IMD_CAN_ID: Id = Id::Standard(0x37);
    // BMS_LIGHTNING_OKAY_MSG_ID
    const BMS_CAN_ID: Id = Id::Standard(0x37);

    // #define RESET_LATCHING_MSG_ID     0x510
    const LATCHING_CAN_ID: ID = 0x510;
    use embassy_time::{Duration, WithTimeout};
    use embedded_can::Id;
    use heapless::mpmc::Queue;
    #[derive(Debug)]
    pub enum FaultframeState {
        BMSFault,
        BMSOk,
        IMDFault,
        IMDOk,
        LatchingOk,
        LatchingFault,
    }

    #[derive(Debug)]
    pub enum CANError {
        Timeout,
        Unkown,
    }

    /// This struct requires an already existing NerCan instance, as well as an already spawned
    /// can_handler task. The point of the Inbox is to parse the can messages and relay them to the
    /// state machine cleanly
    pub struct Inbox {
        receiver: Receiver<'static, ThreadModeRawMutex, Frame, 16>,
    }
    impl Inbox {
        // TODO: Make this a task
        pub async fn populate_queue(
            self,
            quetex: &'static Mutex<CriticalSectionRawMutex, Queue<Option<FaultframeState>, 32>>,
        ) -> ! {
            loop {
                let latest: Option<FaultframeState> =
                    match self.receiver.receive().with_timeout(CAN_RECV_TIMEOUT).await {
                        Ok(frame) => match frame.id() {
                            &IMD_CAN_ID => match frame.data().iter().sum() {
                                0 => Some(FaultframeState::IMDOk),
                                _ => Some(FaultframeState::IMDFault),
                            },
                            &BMS_CAN_ID => match frame.data()[0] & 0x80 {
                                0 => Some(FaultframeState::BMSOk),
                                _ => Some(FaultframeState::BMSFault),
                            },
                            &LATCHING_CAN_ID => match frame.data()[0] & 0x80 {
                                0 => Some(FaultframeState::LatchingOk),
                                _ => Some(FaultframeState::LatchingFault),
                            },
                            _id => {
                                warn!("Unknown ID: {:#?}", _id);
                                None
                            }
                        },
                        Err(e) => {
                            warn!("Did not recive CAN Frame. Error: {}", e);
                            None
                        }
                    };
                quetex.lock().await.enqueue(latest).expect("Queue Full");
                // TODO: This shouldn't be a panic
            }
        }
    }
}
