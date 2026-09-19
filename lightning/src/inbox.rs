pub use self::inbox::FaultframeState;
pub mod inbox {
    use core::fmt::Debug;

    use defmt::warn;
    use embassy_stm32::can::Frame;
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Receiver, mutex::Mutex};
    const CAN_RECV_TIMEOUT: Duration = Duration::from_millis(500);
    // Previously known as IMD_GENERAL_MSG_ID
    pub const IMD_CAN_ID: u16 = 0x307;

    // BMS_LIGHTNING_OKAY_MSG_ID
    pub const BMS_CAN_ID: u32 = 0x01E;

    // #define RESET_LATCHING_MSG_ID     0x510
    pub const LATCHING_CAN_ID: u16 = 0x510;

    const IMD_CAN_ID_PROCESSED: Id = Id::Standard(StandardId::new(IMD_CAN_ID).expect("Invalid ID"));
    const LATCHING_CAN_ID_PROCESSED: Id =
        Id::Standard(StandardId::new(LATCHING_CAN_ID).expect("Invalid ID"));
    const BMS_CAN_ID_PROCESSED: Id = Id::Extended(ExtendedId::new(BMS_CAN_ID).expect("Invalid ID"));

    use embassy_time::{Duration, WithTimeout};
    use embedded_can::{ExtendedId, Id, StandardId};
    use heapless::mpmc::Queue;

    #[derive(Debug, PartialEq)]
    pub enum FaultframeState {
        BMSFault,
        BMSOk,
        IMDFault,
        IMDOk,
        ResetRequested,
    }

    /// This struct requires an already existing NerCan instance, as well as an already spawned
    /// can_handler task. The point of the Inbox is to parse the can messages and relay them to the
    /// state machine cleanly

    #[embassy_executor::task]
    pub async fn populate_queue(
        receiver: Receiver<'static, ThreadModeRawMutex, Frame, 16>,
        quetex: &'static Mutex<ThreadModeRawMutex, &'static Queue<Option<FaultframeState>, 32>>,
    ) -> ! {
        loop {
            let latest: Option<FaultframeState> =
                match receiver.receive().with_timeout(CAN_RECV_TIMEOUT).await {
                    Ok(frame) => match frame.id() {
                        &IMD_CAN_ID_PROCESSED => match frame.data().iter().any(|&b| b != 0) {
                            false => Some(FaultframeState::IMDOk),
                            true => Some(FaultframeState::IMDFault),
                        },
                        &BMS_CAN_ID_PROCESSED => match frame.data()[0] & 0x80 {
                            0 => Some(FaultframeState::BMSOk),
                            _ => Some(FaultframeState::BMSFault),
                        },
                        &LATCHING_CAN_ID_PROCESSED => match frame.data()[0] & 0x80 {
                            0 => None, // Nothing needs to be done if no reset it requested
                            _ => Some(FaultframeState::ResetRequested),
                        },
                        _id => {
                            warn!("Unknown ID ");
                            None
                        }
                    },
                    Err(e) => {
                        warn!("Did not receive CAN Frame. Error: {}", e);
                        None
                    }
                };
            match quetex.lock().await.enqueue(latest) {
                Ok(_) => {}
                Err(_) => warn!("Could not append to queue. Dropping packet."),
            }
        }
    }
}
