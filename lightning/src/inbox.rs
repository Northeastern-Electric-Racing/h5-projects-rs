pub use self::inbox::FaultframeState;
pub mod inbox {
    use core::fmt::Debug;

    use defmt::{debug, warn};
    use embassy_stm32::can::Frame;
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Receiver, mutex::Mutex};
    const CAN_RECV_TIMEOUT: Duration = Duration::from_millis(500);

    pub const GRACE_PERIOD_DURATION: Duration = Duration::from_secs(5); // Needs to be public such
    // that state can read it
    // Previously known as IMD_GENERAL_MSG_ID
    pub const IMD_CAN_ID: u16 = 0x037;

    // BMS_LIGHTNING_OKAY_MSG_ID
    pub const BMS_CAN_ID: u32 = 0x01E;

    // #define RESET_LATCHING_MSG_ID     0x510
    pub const LATCHING_CAN_ID: u16 = 0x510;

    const IMD_CAN_ID_PROCESSED: Id = Id::Standard(StandardId::new(IMD_CAN_ID).expect("Invalid ID"));
    const LATCHING_CAN_ID_PROCESSED: Id =
        Id::Standard(StandardId::new(LATCHING_CAN_ID).expect("Invalid ID"));
    const BMS_CAN_ID_PROCESSED: Id = Id::Extended(ExtendedId::new(BMS_CAN_ID).expect("Invalid ID"));

    use embassy_time::{Duration, Instant, WithTimeout};
    use embedded_can::{ExtendedId, Id, StandardId};
    use heapless::mpmc::Queue;

    #[derive(Debug, PartialEq, defmt::Format)]
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
        let boot_time: Instant = Instant::now();
        receiver.clear(); // The IMD goes a bit crazy on init, this gets rid of the frame.
        loop {
            let latest: Option<FaultframeState> =
                match receiver.receive().with_timeout(CAN_RECV_TIMEOUT).await {
                    Ok(frame) => match frame.id() {
                        &IMD_CAN_ID_PROCESSED => {
                            // Only the 4th and 5th bits mean there is an error on the IMD
                            match (u16::from_le_bytes([
                                *frame.data().get(4).unwrap_or(&1),
                                *frame.data().get(4).unwrap_or(&1),
                            ]) & 0x07FF)
                            {
                                0 => Some(FaultframeState::IMDOk),
                                _ => {
                                    debug!("IMD Fault data: {}", frame.data());
                                    Some(FaultframeState::IMDFault)
                                }
                            }
                        }
                        &BMS_CAN_ID_PROCESSED => match frame.data()[0] & 0x80 {
                            0 => Some(FaultframeState::BMSOk),
                            _ => Some(FaultframeState::BMSFault),
                        },
                        &LATCHING_CAN_ID_PROCESSED => match frame.data()[0] & 0x80 {
                            0 => None, // Nothing needs to be done if no reset it requested
                            _ => Some(FaultframeState::ResetRequested),
                        },
                        _id => {
                            warn!("Unknown ID. Somthing is wrong with the filters. ");
                            None
                        }
                    },
                    Err(e) => {
                        // warn!("Did not receive CAN Frame. Error: {}", e);
                        None
                    }
                };
            // debug!("latest: {}", latest);
            if latest.is_some()
                && (embassy_time::Instant::now() - boot_time) > GRACE_PERIOD_DURATION
            {
                match quetex.lock().await.enqueue(latest) {
                    Ok(_) => {}
                    Err(_) => warn!("Could not append to queue. Dropping packet."),
                }
            }
        }
    }
}
