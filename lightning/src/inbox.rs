pub use self::inbox::FaultframeState;
pub mod inbox {
    use core::fmt::Debug;

    use defmt::warn;
    use embassy_stm32::can::{CanRx, Frame};
    use embassy_sync::channel::Channel;

    static INCOMING: Channel<ThreadModeRawMutex, Frame, 32> = Channel::new();
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
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

    // Queue received frames separately from application command processing.
    #[embassy_executor::task]
    pub async fn can_rx(mut receiver: CanRx<'static>) -> ! {
        loop {
            match receiver.read().await {
                Ok(envelope) => INCOMING.send(envelope.frame).await,
                Err(error) => warn!("CAN receive error: {}", error),
            }
        }
    }

    // Process boot requests and relay fault messages to the state machine.
    #[embassy_executor::task]
    pub async fn populate_queue(
        quetex: &'static Mutex<ThreadModeRawMutex, &'static Queue<Option<FaultframeState>, 32>>,
        mut bootloader: boot_control::Bootloader<'static>,
    ) -> ! {
        let boot_time: Instant = Instant::now();
        loop {
            let latest: Option<FaultframeState> =
                match INCOMING.receive().with_timeout(CAN_RECV_TIMEOUT).await {
                    Ok(frame) => {
                        // Handle boot entry independently of the fault-frame grace period.
                        if bootloader.handle_frame(&frame).is_err() {
                            defmt::error!("Failed to persist bootloader request");
                            continue;
                        }
                        match frame.id() {
                            &IMD_CAN_ID_PROCESSED => {
                                // Only the 4th and 5th bits mean there is an error on the IMD
                                match u16::from_le_bytes([
                                    *frame.data().get(4).unwrap_or(&1),
                                    *frame.data().get(4).unwrap_or(&1),
                                ]) & 0x07FF
                                {
                                    0 => Some(FaultframeState::IMDOk),
                                    _ => Some(FaultframeState::IMDFault),
                                }
                            }
                            // Bitmask on the 7th bit
                            &BMS_CAN_ID_PROCESSED => match frame.data()[0] & 0x80 {
                                0 => Some(FaultframeState::BMSOk),
                                _ => Some(FaultframeState::BMSFault),
                            },
                            // Bitmask on the 7th bit
                            &LATCHING_CAN_ID_PROCESSED => match frame.data()[0] & 0x80 {
                                0 => None, // Nothing needs to be done if no reset it requested
                                _ => Some(FaultframeState::ResetRequested),
                            },
                            _id => {
                                warn!("Unknown ID. Somthing is wrong with the filters. ");
                                None
                            }
                        }
                    }
                    Err(_e) => {
                        // warn!("timeout");
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
