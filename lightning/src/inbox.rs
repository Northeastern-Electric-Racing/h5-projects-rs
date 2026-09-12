mod inbox {
    use can_handler::NerCan;
    use embassy_sync::channel::Receiver;
    #[derive(Debug)]
    pub struct FaultStates {
        BMSFault: bool,
        IMDFault: bool,
    }

    /// This struct requires an already existing NerCan instance, as well as an already spawned
    /// can_handler task. The point of the Inbox is to parse the can messages and relay them to the
    /// state machine cleanly
    pub struct Inbox {
        receiver: Receiver<'static, ThreadModeRawMutex, Frame, 16>,
    }
    impl Inbox {
        pub fn get_latest_unread(self) -> FaultStates {
            // TODO: Handle this
        }
    }
}
