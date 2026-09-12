// TODO: Handle state machine logic here
// TODO: Fix imports
mod state_machine {
    use crate::inbox;
    use embassy_executor::task;
    use embassy_sync::mutex;
    // TODO: Accept the ground truth from a mutex that gets populated from a seperate CAN thread
    #[embassy_executor::task]
    pub async fn state_machine(
        faults: mutex::Mutex<CriticalSectionMutex, Result<inbox::FaultStatus, /*whatever the can error type is*/>>,
    ) -> ! {
        // TODO: Init; have both lights off here
        loop {
            match faults {
                // TODO: Act on hardware here
                Ok(v) => todo!(),
                Err(_) => //TODO: Red lights
            }
        }
    }
}
