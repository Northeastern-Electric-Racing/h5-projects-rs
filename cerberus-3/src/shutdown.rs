use core::sync::atomic::{AtomicBool, Ordering};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BmsShutdownState {
    Shutdown,
    Closed
}

static BMS_SHUTDOWN: AtomicBool = AtomicBool::new(true);

pub fn update_bms_shutdown(state: BmsShutdownState) {
    BMS_SHUTDOWN.store(
        state == BmsShutdownState::Shutdown,
        Ordering::Relaxed,
    );
}


pub fn is_shutdown_closed() -> bool {
    #[cfg(feature = "TSMS_OVERIDE")]
    {
        return true;
    }

    #[cfg(not(feature = "TSMS_OVERIDE"))]
    {
        !BMS_SHUTDOWN.load(Ordering::Relaxed)
    }
}