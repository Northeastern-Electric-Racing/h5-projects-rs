//! Stub until it is not

pub enum ChargerStatus {
    ChargerConnected,
    ChargerNotConnected,
}

/// Indicates if the charger is connected.
pub fn charger_connected() -> ChargerStatus {
    // u_TODO make this real
    ChargerStatus::ChargerNotConnected
}

pub fn is_charger_connected() -> bool {
    // u_TODO make this real
    false
}