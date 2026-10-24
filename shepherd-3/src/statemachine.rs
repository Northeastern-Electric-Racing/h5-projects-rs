use strum::FromRepr;
use embassy_time::Instant;
use crate::units::Voltage;
use crate::{charging, charging::ChargerStatus, faults, faults::CriticalityStatus};

/// The operating state of the pack.
#[derive(PartialEq, Eq, FromRepr, defmt::Format)]
pub enum BmsState {
    /// Bringing the pack up. Startup has not finished.
    Boot = 0,
    /// Normal operation.
    Ready,
    /// The charger is connected and charging is being managed.
    Charging,
    /// A critical fault is active. The shutdown circuit is asserted.
    Faulted,
}
impl BmsState {
    pub fn tick(self) -> Self {

        // hierarchical/flowchart-ish verison #3
        // get rid of the enums and just go back to using `are_critical_faults_active()` and `is_charger_connected()` since they probably are easier to read here. also probably controversial if/else formatting

        match self {
            BmsState::Boot => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else if charging::is_charger_connected() { BmsState::Charging } 
                else                                     { BmsState::Ready }
            },

            BmsState::Ready => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else if charging::is_charger_connected() { BmsState::Charging } 
                else                                     { BmsState::Ready }
            },

            BmsState::Charging => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else if charging::is_charger_connected() { BmsState::Charging } 
                else                                     { BmsState::Ready }
            },

            BmsState::Faulted => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else                                     { BmsState::Boot } 
            },
        }
    }
}

pub fn tick(self) -> Self {

    // hierarchical/flowchart-ish verison #2
    // (probably most similar to an actual state transition diagram since we start by matching on the current state. so it is easy to view each state, and what the possible transitions out from that state are)

    match self {
        BmsState::Boot => {
            match faults::criticality_status() {
                CriticalityStatus::CriticalFaultsActive                         => BmsState::Faulted,

                CriticalityStatus::Ok => match charging::charger_connected() {
                    ChargerStatus::ChargerConnected                             => BmsState::Charging,
                    ChargerStatus::ChargerNotConnected                          => BmsState::Ready,
                }
            }
        },

        BmsState::Ready => {
            match faults::criticality_status() {
                CriticalityStatus::CriticalFaultsActive                         => BmsState::Faulted,

                CriticalityStatus::Ok => match charging::charger_connected() {
                    ChargerStatus::ChargerConnected                             => BmsState::Charging,
                    ChargerStatus::ChargerNotConnected                          => BmsState::Ready,
                }
            }
        },

        BmsState::Charging => {
            match faults::criticality_status() {
                CriticalityStatus::CriticalFaultsActive                         => BmsState::Faulted,
                
                CriticalityStatus::Ok => match charging::charger_connected() {
                    ChargerStatus::ChargerConnected                             => BmsState::Charging,
                    ChargerStatus::ChargerNotConnected                          => BmsState::Ready,
                }
            }
        },

        BmsState::Faulted => {
            match faults::criticality_status() {
                CriticalityStatus::CriticalFaultsActive                         => BmsState::Faulted,
                CriticalityStatus::Ok                                           => BmsState::Boot,
            }
        },
    }
}

pub fn tick(self) -> Self {

    // hierarchical/flowchart-ish verison #1
    // (nice since we don't need to use _ for the CriticalityStatus::Ok case once it has been handled by the highest level branch)

    match faults::criticality_status() {
        CriticalityStatus::CriticalFaultsActive => BmsState::Faulted,

        CriticalityStatus::Ok => match (self, charging::charger_connected()) {
            (BmsState::Boot, ChargerStatus::ChargerConnected)        => BmsState::Charging,
            (BmsState::Boot, ChargerStatus::ChargerNotConnected)     => BmsState::Ready,

            (BmsState::Ready, ChargerStatus::ChargerConnected)       => BmsState::Charging,
            (BmsState::Ready, ChargerStatus::ChargerNotConnected)    => BmsState::Ready,

            (BmsState::Charging, ChargerStatus::ChargerNotConnected) => BmsState::Ready,
            (BmsState::Charging, ChargerStatus::ChargerConnected)    => BmsState::Charging,

            (BmsState::Faulted, _)                                   => BmsState::Boot,
        }
    }
}

pub fn tick(self) -> Self {
    match (self, faults::criticality_status(), charging::charger_connected()) {
        (_, CriticalityStatus::CriticalFaultsActive, _)             => BmsState::Faulted,

        (BmsState::Boot, _, ChargerStatus::ChargerConnected)        => BmsState::Charging,
        (BmsState::Boot, _, ChargerStatus::ChargerNotConnected)     => BmsState::Ready,

        (BmsState::Ready, _, ChargerStatus::ChargerConnected)       => BmsState::Charging,
        (BmsState::Ready, _, ChargerStatus::ChargerNotConnected)    => BmsState::Ready,

        (BmsState::Charging, _, ChargerStatus::ChargerNotConnected) => BmsState::Ready,
        (BmsState::Charging, _, ChargerStatus::ChargerConnected)    => BmsState::Charging,

        (BmsState::Faulted, _, _)                                   => BmsState::Boot,
    }
}

pub fn tick(self) -> Self {
    match (self, faults::criticality_status(), charging::charger_connected()) {
        // State transition table esque syntax #2 (we explicitly list every case, kinda verbose/ugly?) 
        (BmsState::Boot, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerConnected)        => BmsState::Faulted,
        (BmsState::Boot, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerNotConnected)     => BmsState::Faulted,
        (BmsState::Boot, CriticalityStatus::Ok, ChargerStatus::ChargerConnected)                          => BmsState::Charging,
        (BmsState::Boot, CriticalityStatus::Ok, ChargerStatus::ChargerNotConnected)                       => BmsState::Ready,

        (BmsState::Ready, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerConnected)       => BmsState::Faulted,
        (BmsState::Ready, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerNotConnected)    => BmsState::Faulted,
        (BmsState::Ready, CriticalityStatus::Ok, ChargerStatus::ChargerConnected)                         => BmsState::Charging,
        (BmsState::Ready, CriticalityStatus::Ok, ChargerStatus::ChargerNotConnected)                      => BmsState::Ready,

        (BmsState::Charging, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerNotConnected) => BmsState::Faulted,
        (BmsState::Charging, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerConnected)    => BmsState::Faulted,
        (BmsState::Charging, CriticalityStatus::Ok, ChargerStatus::ChargerNotConnected)                   => BmsState::Ready,
        (BmsState::Charging, CriticalityStatus::Ok, ChargerStatus::ChargerConnected)                      => BmsState::Charging,

        (BmsState::Faulted, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerNotConnected)  => BmsState::Faulted,
        (BmsState::Faulted, CriticalityStatus::CriticalFaultsActive, ChargerStatus::ChargerConnected)     => BmsState::Faulted,
        (BmsState::Faulted, CriticalityStatus::Ok, ChargerStatus::ChargerNotConnected)                    => BmsState::Boot,
        (BmsState::Faulted, CriticalityStatus::Ok, ChargerStatus::ChargerConnected)                       => BmsState::Boot,
    }
}

mod sm {
    use super::*;

    use core::cell::Cell;
    use embassy_sync::blocking_mutex::ThreadModeMutex;

    struct State { state: ThreadModeMutex<Cell<BmsState>> }
    impl State {
        pub const fn new() -> Self { Self { state: ThreadModeMutex::new(Cell::new(BmsState::Boot)) } }
        pub fn 
    }

    impl AtomicBmsState {
        fn tick(&self) {

        }
    }
}
