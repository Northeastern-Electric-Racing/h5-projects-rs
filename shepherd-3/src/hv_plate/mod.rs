//! Module for controlling the HV plate and its ADBMS2950B chip.
//!
//! One ADBMS2950B measures the tractive system as a whole:
//! - pack current through a 0.05 mOhm shunt
//! - BATT and TS voltage through dividers
//! - shunt temperature through a thermistor on a voltage channel
//!
//! The chip also drives the HV control relay from GPO4, via `HvPlate::set_hv_relay`.
//!
//! Much of what this module computes is read only by its `#[cfg(defmt_monitor)]` blocks, so the allow below covers every file for monitors-off builds.
#![cfg_attr(not(defmt_monitor), allow(dead_code, unused_mut, unused_variables, unused_assignments))]

mod alias;
mod board;
mod cache;
mod core;
mod debug;
mod isospi_recovery;
mod shunt;

// Tasks
pub use core::task::hv_plate_task;
pub use debug::hv_plate_debug;
pub use shunt::shunt_task;

// -- API for hv_plate module --
use crate::helpers::stamped::Stamped;
use crate::units::{Current, Voltage};

/// One accumulator window, offset-corrected and in physical units.
pub(crate) use shunt::Measurement;

/// The relay is the one thing the plate takes commands about rather than only reporting.
pub(crate) use self::core::relay::{RelayRequest, RelayState};

/// TS voltage off the 2950's own divider behind GPO2.
pub(crate) fn ts_voltage() -> Option<Stamped<Voltage>> {
    cache::CACHE.voltages().reading().try_partial_get(|reading| Stamped::new(reading.nice().ts_voltage, reading.read_at()))
}

/// BATT-side voltage, off the 2950's own divider.
pub(crate) fn instant_batt_voltage() -> Option<Stamped<Voltage>> {
    cache::CACHE.current_voltage().reading().try_partial_get(|reading| Stamped::new(reading.nice().batt_voltage, reading.read_at()))
}

/// Pack current off a single conversion, offset-corrected. Positive is discharge.
pub(crate) fn instant_current() -> Option<Stamped<Current>> {
    shunt::published::instant()
}

/// The last accumulator window to close
pub(crate) const fn shunt_window() -> &'static shunt::published::UnwriteableWindowWatch {
    shunt::published::window()
}

/// What the 2950 last acknowledged about GPO4, which is not necessarily what the contacts are doing.
pub(crate) fn relay_state() -> RelayState {
    self::core::relay::state()
}

/// Asks the plate to open or close the HV relay; it is applied on the plate's next pass.
pub(crate) fn request_relay(request: RelayRequest) {
    self::core::relay::request(request);
}
