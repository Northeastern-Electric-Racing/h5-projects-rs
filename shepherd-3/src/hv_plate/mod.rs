//! Module for controlling the HV plate and its ADBMS2950B chip.
//!
//! One ADBMS2950B measures the tractive system as a whole:
//! - pack current through a 0.05 mOhm shunt
//! - BATT and TS voltage through dividers
//! - shunt temperature through a thermistor on a voltage channel
//!
//! The chip also drives the HV control relay from GPO4, via `HvPlate::set_hv_relay`.

mod board;
mod cache;
mod core;

/// Allows you to read the cache data.
pub const fn cache() -> &'static CacheData {
    &cache::CACHE
}

// Re-exports
pub use core::task::{hv_plate_task, signal::HV_PLATE_FRESH_DATA_SIGNAL};
pub use cache::{CacheData, RegisterCacheData, UpdateError, accumulated, aux, current_voltage, flag, status, voltages};
pub use board::{SHUNT_THERMISTOR_CHANNEL, TS_VOLTAGE_CHANNEL};
