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
#[cfg(feature = "hil")]
mod hil;
mod isospi_recovery;
mod precharge;
mod soc;

// Tasks
pub use core::task::hv_plate_task;
pub use precharge::precharge_task;
pub use soc::soc_task;

// API
/// Allows you to read the cache data.
pub const fn cache() -> &'static CacheData {
    &cache::CACHE
}
pub use core::task::signal::HV_PLATE_FRESH_DATA_SIGNAL;
pub(crate) use soc::api::soc_data;

// Re-export cache types
pub use cache::CacheData;
