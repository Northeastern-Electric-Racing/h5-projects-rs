//! Module for controlling the Segments and their ADBMS6830B chips.
//!
//! For context, there are 5 segments, each with two ADBMS6830B chips. So, there are 10 ADBMS6830B chips total.

mod cache;
mod chips;
mod core;
#[cfg(feature = "hil")]
mod hil;

/// Allows you to read the cache data.
pub const fn cache() -> &'static CacheData {
    &cache::CACHE
}

// Re-exports
pub use core::task::{
    segments_task,
    signal::{SEGMENTS_FRESH_DATA_SIGNAL, SEGMENTS_OPENWIRE_RAN_SIGNAL},
    OPEN_WIRE_FREQUENCY,
};
pub use chips::{
    ChipId, IndexByChip, ChipKind,
    cells::{CellId, IndexByCell, NUM_CELLS_TOTAL},
    gpios::{GpioId, IndexByGpio, ThermistorTemperatures},
    segments::{SegmentId, IndexBySegment, NUM_CELLS_PER_SEGMENT},
};
pub use cache::{CacheData, RegisterCacheData, Reading, fault_counts, redundant_aux, cell_voltages, average_cell_voltages, filtered_cell_voltages, s_voltages, status_c, status_d, aux, status_a, status_b, pwm};
