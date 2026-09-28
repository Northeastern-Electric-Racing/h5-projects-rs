use crate::{
    segments::{
        CellId, ChipId,
    },
    units::{
        Temperature, Voltage,
    }
};

struct CriticalCellValue<T> {
    /// The critical value being stored here.
    value: T,
    /// Chip the critical value was measured from.
    chip: ChipId,
    /// Cell on `chip` that the critical value was measured from.
    cell: CellId,
}

/// 6 consoles 10 computers
struct Analyzer {
    // Max, min, and avg thermistor readings.
    max_temp: CriticalCellValue<Temperature>,
    min_temp: CriticalCellValue<Temperature>,
    avg_temp: Temperature,

    // Max, min, and avg voltage of the cells
    max_voltage: CriticalCellValue<Voltage>,
    min_voltage: CriticalCellValue<Voltage>,
    avg_voltage: Voltage,
    delta_voltage: Voltage,

    // Max, min, and avg Open Cell Voltage (OCV) readings.
    max_ocv: CriticalCellValue<Voltage>,
    min_ocv: CriticalCellValue<Voltage>,
    avg_ocv: Voltage,
    delta_ocv: Voltage,
    pack_ocv: Voltage,

    // The highest current chip temperature, for faulting.
    max_chiptemp: CriticalCellValue<Temperature>,

    segment_average_temps: 
}