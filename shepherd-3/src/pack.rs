use strum::IntoEnumIterator;

use crate::{
    segments::{CellId, ChipId, SegmentId, IndexBySegment, CacheData, NUM_CELLS_PER_SEGMENT, NUM_CELLS_TOTAL},
    units::{Temperature, Voltage, Percentage, degree_celsius, volt},
};

mod analyzer {
    use uom::si::angle::degree;

    use super::*;

    struct CriticalCellValue<T> {
        /// The critical value being stored here.
        value: T,
        /// Chip the critical value was measured from.
        chip: ChipId,
        /// Cell on `chip` that the critical value was measured from.
        cell: CellId,
    }
    impl<T> CriticalCellValue<T> {
        pub const fn value(&self) -> &T {
            &self.value
        }
        pub const fn chip(&self) -> ChipId {
            self.chip
        }
        pub const fn cell(&self) -> CellId {
            self.cell
        }
    }

    struct CriticalChipValue<T> {
        /// The critical value being stored here.
        value: T,
        /// Chip the critical value was measured from.
        chip: ChipId,
    }
    impl<T> CriticalChipValue<T> {
        pub const fn value(&self) -> &T {
            &self.value
        }
        pub const fn chip(&self) -> ChipId {
            self.chip
        }
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

        /// The highest current chip temperature, for faulting.
        max_chiptemp: CriticalChipValue<Temperature>,

        /// Segment temperature averages.
        segment_average_temps: IndexBySegment<Temperature>,
        /// Segment OCV average voltages.
        segment_average_volts: IndexBySegment<Voltage>,
        /// Total voltages for each segment.
        segment_total_volts: IndexBySegment<Voltage>,
        /// Delta voltages for each segment.
        segment_delta_volts: IndexBySegment<Voltage>,

        /// Voltage of pack
        pack_voltage: Voltage,

        /// State of Charge (SoC) of the pack.
        soc: Percentage,
    }
    impl Analyzer {
        /// Creates a new Analyzer with blank data.
        pub fn new() -> Self {
            Self {
                max_temp: CriticalCellValue {
                    value: Temperature::new::<degree_celsius>(f32::MIN),
                    chip: ChipId::Chip0,
                    cell: CellId::Cell1,
                },
                min_temp: CriticalCellValue {
                    value: Temperature::new::<degree_celsius>(f32::MAX),
                    chip: ChipId::Chip0,
                    cell: CellId::Cell1,
                },
                avg_temp: Temperature::new::<degree_celsius>(f32::MIN),

                max_voltage: CriticalCellValue {
                    value: Voltage::new::<volt>(f32::MIN),
                    chip: ChipId::Chip0,
                    cell: CellId::Cell1,
                },
                min_voltage: CriticalCellValue {
                    value: Voltage::new::<volt>(f32::MAX),
                    chip: ChipId::Chip0,
                    cell: CellId::Cell1,
                },
                avg_voltage: Voltage::new::<volt>(f32::MIN),
                delta_voltage: Voltage::new::<volt>(f32::MIN),

                max_ocv: CriticalCellValue {
                    value: Voltage::new::<volt>(f32::MIN),
                    chip: ChipId::Chip0,
                    cell: CellId::Cell1,
                },
                min_ocv: CriticalCellValue {
                    value: Voltage::new::<volt>(f32::MAX),
                    chip: ChipId::Chip0,
                    cell: CellId::Cell1,
                },
                avg_ocv: Voltage::new::<volt>(f32::MIN),
                delta_ocv: Voltage::new::<volt>(f32::MIN),
                pack_ocv: Voltage::new::<volt>(f32::MIN),

                max_chiptemp: CriticalChipValue {
                    value: Temperature::new::<degree_celsius>(f32::MIN),
                    chip: ChipId::Chip0,
                },

                segment_average_temps: IndexBySegment::from_fn(|_| Temperature::new::<degree_celsius>(f32::MIN)),
                segment_average_volts: IndexBySegment::from_fn(|_| Voltage::new::<volt>(f32::MIN)),
                segment_total_volts: IndexBySegment::from_fn(|_| Voltage::new::<volt>(f32::MIN)),
                segment_delta_volts: IndexBySegment::from_fn(|_| Voltage::new::<volt>(f32::MIN)),

                pack_voltage: Voltage::new::<volt>(f32::MIN),

                soc: Percentage::new(0.0_f32),
            }
        }
    }

    impl Analyzer {
        fn calc_pack_temps(&mut self, data: &CacheData) {
            let mut total_temp = 0_f32;
            let mut total_seg_temp = 0_f32;

            let Ok(rdax) = data.get_redundant_aux().try_nice() else {
                return;
            };
            let Ok(stata) = data.get_status_a().try_nice() else {
                return;
            };

            for chip in ChipId::iter() {
                let temps = rdax[chip].to_temps().cell_temperatures;

                for cell in CellId::iter() {
                    if &temps[cell] > self.max_temp.value() {
                        self.max_temp = CriticalCellValue { value: temps[cell], chip, cell }
                    }

                    if &temps[cell] < self.min_temp.value() {
                        self.min_temp = CriticalCellValue { value: temps[cell], chip, cell }
                    }

                    total_temp += temps[cell].get::<degree_celsius>();
                    total_seg_temp += temps[cell].get::<degree_celsius>();
                }

                // Apparently only for NERO according to analyzer.c
                if chip.is_beta() {
                    self.segment_average_temps[chip.segment()] = Temperature::new::<degree_celsius>(total_seg_temp / NUM_CELLS_PER_SEGMENT as f32);
                    total_seg_temp = 0_f32;
                }

                if self.max_chiptemp.value() < &stata[chip].itmp {
                    self.max_chiptemp = CriticalChipValue { value: stata[chip].itmp, chip }
                }
            }

            self.avg_temp = Temperature::new::<degree_celsius>(total_temp / NUM_CELLS_TOTAL as f32);
        }
    }

    impl Analyzer {
        pub fn analyze(&mut self) {
            let cache = crate::segments::cache();

            self.calc_pack_temps(cache);
        }
    }
}
