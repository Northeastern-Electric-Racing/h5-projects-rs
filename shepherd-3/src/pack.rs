use strum::IntoEnumIterator;

use crate::{
    state_machine::{BmsState}, state_machine,
    segments::{CellId, ChipId, ChipKind, SegmentId, IndexByChip, IndexByCell, IndexBySegment, CacheData, NUM_CELLS_PER_SEGMENT, NUM_CELLS_TOTAL},
    units::{Temperature, Voltage, Current, Length, Percentage, Resistance, ResistancePerLength, degree_celsius, volt, consts::{from_ohms, from_millimeters, from_amps}},
};

mod analyzer {
    use uom::si::angle::degree;

    use crate::units::consts::from_ohms_per_millimeter;

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

        /// Current cell voltages. These come from the C-ADC registers when
        /// we are charging, and the Filtered Cell Voltage registers when we
        /// are not charging (aka are in any other state).
        cell_voltages: IndexByChip<IndexByCell<Voltage>>,

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

                cell_voltages: IndexByChip::from_fn(|_| { IndexByCell::from_fn(|_| { Voltage::new::<volt>(f32::MIN) }) }),

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

        /// Doesn't really actually do that much calculating. Basically just moves data into `self.cell_voltages`, choosing the source
        /// register depending on if we are charging or not. Also has to do some post-processing corrections for 25A specifically (since this was in TSECU-Shepherd code).
        /// This doesn't modify anything in the cache itself (since that's raw reads), this just initializes (and post-processes) the Analyzer's cell voltage data.
        fn calc_cell_voltages(&mut self, data: &CacheData) {
            let state = state_machine::bms_state();

            let voltages: IndexByChip<IndexByCell<Voltage>> = match state {
                BmsState::Charging => { 
                    let Ok(data) = data.get_cell_voltages().try_nice() else { 
                        return; 
                    };
                    data.into()
                },

                _ => {
                    let Ok(data) = data.get_filtered_cell_voltages().try_nice() else {
                        return;
                    };
                    data.into()
                }
            };

            for chip in ChipId::iter() {
                // Store the cell voltages from the correct registers depending on if we are charging or not
                for cell in CellId::iter() {
                    self.cell_voltages[chip][cell] = voltages[chip][cell];
                }

                // Constants and comments from TSECU-Shepherd
                // 25A patch only: alpha lowest and beta highest need to be offset correctly
                const UNIT_RES: ResistancePerLength = from_ohms_per_millimeter(0.00139_f32); // from 1/2 oz copper, 0.71mm trace width, 30C
                const TRACE_LEN_ALPHA: Length = from_millimeters(234.56_f32 + 27.23_f32);
                const TRACE_LEN_BETA: Length = from_millimeters(116.36_f32 + 27.431_f32);
                const FUSE_RES: Resistance = from_ohms(0.1637_f32);
                const TRACE_RES_ONBOARD_ALPHA: Resistance = from_ohms(0.015_f32); // ohms, correction offset
                const TRACE_RES_ONBOARD_BETA: Resistance = from_ohms(0.20_f32); // ohms, correction offset

                // The distance times the unit resistance, plus the resistance of the fuse
                let (res, cell): (Resistance, CellId) = match chip.kind() {
                    ChipKind::Beta => {
                        let res: Resistance = (UNIT_RES * TRACE_LEN_BETA) + FUSE_RES + TRACE_RES_ONBOARD_BETA;
                        (res, CellId::Cell13)
                    },
                    ChipKind::Alpha => {
                        let res: Resistance = (UNIT_RES * TRACE_LEN_ALPHA) + FUSE_RES + TRACE_RES_ONBOARD_ALPHA;
                        (res, CellId::Cell1)
                    }
                };

                let curr_bal: Current = match state {
                    // measured on 4/5/2026, the current through the cells when in charging mode single shot C ADCs
			        // redone to be higher 4/8 sans measurement
                    BmsState::Charging => from_amps(0.029_f32),

                    // measured on 4/5/2026, the current through the cells when in active mode continous C/S read compare
                    _ => from_amps(0.031_f32),
                };

                // I*R is the way
                self.cell_voltages[chip][cell] += curr_bal * res;
            }
        }
    }

    impl Analyzer {
        pub fn analyze(&mut self) {
            let cache = crate::segments::cache();

            self.calc_pack_temps(cache);
            self.calc_cell_voltages(cache);
        }
    }
}
