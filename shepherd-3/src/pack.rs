use strum::IntoEnumIterator;

use crate::{
    state_machine::{BmsState}, state_machine,
    segments::{CellId, ChipId, ChipKind, SegmentId, IndexByChip, IndexByCell, IndexBySegment, ThermistorTemperatures, CacheData, NUM_CELLS_PER_SEGMENT, NUM_CELLS_TOTAL},
    units::{Temperature, Voltage, Current, Length, Ratio, Resistance, ResistancePerLength, percent, ratio, degree_celsius, volt, ohm, consts::{from_ohms, from_volts, from_ohms_per_millimeter, from_millimeters, from_amps, from_ratio}},
};
use adbms6830b::chip::registers::pwm::types::PwmDutyCycleConfig;

mod analyzer {
    use super::*;
    use embassy_time::Instant;
    use embassy_sync::{blocking_mutex, blocking_mutex::raw::ThreadModeRawMutex};
    use core::cell::Cell;

    /// Holds analyzer data, plus some hopefully useful metadata for readers.
    #[derive(Copy, Clone)]
    pub struct AnalyzerHolder {
        /// The actual analyzer data.
        pub data: Analyzer,
        /// When the analyzer data was last updated.
        pub last_updated: Instant,
    }

    pub(super) struct Static {
        inner: blocking_mutex::ThreadModeMutex<Cell<Option<AnalyzerHolder>>>,
    }
    impl Static {
        const fn new() -> Self {
            Self { inner: blocking_mutex::ThreadModeMutex::new(Cell::new(None)) }
        }

        /// Copies out the analyzer data.
        fn get(&self) -> Option<AnalyzerHolder> {
            self.inner.lock(|inner| inner.get())
        }
    }

    static ANALYZER: Static = Static::new();

    /// Copies out the analyzer data.
    /// 
    /// If the analyzer data hasn't been updated yet, this returns `None`.
    pub fn analyzer() -> Option<AnalyzerHolder> {
        ANALYZER.get()
    }

    /// Updates the Analyzer stored in the static
    /// with a new Analyzer.
    fn update(analyzer: Analyzer) {
        ANALYZER.inner.lock(|inner| inner.set(
            Some(AnalyzerHolder {
                    data: analyzer,
                    last_updated: Instant::now(),
                })
            )
        )
    }

    #[derive(Copy, Clone)]
    struct CriticalCellValue<T> {
        /// The critical value being stored here.
        value: T,
        /// Chip the critical value was measured from.
        chip: ChipId,
        /// Cell on `chip` that the critical value was measured from.
        cell: CellId,
    }
    impl<T> CriticalCellValue<T> {
        pub const fn value_ref(&self) -> &T {
            &self.value
        }
        pub const fn chip(&self) -> ChipId {
            self.chip
        }
        pub const fn cell(&self) -> CellId {
            self.cell
        }
    }
    impl<T: Copy> CriticalCellValue<T> {
        pub const fn value(&self) -> T {
            self.value
        }
    }

    #[derive(Copy, Clone)]
    struct CriticalChipValue<T> {
        /// The critical value being stored here.
        value: T,
        /// Chip the critical value was measured from.
        chip: ChipId,
    }
    impl<T> CriticalChipValue<T> {
        pub const fn value_ref(&self) -> &T {
            &self.value
        }
        pub const fn chip(&self) -> ChipId {
            self.chip
        }
    }
    impl<T: Copy> CriticalChipValue<T> {
        pub const fn value(&self) -> T {
            self.value
        }
    }

    /// Analyzer's comprehensive view of chip data taken from the cache. This represents data for a single chip.
    /// 
    /// This isn't 100% analgous to the `chipdata_t` struct from TSECU-Shep. This is meant
    /// to be the stuff for Analyzer that can be taken directly from the cache (but doesn't incldue anything it has to calculate itself).
    #[derive(Copy, Clone)]
    struct ChipData {
        pub cell_temp: IndexByCell<Temperature>,
        pub cell_voltages: IndexByCell<Voltage>,
        pub s_cell_voltages: IndexByCell<Voltage>,
        pub s_cell_ow_even_on: IndexByCell<Voltage>,
        pub s_cell_ow_odd_on: IndexByCell<Voltage>,

        pub on_board_temp_1: Temperature,
        pub on_board_temp_2: Temperature,
        pub on_board_temp_3: Temperature, 

        pub die_temp: Temperature,

        pub is_balancing: IndexByCell<bool>,
        pub cs_fault: IndexByCell<bool>,

        pub vpv: Voltage,
        pub vmv: Voltage,
        pub v_res: Voltage,
        pub vref2: Voltage,
        pub v_analog: Voltage,
        pub v_digital: Voltage,
    }
    impl ChipData {
        /// Creates a new `ChipData` with new cache data.
        /// 
        /// If the cache hasn't been updated yet (probably meaning we very recently booted), this
        /// will return Err(()).
        pub fn new() -> Result<IndexByChip<Self>, ()> {
            let cache = crate::segments::cache();
            
            let redundant_aux = cache.get_redundant_aux().try_nice()?;
            let aux = cache.get_aux().try_nice()?;
            let status_a = cache.get_status_a().try_nice()?;
            let status_b = cache.get_status_b().try_nice()?;
            let status_c = cache.get_status_c().try_nice()?;
            
            // Get cell voltagse, either from `cell_voltages` or `filtered_cell_voltages` depending on if we're charging or not
            let cell_voltages: IndexByChip<IndexByCell<Voltage>> = match state_machine::bms_state() {
                BmsState::Charging => { 
                    let Ok(data) = cache.get_cell_voltages().try_nice() else { return Err(()); };
                    data.into()
                },

                _ => {
                    let Ok(data) = cache.get_filtered_cell_voltages().try_nice() else { return Err(()); };
                    data.into()
                }
            };

            let s_voltages: IndexByChip<IndexByCell<Voltage>> = cache.get_s_voltages().try_nice()?.into();
            let s_voltages_ow_even_on: IndexByChip<IndexByCell<Voltage>> = cache.get_s_voltages_ow_even_on().try_nice()?.into();
            let s_voltages_ow_odd_on: IndexByChip<IndexByCell<Voltage>> = cache.get_s_voltages_ow_odd_on().try_nice()?.into();
            let pwm: IndexByChip<IndexByCell<PwmDutyCycleConfig>> = cache.get_pwm().try_nice()?.into();

            Ok(IndexByChip::from_fn(|chip| {
                let temps = redundant_aux[chip].to_temps();

                Self {
                    cell_temp: temps.cell_temperatures,
                    cell_voltages: cell_voltages[chip],
                    s_cell_voltages: s_voltages[chip],
                    s_cell_ow_even_on: s_voltages_ow_even_on[chip],
                    s_cell_ow_odd_on: s_voltages_ow_odd_on[chip],
                    on_board_temp_1: temps.on_board_temp_1,
                    on_board_temp_2: temps.on_board_temp_2,
                    on_board_temp_3: temps.on_board_temp_3,
                    die_temp: status_a[chip].itmp,
                    is_balancing: pwm[chip].map_ref(|cfg| cfg.is_balancing()),
                    cs_fault: status_c[chip].cell_channel_comparison_faults.map_ref(|cfg| cfg.is_set()),
                    vpv: aux[chip].vpv,
                    vmv: aux[chip].vmv,
                    v_res: status_b[chip].vres,
                    vref2: status_a[chip].vref2,
                    v_analog: status_b[chip].va,
                    v_digital: status_b[chip].vd,
                }
            }))
        }
    }

    /// 6 consoles 10 computers
    #[derive(Copy, Clone)]
    struct Analyzer {
        /// Data directly from the ADBMS6830B chips.
        chip_data: IndexByChip<ChipData>,

        // Stuff that was on `chipdata_t` in the C code but wasn't moved over to `ChipData` in the Rust code
        // because it is a calculated value rather than something taken directly from the chips
        cell_resistance: IndexByChip<IndexByCell<Resistance>>,
        open_cell_voltage: IndexByChip<IndexByCell<Voltage>>,
        ow_fault: IndexByChip<IndexByCell<bool>>,

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
        soc: Ratio,
    }
    impl Analyzer {
        /// Creates a new Analyzer with current cache data, and blank data for all the
        /// derived values.
        /// 
        /// If the cache hasn't been updated yet, this returns Err(()).
        pub fn new() -> Result<Self, ()> {
            Ok(Self {
                chip_data: ChipData::new()?,

                cell_resistance: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| Resistance::new::<ohm>(f32::MIN))),
                open_cell_voltage: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| Voltage::new::<volt>(f32::MIN))),
                ow_fault: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| false)),

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

                soc: Ratio::new::<ratio>(0.0_f32),
            })
        }
    }

    impl Analyzer {
        fn calc_pack_temps(&mut self) {
            let mut total_temp = 0_f32;
            let mut total_seg_temp = 0_f32;

            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    let temp: Temperature = self.chip_data[chip].cell_temp[cell];

                    if temp > self.max_temp.value() {
                        self.max_temp = CriticalCellValue { value: temp, chip, cell }
                    }

                    if temp < self.min_temp.value() {
                        self.min_temp = CriticalCellValue { value: temp, chip, cell }
                    }

                    total_temp += temp.get::<degree_celsius>();
                    total_seg_temp += temp.get::<degree_celsius>();
                }

                // Apparently only for NERO according to analyzer.c
                if chip.is_beta() {
                    self.segment_average_temps[chip.segment()] = Temperature::new::<degree_celsius>(total_seg_temp / NUM_CELLS_PER_SEGMENT as f32);
                    total_seg_temp = 0_f32;
                }

                let die_temp: Temperature = self.chip_data[chip].die_temp;
                
                if self.max_chiptemp.value() < die_temp {
                    self.max_chiptemp = CriticalChipValue { value: die_temp, chip }
                }
            }

            self.avg_temp = Temperature::new::<degree_celsius>(total_temp / NUM_CELLS_TOTAL as f32);
        }

        /// Doesn't really actually do that much calculating. Basically just moves data into `self.cell_voltages`, choosing the source
        /// register depending on if we are charging or not. Also has to do some post-processing corrections for 25A specifically (since this was in TSECU-Shepherd code).
        /// This doesn't modify anything in the cache itself (since that's raw reads), this just initializes (and post-processes) the Analyzer's cell voltage data.
        fn calc_cell_voltages(&mut self) {
            let state = state_machine::bms_state();

            for chip in ChipId::iter() {
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
                self.chip_data[chip].cell_voltages[cell] += curr_bal * res;
            }
        }

        /// Calculates pack voltage stats.
        /// 
        /// ### WARNING
        /// This should be called after `open_cell_voltage` has been initialized with actual stuff.
        pub fn calc_pack_voltage_stats(&mut self) {
            let mut total_volt: Voltage = Voltage::new::<volt>(0_f32);
            let mut total_ocv: Voltage = Voltage::new::<volt>(0_f32);
            let mut total_seg_volt: Voltage = Voltage::new::<volt>(0_f32);

            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    if self.chip_data[chip].cell_voltages[cell] > self.max_voltage.value() {
                        self.max_voltage = CriticalCellValue { value: self.chip_data[chip].cell_voltages[cell], chip, cell }
                    }

                    if self.open_cell_voltage[chip][cell] > self.max_ocv.value() {
                        self.max_ocv = CriticalCellValue { value: self.open_cell_voltage[chip][cell], chip, cell }
                    }

                    if self.chip_data[chip].cell_voltages[cell] < self.min_voltage.value() {
                        self.min_voltage = CriticalCellValue { value: self.chip_data[chip].cell_voltages[cell], chip, cell }
                    }

                    if self.open_cell_voltage[chip][cell] < self.max_ocv.value() {
                        self.min_ocv = CriticalCellValue { value: self.open_cell_voltage[chip][cell], chip, cell }
                    }

                    total_volt += self.chip_data[chip].cell_voltages[cell];
                    total_ocv += self.open_cell_voltage[chip][cell];
                    total_seg_volt += self.open_cell_voltage[chip][cell];
                }
                if chip.is_beta() {
                    // calculate average voltage across a segment
                    self.segment_average_volts[chip.segment()] = total_seg_volt / (NUM_CELLS_PER_SEGMENT as f32);
                    self.segment_total_volts[chip.segment()] = total_seg_volt;
                    self.segment_delta_volts[chip.segment()] = self.max_voltage.value() - self.min_voltage.value();
                    total_seg_volt = Voltage::new::<volt>(0_f32);
                }
            }

            // calculate some voltage stats
            self.avg_voltage = total_volt / (NUM_CELLS_TOTAL as f32);
            self.pack_voltage = total_volt;
            self.delta_voltage = self.max_voltage.value() - self.min_voltage.value();
            self.avg_ocv = total_ocv / (NUM_CELLS_TOTAL as f32);
            self.pack_ocv = total_ocv;
            self.delta_ocv = self.max_ocv.value() - self.min_ocv.value();
        }

        pub async fn detect_cell_open_wire(&mut self) {
            use crate::units::consts::ZERO_VOLTS;

            let mut open_wire_fault_active = false;

            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    struct Comparison {
                        excited: Voltage,
                        baseline: Voltage,
                    }

                    // Cell voltage when even cells were excited and odd cells were left normal.
                    let even_excited: Voltage = self.chip_data[chip].s_cell_ow_even_on[cell];
                    // Cell voltage when odd cells were excited and even cells were left normal.
                    let odd_excited: Voltage = self.chip_data[chip].s_cell_ow_odd_on[cell];

                    let depends: Comparison = if cell.is_even() {
                        // This cell is even, so its `excited` voltage comes from when only even cells were excited,
                        // while its `baseline` voltage comes from when only odd cells were excited.
                        Comparison {
                            excited: even_excited,
                            baseline: odd_excited,
                        }
                    } else {
                        // This cell is odd, so its `excited` voltage comes from when only odd cells were excited,
                        // while its `baseline` voltage comes from when only even cells were excited.
                        Comparison {
                            excited: odd_excited,
                            baseline: even_excited,
                        }
                    };

                    let drop: Voltage = depends.baseline - depends.excited;
                    let drop_percent: Ratio = {
                        if depends.baseline > ZERO_VOLTS {
                            drop / depends.baseline
                        } else {
                            Ratio::new::<ratio>(0_f32)
                        }
                    };

                    /// Open-wire threshold while the S-ADC switch is active.
                    const CELL_OPEN_WIRE_MAX_DROP_PERCENT: Ratio = from_ratio(0.15).expect("Invalid Ratio.");

                    let is_open = drop_percent > CELL_OPEN_WIRE_MAX_DROP_PERCENT;
                    self.ow_fault[chip][cell] = is_open;
                    open_wire_fault_active = open_wire_fault_active || is_open;
                    if is_open {
                        defmt::warn!("[OW] Open wire: Chip={}, Cell={}, even_excited={} V, odd_excited={} V, drop={} V, drop_percent={} %", chip, cell, even_excited.get::<volt>(), odd_excited.get::<volt>(), drop.get::<volt>(), drop_percent.get::<percent>());
                    }
                }
            }

            use crate::faults;
            use crate::faults::{FaultCommand, PassFailAction};

            if open_wire_fault_active {
                faults::queue(FaultCommand::CellOpenWireFault(PassFailAction::NotifyBad)).await;
            } else {
                faults::queue(FaultCommand::CellOpenWireFault(PassFailAction::NotifyOkay)).await;
            }
        }
    }

    /// Task that runs and updates the analyzer (to do run some calculations on chip data).
    #[embassy_executor::task]
    pub async fn analyzer_task() {
        use crate::segments::{SEGMENTS_FRESH_DATA_SIGNAL, SEGMENTS_OPENWIRE_RAN_SIGNAL, ChipId, ChipKind, CellId};
        use crate::units::{degree_celsius, volt};
        use crate::can;

        // Subscribe to Segments fresh data signal subscription so we are notified when new segments data comes in.
        let mut segments_freshdata_subscription = SEGMENTS_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");
        let mut segments_openwire_subscription = SEGMENTS_OPENWIRE_RAN_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

        loop {
            // Run one loop of this task every time new Segments data arrives.
            segments_freshdata_subscription.wait().await;

            let Ok(mut analyzer) = Analyzer::new() else { continue; };

            // this whole section is supposed to look pretty similar to the C code just so
            // we make sure we bring everything over correctly.
            // calc_cell_temps() is in the C code, but is not needed here because the chipdata already calculates cell temps.
            analyzer.calc_pack_temps();
            analyzer.calc_cell_voltages();
            // calc_open_cell_voltage u_TODO - probably do this later once full scope of how much hv plate data is needed here is known
            analyzer.calc_pack_voltage_stats();
            // calc_celL_resistances u_TODO - also needs hv_plate so see above

            if segments_openwire_subscription.has_been_signaled() {
                analyzer.detect_cell_open_wire().await;
            }

            update(analyzer);
        }
    }
}