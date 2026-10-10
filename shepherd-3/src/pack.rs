use crate::{
    helpers::deadline::Deadline,
    state_machine::{BmsState},
    state_machine,
    segments::{CellId, ChipId, ChipKind, IndexByChip, IndexByCell, IndexBySegment, NUM_CELLS_PER_SEGMENT, NUM_CELLS_TOTAL},
    units::{Temperature, Voltage, Current, Length, Ratio, Resistance, ResistancePerLength, percent, ratio, degree_celsius, volt, ohm},
};
use adbms6830b::chip::registers::pwm::types::PwmDutyCycleConfig;

pub mod analyzer {
    use super::*;
    use embassy_time::Instant;
    use crate::helpers::snapshot_cell::SnapshotCell;

    /// Holds analyzer data, plus some hopefully useful metadata for readers.
    #[derive(Copy, Clone)]
    pub struct AnalyzerHolder {
        /// The actual analyzer data.
        pub data: Analyzer,
        /// When the analyzer data was last updated.
        pub last_updated: Instant,
    }

    /// The most recently published analyzer data.
    static ANALYZER: SnapshotCell<Option<AnalyzerHolder>> = SnapshotCell::new(None);

    /// Copies out the analyzer data.
    ///
    /// If the analyzer data hasn't been updated yet, this returns `None`.
    pub fn analyzer() -> Option<AnalyzerHolder> {
        ANALYZER.take_snapshot()
    }

    /// Publishes a copy of `analyzer` for other tasks to read.
    fn update(analyzer: Analyzer) {
        ANALYZER.set(Some(AnalyzerHolder { data: analyzer, last_updated: Instant::now() }))
    }

    #[derive(Copy, Clone)]
    pub struct CriticalCellValue<T> {
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

    #[allow(dead_code)]
    #[derive(Copy, Clone)]
    pub struct CriticalChipValue<T> {
        /// The critical value being stored here.
        value: T,
        /// Chip the critical value was measured from.
        chip: ChipId,
    }
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    pub struct ChipData {
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
                    let Ok(data) = cache.get_cell_voltages().try_nice() else {
                        return Err(());
                    };
                    data.into()
                },

                _ => {
                    let Ok(data) = cache.get_filtered_cell_voltages().try_nice() else {
                        return Err(());
                    };
                    data.into()
                },
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
    #[allow(dead_code)]
    pub struct Analyzer {
        /// Data directly from the ADBMS6830B chips.
        pub chip_data: IndexByChip<ChipData>,

        // Stuff that was on `chipdata_t` in the C code but wasn't moved over to `ChipData` in the Rust code
        // because it is a calculated value rather than something taken directly from the chips
        pub cell_resistance: IndexByChip<IndexByCell<Resistance>>,
        pub open_cell_voltage: IndexByChip<IndexByCell<Voltage>>,
        pub ow_fault: IndexByChip<IndexByCell<bool>>,

        // Max, min, and avg thermistor readings.
        pub max_temp: CriticalCellValue<Temperature>,
        pub min_temp: CriticalCellValue<Temperature>,
        pub avg_temp: Temperature,

        // Max, min, and avg voltage of the cells
        pub max_voltage: CriticalCellValue<Voltage>,
        pub min_voltage: CriticalCellValue<Voltage>,
        pub avg_voltage: Voltage,
        pub delta_voltage: Voltage,

        // Max, min, and avg Open Cell Voltage (OCV) readings.
        pub max_ocv: CriticalCellValue<Voltage>,
        pub min_ocv: CriticalCellValue<Voltage>,
        pub avg_ocv: Voltage,
        pub delta_ocv: Voltage,
        pub pack_ocv: Voltage,

        /// Deadline after which `open_cell_voltage` can be updated.
        ocv_timer: Option<Deadline>,
        /// Whether we are still waiting for the first valid cell voltage reading to initialize `open_cell_voltage` with.
        ocv_is_first_reading: bool,

        /// The highest current chip temperature.
        pub max_chiptemp: CriticalChipValue<Temperature>,

        /// Segment temperature averages.
        pub segment_average_temps: IndexBySegment<Temperature>,
        /// Segment OCV average voltages.
        pub segment_average_volts: IndexBySegment<Voltage>,
        /// Total voltages for each segment.
        pub segment_total_volts: IndexBySegment<Voltage>,
        /// Delta voltages for each segment.
        pub segment_delta_volts: IndexBySegment<Voltage>,

        /// Voltage of pack
        pub pack_voltage: Voltage,

        /// State of Charge (SoC) of the pack.
        pub soc: Ratio,
    }
    impl Analyzer {
        /// Creates a new Analyzer with `chip_data`, and blank data for all the
        /// derived values.
        pub fn new(chip_data: IndexByChip<ChipData>) -> Self {
            Self {
                chip_data,

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

                ocv_timer: None,
                ocv_is_first_reading: true,

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
            }
        }
    }

    impl Analyzer {
        fn calc_pack_temps(&mut self) {
            // Reset the last cycle's critical values.
            self.max_temp = CriticalCellValue {
                value: Temperature::new::<degree_celsius>(f32::MIN),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };
            self.min_temp = CriticalCellValue {
                value: Temperature::new::<degree_celsius>(f32::MAX),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };
            self.max_chiptemp = CriticalChipValue {
                value: Temperature::new::<degree_celsius>(f32::MIN),
                chip: ChipId::Chip0,
            };

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
                const UNIT_RES: ResistancePerLength = ResistancePerLength::from_ohms_per_millimeter(0.00139_f32); // from 1/2 oz copper, 0.71mm trace width, 30C
                const TRACE_LEN_ALPHA: Length = Length::from_millimeters(234.56_f32 + 27.23_f32);
                const TRACE_LEN_BETA: Length = Length::from_millimeters(116.36_f32 + 27.431_f32);
                const FUSE_RES: Resistance = Resistance::from_ohms(0.1637_f32);
                const TRACE_RES_ONBOARD_ALPHA: Resistance = Resistance::from_ohms(0.015_f32); // ohms, correction offset
                const TRACE_RES_ONBOARD_BETA: Resistance = Resistance::from_ohms(0.20_f32); // ohms, correction offset

                // The distance times the unit resistance, plus the resistance of the fuse
                let (res, cell): (Resistance, CellId) = match chip.kind() {
                    ChipKind::Beta => {
                        let res = (UNIT_RES * TRACE_LEN_BETA) + FUSE_RES + TRACE_RES_ONBOARD_BETA;
                        (res, CellId::Cell13)
                    },
                    ChipKind::Alpha => {
                        let res = (UNIT_RES * TRACE_LEN_ALPHA) + FUSE_RES + TRACE_RES_ONBOARD_ALPHA;
                        (res, CellId::Cell1)
                    },
                };

                let curr_bal = match state {
                    // measured on 4/5/2026, the current through the cells when in charging mode single shot C ADCs
                    // redone to be higher 4/8 sans measurement
                    BmsState::Charging => Current::from_amps(0.029_f32),

                    // measured on 4/5/2026, the current through the cells when in active mode continous C/S read compare
                    _ => Current::from_amps(0.031_f32),
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
            // Reset the last cycle's critical values.
            self.max_voltage = CriticalCellValue {
                value: Voltage::new::<volt>(f32::MIN),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };
            self.max_ocv = CriticalCellValue {
                value: Voltage::new::<volt>(f32::MIN),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };
            self.min_voltage = CriticalCellValue {
                value: Voltage::new::<volt>(f32::MAX),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };
            self.min_ocv = CriticalCellValue {
                value: Voltage::new::<volt>(f32::MAX),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };

            let mut total_volt: Voltage = Voltage::new::<volt>(0_f32);
            let mut total_ocv: Voltage = Voltage::new::<volt>(0_f32);
            let mut total_seg_volt: Voltage = Voltage::new::<volt>(0_f32);

            static BETA_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

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

                    if self.open_cell_voltage[chip][cell] < self.min_ocv.value() {
                        self.min_ocv = CriticalCellValue { value: self.open_cell_voltage[chip][cell], chip, cell }
                    }

                    total_volt += self.chip_data[chip].cell_voltages[cell];
                    total_ocv += self.open_cell_voltage[chip][cell];
                    total_seg_volt += self.open_cell_voltage[chip][cell];
                }
                if chip.is_beta() {
                    BETA_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    // calculate average voltage across a segment
                    self.segment_average_volts[chip.segment()] = total_seg_volt / (NUM_CELLS_PER_SEGMENT as f32);
                    self.segment_total_volts[chip.segment()] = total_seg_volt;
                    self.segment_delta_volts[chip.segment()] = self.max_voltage.value() - self.min_voltage.value();
                    total_seg_volt = Voltage::new::<volt>(0_f32);
                }
                defmt_monitor::monitor!("AnalyzerDebug/Misc/BETA_COUNT", desc = "Times is_beta() has been true!", "{=u32}", BETA_COUNT.load(core::sync::atomic::Ordering::Relaxed));
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
                        Comparison { excited: even_excited, baseline: odd_excited }
                    } else {
                        // This cell is odd, so its `excited` voltage comes from when only odd cells were excited,
                        // while its `baseline` voltage comes from when only even cells were excited.
                        Comparison { excited: odd_excited, baseline: even_excited }
                    };

                    let drop: Voltage = depends.baseline - depends.excited;
                    let drop_percent: Ratio = { if depends.baseline > Voltage::zero() { drop / depends.baseline } else { Ratio::new::<ratio>(0_f32) } };

                    /// Open-wire threshold while the S-ADC switch is active.
                    const CELL_OPEN_WIRE_MAX_DROP_PERCENT: Ratio = Ratio::from_ratio(0.15).expect("Invalid Ratio.");

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

        fn calc_open_cell_voltage(&mut self) {
            use crate::helpers::deadline::Deadline;
            use embassy_time::{Duration};

            const OCV_CURR_THRESH: Current = Current::from_amps(0.5_f32);
            const OCV_TIMER_DURATION: Duration = Duration::from_millis(1500);

            let Ok(data) = crate::hv_plate::cache().get_current_voltage().try_nice() else {
                return;
            };
            let mut pack_current = data.pack_current;
            let mut update_ocv = false;

            let ocv_update_allowed: bool = {
                pack_current.abs() < OCV_CURR_THRESH
                // u_TODO: && crate::state_machine::charger_output_disabled()
                // u_TODO: && crate::state_machine::balancing_active()
            };

            if self.ocv_is_first_reading {
                let last_cell: Voltage = self.chip_data[ChipId::last()].cell_voltages[CellId::last()];

                if last_cell > Voltage::from_volts(1.0_f32) && last_cell < Voltage::from_volts(5.0_f32) {
                    self.ocv_is_first_reading = false;
                    update_ocv = true;
                }
            }

            if ocv_update_allowed {
                match self.ocv_timer {
                    // Timer is expired so we should update OCV.
                    Some(deadline) if deadline.past() => update_ocv = true,

                    // Timer is still active so keep waiting.
                    Some(_) => {},

                    // No timer, but update is allowed now, so we should start it
                    None => self.ocv_timer = Some(Deadline::expire_in(OCV_TIMER_DURATION)),
                }
            } else {
                self.ocv_timer = None;
            }

            if update_ocv {
                for chip in ChipId::iter() {
                    for cell in CellId::iter() {
                        self.open_cell_voltage[chip][cell] = self.chip_data[chip].cell_voltages[cell];
                    }
                }
            }
        }
    }

    /// Task that runs and updates the analyzer (to do run some calculations on chip data).
    #[embassy_executor::task]
    pub async fn analyzer_task() {
        use crate::{
            can,
            can::{
                types::{CellTemperatures, CellVoltage, PackSocStatus, SegmentAverageVoltages, SegmentTotalVoltages, SegmentTemperatures},
            },
            segments::{SEGMENTS_FRESH_DATA_SIGNAL, SEGMENTS_OPENWIRE_RAN_SIGNAL, SegmentId},
            units::{degree_celsius, volt},
        };

        // Subscribe to Segments fresh data signal subscription so we are notified when new segments data comes in.
        let mut segments_freshdata_subscription = SEGMENTS_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");
        let mut segments_openwire_subscription = SEGMENTS_OPENWIRE_RAN_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

        // This is the "working"/persistent analyzer that this task mutates
        // continuously. At the end of every cycle, this gets published to the global
        // ANALYZER that other tasks read/take snapshots of.
        let mut working_analyzer: Option<Analyzer> = None;

        #[allow(unused)]
        let mut analyzer_task_run_count: usize = 0;

        loop {
            // Run one loop of this task every time new Segments data arrives.
            segments_freshdata_subscription.wait().await;

            let Ok(chip_data) = ChipData::new() else {
                defmt::warn!("pack: analyzer: skipped running `analyzer_task()` because the Cache has not been updated yet. Will try again next loop.");
                continue;
            };
            let analyzer = working_analyzer.get_or_insert_with(|| Analyzer::new(chip_data));
            analyzer.chip_data = chip_data;

            // this whole section is supposed to look pretty similar to the C code just so
            // we make sure we bring everything over correctly.
            // calc_cell_temps() is in the C code, but is not needed here because the chipdata already calculates cell temps.
            analyzer.calc_pack_temps();
            analyzer.calc_cell_voltages();
            analyzer.calc_open_cell_voltage();
            analyzer.calc_pack_voltage_stats();
            // calc_celL_resistances u_TODO - also needs hv_plate so see above
            // we don't need `update_chip_status()` from the C code since all of that stuff is just done by ChipData::new()

            if segments_openwire_subscription.has_been_signaled() {
                analyzer.detect_cell_open_wire().await;
            }

            update(*analyzer);

            // u_TODO - i'm pretty sure we can just move the stuff in `cell_temp_sanitizer.c` directly into the analyzer task. there doesn't seem to be a reason to have it in its own task like the C code. so we should add that here (after the analyzer is done)

            can::send(
                CellVoltage {
                    high_val: analyzer.max_ocv.value().get::<volt>(),
                    high_cell: analyzer.max_ocv.cell().as_u8(),
                    high_chip: analyzer.max_ocv.chip().as_u8(),
                    low_val: analyzer.min_ocv.value().get::<volt>(),
                    low_chip: analyzer.min_ocv.chip().as_u8(),
                    low_cell: analyzer.min_ocv.cell().as_u8(),
                    avg_val: analyzer.avg_ocv.get::<volt>(),
                }
                .as_frame(),
            )
            .await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/high_val", desc = "Value of highest cell voltage, in volts.", "{=f32}", analyzer.max_ocv.value().get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/high_cell", desc = "The cell `high_val` was measured from.", "{}", analyzer.max_ocv.cell());
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/high_chip", desc = "The chip `high_val` was measured from.", "{}", analyzer.max_ocv.chip());
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/low_val", desc = "Value of lowest cell voltage, in volts.", "{=f32}", analyzer.min_ocv.value().get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/low_cell", desc = "The cell `low_val` was measured from.", "{}", analyzer.min_ocv.cell());
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/low_chip", desc = "The chip `low_val` was measured from.", "{}", analyzer.min_ocv.chip());
                defmt_monitor::monitor!("AnalyzerDebug/CellVoltage/avg_val", desc = "The average cell voltage, in volts.", "{=f32}", analyzer.avg_ocv.get::<volt>());
            }

            can::send(
                SegmentAverageVoltages {
                    seg1: analyzer.segment_average_volts[SegmentId::Segment0].get::<volt>(),
                    seg2: analyzer.segment_average_volts[SegmentId::Segment1].get::<volt>(),
                    seg3: analyzer.segment_average_volts[SegmentId::Segment2].get::<volt>(),
                    seg4: analyzer.segment_average_volts[SegmentId::Segment3].get::<volt>(),
                    seg5: analyzer.segment_average_volts[SegmentId::Segment4].get::<volt>(),
                }
                .as_frame(),
            )
            .await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                for segment in SegmentId::iter() {
                    defmt_monitor::monitor!(["AnalyzerDebug/SegmentAverageVoltages/seg_{=u8}/", segment.as_u8()], desc = "Average voltage for this segment, in volts.", "{=f32}", analyzer.segment_average_volts[segment].get::<volt>());
                }
            }

            can::send(
                SegmentTotalVoltages {
                    seg1: analyzer.segment_total_volts[SegmentId::Segment0].get::<volt>(),
                    seg2: analyzer.segment_total_volts[SegmentId::Segment1].get::<volt>(),
                    seg3: analyzer.segment_total_volts[SegmentId::Segment2].get::<volt>(),
                    seg4: analyzer.segment_total_volts[SegmentId::Segment3].get::<volt>(),
                    seg5: analyzer.segment_total_volts[SegmentId::Segment4].get::<volt>(),
                }
                .as_frame(),
            )
            .await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                for segment in SegmentId::iter() {
                    defmt_monitor::monitor!(["AnalyzerDebug/SegmentTotalVoltages/seg_{=u8}/", segment.as_u8()], desc = "Total voltage for this segment, in volts.", "{=f32}", analyzer.segment_total_volts[segment].get::<volt>());
                }
            }

            can::send(
                CellTemperatures {
                    high_val: analyzer.max_temp.value().get::<degree_celsius>(),
                    high_cell: analyzer.max_temp.cell().as_u8(),
                    high_chip: analyzer.max_temp.chip().as_u8(),
                    low_val: analyzer.min_temp.value().get::<degree_celsius>(),
                    low_chip: analyzer.min_temp.chip().as_u8(),
                    low_cell: analyzer.min_temp.cell().as_u8(),
                    avg_val: analyzer.avg_temp.get::<degree_celsius>(),
                }
                .as_frame(),
            )
            .await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/high_val", desc = "Value of highest cell temperature, in degrees celsius.", "{=f32}", analyzer.max_temp.value().get::<degree_celsius>());
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/high_cell", desc = "The cell `high_val` was measured from.", "{}", analyzer.max_temp.cell());
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/high_chip", desc = "The chip `high_val` was measured from.", "{}", analyzer.max_temp.chip());
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/low_val", desc = "Value of lowest cell temperature, in degree celsius.", "{=f32}", analyzer.min_temp.value().get::<degree_celsius>());
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/low_cell", desc = "The cell `low_val` was measured from.", "{}", analyzer.min_temp.cell());
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/low_chip", desc = "The chip `low_val` was measured from.", "{}", analyzer.min_temp.chip());
                defmt_monitor::monitor!("AnalyzerDebug/CellTemperatures/avg_val", desc = "The average cell temperature in degrees celsius.", "{}", analyzer.avg_temp.get::<degree_celsius>());
            }

            can::send(
                SegmentTemperatures {
                    seg1: analyzer.segment_average_temps[SegmentId::Segment0].get::<degree_celsius>(),
                    seg2: analyzer.segment_average_temps[SegmentId::Segment1].get::<degree_celsius>(),
                    seg3: analyzer.segment_average_temps[SegmentId::Segment2].get::<degree_celsius>(),
                    seg4: analyzer.segment_average_temps[SegmentId::Segment3].get::<degree_celsius>(),
                    seg5: analyzer.segment_average_temps[SegmentId::Segment4].get::<degree_celsius>(),
                }
                .as_frame(),
            )
            .await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                for segment in SegmentId::iter() {
                    defmt_monitor::monitor!(["AnalyzerDebug/SegmentTemperatures/seg_{=u8}/", segment.as_u8()], desc = "Temperature for this segment, in degrees celsius.", "{=f32}", analyzer.segment_average_temps[segment].get::<degree_celsius>());
                }
            }

            can::send(
                PackSocStatus {
                    pack_soc: analyzer.soc.get::<ratio>(),
                    pack_soc_drift: f32::MIN, // u_TODO make this real eventually
                }
                .as_frame(),
            )
            .await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/PackSocStatus/pack_soc", desc = "Pack state of charge. This is a ratio/percentage from 0.0 to 1.0", "{=f32}", analyzer.soc.get::<ratio>());
                defmt_monitor::monitor!("AnalyzerDebug/PackSocStatus/pack_soc_drift", desc = "Pack SoC drift. CURRENTLY NOT A REAL VALUE.", "{=f32}", f32::MIN);
            }

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/Misc/avg_voltage", desc = "Average voltage, in volts.", "{=f32}", analyzer.avg_voltage.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/pack_voltage", desc = "Pack voltage, in volts.", "{=f32}", analyzer.pack_voltage.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/delta_voltage", desc = "Delta voltage, in volts.", "{=f32}", analyzer.delta_voltage.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/avg_ocv", desc = "Average Open Cell Voltage, in volts.", "{=f32}", analyzer.avg_ocv.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/pack_ocv", desc = "Pack Open Cell Voltage, in volts.", "{=f32}", analyzer.pack_ocv.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/delta_ocv", desc = "Delta Open Cell Voltage, in volts.", "{=f32}", analyzer.delta_ocv.get::<volt>());
            }

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                analyzer_task_run_count += 1;
                defmt_monitor::monitor!("AnalyzerDebug/analyzer_task_run_count", desc = "Times this task has run.", "{=usize}", &analyzer_task_run_count);
            }
        }
    }
}
