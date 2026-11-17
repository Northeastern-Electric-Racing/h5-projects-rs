use crate::{
    segments::{CellId, ChipId, IndexByChip, IndexByCell, IndexBySegment, IndexByOnboardTherm, ThermistorTemperatures, NUM_CELLS_TOTAL},
    units::{Temperature, Voltage, Current, Ratio, Resistance, percent, ratio, degree_celsius, volt, ohm},
};

// Only the 25A balancing patch names these, and that is compiled out under `hil`.
#[cfg(not(feature = "hil"))]
use crate::{
    segments::ChipKind,
    state_machine::BmsState,
    units::{Length, ResistancePerLength},
};

pub mod analyzer {
    use super::*;
    use embassy_time::Instant;
    use crate::helpers::startup_cell::{StartupCell, UnwriteableStartupCell};
    use crate::faults;
    use crate::faults::{FaultCommand, PassFailAction};

    // u_TODO - Should this be another file somehwere?
    /// Which measurement to believe, where more than one instrument measures the same thing.
    pub mod selected {
        use embassy_time::{Duration, Instant};
        use crate::helpers::stamped::Stamped;

        use crate::can::reported::{self, Reporter};
        use crate::units::{Current, Voltage};
        use crate::hv_plate;

        /// Several passes of the plate loop. Past this the link is not merely slow.
        #[cfg(not(feature = "hil"))]
        const PLATE_MAX_AGE: Duration = Duration::from_millis(250);
        // The Simulink model responds slower than the chip
        #[cfg(feature = "hil")]
        const PLATE_MAX_AGE: Duration = Duration::from_millis(1_000);

        /// Generous next to any sender's frame rate, so a single dropped frame does not flap the source.
        const REPORTED_MAX_AGE: Duration = Duration::from_millis(1_000);

        /// Where a reading came from.
        // u_TODO: bring in segments as source here
        #[derive(Copy, Clone, PartialEq, Eq)]
        #[derive(defmt::Format)]
        pub enum Source {
            /// The 2950's own measurement.
            Plate,
            /// Reported over CAN by another node.
            Reported(Reporter),
        }

        /// A reading, when it was measured, and which instrument measured it.
        #[derive(Copy, Clone)]
        pub struct Sourced<T> {
            value: Stamped<T>,
            source: Source,
        }

        impl<T: Copy> Sourced<T> {
            /// The value. Already bounded by the selector's per-source staleness limit, which is the
            /// loosest any caller should accept -- tighten it with [`Self::fresh`].
            pub const fn get(&self) -> T {
                self.value.get()
            }

            /// Which instrument measured it.
            pub const fn source(&self) -> Source {
                self.source
            }

            /// When it was measured, for anything differencing two readings or integrating between them.
            /// A plate reading is stamped at the SPI read; a reported one at the frame's arrival, which
            /// lags the sender's own measurement by however long its pipeline is.
            pub const fn at(&self) -> Instant {
                self.value.at()
            }

            /// How old it is now. What the selector's own bound is judged against.
            pub fn age(&self) -> Duration {
                self.value.age()
            }

            /// The value, but only if it is newer than `within`. For a caller stricter than the selector.
            pub fn fresh(&self, within: Duration) -> Option<T> {
                self.value.fresh(within)
            }
        }

        fn plate<T: Copy>(reading: Stamped<T>) -> Sourced<T> {
            Sourced { value: reading, source: Source::Plate }
        }

        fn claimed<T: Copy>(claim: reported::Claim<T>, within: Duration) -> Option<Sourced<T>> {
            claim.value.is_fresh(within).then_some(Sourced { value: claim.value, source: Source::Reported(claim.from) })
        }

        /// Pack current, discharge positive.
        pub fn pack_current() -> Option<Sourced<Current>> {
            hv_plate::instant_current().filter(|reading| reading.is_fresh(PLATE_MAX_AGE)).map(plate).or_else(|| reported::pack_current().and_then(|claim| claimed(claim, REPORTED_MAX_AGE)))
        }

        /// BATT-side voltage. The plate's own divider, failing over to a reported claim.
        pub fn batt_voltage() -> Option<Sourced<Voltage>> {
            hv_plate::instant_batt_voltage().filter(|reading| reading.is_fresh(PLATE_MAX_AGE)).map(plate)
        }

        /// Tractive-system voltage, off the 2950's divider behind GPO2 or whoever else is talking.
        pub fn ts_voltage() -> Option<Sourced<Voltage>> {
            hv_plate::ts_voltage().filter(|reading| reading.is_fresh(PLATE_MAX_AGE)).map(plate).or_else(|| reported::ts_voltage().and_then(|claim| claimed(claim, REPORTED_MAX_AGE)))
        }
    }

    pub use selected::{Source, Sourced};

    /// Holds analyzer data, plus some hopefully useful metadata for readers.
    #[derive(Copy, Clone)]
    pub struct AnalyzerHolder {
        /// The actual analyzer data.
        pub data: Analyzer,
        /// When the analyzer data was last updated.
        pub last_updated: Instant,
    }

    /// The most recently published analyzer data.
    static ANALYZER: StartupCell<AnalyzerHolder> = StartupCell::new();

    /// Gets a handle to the most recently published analyzer data, empty until the analyzer task first publishes.
    pub fn analyzer() -> &'static UnwriteableStartupCell<AnalyzerHolder> {
        ANALYZER.to_unwriteable()
    }

    /// Publishes a copy of `analyzer` for other tasks to read.
    fn update(analyzer: Analyzer) {
        ANALYZER.set(AnalyzerHolder { data: analyzer, last_updated: Instant::now() })
    }

    /// Number of consecutive invalid samples before a thermistor is marked invalid.
    const THERM_FAULT_COUNT_THRESHOLD: u8 = 2;
    /// Max allowed change from the last trusted sample in degrees C
    const TEMP_MAX_DELTA_C: f32 = 5.0;
    // from bms_config.h in the C code
    const MIN_TEMP: Temperature = Temperature::from_celsius(-40.0);
    const MAX_CELL_TEMP: Temperature = Temperature::from_celsius(60.0);

    #[derive(Copy, Clone)]
    pub enum ThermState {
        Startup { faults: u8 },
        Trusted { last_temp: Temperature, faults: u8 },
        Invalid,
    }
    impl ThermState {
        const fn new() -> Self {
            ThermState::Startup { faults: 0 }
        }

        /// Update `ThermState` with `temp`. Returns `true` if `temp` was accepted.
        fn update(&mut self, temp: Temperature) -> bool {
            let in_range = (temp >= MIN_TEMP) && (temp <= MAX_CELL_TEMP);
            match self {
                Self::Startup { faults } => {
                    if in_range {
                        *self = Self::Trusted { last_temp: temp, faults: 0 };
                        true
                    } else {
                        *faults += 1;
                        if *faults >= THERM_FAULT_COUNT_THRESHOLD {
                            *self = Self::Invalid;
                        }
                        false
                    }
                },

                Self::Trusted { last_temp, faults } => {
                    let delta = temp.get::<degree_celsius>() - last_temp.get::<degree_celsius>();
                    if in_range && delta <= TEMP_MAX_DELTA_C && delta >= -TEMP_MAX_DELTA_C {
                        *last_temp = temp;
                        *faults = 0;
                        true
                    } else {
                        *faults += 1;
                        if *faults >= THERM_FAULT_COUNT_THRESHOLD {
                            *self = Self::Invalid;
                        }
                        false
                    }
                },

                Self::Invalid => false,
            }
        }

        pub const fn is_invalid(&self) -> bool {
            matches!(self, Self::Invalid)
        }
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
        pub const fn new(value: T, chip: ChipId, cell: CellId) -> Self {
            Self { value, chip, cell }
        }
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

    /// 6 consoles 10 computers
    #[derive(Copy, Clone)]
    #[allow(dead_code)]
    pub struct Analyzer {
        pub corrected_cell_voltages: IndexByChip<IndexByCell<Voltage>>,

        pub cell_resistance: IndexByChip<IndexByCell<Resistance>>,
        pub ow_fault: IndexByChip<IndexByCell<bool>>,

        // Max, min, and avg thermistor readings.
        pub max_temp: CriticalCellValue<Temperature>,
        pub min_temp: CriticalCellValue<Temperature>,
        pub avg_temp: Temperature,

        // On board temps
        pub on_board_temps: IndexByChip<IndexByOnboardTherm<Temperature>>,

        // Max, min, and avg voltage of the cells
        pub max_voltage: CriticalCellValue<Voltage>,
        pub min_voltage: CriticalCellValue<Voltage>,
        pub avg_voltage: Voltage,
        pub delta_voltage: Voltage,

        /// Positive is discharge. Carries its own instant and instrument.
        pub pack_current: Option<Sourced<Current>>,
        /// Carries its own instant and instrument.
        pub ts_voltage: Option<Sourced<Voltage>>,
        pub soc: Option<Ratio>,

        /// The highest current chip temperature.
        pub max_chiptemp: CriticalChipValue<Temperature>,

        /// Segment temperature averages.
        pub segment_average_temps: IndexBySegment<Temperature>,
        /// Delta voltages for each segment.
        pub segment_delta_volts: IndexBySegment<Voltage>,

        /// Voltage of pack
        pub pack_voltage: Voltage,

        /// Per-cell trust state for the sanitizer.
        pub sanitized_therms: IndexByChip<IndexByCell<ThermState>>,
    }
    impl Analyzer {
        pub fn new() -> Self {
            Self {
                corrected_cell_voltages: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| Voltage::new::<volt>(f32::MIN))),

                cell_resistance: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| Resistance::new::<ohm>(f32::MIN))),
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

                on_board_temps: IndexByChip::from_fn(|_| IndexByOnboardTherm::from_fn(|_| Temperature::new::<degree_celsius>(f32::MIN))),

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

                max_chiptemp: CriticalChipValue {
                    value: Temperature::new::<degree_celsius>(f32::MIN),
                    chip: ChipId::Chip0,
                },

                segment_average_temps: IndexBySegment::from_fn(|_| Temperature::new::<degree_celsius>(f32::MIN)),
                segment_delta_volts: IndexBySegment::from_fn(|_| Voltage::new::<volt>(f32::MIN)),

                pack_voltage: Voltage::new::<volt>(f32::MIN),

                pack_current: None,
                ts_voltage: None,
                soc: None,

                sanitized_therms: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| ThermState::new())),
            }
        }

        /// Rejects thermistor samples that are out of range or jump too far, and tracks min/max over only the trusted ones.
        fn sanitize_temps(&mut self, cell_temps: &IndexByChip<IndexByCell<Temperature>>, die_temps: &IndexByChip<Temperature>) {
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

            let mut total_temp = 0.0_f32;
            let mut valid_temp_count = 0_usize;
            let mut total_seg_temp = 0.0_f32;
            let mut valid_seg_temp_count = 0_usize;

            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    let temp = cell_temps[chip][cell];
                    let therm = &mut self.sanitized_therms[chip][cell];
                    let sample_valid = therm.update(temp);

                    if sample_valid {
                        if temp > self.max_temp.value() {
                            self.max_temp = CriticalCellValue { value: temp, chip, cell }
                        }

                        if temp < self.min_temp.value() {
                            self.min_temp = CriticalCellValue { value: temp, chip, cell }
                        }

                        total_temp += temp.get::<degree_celsius>();
                        total_seg_temp += temp.get::<degree_celsius>();
                        valid_temp_count += 1;
                        valid_seg_temp_count += 1;
                    }
                }

                if chip.is_beta() {
                    if valid_seg_temp_count > 0 {
                        self.segment_average_temps[chip.segment()] = Temperature::new::<degree_celsius>(total_seg_temp / valid_seg_temp_count as f32);
                    }

                    total_seg_temp = 0.0;
                    valid_seg_temp_count = 0;
                }

                let die_temp = die_temps[chip];
                if die_temp > self.max_chiptemp.value() {
                    self.max_chiptemp = CriticalChipValue { value: die_temp, chip };
                }
            }

            if valid_temp_count > 0 {
                self.avg_temp = Temperature::new::<degree_celsius>(total_temp / valid_temp_count as f32);
            }
        }

        /// Doesn't really actually do that much calculating. Basically just moves data into `self.cell_voltages`, choosing the source
        /// register depending on if we are charging or not. Also has to do some post-processing corrections for 25A specifically (since this was in TSECU-Shepherd code).
        /// This doesn't modify anything in the cache itself (since that's raw reads), this just initializes (and post-processes) the Analyzer's cell voltage data.
        fn calc_cell_voltages(&mut self, cell_voltages: &IndexByChip<IndexByCell<Voltage>>) {
            #[cfg(not(feature = "hil"))]
            let state = crate::state_machine::state();

            for chip in ChipId::iter() {
                // Copy cell voltages into struct then patch
                self.corrected_cell_voltages[chip].iter_mut().for_each(|(cell, voltage)| {
                    *voltage = cell_voltages[chip][cell];
                });

                #[cfg(not(feature = "hil"))]
                {
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
                    self.corrected_cell_voltages[chip][cell] += curr_bal * res;
                }
            }
        }

        /// Calculates pack voltage stats.
        pub async fn calc_pack_voltage_stats(&mut self) {
            // Reset the last cycle's critical values.
            self.max_voltage = CriticalCellValue {
                value: Voltage::new::<volt>(f32::MIN),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };
            self.min_voltage = CriticalCellValue {
                value: Voltage::new::<volt>(f32::MAX),
                chip: ChipId::Chip0,
                cell: CellId::Cell1,
            };

            let mut total_volt: Voltage = Voltage::new::<volt>(0_f32);

            static BETA_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    if self.corrected_cell_voltages[chip][cell] > self.max_voltage.value() {
                        self.max_voltage = CriticalCellValue { value: self.corrected_cell_voltages[chip][cell], chip, cell }
                    }

                    if self.corrected_cell_voltages[chip][cell] < self.min_voltage.value() {
                        self.min_voltage = CriticalCellValue { value: self.corrected_cell_voltages[chip][cell], chip, cell }
                    }

                    total_volt += self.corrected_cell_voltages[chip][cell];
                }
                if chip.is_beta() {
                    BETA_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    self.segment_delta_volts[chip.segment()] = self.max_voltage.value() - self.min_voltage.value();
                }
                defmt_monitor::monitor!("AnalyzerDebug/Misc/BETA_COUNT", desc = "Times is_beta() has been true!", "{=u32}", BETA_COUNT.load(core::sync::atomic::Ordering::Relaxed));
            }

            // calculate some voltage stats
            self.avg_voltage = total_volt / (NUM_CELLS_TOTAL as f32);
            self.pack_voltage = total_volt;
            self.delta_voltage = self.max_voltage.value() - self.min_voltage.value();

            // Update chip temp fault
            if self.max_chiptemp.value() > Temperature::from_celsius(60.0) {
                faults::queue(FaultCommand::DieTempMaximumFault(PassFailAction::NotifyBad)).await;
            } else {
                faults::queue(FaultCommand::DieTempMaximumFault(PassFailAction::NotifyOkay)).await;
            }
        }

        pub async fn detect_cell_open_wire(&mut self, _s_voltages_ow_even_on: &IndexByChip<IndexByCell<Voltage>>, _s_voltages_ow_odd_on: &IndexByChip<IndexByCell<Voltage>>) {
            let mut open_wire_fault_active = false;

            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    struct Comparison {
                        excited: Voltage,
                        baseline: Voltage,
                    }

                    // Cell voltage when even cells were excited and odd cells were left normal.
                    let even_excited: Voltage = _s_voltages_ow_even_on[chip][cell];
                    // Cell voltage when odd cells were excited and even cells were left normal.
                    let odd_excited: Voltage = _s_voltages_ow_odd_on[chip][cell];

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
        use crate::{
            can,
            can::{
                types::{CellTemperatures, PackSocStatus, SegmentTemperatures},
            },
            segments::{SEGMENTS_FRESH_DATA_SIGNAL, SEGMENTS_OPENWIRE_RAN_SIGNAL, SegmentId},
            units::degree_celsius,
        };
        use crate::segments;
        #[cfg(defmt_monitor)]
        use crate::units::volt;
        use crate::state_machine::{BmsState};

        // Subscribe to Segments fresh data signal subscription so we are notified when new segments data comes in.
        let mut segments_freshdata_subscription = SEGMENTS_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");
        #[cfg_attr(feature = "hil", allow(unused_mut, unused_variables))]
        let mut segments_openwire_subscription = SEGMENTS_OPENWIRE_RAN_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

        // This is the "working"/persistent analyzer that this task mutates
        // continuously. At the end of every cycle, this gets published to the global
        // ANALYZER that other tasks read/take snapshots of.
        let mut working_analyzer: Option<Analyzer> = None;

        #[allow(unused)]
        let mut analyzer_task_run_count: usize = 0;

        // This block of code assumes that all of these are getting filled by the segments task every cycle. If in the future,
        // someone modifies the segments task to only update some depending on car state, these might sleep this task for a long time
        // if a group in the cache ends up not getting initialized for a while after boot
        let normal_cell_voltages = segments::cache().get_cell_voltages().nice().ready().await;
        let filtered_cell_voltages = segments::cache().get_filtered_cell_voltages().nice().ready().await;
        let redundant_aux = segments::cache().get_redundant_aux().nice().ready().await;
        let status_a = segments::cache().get_status_a().nice().ready().await;
        #[cfg(not(feature = "hil"))]
        let s_voltages_ow_even_on = segments::cache().get_s_voltages_ow_even_on().nice().ready().await;
        #[cfg(not(feature = "hil"))]
        let s_voltages_ow_odd_on = segments::cache().get_s_voltages_ow_odd_on().nice().ready().await;

        loop {
            // Run one loop of this task every time new Segments data arrives.
            segments_freshdata_subscription.wait().await;

            // Get cell voltages depending on if we are charging or not.
            let cell_voltages = match crate::state_machine::state() {
                // If we are charging, get normal cell voltages.
                BmsState::Charging => *normal_cell_voltages.get().data(),
                // If we are not charging, get filtered cell voltages.
                _ => *filtered_cell_voltages.get().data(),
            };

            let analyzer = working_analyzer.get_or_insert_with(|| Analyzer::new());

            let temps: IndexByChip<ThermistorTemperatures> = {
                let rdax = *redundant_aux.get().data();

                IndexByChip::from_fn(|chip| rdax[chip].to_temps())
            };
            let cell_temps = IndexByChip::from_fn(|chip| temps[chip].cell_temperatures);
            let onboard_temps = IndexByChip::from_fn(|chip| temps[chip].on_board_temps);
            analyzer.on_board_temps = onboard_temps;

            let die_temps: IndexByChip<Temperature> = IndexByChip::from_fn(|chip| status_a.get().data()[chip].itmp);

            let soc_data = crate::soc::soc_data();
            // u_TODO - just use the 2950 (swap for 27)
            analyzer.pack_current = selected::pack_current();
            // u_TODO - switch to use 2950 sense for 27A.
            analyzer.ts_voltage = selected::ts_voltage();
            analyzer.soc = soc_data.and_then(|soc| soc.data.state_of_charge);

            // This section mirrors the C code so we can check everything came over correctly.
            analyzer.sanitize_temps(&cell_temps, &die_temps);
            analyzer.calc_cell_voltages(&cell_voltages);
            analyzer.calc_pack_voltage_stats().await;

            // calc_celL_resistances u_TODO - also needs hv_plate so see above
            // we don't need `update_chip_status()` from the C code since all of that stuff is just done by ChipData::new()

            #[cfg(not(feature = "hil"))]
            if segments_openwire_subscription.has_been_signaled() {
                analyzer.detect_cell_open_wire(s_voltages_ow_even_on.get().data(), s_voltages_ow_odd_on.get().data()).await;
            }

            update(*analyzer);

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                let mut count = 0;
                for chip in ChipId::iter() {
                    for cell in CellId::iter() {
                        let therm = &analyzer.sanitized_therms[chip][cell];
                        let invalid = therm.is_invalid();
                        if invalid {
                            count += 1;
                        }
                        defmt_monitor::monitor!(["AnalyzerDebug/ThermState/chip_{=u8}/cell_{=u8}/valid", chip.as_u8(), cell.as_u8()], desc = "Whether the sanitizer currently trusts this thermistor. Once false, it stays false until reboot.", "{=bool}", !invalid)
                    }
                }
                defmt_monitor::monitor!("AnalyzerDebug/ThermState/invalid_count", desc = "Number of thermistors the sanitizer currently does not trust.", "{=u32}", count);
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

                    seg5: {
                        #[cfg(feature = "four-segments")]
                        {
                            0_f32
                        }

                        #[cfg(not(feature = "four-segments"))]
                        {
                            analyzer.segment_average_temps[SegmentId::Segment4].get::<degree_celsius>()
                        }
                    },
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

            // Both signals are scaled 16-bit ints on the wire, so "no estimate" cannot be sent as NaN the way the monitors do.
            let pack_soc = analyzer.soc.map_or(0.0_f32, |soc| soc.get::<ratio>());
            let pack_soc_drift = soc_data.map_or(0.0_f32, |soc| soc.data.drift);
            can::send(PackSocStatus { pack_soc, pack_soc_drift }.as_frame()).await;

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/PackSocStatus/pack_soc", desc = "Pack state of charge, 0.0 to 1.0. NaN until an open-circuit voltage has seeded the count, so a gap in the trace means no estimate rather than an empty pack.", "{=f32}", analyzer.soc.map_or(f32::NAN, |soc| soc.get::<ratio>()));
                defmt_monitor::monitor!("AnalyzerDebug/PackSocStatus/pack_soc_drift", desc = "Gap the last OCV seed found against the coulomb count, in state of charge. Positive means the OCV read higher. NaN until the estimator has published.", "{=f32}", soc_data.map_or(f32::NAN, |soc| soc.data.drift));
                defmt_monitor::monitor!(
                    "AnalyzerDebug/PackSocStatus/soc_age_ms",
                    desc = "Age of the SoC estimate when the analyzer read it, in ms. Should sit near the estimator's publish period; a climbing value means it has stopped publishing and pack_soc is frozen rather than current. NaN until it publishes once.",
                    "{=f32}",
                    soc_data.map_or(f32::NAN, |soc| soc.last_updated.elapsed().as_millis() as f32)
                );
            }

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/Misc/avg_voltage", desc = "Average voltage, in volts.", "{=f32}", analyzer.avg_voltage.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/pack_voltage", desc = "Pack voltage, in volts.", "{=f32}", analyzer.pack_voltage.get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/Misc/delta_voltage", desc = "Delta voltage, in volts.", "{=f32}", analyzer.delta_voltage.get::<volt>());
            }

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                use crate::segments::OnboardThermIndex;

                // Per-cell data.
                for chip in ChipId::iter() {
                    for cell in CellId::iter() {
                        defmt_monitor::monitor!(["AnalyzerDebug/Cells/chip_{=u8}/cell_{=u8}/corrected_voltage", chip.as_u8(), cell.as_u8()], desc = "Cell voltage after the 25A trace/fuse correction, in volts.", "{=f32}", analyzer.corrected_cell_voltages[chip][cell].get::<volt>());
                        defmt_monitor::monitor!(["AnalyzerDebug/Cells/chip_{=u8}/cell_{=u8}/ow_fault", chip.as_u8(), cell.as_u8()], desc = "Whether the last open-wire check flagged this cell.", "{=bool}", analyzer.ow_fault[chip][cell]);
                    }
                }

                // On-board thermistors.
                for chip in ChipId::iter() {
                    let temps = &analyzer.on_board_temps[chip];
                    defmt_monitor::monitor!(["AnalyzerDebug/OnboardTemps/chip_{=u8}/alpha_balancing_bank", chip.as_u8()], desc = "Alpha balancing bank thermistor, in degrees celsius.", "{=f32}", temps[OnboardThermIndex::ThermAlphaBalancingBank].get::<degree_celsius>());
                    defmt_monitor::monitor!(["AnalyzerDebug/OnboardTemps/chip_{=u8}/chips", chip.as_u8()], desc = "Thermistor near the chips, in degrees celsius.", "{=f32}", temps[OnboardThermIndex::ThermChips].get::<degree_celsius>());
                    defmt_monitor::monitor!(["AnalyzerDebug/OnboardTemps/chip_{=u8}/beta_balancing_bank", chip.as_u8()], desc = "Beta balancing bank thermistor, in degrees celsius.", "{=f32}", temps[OnboardThermIndex::ThermBetaBalancingBank].get::<degree_celsius>());
                }

                // Corrected cell voltage max/mins.
                defmt_monitor::monitor!("AnalyzerDebug/CorrectedCellVoltage/high_val", desc = "Value of highest corrected cell voltage, in volts.", "{=f32}", analyzer.max_voltage.value().get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/CorrectedCellVoltage/high_cell", desc = "The cell `high_val` was measured from.", "{}", analyzer.max_voltage.cell());
                defmt_monitor::monitor!("AnalyzerDebug/CorrectedCellVoltage/high_chip", desc = "The chip `high_val` was measured from.", "{}", analyzer.max_voltage.chip());
                defmt_monitor::monitor!("AnalyzerDebug/CorrectedCellVoltage/low_val", desc = "Value of lowest corrected cell voltage, in volts.", "{=f32}", analyzer.min_voltage.value().get::<volt>());
                defmt_monitor::monitor!("AnalyzerDebug/CorrectedCellVoltage/low_cell", desc = "The cell `low_val` was measured from.", "{}", analyzer.min_voltage.cell());
                defmt_monitor::monitor!("AnalyzerDebug/CorrectedCellVoltage/low_chip", desc = "The chip `low_val` was measured from.", "{}", analyzer.min_voltage.chip());

                // Die temps.
                defmt_monitor::monitor!("AnalyzerDebug/ChipTemperature/high_val", desc = "Highest ADBMS6830 die temperature, in degrees celsius.", "{=f32}", analyzer.max_chiptemp.value().get::<degree_celsius>());
                defmt_monitor::monitor!("AnalyzerDebug/ChipTemperature/high_chip", desc = "The chip `high_val` was measured from.", "{}", analyzer.max_chiptemp.chip());

                for segment in SegmentId::iter() {
                    defmt_monitor::monitor!(["AnalyzerDebug/SegmentDeltaVoltages/seg_{=u8}/", segment.as_u8()], desc = "Delta voltage stored for this segment, in volts.", "{=f32}", analyzer.segment_delta_volts[segment].get::<volt>());
                }

                // Pack current and TS voltage, then each source that fed them. Published next to
                // their inputs so a wrong number can be traced to the source that produced it
                // rather than guessing which one won.
                defmt_monitor::monitor!("AnalyzerDebug/Misc/pack_current", desc = "Pack current in amps, positive is discharge. The CAN-reported value if a node has sent one, otherwise the shunt. NaN if neither has a reading.", "{=f32}", analyzer.pack_current.map_or(f32::NAN, |current| current.get().as_amps()));
                defmt_monitor::monitor!("AnalyzerDebug/Misc/ts_voltage", desc = "TS voltage in volts, as selected: the CAN-reported value if a node has sent one, otherwise the 2950's own divider. NaN if neither has a reading.", "{=f32}", analyzer.ts_voltage.map_or(f32::NAN, |voltage| voltage.get().get::<volt>()));

                // What the DTI or charger reported, before selection. Nothing clears these, so a
                // node that has gone quiet leaves its last value here -- a steady number is not
                // proof that anything is still sending.
                defmt_monitor::monitor!("AnalyzerDebug/Reported/pack_current", desc = "Pack current as last reported over CAN by the DTI or the charger, in amps. NaN until a frame has arrived.", "{=f32}", crate::can::reported::pack_current().map_or(f32::NAN, |claim| claim.value.get().as_amps()));
                defmt_monitor::monitor!("AnalyzerDebug/Reported/ts_voltage", desc = "TS voltage as last reported over CAN by the DTI or the charger, in volts. NaN until a frame has arrived.", "{=f32}", crate::can::reported::ts_voltage().map_or(f32::NAN, |claim| claim.value.get().get::<volt>()));

                // The 2950's own readings, for cross-checking the reported ones against the plate.
                defmt_monitor::monitor!("AnalyzerDebug/Plate/ts_voltage", desc = "TS voltage off the 2950's own divider behind GPO2, in volts. NaN until the voltages group has been read.", "{=f32}", crate::hv_plate::ts_voltage().map_or(f32::NAN, |ts| ts.get().get::<volt>()));
                defmt_monitor::monitor!("AnalyzerDebug/Plate/shunt_current", desc = "Pack current off the 2950's shunt, in amps, discharge-positive. NaN until the estimator has published.", "{=f32}", crate::soc::soc_data().and_then(|soc| soc.data.shunt_current).map_or(f32::NAN, |current| current.as_amps()));
            }

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                analyzer_task_run_count += 1;
                defmt_monitor::monitor!("AnalyzerDebug/analyzer_task_run_count", desc = "Times this task has run.", "{=usize}", &analyzer_task_run_count);
            }
        }
    }
}

/// The validated values, each at its own source's cadence.
pub use analyzer::selected::{batt_voltage, pack_current, ts_voltage};
pub use analyzer::{Source, Sourced};
