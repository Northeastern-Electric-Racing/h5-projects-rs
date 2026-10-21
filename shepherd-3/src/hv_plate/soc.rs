use crate::hv_plate;
use adbms2950::chip::registers::config_a::types::AccumulatorDepth;
use adbms2950::chip::registers::measurement::accumulation_count;

/// Accumulation depth, written into ConfigA's `ACCI` by `HvPlate::startup`.
pub(in crate::hv_plate) const ACCUMULATOR_DEPTH: AccumulatorDepth = AccumulatorDepth::Samples32;
const ACCN: u16 = accumulation_count(ACCUMULATOR_DEPTH as u8);

const NANOS_PER_MICRO: u32 = 1_000;

/// Handles reading 2950's acc registers - does window bookkeeping, `t_CONV` measurement, restart handling.
mod accumulator {
    use super::{ACCN, NANOS_PER_MICRO, hv_plate};
    use embassy_time::{Duration, Instant};

    const I1CNTPHA_MODULUS: u32 = 1 << 13;
    const PHA_PER_CNT: u32 = 4;

    const CONVERSION_STARTUP_MASK: Duration = Duration::from_micros(adbms2950::line::conversion_times::IXADC_INIT_MAX_MS as u64 * 1_000 + NOMINAL_CONVERSION_NANOS as u64 * ACCN as u64 / 1_000);
    pub const NOMINAL_CONVERSION_NANOS: u32 = adbms2950::line::conversion_times::IXADC_CONVERSION_MS * 1_000_000;

    /// Derived from this equation: `elapsed(in micros) * NANOS_PER_MICRO * PHASE_PER_CONVERSION / ticks`, such that the result doesn't overflow or narrow.
    const MAX_ELAPSED_MICROS: u32 = u32::MAX / (NANOS_PER_MICRO * PHA_PER_CNT);
    const MAX_ELAPSED: Duration = Duration::from_micros(MAX_ELAPSED_MICROS as u64);

    /// Any measured window > ~20% (datasheet says worst case of +- 10%) of nominal isn't measured as drift but a desync.
    const CONVERSION_TOLERANCE_NANOS: u32 = NOMINAL_CONVERSION_NANOS / 5;

    /// How long `I1CNTPHA` has to run before the measurement against it is worth believing.
    /// Jitter in tick will go down 1/x with this duration
    const CONVERSION_TIME_BASELINE: Duration = Duration::from_millis(500);

    /// Registers one FLAG + IVB1ACC read.
    #[derive(Copy, Clone)]
    pub struct Registers {
        i1cnt: u16,
        i1cntpha: u16,
        current_sum_microvolts: i32,
        batt_sum_microvolts: i32,
        read_at: Instant,
    }

    impl Registers {
        pub fn read(flag: &hv_plate::cache::flag::Nice, accumulated: &hv_plate::cache::accumulated::Nice) -> Self {
            Self {
                i1cnt: flag.data().i1cnt,
                i1cntpha: flag.data().i1cntpha,
                current_sum_microvolts: hv_plate::board::discharge_positive(accumulated.data().current_sum_microvolts),
                batt_sum_microvolts: accumulated.data().batt_sum_microvolts,
                read_at: accumulated.oldest_read(),
            }
        }
    }

    /// Data from one accumulator window
    #[derive(Copy, Clone)]
    pub struct Window {
        pub current_sum_microvolts: i32,
        pub batt_sum_microvolts: i32,
        /// How many windows this reading covers. Anything above 1 means we lost some.
        pub windows_elapsed: u16,
        /// Measured time for a single conversion; `t_CONV` is this times [`ACCN`].
        pub conversion_nanos: u32,
    }

    impl Window {
        /// Wall time this reading accounts for.
        pub const fn duration_nanos(&self) -> u64 {
            self.windows_elapsed as u64 * ACCN as u64 * self.conversion_nanos as u64
        }
    }

    pub struct WindowReader {
        /// Readings before this are rejected.
        startup_deadline: Option<Instant>,
        /// Windows consumed so far.
        n: u32,
        i1cnt_old: u16,
        /// `I1CNTPHA` and host time at the start of the open measurement window.
        measurement_base: Option<(u16, Instant)>,
        /// The last measured conversion nanos measured over a full [`CONVERSION_TIME_BASELINE`]
        measured_conversion_nanos: Option<u32>,
        missed_windows: u32,
        desyncs: u32,
    }

    impl WindowReader {
        pub fn new(now: Instant) -> Self {
            let mut accumulator = Self {
                startup_deadline: None,
                n: 0,
                i1cnt_old: 0,
                measurement_base: None,
                measured_conversion_nanos: None,
                missed_windows: 0,
                desyncs: 0,
            };
            accumulator.restart(now);
            accumulator
        }

        /// Re-baselines after an `ADI1`, which zeroes `I1CNT`.
        pub fn restart(&mut self, now: Instant) {
            self.n = 0;
            self.i1cnt_old = 0;
            self.measurement_base = None;
            self.measured_conversion_nanos = None;
            self.startup_deadline = Some(now + CONVERSION_STARTUP_MASK);
        }

        /// `None` while the conversion is still settling, or when the window has not advanced.
        pub fn convert(&mut self, Registers { i1cnt, i1cntpha, current_sum_microvolts, batt_sum_microvolts, read_at }: Registers) -> Option<Window> {
            if let Some(deadline) = self.startup_deadline {
                if read_at < deadline {
                    return None;
                }
                self.startup_deadline = None;
                self.n = u32::from(i1cnt) / u32::from(ACCN) + 1;
                self.i1cnt_old = i1cnt;
                defmt::info!("HvPlate: soc: counters live at I1CNT {}, accumulating.", i1cnt);
            }

            if i1cnt < self.i1cnt_old {
                self.n = 0;
            }
            self.i1cnt_old = i1cnt;

            let i1cnt_windows = u32::from(i1cnt) / u32::from(ACCN);
            if i1cnt_windows < self.n {
                return None;
            }
            let missed = i1cnt_windows - self.n;
            self.n = i1cnt_windows + 1;
            if missed > 0 {
                self.missed_windows = self.missed_windows.saturating_add(missed);
                defmt::warn!("HvPlate: soc: {} window(s) overwritten before the task read them; their charge is approximated from this window's average.", missed);
            }

            Some(Window {
                current_sum_microvolts,
                batt_sum_microvolts,
                windows_elapsed: (missed + 1) as u16,
                conversion_nanos: self.measure_conversion_time(i1cntpha, read_at).unwrap_or(NOMINAL_CONVERSION_NANOS),
            })
        }

        /// `t_CONV` measured against the host clock: `t / dI1CNTPHA * 4`, divided by [`ACCN`].
        fn measure_conversion_time(&mut self, i1cntpha: u16, read_at: Instant) -> Option<u32> {
            let Some((base_pha, base_at)) = self.measurement_base else {
                // We have to init the base measurement to something and skip this cycle
                self.measurement_base = Some((i1cntpha, read_at));
                return None;
            };

            let elapsed = read_at.saturating_duration_since(base_at);
            if elapsed < CONVERSION_TIME_BASELINE {
                // Return previous measured conversion nanos
                return self.measured_conversion_nanos;
            }

            // Reset base and measure conversion nanos
            self.measurement_base = Some((i1cntpha, read_at));

            let mesaured_cntpha = (u32::from(i1cntpha) + I1CNTPHA_MODULUS - u32::from(base_pha)) % I1CNTPHA_MODULUS;

            if mesaured_cntpha == 0 || elapsed > MAX_ELAPSED {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: no usable conversion time: {} phase ticks across {=u64} us.", mesaured_cntpha, elapsed.as_micros());
                return self.measured_conversion_nanos;
            }

            // Bounded by MAX_ELAPSED above, so neither the narrowing nor the product can overflow.
            let elapsed_micros = elapsed.as_micros() as u32;
            let conversion_nanos = (elapsed_micros * NANOS_PER_MICRO * PHA_PER_CNT) / mesaured_cntpha;
            if conversion_nanos.abs_diff(NOMINAL_CONVERSION_NANOS) > CONVERSION_TOLERANCE_NANOS {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: measured a conversion time of {} ns, too far from nominal to believe.", conversion_nanos);
                return self.measured_conversion_nanos;
            }

            self.measured_conversion_nanos = Some(conversion_nanos);
            Some(conversion_nanos)
        }

        /// Should stay at zero. Non-zero means the task is not keeping up.
        pub const fn missed_windows(&self) -> u32 {
            self.missed_windows
        }

        pub const fn desyncs(&self) -> u32 {
            self.desyncs
        }
    }
}

/// Calibrates the shunt as a current sensor
///
/// Everything downstream sees amps, volts and coulombs, so no estimator needs to know the shunt
/// has an offset or that its resistance moves with temperature.
mod shunt {
    use super::accumulator;
    use super::{ACCN, NANOS_PER_MICRO, hv_plate};
    use crate::units::{Current, Temperature, Voltage};
    use uom::si::electric_current::ampere;

    /// u_TODO: worth replacing with a bench measurement of the installed part,
    /// we also can do stuff to get temp coeff but it is tricky with MANGANIN resistor as
    /// the coeff doesn't apply linearly across delta in temp.
    const NOMINAL_NANOOHMS: i64 = hv_plate::board::SHUNT_MICROOHMS * 1_000;
    const SUM_NANOVOLTS_PER_AMP: i64 = ACCN as i64 * NOMINAL_NANOOHMS;

    /// 32 averages the ~9 uV RMS window noise (32 x 0.283 uV avg noise with ACC32) down to 1.00 mA, over ~1.0 s.
    /// A constant offset only has time to show as drift across a long idle, and 1 mA there is 0.05% of pack over 8 hours.
    ///
    /// 82 is the other defensible value: it reaches the 1 uV storage quantum, 0.625 mA, past which
    /// the `i32` of summed microvolts floors the estimate and collecting longer buys nothing at
    /// all. Worth 0.02% of pack, costs ~2.6 s of AIRs-open time instead of ~1.0 s.
    const CALIBRATION_WINDOWS: u32 = 32;

    /// Table 2, conversion noise at a ACC32 conversion time: 0.283 uV RMS.
    /// When we sum over 32 conversions individually we get 0.283 x 32 uV = 9.05 uV RMS.
    const WINDOW_NOISE_SUM_MICROVOLTS: i32 = 9;
    /// The expected spread of [`CALIBRATION_WINDOWS`] samples is ~4 sigma, so 9 will cover that in both directions.
    const CALIBRATION_MAX_SPREAD_SUM_MICROVOLTS: i32 = 9 * WINDOW_NOISE_SUM_MICROVOLTS;

    // u_TODO: tighten once a bench run shows the actual total on `HvPlate/Soc/ShuntOffset`.
    const CALIBRATION_MAX_OFFSET_MILLIAMPS: i64 = 500;
    const CALIBRATION_MAX_OFFSET_SUM_MICROVOLTS: i32 = (CALIBRATION_MAX_OFFSET_MILLIAMPS * SUM_NANOVOLTS_PER_AMP / 1_000_000) as i32;
    const CALIBRATION_MAX_REFERENCE_AMPS: f32 = 0.15;

    /// One window, corrected and in physical units. What every estimator consumes.
    #[derive(Copy, Clone)]
    pub struct Measurement {
        /// Mean current across the window. Positive is discharge.
        pub shunt_current: Current,
        /// Mean pack voltage across the same window, off the BATT divider.
        pub pack_voltage: Voltage,
        pub charge_microcoulombs: i64,
        pub duration_nanos: u64,
        /// How many windows this covers. 1 is clean.
        pub windows_elapsed: u16,
    }

    /// A calibration measurement in progress.
    struct Run {
        sum: i64,
        windows: u32,
        min: i32,
        max: i32,
    }

    pub struct Calibrator {
        /// Subtracted from every window's sum. Zero until a run is accepted.
        offset_sum_microvolts: i32,
        offset_taken_at: Option<Temperature>,
        run: Option<Run>,
        runs_accepted: u32,
        runs_rejected: u32,
    }

    impl Calibrator {
        pub const fn new() -> Self {
            Self {
                offset_sum_microvolts: 0,
                offset_taken_at: None,
                run: None,
                runs_accepted: 0,
                runs_rejected: 0,
            }
        }

        /// Offers a window to the calibration routine. `zero_current` is essentially only when the AIR's are open.
        pub fn observe(&mut self, window: &accumulator::Window, contacts_open: bool, reference_current: Option<Current>, temp: Temperature) {
            let zero_current = contacts_open && reference_current.is_some_and(|current| current.get::<ampere>().abs() < CALIBRATION_MAX_REFERENCE_AMPS);

            if !zero_current {
                self.abort();
                return;
            }

            let (sum, windows, min, max) = {
                let run = self.run.get_or_insert(Run { sum: 0, windows: 0, min: i32::MAX, max: i32::MIN });

                run.sum += i64::from(window.current_sum_microvolts);
                run.windows += 1;
                run.min = run.min.min(window.current_sum_microvolts);
                run.max = run.max.max(window.current_sum_microvolts);

                (run.sum, run.windows, run.min, run.max)
            };

            if windows < CALIBRATION_WINDOWS {
                return;
            }

            self.run = None;
            let offset = (sum / i64::from(windows)) as i32;
            let spread = max - min;

            if offset.abs() > CALIBRATION_MAX_OFFSET_SUM_MICROVOLTS || spread > CALIBRATION_MAX_SPREAD_SUM_MICROVOLTS {
                self.runs_rejected = self.runs_rejected.saturating_add(1);
                defmt::warn!("HvPlate: soc: rejected a shunt calibration: offset {} uV, spread {} uV over {} windows.", offset, spread, windows);
                return;
            }

            self.runs_accepted = self.runs_accepted.saturating_add(1);
            self.offset_sum_microvolts = offset;
            self.offset_taken_at = Some(temp);
            defmt::info!("HvPlate: soc: shunt offset {} uV over {} windows, spread {} uV.", offset, windows, spread);
        }

        /// Drops any run in progress. The AIRs closing or the chip restarting both invalidate one.
        pub fn abort(&mut self) {
            self.run = None;
        }

        pub fn correct(&self, window: &accumulator::Window) -> Measurement {
            let corrected = window.current_sum_microvolts - self.offset_sum_microvolts;
            let duration_nanos = window.duration_nanos();

            Measurement {
                shunt_current: Current::new::<ampere>(corrected as f32 * NANOS_PER_MICRO as f32 / SUM_NANOVOLTS_PER_AMP as f32),
                pack_voltage: hv_plate::board::batt_voltage(window.batt_sum_microvolts / i32::from(ACCN)),
                charge_microcoulombs: i64::from(corrected) * duration_nanos as i64 / SUM_NANOVOLTS_PER_AMP,
                duration_nanos,
                windows_elapsed: window.windows_elapsed,
            }
        }

        /// Whether a calibration run has been accepted.
        /// Until it has, `correct` doesn't actually correct.
        pub const fn is_calibrated(&self) -> bool {
            self.offset_taken_at.is_some()
        }

        pub const fn offset_sum_microvolts(&self) -> i32 {
            self.offset_sum_microvolts
        }

        pub fn offset_taken_at(&self) -> Option<Temperature> {
            self.offset_taken_at
        }

        pub const fn runs_accepted(&self) -> u32 {
            self.runs_accepted
        }

        pub const fn runs_rejected(&self) -> u32 {
            self.runs_rejected
        }
    }
}

mod soc_estimation {
    use super::shunt::Measurement;
    use crate::units::Voltage;
    use embassy_time::Instant;

    /// 130s3p of 5 Ah cells, in microcoulombs.
    const CAPACITY_MICROCOULOMBS: i64 = 5 * 3 * 3600 * 1_000_000;

    fn soc_from_ocv(ocv: Voltage) -> Option<f32> {
        use uom::si::electric_potential::volt;

        let v = ocv.get::<volt>();
        if !(2.5..=4.2).contains(&v) {
            return None;
        }

        let soc = ((((-0.0179218 * v + 0.0830236) * v + 0.905277) * v - 7.43367) * v + 18.7306) * v - 16.0039;
        Some(soc.clamp(0.0, 1.0))
    }

    /// An `soc` reading taken when the count stood at `microcoulombs`.
    #[derive(Copy, Clone)]
    struct Reference {
        soc: f32,
        microcoulombs: i64,
    }

    impl Reference {
        fn soc_at(&self, microcoulombs: i64) -> f32 {
            let moved = microcoulombs - self.microcoulombs;
            (self.soc - moved as f32 / CAPACITY_MICROCOULOMBS as f32).clamp(0.0, 1.0)
        }
    }

    /// Does plain coulomb counting referenced to an open-circuit voltage to estimate SOC.
    pub struct CoulombCounter {
        cc_microcoulombs: i64,
        reference: Option<Reference>,
        state_of_charge: Option<f32>,
        drift: f32,
        seeded_from: Option<Instant>,
    }

    impl CoulombCounter {
        pub const fn new() -> Self {
            Self {
                cc_microcoulombs: 0,
                reference: None,
                state_of_charge: None,
                drift: 0.0,
                seeded_from: None,
            }
        }

        pub fn accumulate(&mut self, measurement: &Measurement) {
            self.cc_microcoulombs += measurement.charge_microcoulombs;
            self.recalculate();
        }

        /// Latches a [`Reference`] at the current total.
        pub fn seed(&mut self, min_cell_ocv: Voltage, settled_at: Instant) {
            if self.seeded_from == Some(settled_at) {
                return;
            }

            let Some(soc) = soc_from_ocv(min_cell_ocv) else {
                defmt::warn!("HvPlate: soc: rejected an OCV seed outside the curve's valid range.");
                return;
            };

            // `drift = coulomb_counting - from_ocv`, so positive means the count is reading higher than the pack really is.
            self.drift = self.state_of_charge.map_or(0.0, |reported| reported - soc);
            self.seeded_from = Some(settled_at);
            self.reference = Some(Reference { soc, microcoulombs: self.cc_microcoulombs });
            self.recalculate();

            let age_millis = Instant::now().saturating_duration_since(settled_at).as_millis();
            defmt::info!("HvPlate: soc: referenced at {} from an OCV taken {} ms ago; drift {}.", soc, age_millis, self.drift);
        }

        fn recalculate(&mut self) {
            self.state_of_charge = self.reference.map(|reference| reference.soc_at(self.cc_microcoulombs));
        }

        /// Net charge since boot. Positive is discharge. Never reset
        pub const fn cc_microcoulombs(&self) -> i64 {
            self.cc_microcoulombs
        }

        /// `None` until an OCV has seeded the count.
        pub const fn state_of_charge(&self) -> Option<f32> {
            self.state_of_charge
        }

        /// Positive means the coulomb count was reading higher than the OCV.
        pub const fn drift(&self) -> f32 {
            self.drift
        }
    }
}

/// Api used for communication with analyzer
pub(super) mod api {
    use crate::helpers::snapshot_cell::SnapshotCell;
    use crate::units::{Current, Ratio};

    #[derive(Copy, Clone)]
    pub(crate) struct SocData {
        /// Pack current as the 2950's accumulator measured it. Positive is discharge.
        pub shunt_current: Option<Current>,
        /// State of charge, 0..1. `None` until an open-circuit voltage has seeded the count.
        pub state_of_charge: Option<Ratio>,
        /// Positive means the coulomb count was reading higher than the OCV.
        pub drift: f32,
    }

    impl SocData {
        const fn new() -> Self {
            Self { shunt_current: None, state_of_charge: None, drift: 0.0 }
        }
    }

    static SOC_DATA: SnapshotCell<SocData> = SnapshotCell::new(SocData::new());

    /// Copies out what the estimator currently knows.
    pub(crate) fn soc_data() -> SocData {
        SOC_DATA.take_snapshot()
    }

    /// Publishes what the estimator currently knows. Called only by [`super::task`].
    pub(super) fn store(data: SocData) {
        SOC_DATA.set(data);
    }
}

mod task {
    use super::accumulator::{WindowReader, Registers};
    use super::soc_estimation;
    use super::shunt::{Measurement, Calibrator};
    use crate::hv_plate;
    use crate::units::{degree_celsius, volt};
    use uom::si::electric_current::ampere;
    use embassy_time::{Duration, Instant};

    const DIAGNOSTIC_PERIOD: Duration = Duration::from_millis(500);

    #[embassy_executor::task]
    pub async fn soc_task() {
        use crate::helpers::deadline::Deadline;
        use crate::hv_plate::core::task::signal::ACC_WINDOW_SIGNAL;

        let mut acc_reader = WindowReader::new(Instant::now());
        let mut shunt_calibrator = Calibrator::new();
        let mut cc_estimator = soc_estimation::CoulombCounter::new();
        let mut latest_measurement: Option<Measurement> = None;

        let Some(mut acc_data_recv) = ACC_WINDOW_SIGNAL.receiver() else {
            defmt::error!("HvPlate: soc: no acc data receiver left; coulomb counting will not run!");
            return;
        };

        let flag = hv_plate::cache().get_flag().nice().ready().await;
        let accumulated = hv_plate::cache().get_accumulated().nice().ready().await;
        let voltages = hv_plate::cache().get_voltages().nice().ready().await;

        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();
        loop {
            // Period of ~25ms (how often we read acc data registers)
            let acc_window_status = acc_data_recv.changed().await;

            // Read registers
            let acc_registers = acc_window_status.fresh.then(|| Registers::read(&flag.get(), &accumulated.get()));
            let shunt_temp = voltages.get().data().shunt_temperature;

            // u_TODO - once Blake's PR lands these can be grouped I think
            let ocv_seed = crate::pack::analyzer::analyzer().and_then(|analyzer| analyzer.data.last_settled_at.map(|settled_at| (analyzer.data.min_ocv.value(), settled_at)));
            let reference_current = crate::pack::analyzer::analyzer().and_then(|analyzer| analyzer.data.pack_current);

            // Open AIRs mean we can do calibration, cross-checked against the reported current
            let contacts_open = !hv_plate::precharge::relay_should_be_closed();

            // Reset state in acc register reader and shunt calibrator
            if let Some(at) = acc_window_status.chip_restarted_at {
                acc_reader.restart(at);
                shunt_calibrator.abort();
            }

            // Accumulate acc_registers
            let window = acc_registers.and_then(|regs| acc_reader.convert(regs));

            // Seed estimator with OCV values if available
            if let Some((min_ocv, settled_at)) = ocv_seed {
                cc_estimator.seed(min_ocv, settled_at);
            }

            // Step the estimator
            if let Some(window) = window {
                shunt_calibrator.observe(&window, contacts_open, reference_current, shunt_temp);

                let measurement = shunt_calibrator.correct(&window);

                // Estimators wait for an accepted calibration
                if shunt_calibrator.is_calibrated() {
                    cc_estimator.accumulate(&measurement);
                }

                latest_measurement = Some(measurement);
            }

            // Store
            super::api::store(super::api::SocData {
                shunt_current: latest_measurement.map(|measurement| measurement.shunt_current),
                state_of_charge: cc_estimator.state_of_charge().and_then(crate::units::Ratio::from_ratio),
                drift: cc_estimator.drift(),
            });

            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt_monitor::monitor!("HvPlate/Soc/MissedWindows", desc = "Accumulator windows overwritten before the task read them. Should stay at zero; non-zero means the 25 ms budget is blown.", "{=u32}", acc_reader.missed_windows());
                defmt_monitor::monitor!("HvPlate/Soc/Desyncs", desc = "Observations rejected because I1CNT could have lapped (a gap over ~2 s).", "{=u32}", acc_reader.desyncs());
                defmt_monitor::monitor!("HvPlate/Soc/ShuntOffset", desc = "Calibrated zero-current offset on the summed window, in microvolts. Datasheet bound is +/-32 uV across ACCN=32.", "{=i32}", shunt_calibrator.offset_sum_microvolts());
                defmt_monitor::monitor!("HvPlate/Soc/ShuntOffsetTemp", desc = "Shunt temperature when the offset was taken, in celsius. NaN until a calibration run is accepted.", "{=f32}", shunt_calibrator.offset_taken_at().map_or(f32::NAN, |temp| temp.get::<degree_celsius>()));
                defmt_monitor::monitor!("HvPlate/Soc/CalibrationsAccepted", desc = "Shunt calibration runs that passed the offset and spread checks.", "{=u32}", shunt_calibrator.runs_accepted());
                defmt_monitor::monitor!("HvPlate/Soc/CalibrationsRejected", desc = "Shunt calibration runs thrown out. Persistent rejections with the AIRs commanded open suggest real current, i.e. a welded contactor.", "{=u32}", shunt_calibrator.runs_rejected());

                if let Some(measurement) = latest_measurement {
                    defmt_monitor::monitor!("HvPlate/Soc/BattVolts", desc = "Mean pack voltage over the window, off the BATT divider. Compare against analyzer.pack_voltage: that one sums the cells and is the more accurate figure, so the gap here is this divider's tolerance.", "{=f32}", measurement.pack_voltage.get::<volt>());
                    defmt_monitor::monitor!("HvPlate/Soc/AverageCurrent", desc = "Mean shunt current over the most recent accumulator window, in amps. Cross-check against the instantaneous reading from IVB1.", "{=f32}", measurement.shunt_current.get::<ampere>());
                    defmt_monitor::monitor!("HvPlate/Soc/StateOfCharge", desc = "State of charge 0..1. NaN until an open-circuit voltage has seeded the count -- the estimate does not exist yet, rather than being zero.", "{=f32}", cc_estimator.state_of_charge().unwrap_or(f32::NAN));
                    defmt_monitor::monitor!("HvPlate/Soc/SampleCharge", desc = "Charge moved in the most recent accumulator window, in microcoulombs.", "{=i64}", measurement.charge_microcoulombs);
                    defmt_monitor::monitor!("HvPlate/Soc/WindowsElapsed", desc = "Accumulator windows covered by the most recent sample. 1 is clean.", "{=u16}", measurement.windows_elapsed);
                    defmt_monitor::monitor!("HvPlate/Soc/TConvUs", desc = "Measured t_CONV, the accumulator window, in us. Nominally 32000; drift from that is the internal oscillator, specified at +/-10%.", "{=u64}", measurement.duration_nanos / u64::from(measurement.windows_elapsed) / 1_000);
                    defmt_monitor::monitor!("HvPlate/Soc/CC", desc = "The datasheet's CC: net charge moved since boot, in microcoulombs. Positive is discharge. Never reset -- not by a chip restart, not by a reseed.", "{=i64}", cc_estimator.cc_microcoulombs());
                    defmt_monitor::monitor!("HvPlate/Soc/Drift", desc = "Gap the last accepted OCV found against the coulomb count, in state of charge. Positive means the OCV read higher.", "{=f32}", cc_estimator.drift());
                }
            }
        }
    }
}

pub use task::soc_task;
