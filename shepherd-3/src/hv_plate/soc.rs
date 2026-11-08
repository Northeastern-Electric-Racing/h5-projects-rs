use crate::hv_plate;

/// Samples each accumulator window sums, from the depth `core` configures the chip with.
const ACCN: u16 = hv_plate::core::ACCUMULATOR_DEPTH.samples();

const NANOS_PER_MICRO: u32 = 1_000;

/// Handles reading 2950's acc registers - does window bookkeeping, `t_CONV` measurement, restart handling.
mod accumulator {
    use super::{ACCN, NANOS_PER_MICRO, hv_plate};
    use adbms2950::chip::registers::flag::Flag;
    use adbms2950::line::conversion_times;
    use embassy_time::{Duration, Instant};

    /// Phase ticks per conversion, as [`Flag::i1cntpha`] packs them.
    const PHA_PER_CNT: u32 = Flag::PHASES_PER_CONVERSION as u32;

    const CONVERSION_STARTUP_MASK: Duration = Duration::from_micros(conversion_times::IXADC_INIT_MAX_MS as u64 * 1_000 + NOMINAL_CONVERSION_NANOS as u64 * ACCN as u64 / 1_000);
    pub const NOMINAL_CONVERSION_NANOS: u32 = conversion_times::IXADC_CONVERSION_NS;

    /// Derived from this equation: `elapsed(in micros) * NANOS_PER_MICRO * PHASE_PER_CONVERSION / ticks`, such that the result doesn't overflow or narrow.
    const MAX_ELAPSED_MICROS: u32 = u32::MAX / (NANOS_PER_MICRO * PHA_PER_CNT);
    const MAX_ELAPSED: Duration = Duration::from_micros(MAX_ELAPSED_MICROS as u64);

    /// Twice the datasheet's worst-case oscillator tolerance. Beyond that a measured window is
    /// taken as a desynced counter rather than real drift.
    const CONVERSION_TOLERANCE_NANOS: u32 = NOMINAL_CONVERSION_NANOS * conversion_times::IXADC_CONVERSION_TOLERANCE_PERCENT * 2 / 100;

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

    #[cfg_attr(not(defmt_monitor), allow(dead_code))]
    impl Registers {
        pub fn read(flag: &hv_plate::cache::flag::Nice, accumulated: &hv_plate::cache::accumulated::Nice) -> Self {
            Self {
                i1cnt: flag.data().i1cnt,
                i1cntpha: flag.data().i1cntpha,
                current_sum_microvolts: accumulated.data().current_sum_microvolts,
                batt_sum_microvolts: accumulated.data().batt_sum_microvolts,
                read_at: accumulated.oldest_read(),
            }
        }

        /// The conversion counter as read. Published per window so a counter that has stalled or
        /// lapped is visible directly, rather than inferred from what it did to the charge.
        pub const fn i1cnt(&self) -> u16 {
            self.i1cnt
        }

        /// The counter's sub-phase. `measure_conversion_time` divides elapsed time by the change
        /// in this, so it is the other half of every `t_CONV` figure.
        pub const fn i1cntpha(&self) -> u16 {
            self.i1cntpha
        }

        /// When this reading was taken, on whichever clock stamped it: the MCU's on a car, the
        /// model's simulation time on a rig.
        pub const fn read_at(&self) -> Instant {
            self.read_at
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

    /// Rejects readings until the conversion has had time to settle after a restart.
    #[derive(Copy, Clone)]
    enum StartupMask {
        /// Waiting for a reading to measure the mask from.
        Unbased,
        /// Readings before this instant are rejected.
        Until(Instant),
        /// The conversion has settled; readings count.
        Elapsed,
    }

    pub struct Reader {
        /// Rejects readings until the conversion has settled.
        startup_mask: StartupMask,
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

    // `missed_windows` and `desyncs` are read only by the `#[cfg(defmt_monitor)]` block in `task`.
    #[cfg_attr(not(defmt_monitor), allow(dead_code))]
    impl Reader {
        pub const fn new() -> Self {
            Self {
                startup_mask: StartupMask::Unbased,
                n: 0,
                i1cnt_old: 0,
                measurement_base: None,
                measured_conversion_nanos: None,
                missed_windows: 0,
                desyncs: 0,
            }
        }

        /// Re-baselines after an `ADI1`, which zeroes `I1CNT`.
        pub fn restart(&mut self) {
            self.n = 0;
            self.i1cnt_old = 0;
            self.measurement_base = None;
            self.measured_conversion_nanos = None;
            self.startup_mask = StartupMask::Unbased;
        }

        /// `None` while the conversion is still settling, or when the window has not advanced.
        pub fn convert(&mut self, Registers { i1cnt, i1cntpha, current_sum_microvolts, batt_sum_microvolts, read_at }: Registers) -> Option<Window> {
            match self.startup_mask {
                StartupMask::Unbased => {
                    self.startup_mask = StartupMask::Until(read_at + CONVERSION_STARTUP_MASK);
                    return None;
                },
                StartupMask::Until(deadline) if read_at < deadline => return None,
                StartupMask::Until(_) => {
                    self.startup_mask = StartupMask::Elapsed;
                    self.n = u32::from(i1cnt) / u32::from(ACCN) + 1;
                    self.i1cnt_old = i1cnt;
                    defmt::info!("HvPlate: soc: counters live at I1CNT {}, accumulating.", i1cnt);
                },
                StartupMask::Elapsed => {},
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

            let measured_cntpha = u32::from(Flag::i1cntpha_delta(base_pha, i1cntpha));

            if measured_cntpha == 0 || elapsed > MAX_ELAPSED {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: no usable conversion time: {} phase ticks across {=u64} us.", measured_cntpha, elapsed.as_micros());
                return self.measured_conversion_nanos;
            }

            // Bounded by MAX_ELAPSED above, so neither the narrowing nor the product can overflow.
            let elapsed_micros = elapsed.as_micros() as u32;
            let conversion_nanos = (elapsed_micros * NANOS_PER_MICRO * PHA_PER_CNT) / measured_cntpha;
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

    /// Table 2, conversion noise at a ACC32 conversion time: 0.283 uV RMS.
    /// When we sum over 32 conversions individually we get 0.283 x 32 uV = 9.05 uV RMS.
    const WINDOW_NOISE_SUM_MICROVOLTS: i32 = 9;
    /// The expected spread of [`CALIBRATION_WINDOWS`] samples is ~4 sigma, so 9 sigmas will cover that in both directions.
    const CALIBRATION_MAX_SPREAD_SUM_MICROVOLTS: i32 = 9 * WINDOW_NOISE_SUM_MICROVOLTS;

    /// 32 averages the ~9 uV RMS window noise down to 1.00 mA over ~1.0 s.
    /// A constant offset only has time to show as drift across a long idle, and 1 mA there is 0.05% of pack over 8 hours.
    ///
    /// 82 is the other defensible value: it reaches the 1 uV storage quantum, 0.625 mA, past which
    /// the `i32` of summed microvolts floors the estimate and collecting longer buys nothing at
    /// all. Worth 0.02% of pack, costs ~2.6 s of AIRs-open time instead of ~1.0 s.
    const CALIBRATION_WINDOWS: u32 = 32;

    // u_TODO: tighten once a bench run shows the actual total on `HvPlate/Soc/ShuntOffset`.
    const CALIBRATION_MAX_OFFSET_MILLIAMPS: i64 = 500;
    const CALIBRATION_MAX_OFFSET_SUM_MICROVOLTS: i32 = (CALIBRATION_MAX_OFFSET_MILLIAMPS * SUM_NANOVOLTS_PER_AMP / 1_000_000) as i32;
    const CALIBRATION_MAX_REFERENCE_AMPS: f32 = 0.15;

    /// One window, corrected and in physical units. What every estimator consumes.
    #[cfg_attr(not(defmt_monitor), allow(dead_code))]
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

    // The offset and run tallies are read only by the `#[cfg(defmt_monitor)]` block in `task`.
    #[cfg_attr(not(defmt_monitor), allow(dead_code))]
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
            let corrected_shunt_reading = window.current_sum_microvolts - self.offset_sum_microvolts;
            let duration_nanos = window.duration_nanos();

            Measurement {
                shunt_current: Current::new::<ampere>((corrected_shunt_reading * NANOS_PER_MICRO as i32) as f32 / SUM_NANOVOLTS_PER_AMP as f32),
                pack_voltage: hv_plate::board::batt_voltage(window.batt_sum_microvolts / i32::from(ACCN)),
                charge_microcoulombs: i64::from(corrected_shunt_reading) * duration_nanos as i64 / SUM_NANOVOLTS_PER_AMP,
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

mod charge {
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

        // Calculated based on cell discharge graph and fit to 5th degree and approximated using Horner Form.
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
    pub struct Counter {
        microcoulombs: i64,
        reference: Option<Reference>,
        state_of_charge: Option<f32>,
        drift: f32,
        seeded_from: Option<Instant>,
    }

    #[cfg_attr(not(defmt_monitor), allow(dead_code))]
    impl Counter {
        pub const fn new() -> Self {
            Self {
                microcoulombs: 0,
                reference: None,
                state_of_charge: None,
                drift: 0.0,
                seeded_from: None,
            }
        }

        pub fn accumulate(&mut self, measurement: &Measurement) {
            self.microcoulombs += measurement.charge_microcoulombs;
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
            self.reference = Some(Reference { soc, microcoulombs: self.microcoulombs });
            self.recalculate();

            let age_millis = Instant::now().saturating_duration_since(settled_at).as_millis();
            defmt::info!("HvPlate: soc: referenced at {} from an OCV taken {} ms ago; drift {}.", soc, age_millis, self.drift);
        }

        fn recalculate(&mut self) {
            self.state_of_charge = self.reference.map(|reference| reference.soc_at(self.microcoulombs));
        }

        /// Net charge since boot. Positive is discharge. Never reset
        pub const fn cc_microcoulombs(&self) -> i64 {
            self.microcoulombs
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
pub mod api {
    use crate::helpers::snapshot_cell::SnapshotCell;
    use crate::units::{Current, Ratio};
    use embassy_time::Instant;

    #[derive(Copy, Clone)]
    pub struct SocData {
        /// Pack current as the 2950's accumulator measured it. Positive is discharge.
        pub shunt_current: Option<Current>,
        /// State of charge, 0..1. `None` until an open-circuit voltage has seeded the count.
        pub state_of_charge: Option<Ratio>,
        /// Positive means the coulomb count was reading higher than the OCV.
        pub drift: f32,
    }

    #[cfg_attr(not(defmt_monitor), allow(dead_code))]
    #[derive(Copy, Clone)]
    pub struct SocDataHolder {
        /// The actual estimator data.
        pub data: SocData,
        /// When the estimator last published.
        pub last_updated: Instant,
    }

    /// The most recently published estimator data.
    static SOC_DATA: SnapshotCell<Option<SocDataHolder>> = SnapshotCell::new(None);

    /// Copies out what the estimator currently knows.
    pub fn soc_data() -> Option<SocDataHolder> {
        SOC_DATA.read()
    }

    /// Publishes what the estimator currently knows. Called only by [`super::task`].
    pub(super) fn store(data: SocData) {
        SOC_DATA.set(Some(SocDataHolder { data, last_updated: Instant::now() }));
    }
}

mod task {
    use super::*;
    #[cfg(defmt_monitor)]
    use crate::helpers::deadline::Deadline;
    use crate::{hv_plate, pack};
    #[cfg(defmt_monitor)]
    use crate::units::{degree_celsius, volt};
    #[cfg(defmt_monitor)]
    use uom::si::electric_current::ampere;
    #[cfg(defmt_monitor)]
    use embassy_time::Duration;

    #[cfg(defmt_monitor)]
    const DIAGNOSTIC_PERIOD: Duration = Duration::from_millis(500);

    #[embassy_executor::task]
    pub async fn soc_task() {
        use hv_plate::core::task::signal::ACC_WINDOW_SIGNAL;
        use pack::analyzer;

        let mut reader = accumulator::Reader::new();
        let mut calibrator = shunt::Calibrator::new();
        let mut counter = charge::Counter::new();

        let mut latest_measurement: Option<shunt::Measurement> = None;
        #[cfg(defmt_monitor)]
        let mut latest_window: Option<accumulator::Window> = None;
        let mut latest_registers: Option<accumulator::Registers> = None;

        let Some(mut acc_data_recv) = ACC_WINDOW_SIGNAL.receiver() else {
            defmt::error!("HvPlate: soc: no acc data receiver left; coulomb counting will not run!");
            return;
        };

        let flag = hv_plate::cache().get_flag().nice().ready().await;
        let accumulated = hv_plate::cache().get_accumulated().nice().ready().await;
        let voltages = hv_plate::cache().get_voltages().nice().ready().await;

        #[cfg(defmt_monitor)]
        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();
        loop {
            // Period of ~25ms (how often we read acc data registers)
            let acc_window_status = acc_data_recv.changed().await;

            // Read registers
            let acc_registers = acc_window_status.fresh.then(|| accumulator::Registers::read(&flag.get(), &accumulated.get()));
            let shunt_temp = voltages.get().data().shunt_temperature;

            // Both are read under one lock so the OCV seed and the reference current always come from the same analyzer publish.
            #[rustfmt::skip]
            let (ocv_seed, reference_current) = analyzer::analyzer()
                .try_partial_get(|inner| {
                    let seed = inner.data.last_settled_at.map(|settled_at| (inner.data.min_ocv.value(), settled_at));
                    (seed, inner.data.pack_current)
                })
                .unwrap_or((None, None));

            // u_TODO - should pull from hv plates understanding of the contacts state (which is in precharge PR for now)
            // Open AIRs mean we can do calibration, cross-checked against the reported current
            let contacts_open = true; // Set to true for HIL testing without precharge or shutdown setup

            // Reset state in acc register reader and shunt calibrator
            if acc_window_status.chip_restarted {
                reader.restart();
                calibrator.abort();
            }

            // Accumulate acc_registers
            let window = acc_registers.and_then(|regs| reader.convert(regs));

            latest_registers = acc_registers.or(latest_registers);

            // Seed estimator with OCV values if available
            if let Some((min_ocv, settled_at)) = ocv_seed {
                counter.seed(min_ocv, settled_at);
            }

            // Step the estimator
            if let Some(window) = window {
                calibrator.observe(&window, contacts_open, reference_current, shunt_temp);

                let measurement = calibrator.correct(&window);

                // Estimators wait for an accepted calibration
                if calibrator.is_calibrated() {
                    counter.accumulate(&measurement);
                }

                latest_measurement = Some(measurement);
                #[cfg(defmt_monitor)]
                {
                    latest_window = Some(window);
                }
            }

            // Store
            api::store(api::SocData {
                shunt_current: latest_measurement.map(|measurement| measurement.shunt_current),
                state_of_charge: counter.state_of_charge().and_then(crate::units::Ratio::from_ratio),
                drift: counter.drift(),
            });

            #[cfg(defmt_monitor)]
            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt_monitor::monitor!("Soc/MissedWindows", desc = "Accumulator windows overwritten before the task read them. Should stay at zero; non-zero means the 25 ms budget is blown.", "{=u32}", reader.missed_windows());
                defmt_monitor::monitor!("Soc/Desyncs", desc = "Observations rejected because I1CNT could have lapped (a gap over ~2 s).", "{=u32}", reader.desyncs());
                #[cfg(feature = "hil")]
                defmt_monitor::monitor!("Soc/HilSimClockUs", desc = "Simulation time the rig last reported over RDHILTIME, in us. Stuck at zero means the model is not answering and no window will ever close.", "{=u64}", hv_plate::hil::sim_time::now().as_micros());
                defmt_monitor::monitor!("Soc/ShuntOffset", desc = "Calibrated zero-current offset on the summed window, in microvolts. Datasheet bound is +/-32 uV across ACCN=32.", "{=i32}", calibrator.offset_sum_microvolts());
                defmt_monitor::monitor!("Soc/ShuntOffsetTemp", desc = "Shunt temperature when the offset was taken, in celsius. NaN until a calibration run is accepted.", "{=f32}", calibrator.offset_taken_at().map_or(f32::NAN, |temp| temp.get::<degree_celsius>()));
                defmt_monitor::monitor!("Soc/CalibrationsAccepted", desc = "Shunt calibration runs that passed the offset and spread checks.", "{=u32}", calibrator.runs_accepted());
                defmt_monitor::monitor!("Soc/CalibrationsRejected", desc = "Shunt calibration runs thrown out. Persistent rejections with the AIRs commanded open suggest real current, i.e. a welded contactor.", "{=u32}", calibrator.runs_rejected());

                if let Some(regs) = latest_registers {
                    defmt_monitor::monitor!("Soc/Window/I1cnt", desc = "Conversion counter as read from FLAG. Climbs by ACCN each window and wraps at 2048; standing still means the chip stopped converting.", "{=u16}", regs.i1cnt());
                    defmt_monitor::monitor!("Soc/Window/I1cntPha", desc = "Sub-phase of the conversion counter, 13 bits with I1CNT. The change in this across two reads is what t_CONV is measured from.", "{=u16}", regs.i1cntpha());
                    defmt_monitor::monitor!("Soc/Window/ReadAtUs", desc = "Timestamp the reading carries, in us. On a rig this is the model's simulation time, so it should track Soc/HilSimClockUs rather than the MCU's uptime.", "{=u64}", regs.read_at().as_micros());
                }

                // A measurement is only present if a window is captured
                if let (Some(window), Some(measurement)) = (latest_window, latest_measurement) {
                    defmt_monitor::monitor!("Soc/Window/CurrentSumUv", desc = "Shunt voltage summed over the window, in uV, discharge positive and before the calibrated offset is taken off.", "{=i32}", window.current_sum_microvolts);
                    defmt_monitor::monitor!("Soc/Window/CorrectedSumUv", desc = "The same sum with the calibrated offset removed. Equal to the raw sum until a calibration run has been accepted.", "{=i32}", window.current_sum_microvolts - calibrator.offset_sum_microvolts());
                    defmt_monitor::monitor!("Soc/Window/BattSumUv", desc = "BATT-tap voltage summed over the window, in uV. Divide by ACCN for the mean tap reading.", "{=i32}", window.batt_sum_microvolts);
                    defmt_monitor::monitor!("Soc/Window/WindowsElapsed", desc = "Windows this reading covers. 1 is clean; more means windows were overwritten before the task read them and their charge is being approximated.", "{=u16}", window.windows_elapsed);
                    defmt_monitor::monitor!("Soc/Window/ConversionNs", desc = "Conversion time behind this window, in ns. Nominally 1000000. Pinned exactly there every window means the measurement was rejected and the nominal fallback is in use -- cross-check Soc/Desyncs.", "{=u32}", window.conversion_nanos);
                    defmt_monitor::monitor!("Soc/Window/DurationUs", desc = "Wall time this window accounts for, in us. Nominally 32000 for one window at ACCN=32.", "{=u64}", window.duration_nanos() / 1_000);
                    defmt_monitor::monitor!("Soc/Window/CurrentAmps", desc = "Mean current across the window, in amps. Positive is discharge.", "{=f32}", measurement.shunt_current.get::<ampere>());
                    defmt_monitor::monitor!("Soc/Window/BattVolts", desc = "Mean pack voltage across the window, off the BATT divider.", "{=f32}", measurement.pack_voltage.get::<volt>());
                    defmt_monitor::monitor!("Soc/Window/ChargeUc", desc = "Charge moved in this window, in microcoulombs. This is what the coulomb count integrates.", "{=i64}", measurement.charge_microcoulombs);
                    defmt_monitor::monitor!("Soc/Window/Calibrated", desc = "Whether a shunt calibration has been accepted. False means CorrectedSumUv is uncorrected and the coulomb count is not running at all.", "{=bool}", calibrator.is_calibrated());
                }

                if let Some(measurement) = latest_measurement {
                    defmt_monitor::monitor!("Soc/Measurement/BattVolts", desc = "Mean pack voltage over the window, off the BATT divider. Compare against analyzer.pack_voltage: that one sums the cells and is the more accurate figure, so the gap here is this divider's tolerance.", "{=f32}", measurement.pack_voltage.get::<volt>());
                    defmt_monitor::monitor!("Soc/Measurement/AverageCurrent", desc = "Mean shunt current over the most recent accumulator window, in amps. Cross-check against the instantaneous reading from IVB1.", "{=f32}", measurement.shunt_current.get::<ampere>());
                    defmt_monitor::monitor!("Soc/Measurement/StateOfCharge", desc = "State of charge 0..1. NaN until an open-circuit voltage has seeded the count -- the estimate does not exist yet, rather than being zero.", "{=f32}", counter.state_of_charge().unwrap_or(f32::NAN));
                    defmt_monitor::monitor!("Soc/Measurement/SampleCharge", desc = "Charge moved in the most recent accumulator window, in microcoulombs.", "{=i64}", measurement.charge_microcoulombs);
                    defmt_monitor::monitor!("Soc/Measurement/WindowsElapsed", desc = "Accumulator windows covered by the most recent sample. 1 is clean.", "{=u16}", measurement.windows_elapsed);
                    defmt_monitor::monitor!("Soc/Measurement/TConvUs", desc = "Measured t_CONV, the accumulator window, in us. Nominally 32000; drift from that is the internal oscillator, specified at +/-10%.", "{=u64}", measurement.duration_nanos / u64::from(measurement.windows_elapsed) / 1_000);
                    defmt_monitor::monitor!("Soc/Measurement/CC", desc = "The datasheet's CC: net charge moved since boot, in microcoulombs. Positive is discharge. Never reset -- not by a chip restart, not by a reseed.", "{=i64}", counter.cc_microcoulombs());
                    defmt_monitor::monitor!("Soc/Measurement/Drift", desc = "Gap the last accepted OCV found against the coulomb count, in state of charge. Positive means the OCV read higher.", "{=f32}", counter.drift());
                }
            }
        }
    }
}

pub use task::soc_task;
