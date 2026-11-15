//! The shunt as a current sensor: accumulator window bookkeeping and zero-offset calibration.

use crate::hv_plate;

const NANOS_PER_MICRO: u32 = 1_000;

/// Samples each accumulator window sums, from the depth `core` configures the chip with.
const ACCN: u16 = hv_plate::core::ACCUMULATOR_DEPTH.samples();

/// Handles reading 2950's acc registers - does window bookkeeping, `t_CONV` measurement, restart handling.
mod accumulator {
    use adbms2950::chip::registers::flag::Flag;
    use adbms2950::line::conversion_times;
    use crate::time::PlantTime;
    use embassy_time::Duration;
    use super::{ACCN, NANOS_PER_MICRO, hv_plate};

    /// Phase ticks per conversion, as [`Flag::i1cntpha`] packs them.
    const PHA_PER_CNT: u32 = Flag::PHASES_PER_CONVERSION as u32;

    const CONVERSION_STARTUP_MASK: Duration = Duration::from_micros(conversion_times::IXADC_INIT_MAX_MS as u64 * 1_000 + NOMINAL_CONVERSION_NANOS as u64 * ACCN as u64 / 1_000);
    pub const NOMINAL_CONVERSION_NANOS: u32 = conversion_times::IXADC_CONVERSION_NS;

    /// Derived from this equation: `elapsed(in micros) * NANOS_PER_MICRO * PHASE_PER_CONVERSION / ticks`, such that the result doesn't overflow or narrow.
    const MAX_ELAPSED_MICROS: u32 = u32::MAX / (NANOS_PER_MICRO * PHA_PER_CNT);
    const MAX_ELAPSED: Duration = Duration::from_micros(MAX_ELAPSED_MICROS as u64);

    /// Twice the datasheet's worst-case oscillator tolerance.
    const CONVERSION_TOLERANCE_NANOS: u32 = NOMINAL_CONVERSION_NANOS * conversion_times::IXADC_CONVERSION_TOLERANCE_PERCENT * 2 / 100;

    /// How long `I1CNTPHA` has to run before the measurement against it is worth believing.
    const CONVERSION_TIME_BASELINE: Duration = Duration::from_millis(500);

    /// Registers one FLAG + IVB1ACC read.
    #[derive(Copy, Clone)]
    pub struct Registers {
        i1cnt: u16,
        i1cntpha: u16,
        current_sum_microvolts: i32,
        batt_sum_microvolts: i32,
        read_at: PlantTime,
    }

    impl Registers {
        /// `None` when the plant clock could not be read, which on a rig means the model has not
        /// answered yet. Everything below measures an interval the pack actually lived through, so
        /// there is nothing to compute without it.
        pub fn read(flag: &hv_plate::cache::flag::Reading, accumulated: &hv_plate::cache::accumulated::Reading) -> Option<Self> {
            Some(Self {
                i1cnt: flag.registers().i1cnt(),
                i1cntpha: flag.registers().i1cntpha(),
                current_sum_microvolts: accumulated.nice().current_sum_microvolts,
                batt_sum_microvolts: accumulated.nice().batt_sum_microvolts,
                read_at: accumulated.plant_at()?,
            })
        }

        /// The conversion counter as read.
        pub const fn i1cnt(&self) -> u16 {
            self.i1cnt
        }

        /// The counter's sub-phase, which `t_CONV` is measured from.
        pub const fn i1cntpha(&self) -> u16 {
            self.i1cntpha
        }

        /// When this reading was taken, on the plant's clock.
        pub const fn read_at(&self) -> PlantTime {
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
        Until(PlantTime),
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
        measurement_base: Option<(u16, PlantTime)>,
        /// The last measured conversion nanos measured over a full [`CONVERSION_TIME_BASELINE`]
        measured_conversion_nanos: Option<u32>,
        missed_windows: u32,
        desyncs: u32,
    }

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
                    defmt::info!("HvPlate: shunt: counters live at I1CNT {}, accumulating.", i1cnt);
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
                defmt::warn!("HvPlate: shunt: {} window(s) overwritten before the task read them; their charge is approximated from this window's average.", missed);
            }

            Some(Window {
                current_sum_microvolts,
                batt_sum_microvolts,
                windows_elapsed: (missed + 1) as u16,
                conversion_nanos: self.measure_conversion_time(i1cntpha, read_at).unwrap_or(NOMINAL_CONVERSION_NANOS),
            })
        }

        /// `t_CONV` measured against the plant clock: `t / dI1CNTPHA * 4`, divided by [`ACCN`].
        fn measure_conversion_time(&mut self, i1cntpha: u16, read_at: PlantTime) -> Option<u32> {
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
                defmt::warn!("HvPlate: shunt: no usable conversion time: {} phase ticks across {=u64} us.", measured_cntpha, elapsed.as_micros());
                return self.measured_conversion_nanos;
            }

            // Bounded by MAX_ELAPSED above, so neither the narrowing nor the product can overflow.
            let elapsed_micros = elapsed.as_micros() as u32;
            let conversion_nanos = (elapsed_micros * NANOS_PER_MICRO * PHA_PER_CNT) / measured_cntpha;
            if conversion_nanos.abs_diff(NOMINAL_CONVERSION_NANOS) > CONVERSION_TOLERANCE_NANOS {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: shunt: measured a conversion time of {} ns, too far from nominal to believe.", conversion_nanos);
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

/// Turning a summed shunt reading into a current, and measuring the sensor's zero offset.
mod calibration {
    use embassy_time::Duration;
    use uom::si::electric_current::ampere;
    use super::{ACCN, NANOS_PER_MICRO, accumulator};
    use crate::hv_plate;
    use crate::units::{Current, Temperature, Voltage};

    /// u_TODO: replace with a bench measurement of the installed part.
    const NOMINAL_NANOOHMS: i64 = hv_plate::board::SHUNT_MICROOHMS * 1_000;
    const SUM_NANOVOLTS_PER_AMP: i64 = ACCN as i64 * NOMINAL_NANOOHMS;

    /// Table 2, conversion noise at a ACC32 conversion time: 0.283 uV RMS.
    const WINDOW_NOISE_SUM_MICROVOLTS: i32 = 9;
    /// A run's samples spread ~4 sigma, so 9 sigmas covers that in both directions.
    const CALIBRATION_MAX_SPREAD_SUM_MICROVOLTS: i32 = 9 * WINDOW_NOISE_SUM_MICROVOLTS;

    /// How long a run collects before it is judged.
    const CALIBRATION_PERIOD: Duration = Duration::from_millis(500);

    // u_TODO: tighten once a bench run shows the actual total on `HvPlate/Shunt/Offset`.
    const CALIBRATION_MAX_OFFSET_MILLIAMPS: i64 = 500;
    const CALIBRATION_MAX_OFFSET_SUM_MICROVOLTS: i32 = (CALIBRATION_MAX_OFFSET_MILLIAMPS * SUM_NANOVOLTS_PER_AMP / 1_000_000) as i32;

    /// Converts a summed shunt reading to amps.
    fn amps_from_sum_microvolts(sum_microvolts: i32) -> f32 {
        (i64::from(sum_microvolts) * i64::from(NANOS_PER_MICRO)) as f32 / SUM_NANOVOLTS_PER_AMP as f32
    }

    /// One window, corrected and in physical units. What every estimator consumes.
    #[derive(Copy, Clone)]
    pub struct Measurement {
        /// Mean current across the window. Positive is discharge.
        pub shunt_current: Current,
        /// Mean BATT-side voltage across the same window, off the 2950's divider.
        pub batt_voltage: Voltage,
        pub charge_microcoulombs: i64,
        /// Whether a calibration run has been accepted.
        pub calibrated: bool,
    }

    /// A calibration measurement in progress.
    struct Run {
        sum: i64,
        windows: u32,
        elapsed_nanos: u64,
        min: i32,
        max: i32,
    }

    impl Run {
        const fn new() -> Self {
            Self { sum: 0, windows: 0, elapsed_nanos: 0, min: i32::MAX, max: i32::MIN }
        }

        fn push(&mut self, window: &accumulator::Window) {
            self.sum += i64::from(window.current_sum_microvolts);
            self.windows += 1;
            self.elapsed_nanos += window.duration_nanos();
            self.min = self.min.min(window.current_sum_microvolts);
            self.max = self.max.max(window.current_sum_microvolts);
        }

        /// Whether the run has collected for [`CALIBRATION_PERIOD`] and can be judged.
        const fn complete(&self) -> bool {
            self.elapsed_nanos >= CALIBRATION_PERIOD.as_nanos()
        }

        /// Mean sum across the run, which is the offset it proposes.
        const fn offset(&self) -> i32 {
            (self.sum / self.windows as i64) as i32
        }

        const fn spread(&self) -> i32 {
            self.max - self.min
        }
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

        /// Offers a window to the calibration routine, kept only while `can_calibrate`.
        pub fn observe(&mut self, window: &accumulator::Window, can_calibrate: bool, temp: Temperature) {
            if !can_calibrate {
                self.abort();
                return;
            }

            let run = self.run.get_or_insert_with(Run::new);
            run.push(window);
            if !run.complete() {
                return;
            }

            let (offset, spread, windows) = (run.offset(), run.spread(), run.windows);
            self.run = None;

            if offset.abs() > CALIBRATION_MAX_OFFSET_SUM_MICROVOLTS || spread > CALIBRATION_MAX_SPREAD_SUM_MICROVOLTS {
                self.runs_rejected = self.runs_rejected.saturating_add(1);
                defmt::warn!("HvPlate: shunt: rejected a shunt calibration: offset {} uV, spread {} uV over {} windows.", offset, spread, windows);
                return;
            }

            self.runs_accepted = self.runs_accepted.saturating_add(1);
            self.offset_sum_microvolts = offset;
            self.offset_taken_at = Some(temp);
            defmt::info!("HvPlate: shunt: shunt offset {} uV over {} windows, spread {} uV.", offset, windows, spread);
        }

        /// Drops any run in progress. The AIRs closing or the chip restarting both invalidate one.
        pub fn abort(&mut self) {
            self.run = None;
        }

        pub fn correct(&self, window: &accumulator::Window) -> Measurement {
            let corrected_shunt_reading = window.current_sum_microvolts - self.offset_sum_microvolts;
            let duration_nanos = window.duration_nanos();

            Measurement {
                shunt_current: Current::new::<ampere>(amps_from_sum_microvolts(corrected_shunt_reading)),
                batt_voltage: hv_plate::board::batt_voltage(window.batt_sum_microvolts / i32::from(ACCN)),
                charge_microcoulombs: i64::from(corrected_shunt_reading) * duration_nanos as i64 / SUM_NANOVOLTS_PER_AMP,
                calibrated: self.is_calibrated(),
            }
        }

        /// The offset as a current, so it comes off a single IVB1 reading too.
        pub fn offset_current(&self) -> Current {
            Current::new::<ampere>(amps_from_sum_microvolts(self.offset_sum_microvolts))
        }

        /// Whether a calibration run has been accepted.
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

/// Where the shunt's published values live.
pub mod published {
    use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
    use embassy_sync::watch::Watch;
    use super::Measurement;
    use crate::helpers::stamped::Stamped;
    use crate::helpers::snapshot_cell::SnapshotCell;
    use crate::helpers::unwriteable_watch::UnwriteableWatch;
    use crate::units::Current;

    /// Slots for tasks that `await` each window; `try_get` and `anon_receiver` take none.
    pub const WINDOW_RECEIVERS: usize = 1;

    pub type UnwriteableWindowWatch = UnwriteableWatch<ThreadModeRawMutex, Stamped<Measurement>, WINDOW_RECEIVERS>;

    static WINDOW: Watch<ThreadModeRawMutex, Stamped<Measurement>, WINDOW_RECEIVERS> = Watch::new();
    static INSTANT: SnapshotCell<Option<Stamped<Current>>> = SnapshotCell::new(None);

    pub fn window() -> &'static UnwriteableWindowWatch {
        UnwriteableWatch::of(&WINDOW)
    }

    pub fn instant() -> Option<Stamped<Current>> {
        INSTANT.read()
    }

    /// A `Watch` rather than a cell beside a signal: it is already both.
    pub(super) fn publish_window(window: Stamped<Measurement>) {
        WINDOW.sender().send(window);
    }

    pub(super) fn publish_instant(current: Current, read_at: embassy_time::Instant) {
        INSTANT.set(Some(Stamped::new(current, read_at)));
    }
}

mod task {
    use super::*;
    use crate::helpers::deadline::Deadline;
    use crate::helpers::stamped::Stamped;
    use crate::hv_plate::core::relay;
    #[cfg(defmt_monitor)]
    use {crate::units::degree_celsius, embassy_time::Duration, uom::si::electric_current::ampere, uom::si::electric_potential::volt};

    #[cfg(defmt_monitor)]
    const DIAGNOSTIC_PERIOD: Duration = Duration::from_millis(500);

    /// Whether nothing can be moving through the shunt, so a window measures only its offset.
    fn can_calibrate() -> bool {
        // if cfg!(feature = "hil") {
        //     return false;
        // }
        relay::state() == relay::RelayState::Open && !crate::shutdown::acc_sense_closed()
    }

    /// Turns the 2950's accumulator registers into corrected windows.
    #[embassy_executor::task]
    pub async fn shunt_task() {
        let mut reader = accumulator::Reader::new();
        let mut calibrator = calibration::Calibrator::new();

        // One slot, one consumer. `None` is a wiring bug, not a runtime condition.
        let mut acc_data_recv = hv_plate::core::task::signal::acc_window().receiver().expect("HvPlate: shunt: ACC_WINDOW_SIGNAL has no receiver slot left.");

        let current_voltage = hv_plate::cache::CACHE.current_voltage().reading().ready().await;
        let flag = hv_plate::cache::CACHE.flag().reading().ready().await;
        let accumulated = hv_plate::cache::CACHE.accumulated().reading().ready().await;
        let voltages = hv_plate::cache::CACHE.voltages().reading().ready().await;

        // Hold data for logging with monitor
        let mut latest_registers: Option<accumulator::Registers> = None;
        let mut latest_window: Option<accumulator::Window> = None;
        let mut latest_measurement: Option<Measurement> = None;

        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();
        loop {
            // Wait for a closed accumulator window
            let acc_window_status = acc_data_recv.changed().await;

            let acc_registers = acc_window_status.fresh.then(|| accumulator::Registers::read(&flag.get(), &accumulated.get())).flatten();
            let shunt_temp = voltages.get().nice().shunt_temperature;
            let can_calibrate = can_calibrate();

            // Reset state in acc register reader and shunt calibrator
            if acc_window_status.chip_restarted {
                reader.restart();
                calibrator.abort();
            }

            if let Some(registers) = acc_registers
                && let Some(window) = reader.convert(registers)
            {
                calibrator.observe(&window, can_calibrate, shunt_temp);
                let measurement = calibrator.correct(&window);
                // The host stamp, not the plant one: `Stamped` ages against the MCU's clock, and
                // mixing the two would make every freshness check downstream meaningless.
                published::publish_window(Stamped::new(measurement, accumulated.get().read_at()));

                latest_window = Some(window);
                latest_measurement = Some(measurement);
            }

            // Publish corrected instant current
            let reading = current_voltage.get();
            published::publish_instant(reading.nice().pack_current - calibrator.offset_current(), reading.read_at());

            #[cfg(defmt_monitor)]
            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt_monitor::monitor!("HvPlate/Shunt/MissedWindows", desc = "Accumulator windows overwritten before the task read them. Should stay at zero; non-zero means the 25 ms budget is blown.", "{=u32}", reader.missed_windows());
                defmt_monitor::monitor!("HvPlate/Shunt/Desyncs", desc = "Observations rejected because I1CNT could have lapped (a gap over ~2 s).", "{=u32}", reader.desyncs());
                #[cfg(feature = "hil")]
                defmt_monitor::monitor!("HvPlate/Shunt/HilSimClockUs", desc = "Simulation time the rig last reported over RDHILTIME, in us. NaN means the model has never answered and nothing measured in plant time is running at all.", "{=f32}", hv_plate::hil::sim_time::now().map_or(f32::NAN, |now| now.as_micros() as f32));
                defmt_monitor::monitor!("HvPlate/Shunt/Offset", desc = "Calibrated zero-current offset on the summed window, in microvolts. Datasheet bound is +/-32 uV across ACCN=32.", "{=i32}", calibrator.offset_sum_microvolts());
                defmt_monitor::monitor!("HvPlate/Shunt/OffsetTemp", desc = "Shunt temperature when the offset was taken, in celsius. NaN until a calibration run is accepted.", "{=f32}", calibrator.offset_taken_at().map_or(f32::NAN, |temp| temp.get::<degree_celsius>()));
                defmt_monitor::monitor!("HvPlate/Shunt/CanCalibrate", desc = "Whether windows are currently being collected for a shunt offset. Needs the relay read back open from CFGA and the accumulator shutdown sense low; always false under HIL, where the rig drives neither.", "{=bool}", can_calibrate);
                defmt_monitor::monitor!("HvPlate/Shunt/CalibrationsAccepted", desc = "Shunt calibration runs that passed the offset and spread checks.", "{=u32}", calibrator.runs_accepted());
                defmt_monitor::monitor!("HvPlate/Shunt/CalibrationsRejected", desc = "Shunt calibration runs thrown out. Persistent rejections with the AIRs commanded open suggest real current, i.e. a welded contactor.", "{=u32}", calibrator.runs_rejected());

                latest_registers = acc_registers.or(latest_registers);
                if let Some(regs) = latest_registers {
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/I1cnt", desc = "Conversion counter as read from FLAG. Climbs by ACCN each window and wraps at 2048; standing still means the chip stopped converting.", "{=u16}", regs.i1cnt());
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/I1cntPha", desc = "Sub-phase of the conversion counter, 13 bits with I1CNT. The change in this across two reads is what t_CONV is measured from.", "{=u16}", regs.i1cntpha());
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/ReadAtUs", desc = "Plant-clock timestamp the reading carries, in us. On a rig this is the model's simulation time, so it should track HvPlate/Shunt/HilSimClockUs rather than the MCU's uptime.", "{=u64}", regs.read_at().as_micros());
                }

                // A measurement is only present if a window is captured
                if let (Some(window), Some(measurement)) = (latest_window, latest_measurement) {
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/CurrentSumUv", desc = "Shunt voltage summed over the window, in uV, discharge positive and before the calibrated offset is taken off.", "{=i32}", window.current_sum_microvolts);
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/CorrectedSumUv", desc = "The same sum with the calibrated offset removed. Equal to the raw sum until a calibration run has been accepted.", "{=i32}", window.current_sum_microvolts - calibrator.offset_sum_microvolts());
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/BattSumUv", desc = "BATT-tap voltage summed over the window, in uV. Divide by ACCN for the mean tap reading.", "{=i32}", window.batt_sum_microvolts);
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/WindowsElapsed", desc = "Windows this reading covers. 1 is clean; more means windows were overwritten before the task read them and their charge is being approximated.", "{=u16}", window.windows_elapsed);
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/ConversionNs", desc = "Conversion time behind this window, in ns. Nominally 1000000. Pinned exactly there every window means the measurement was rejected and the nominal fallback is in use -- cross-check HvPlate/Shunt/Desyncs.", "{=u32}", window.conversion_nanos);
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/DurationUs", desc = "Wall time this window accounts for, in us. Nominally 32000 for one window at ACCN=32.", "{=u64}", window.duration_nanos() / 1_000);
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/CurrentAmps", desc = "Mean current across the window, in amps. Positive is discharge.", "{=f32}", measurement.shunt_current.get::<ampere>());
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/BattVolts", desc = "Mean BATT-side voltage across the window, off the 2950's divider -- not the segments' cell sum.", "{=f32}", measurement.batt_voltage.get::<volt>());
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/ChargeUc", desc = "Charge moved in this window, in microcoulombs. This is what the coulomb count integrates.", "{=i64}", measurement.charge_microcoulombs);
                    defmt_monitor::monitor!("HvPlate/Shunt/Window/Calibrated", desc = "Whether a shunt calibration has been accepted. False means CorrectedSumUv is uncorrected and the coulomb count is not running at all.", "{=bool}", measurement.calibrated);
                }
            }
        }
    }
}

pub use calibration::Measurement;
pub use task::shunt_task;
