use crate::hv_plate;
use crate::units::{Current, Voltage};
use adbms2950::chip::registers::config_a::types::AccumulatorDepth;
use adbms2950::chip::registers::measurement::accumulation_count;
use embassy_time::{Duration, Instant};
use uom::si::electric_current::ampere;

const I1CNTPHA_MODULUS: u32 = 1 << 13;
const PHASE_PER_CONVERSION: u32 = 4;
const NANOS_PER_MICRO: u32 = 1_000;

/// Accumulation depth, written into ConfigA's `ACCI` by `HvPlate::startup`.
pub(super) const ACCUMULATOR_DEPTH: AccumulatorDepth = AccumulatorDepth::Samples32;
const ACCN: u16 = accumulation_count(ACCUMULATOR_DEPTH as u8);

const NOMINAL_CONVERSION_NANOS: u32 = adbms2950::line::conversion_times::IXADC_CONVERSION_MS as u32 * 1_000_000;
const CONVERSION_STARTUP_MASK: Duration = Duration::from_micros(adbms2950::line::conversion_times::IXADC_INIT_MAX_MS as u64 * 1_000 + NOMINAL_CONVERSION_NANOS as u64 * ACCN as u64 / 1_000);
/// How long to watch `I1CNTPHA` before trusting a new `t_CONV`.
const MEASUREMENT_WINDOW: Duration = Duration::from_millis(500);

/// 5Ah/cell * 3600 sec/hour * 1e6 (microamps/amps) * 3 (cells/group) 
const CAPACITY_MICROCOULOMBS: i64 = 5 * 3600 * 1_000_000 * 3;

const CURRENT_HYSTERESIS_MICROAMPS: i64 = 10_000;
const SUM_MICROVOLTS_PER_AMP: i64 = hv_plate::board::SHUNT_MICROOHMS * ACCN as i64;
const NOISE_FLOOR_MICROVOLTS: i32 = (CURRENT_HYSTERESIS_MICROAMPS * SUM_MICROVOLTS_PER_AMP / 1_000_000) as i32;

fn soc_from_ocv(ocv: Voltage) -> Option<f32> {
    use uom::si::electric_potential::volt;

    let v = ocv.get::<volt>();
    if !(2.5..=4.2).contains(&v) {
        return None;
    }

    let soc = ((((-0.0179218 * v + 0.0830236) * v + 0.905277) * v - 7.43367) * v + 18.7306) * v - 16.0039;
    Some(soc.clamp(0.0, 1.0))
}

/// One accumulator window: FLAG and IVB1ACC
#[derive(Copy, Clone)]
struct AccWindow {
    i1cnt: u16,
    i1cntpha: u16,
    sum_microvolts: i32,
    read_at: Instant,
}

impl AccWindow {
    fn read(flag: &hv_plate::cache::flag::Nice, accumulated: &hv_plate::cache::accumulated::Nice) -> Self {
        Self {
            i1cnt: flag.data().i1cnt,
            i1cntpha: flag.data().i1cntpha,
            sum_microvolts: hv_plate::board::discharge_positive(accumulated.data().current_sum_microvolts),
            read_at: accumulated.oldest_read(),
        }
    }
}

#[derive(Copy, Clone)]
struct Sample {
    /// Mean current across the window. Positive is discharge.
    average_current: Current,
    charge_microcoulombs: i64,
    /// How many windows were overwritten unread, anything > 1 is bad.
    windows_elapsed: u16,
    /// The conversion time this window's charge was integrated at (`t_CONV` / `ACCN`)
    conversion_nanos: u32,
}

/// Tuple of `soc` measurement which was measured at `microcoulombs` counted.
#[derive(Copy, Clone)]
struct SocReference {
    soc: f32,
    microcoulombs: i64,
}

impl SocReference {
    fn calculate_soc(&self, current_net_microcoulombs: i64) -> f32 {
        let moved = current_net_microcoulombs - self.microcoulombs;
        (self.soc - moved as f32 / CAPACITY_MICROCOULOMBS as f32).clamp(0.0, 1.0)
    }
}

#[derive(Copy, Clone)]
struct Estimator {
    /// Readings before this are rejected.
    conversion_startup_deadline: Option<Instant>,

    /// Windows consumed so far.
    n: u32,
    i1cnt_old: u16,

    /// Measured i1cnt + time at the start of the open measurement window.
    measurement_base: Option<(u16, Instant)>,
    last_sample: Option<Sample>,
    cc_microcoulombs: i64,
    reference: Option<SocReference>,
    state_of_charge: Option<f32>,
    drift: f32,
    seeded_from: Option<Instant>,

    /// Diagnostics
    missed_windows: u32,
    desyncs: u32,
}

impl Estimator {
    fn new(now: Instant) -> Self {
        let mut tracker = Self {
            conversion_startup_deadline: None,
            n: 0,
            i1cnt_old: 0,
            measurement_base: None,
            cc_microcoulombs: 0,
            reference: None,
            state_of_charge: None,
            drift: 0.0,
            seeded_from: None,
            missed_windows: 0,
            desyncs: 0,
            last_sample: None,
        };
        tracker.restart(now);
        tracker
    }

    /// Re-baselines after an `ADI1`, which zeroes `I1CNT`. Leaves the estimate and the running
    /// total alone
    fn restart(&mut self, now: Instant) {
        self.n = 0;
        self.i1cnt_old = 0;
        self.measurement_base = None;
        self.conversion_startup_deadline = Some(now + CONVERSION_STARTUP_MASK);
    }

    /// Folds one accumulator window into the count.
    fn accumulate(&mut self, AccWindow { i1cnt, i1cntpha, sum_microvolts, read_at }: AccWindow) {
        // Check that conversion has actually settled before doing coloumb counting
        if let Some(deadline) = self.conversion_startup_deadline {
            if read_at < deadline {
                return;
            }
            self.conversion_startup_deadline = None;
            // Calculate how many windows have gone by and save into `n`
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
            return;
        }
        let missed = i1cnt_windows - self.n;
        self.n = i1cnt_windows + 1;
        if missed > 0 {
            self.missed_windows = self.missed_windows.saturating_add(missed);
            defmt::warn!("HvPlate: soc: {} window(s) overwritten before the task read them; their charge is approximated from this window's average.", missed);
        }

        let windows_elapsed = (missed + 1) as u16;

        let conversion_nanos = self.get_conversion_time(i1cntpha, read_at).unwrap_or(NOMINAL_CONVERSION_NANOS);
        let window_nanos = u32::from(windows_elapsed) * conversion_nanos;
        let charge_microcoulombs = if sum_microvolts.abs() >= NOISE_FLOOR_MICROVOLTS { i64::from(sum_microvolts) / (hv_plate::board::SHUNT_MICROOHMS * 1_000) * i64::from(window_nanos) } else { 0 };
        self.cc_microcoulombs += charge_microcoulombs;

        // The window's reading, which is both the reported pack current and the diagnostics.
        self.last_sample = Some(Sample {
            average_current: Current::new::<ampere>(sum_microvolts as f32 / SUM_MICROVOLTS_PER_AMP as f32),
            charge_microcoulombs,
            windows_elapsed,
            conversion_nanos,
        });

        self.calculate_state_of_charge();
    }

    /// Latches a [`Reference`] at the current total, returning whether it was accepted.
    fn seed(&mut self, min_cell_ocv: Voltage, settled_at: Instant) {
        // Only reseed on new rest data
        if self.seeded_from == Some(settled_at) {
            return;
        }

        let Some(soc) = soc_from_ocv(min_cell_ocv) else {
            defmt::warn!("HvPlate: soc: rejected an OCV seed outside the curve's valid range.");
            return;
        };

        // `soc_drift = coulomb_counting - from_ocv`, so positive means
        // the count is reading higher than the pack really is.
        self.drift = self.state_of_charge.map_or(0.0, |reported| reported - soc);
        self.seeded_from = Some(settled_at);
        self.reference = Some(SocReference { soc, microcoulombs: self.cc_microcoulombs });
        self.calculate_state_of_charge();
        let age_millis = Instant::now().saturating_duration_since(settled_at).as_millis();
        defmt::info!("HvPlate: soc: referenced at {} from an OCV taken {} ms ago; drift {}.", soc, age_millis, self.drift);
    }

    /// `t_CONV` measured against the host clock: `t / dI1CNTPHA * 4`, divided by [`ACCN`].
    fn get_conversion_time(&mut self, i1cntpha: u16, read_at: Instant) -> Option<u32> {
        let Some((base_pha, base_at)) = self.measurement_base else {
            self.measurement_base = Some((i1cntpha, read_at));
            return None;
        };

        let elapsed = read_at.saturating_duration_since(base_at);
        let elapsed_micros = u32::try_from(elapsed.as_micros()).unwrap_or(u32::MAX);
        let ticks = (u32::from(i1cntpha) + I1CNTPHA_MODULUS - u32::from(base_pha)) % I1CNTPHA_MODULUS;

        // Only the full-window reading is a formal measurement, so only it re-bases and only it
        // counts a desync -- otherwise a stuck counter would tally one every tick.
        let measured = elapsed >= MEASUREMENT_WINDOW;
        if measured {
            self.measurement_base = Some((i1cntpha, read_at));
        }

        const MAX_ELAPSED_MICROS: u32 = u32::MAX / (NANOS_PER_MICRO * PHASE_PER_CONVERSION);
        if ticks == 0 || elapsed_micros > MAX_ELAPSED_MICROS {
            if measured {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: no usable conversion time: {} phase ticks across {} us.", ticks, elapsed_micros);
            }
            return None;
        }

        // A quarter either way against an oscillator specified at a tenth: past this the reading
        // caught a lapped counter, not drift.
        let nanos = elapsed_micros * NANOS_PER_MICRO * PHASE_PER_CONVERSION / ticks;
        const TOLERANCE: u32 = NOMINAL_CONVERSION_NANOS / 4;
        if nanos.abs_diff(NOMINAL_CONVERSION_NANOS) > TOLERANCE {
            if measured {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: measured a conversion time of {} ns, too far from nominal to believe.", nanos);
            }
            return None;
        }

        Some(nanos)
    }

    /// Re-derives [`Self::state_of_charge`]. Call wherever the reference or the total moves.
    fn calculate_state_of_charge(&mut self) {
        self.state_of_charge = self.reference.map(|r| r.calculate_soc(self.cc_microcoulombs));
    }

    /// Net charge since boot. Positive is discharge. Never reset, by a restart or a reseed.
    const fn cc_microcoulombs(&self) -> i64 {
        self.cc_microcoulombs
    }

    /// Positive means the coulomb count was reading higher than the OCV.
    const fn soc_drift(&self) -> f32 {
        self.drift
    }

    /// `None` until an OCV has seeded the count.
    const fn state_of_charge(&self) -> Option<f32> {
        self.state_of_charge
    }

    fn pack_current(&self) -> Option<Current> {
        self.last_sample.map(|sample| sample.average_current)
    }

    fn t_conv(&self) -> Option<Duration> {
        self.last_sample.map(|sample| Duration::from_micros(u64::from(sample.conversion_nanos * u32::from(ACCN) / NANOS_PER_MICRO)))
    }

    const fn last_sample(&self) -> Option<Sample> {
        self.last_sample
    }

    /// Should stay at zero. Non-zero means the task is not keeping up.
    const fn missed_windows(&self) -> u32 {
        self.missed_windows
    }

    const fn desyncs(&self) -> u32 {
        self.desyncs
    }
}

/// What the SoC estimator publishes.
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
    use embassy_time::{Instant, Duration};
    use super::{AccWindow, Estimator};
    use crate::hv_plate;

    const DIAGNOSTIC_PERIOD: Duration = Duration::from_millis(500);

    #[embassy_executor::task]
    pub async fn soc_task() {
        use crate::helpers::deadline::Deadline;
        use crate::hv_plate::core::task::signal::ACC_DATA_SIGNAL;

        let mut estimator = Estimator::new(Instant::now());

        let Some(mut acc_data_recv) = ACC_DATA_SIGNAL.receiver() else {
            defmt::error!("HvPlate: soc: no acc data receiver left; coulomb counting will not run!");
            return;
        };

        let flag = hv_plate::cache().get_flag().nice().ready().await;
        let accumulated = hv_plate::cache().get_accumulated().nice().ready().await;

        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();
        loop {
            // Period of ~25ms (how often we read acc data registers)
            let acc_data = acc_data_recv.changed().await;

            // Get data from cache and pack analyzer
            let window = acc_data.fresh.then(|| AccWindow::read(&flag.get(), &accumulated.get()));
            let ocv_seed = crate::pack::analyzer::analyzer().and_then(|analyzer| analyzer.data.last_settled_at.map(|settled_at| (analyzer.data.min_ocv.value(), settled_at)));

            // Run estimator
            if let Some(at) = acc_data.restarted_at {
                estimator.restart(at);
            }

            if let Some((min_ocv, settled_at)) = ocv_seed {
                estimator.seed(min_ocv, settled_at);
            }

            if let Some(window) = window {
                estimator.accumulate(window);
            } else {
                defmt::warn!("HvPlate: soc: invalid data for window; skipping accumulation for this window!");
            }

            super::api::store(super::api::SocData {
                shunt_current: estimator.pack_current(),
                state_of_charge: estimator.state_of_charge().and_then(crate::units::Ratio::from_ratio),
                drift: estimator.soc_drift(),
            });

            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt_monitor::monitor!("HvPlate/Soc/MissedWindows", desc = "Accumulator windows overwritten before the task read them. Should stay at zero; non-zero means the 25 ms budget is blown.", "{=u32}", estimator.missed_windows());
                defmt_monitor::monitor!("HvPlate/Soc/Desyncs", desc = "Observations rejected because I1CNT could have lapped (a gap over ~2 s).", "{=u32}", estimator.desyncs());

                if let Some(sample) = estimator.last_sample() {
                    defmt_monitor::monitor!("HvPlate/Soc/AverageCurrent", desc = "Mean shunt current over the most recent accumulator window, in amps. Cross-check against the instantaneous reading from IVB1.", "{=f32}", sample.average_current.get::<uom::si::electric_current::ampere>());
                    defmt_monitor::monitor!("HvPlate/Soc/StateOfCharge", desc = "State of charge 0..1. NaN until an open-circuit voltage has seeded the count -- the estimate does not exist yet, rather than being zero.", "{=f32}", estimator.state_of_charge().unwrap_or(f32::NAN));
                    defmt_monitor::monitor!("HvPlate/Soc/SampleCharge", desc = "Charge moved in the most recent accumulator window, in microcoulombs.", "{=i64}", sample.charge_microcoulombs);
                    defmt_monitor::monitor!("HvPlate/Soc/WindowsElapsed", desc = "Accumulator windows covered by the most recent sample. 1 is clean.", "{=u16}", sample.windows_elapsed);
                    defmt_monitor::monitor!("HvPlate/Soc/TConvUs", desc = "Measured t_CONV, the accumulator window, in us. 0 until the first window lands. Nominally 32000; drift from that is the internal oscillator, specified at +/-10%.", "{=u64}", estimator.t_conv().map_or(0, |window| window.as_micros()));
                    defmt_monitor::monitor!("HvPlate/Soc/CC", desc = "The datasheet's CC: net charge moved since boot, in microcoulombs. Positive is discharge. Never reset -- not by a chip restart, not by a reseed.", "{=i64}", estimator.cc_microcoulombs());
                    defmt_monitor::monitor!("HvPlate/Soc/Drift", desc = "Gap the last accepted OCV found against the coulomb count, in state of charge. Positive means the OCV read higher.", "{=f32}", estimator.soc_drift());
                }
            }
        }
    }
}

pub use task::soc_task;
