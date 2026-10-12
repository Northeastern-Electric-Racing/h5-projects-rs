use crate::hv_plate::board;
use crate::hv_plate::cache;
use crate::units::{Current, Voltage};
use adbms2950::chip::registers::config_a::types::AccumulatorDepth;
use adbms2950::chip::registers::measurement::accumulation_count;
use embassy_time::{Duration, Instant};
use uom::si::electric_current::ampere;

const I1CNTPHA_MODULUS: u32 = 1 << 13;
const PHASE_PER_CONVERSION: u64 = 4;
/// Accumulation depth, written into ConfigA's `ACCI` by `HvPlate::startup`.
pub(super) const ACCUMULATOR_DEPTH: AccumulatorDepth = AccumulatorDepth::Samples32;
/// `ACCN`: samples per accumulator window, `4 * (ACCI + 1)`.
const ACCN: u16 = accumulation_count(ACCUMULATOR_DEPTH as u8);

const NOMINAL_CONVERSION_NANOS: u64 = adbms2950::line::conversion_times::IXADC_CONVERSION_MS as u64 * 1_000_000;
const CONVERSION_STARTUP_MASK: Duration = Duration::from_micros(adbms2950::line::conversion_times::IXADC_INIT_MAX_MS as u64 * 1_000 + NOMINAL_CONVERSION_NANOS * ACCN as u64 / 1_000);

const CAPACITY_MICROCOULOMBS: i64 = 5 * 3600 * 1_000_000;
const CURRENT_HYSTERESIS_MICROAMPS: i64 = 10_000;
/// `I * R * ACCN`, so 16 uV. On the sum, where half a code is still visible.
/// Accumulator sum one amp produces: `R * ACCN`, so 1600 uV.
const SUM_MICROVOLTS_PER_AMP: i64 = board::SHUNT_MICROOHMS * ACCN as i64;
const NOISE_FLOOR_MICROVOLTS: i32 = (CURRENT_HYSTERESIS_MICROAMPS * SUM_MICROVOLTS_PER_AMP / 1_000_000) as i32;

/// How long to watch `I1CNTPHA` before trusting a new `t_CONV`.
const MEASUREMENT_WINDOW: Duration = Duration::from_millis(500);

fn soc_from_ocv(ocv: Voltage) -> Option<f32> {
    use uom::si::electric_potential::volt;

    let v = ocv.get::<volt>();
    if !(2.5..=4.2).contains(&v) {
        return None;
    }

    let soc = ((((-0.0179218 * v + 0.0830236) * v + 0.905277) * v - 7.43367) * v + 18.7306) * v - 16.0039;
    Some(soc.clamp(0.0, 1.0))
}

#[derive(Copy, Clone)]
pub struct Sample {
    /// Mean current across the window. Positive is discharge.
    pub average_current: Current,
    pub charge_microcoulombs: i64,
    /// How many windows were overwritten unread, anything > 1 is bad.
    pub windows_elapsed: u16,
    /// The conversion time this window's charge was integrated at (`t_CONV` / `ACCN`)
    pub conversion_nanos: u32,
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
pub struct Estimator {
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
    pub fn new(now: Instant) -> Self {
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
    pub fn restart(&mut self, now: Instant) {
        self.n = 0;
        self.i1cnt_old = 0;
        self.measurement_base = None;
        self.conversion_startup_deadline = Some(now + CONVERSION_STARTUP_MASK);
    }

    /// Folds this cycle's reading into the count.
    pub fn accumulate(&mut self) {
        // Read data from cache
        let flag = cache::CACHE.get_flag();
        let accumulated = cache::CACHE.get_accumulated();
        let (Some(read_at), Ok(flag), Ok(accumulated)) = (accumulated.ivb1acc.last_successful_read(), flag.try_nice(), accumulated.try_nice()) else {
            return;
        };

        let (i1cnt, i1cntpha) = (flag.i1cnt, flag.i1cntpha);
        // Matches sign for `api::pack_current`
        let sum_microvolts = -accumulated.current_sum_microvolts;

        // Check that conversion has actually settled before doing coloumb counting
        if let Some(deadline) = self.conversion_startup_deadline {
            if read_at < deadline {
                return;
            }
            self.conversion_startup_deadline = None;
            // The datasheet's `INIT` is `N = 1, I1CNT_OLD = 0` because it starts reading at
            // t = 0. We mask out the startup window instead, so windows have already gone by --
            // baseline off the counter or the first sample claims every one of them as missed.
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

        // `Q = V / R * t`. Microvolt-nanoseconds over microohms is nanocoulombs, hence the final
        // thousand. [`ACCN`] cancels: a sum times the per-sample period is already the average
        // times the window.
        //
        // u_TODO - the guidance from the datasheet also applies a shunt temp coeff here
        let conversion_nanos = self.get_conversion_time(i1cntpha, read_at).unwrap_or(NOMINAL_CONVERSION_NANOS as u32);
        let charge_microcoulombs = if sum_microvolts.abs() >= NOISE_FLOOR_MICROVOLTS { i64::from(sum_microvolts) * i64::from(windows_elapsed) * i64::from(conversion_nanos) / (board::SHUNT_MICROOHMS * 1_000) } else { 0 };
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
    pub fn seed(&mut self, min_cell_ocv: Voltage, settled_at: Instant) {
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
        let ticks = (u32::from(i1cntpha) + I1CNTPHA_MODULUS - u32::from(base_pha)) % I1CNTPHA_MODULUS;

        // Only the full-window reading is a formal measurement, so only it re-bases and only it
        // counts a desync -- otherwise a stuck counter would tally one every tick.
        let measured = elapsed >= MEASUREMENT_WINDOW;
        if measured {
            self.measurement_base = Some((i1cntpha, read_at));
        }

        if ticks == 0 {
            if measured {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: counter did not move across a measurement window; conversions may have stopped.");
            }
            return None;
        }

        // A quarter either way against an oscillator specified at a tenth: past this the reading
        // caught a lapped counter, not drift.
        let nanos = elapsed.as_micros() * 1_000 * PHASE_PER_CONVERSION / u64::from(ticks);
        const TOLERANCE: u64 = NOMINAL_CONVERSION_NANOS / 4;
        if nanos.abs_diff(NOMINAL_CONVERSION_NANOS) > TOLERANCE {
            if measured {
                self.desyncs = self.desyncs.saturating_add(1);
                defmt::warn!("HvPlate: soc: measured a conversion time of {} ns, too far from nominal to believe.", nanos);
            }
            return None;
        }

        Some(nanos as u32)
    }

    /// Re-derives [`Self::state_of_charge`]. Call wherever the reference or the total moves.
    fn calculate_state_of_charge(&mut self) {
        self.state_of_charge = self.reference.map(|r| r.calculate_soc(self.cc_microcoulombs));
    }

    /// Net charge since boot. Positive is discharge. Never reset, by a restart or a reseed.
    pub const fn cc_microcoulombs(&self) -> i64 {
        self.cc_microcoulombs
    }

    /// C's `soc_drift`. Positive means the coulomb count was reading higher than the OCV.
    pub const fn soc_drift(&self) -> f32 {
        self.drift
    }

    /// State of charge, 0..1. `None` until an OCV has seeded the count.
    pub const fn state_of_charge(&self) -> Option<f32> {
        self.state_of_charge
    }

    /// Mean pack current over the most recent accumulator window. Positive is discharge.
    pub fn pack_current(&self) -> Option<Current> {
        self.last_sample.map(|sample| sample.average_current)
    }

    /// `t_CONV`: the measured accumulator window, `ACCN` conversions. Nominally `ACCN` ms, and
    /// the figure to watch for oscillator drift. `None` until a window has landed.
    pub fn t_conv(&self) -> Option<Duration> {
        self.last_sample.map(|sample| Duration::from_micros(u64::from(sample.conversion_nanos) * u64::from(ACCN) / 1_000))
    }

    pub const fn last_sample(&self) -> Option<Sample> {
        self.last_sample
    }

    /// Should stay at zero. Non-zero means the task is not keeping up.
    pub const fn missed_windows(&self) -> u32 {
        self.missed_windows
    }

    pub const fn desyncs(&self) -> u32 {
        self.desyncs
    }
}
