//! State-of-charge estimation from the shunt's accumulated windows.
#![cfg_attr(not(defmt_monitor), allow(dead_code))]

use crate::time::PlantTime;
use embassy_time::{Duration, Instant};
use crate::helpers::snapshot_cell::SnapshotCell;
use crate::helpers::stamped::Stamped;
use crate::hv_plate::{self, Measurement};
use crate::units::{Current, Ratio, Voltage};

/// 3p of 5 Ah cells, in microcoulombs
const CAPACITY_MICROCOULOMBS: i64 = 5 * 3 * 3600 * 1_000_000;

/// Shortest gap between two references. Plant time, to match the charge the count integrates over
/// between them: on a rig running slower than real time, ten wall seconds are not ten seconds of
/// pack.
const SEED_MIN_INTERVAL: Duration = Duration::from_secs(10);

/// How stale a snapshot may be and still be referenced off. Host time: this asks whether the OCV
/// tracker is still publishing, not how long the pack has sat.
const SEED_MAX_AGE: Duration = Duration::from_secs(1);

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
struct Counter {
    microcoulombs: i64,
    reference: Option<Reference>,
    state_of_charge: Option<f32>,
    drift: f32,
    seeded_from: Option<Instant>,
    /// When the last reference was taken, on the plant's clock, which [`SEED_MIN_INTERVAL`] is
    /// measured from.
    last_seed_at: Option<PlantTime>,
}

impl Counter {
    const fn new() -> Self {
        Self {
            microcoulombs: 0,
            reference: None,
            state_of_charge: None,
            drift: 0.0,
            seeded_from: None,
            last_seed_at: None,
        }
    }

    fn accumulate(&mut self, measurement: &Measurement) {
        self.microcoulombs += measurement.charge_microcoulombs;
        if self.reference.is_some() {
            self.calculate_soc();
        }
    }

    /// Latches a [`Reference`] at the current total, at most once per [`SEED_MIN_INTERVAL`].
    ///
    /// `now` is plant time, to match the charge the count integrates between two references. The
    /// caller skips the seed entirely when that clock cannot be read: an unenforced minimum would
    /// let a single noisy snapshot be referenced off repeatedly.
    fn seed(&mut self, now: PlantTime, reading: Stamped<Voltage>) {
        // Already referenced off this snapshot. Also stops a stale one being re-used once the
        // pack leaves rest and the stamp stops advancing.
        if self.seeded_from == Some(reading.at()) {
            return;
        }

        if let Some(last_seed_at) = self.last_seed_at
            && now.saturating_duration_since(last_seed_at) < SEED_MIN_INTERVAL
        {
            return;
        }

        let Some(min_cell_ocv) = reading.fresh(SEED_MAX_AGE) else {
            return;
        };

        let Some(soc) = soc_from_ocv(min_cell_ocv) else {
            defmt::warn!("HvPlate: soc: rejected an OCV seed outside the curve's valid range.");
            return;
        };

        // `drift = coulomb_counting - from_ocv`, so positive means the count is reading higher than the pack really is.
        self.drift = self.state_of_charge.map_or(0.0, |reported| reported - soc);
        self.seeded_from = Some(reading.at());
        self.last_seed_at = Some(now);
        self.reference = Some(Reference { soc, microcoulombs: self.microcoulombs });
        self.calculate_soc();

        defmt::info!("HvPlate: soc: referenced at {} from an OCV taken {} ms ago; drift {}.", soc, reading.age().as_millis(), self.drift);
    }

    fn calculate_soc(&mut self) {
        self.state_of_charge = self.reference.map(|reference| reference.soc_at(self.microcoulombs));
    }

    /// Net charge since boot. Positive is discharge. Never reset
    const fn cc_microcoulombs(&self) -> i64 {
        self.microcoulombs
    }

    /// `None` until an OCV has seeded the count.
    const fn state_of_charge(&self) -> Option<f32> {
        self.state_of_charge
    }

    /// Positive means the coulomb count was reading higher than the OCV.
    const fn drift(&self) -> f32 {
        self.drift
    }

    /// Seconds of plant time since the last reference, or `None` if the count has never been
    /// seeded or the plant clock cannot be read.
    fn secs_since_seed(&self, now: Option<PlantTime>) -> Option<f32> {
        let (at, now) = (self.last_seed_at?, now?);
        Some(now.saturating_duration_since(at).as_millis() as f32 / 1000.0)
    }
}

/// What the estimator publishes
#[derive(Copy, Clone)]
pub struct SocData {
    /// Pack current as the 2950's accumulator measured it. Positive is discharge.
    pub shunt_current: Option<Current>,
    /// State of charge, 0..1. `None` until an open-circuit voltage has seeded the count.
    pub state_of_charge: Option<Ratio>,
    /// Positive means the coulomb count was reading higher than the OCV.
    pub drift: f32,
}

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

/// Publishes what the estimator currently knows. Called only by [`task`].
fn store(data: SocData) {
    // The MCU's clock: this stamp answers "is the estimator still running", which has to keep
    // working when a rig's model stops.
    SOC_DATA.set(Some(SocDataHolder { data, last_updated: Instant::now() }));
}

mod task {
    use super::*;
    use crate::ocv;

    #[embassy_executor::task]
    pub async fn soc_task() {
        let mut counter = Counter::new();

        let mut window_recv = hv_plate::shunt_window().receiver().expect("soc: shunt window watch has no receiver slot left.");

        loop {
            // One wake per closed accumulator window
            let measurement = window_recv.changed().await.get();

            // Get latest OCV value
            let ocv_seed = ocv::ocv().try_partial_get(|snapshot| Stamped::new(snapshot.get().stats.min.value(), snapshot.at()));
            // Plant time: the gap between two references has to match the charge the count
            // integrated across it.
            let plant = crate::time::plant_now();

            if measurement.calibrated {
                counter.accumulate(&measurement);
            }

            // Seed estimator with OCV values if available
            if let (Some(now), Some(reading)) = (plant, ocv_seed) {
                counter.seed(now, reading);
            }

            store(SocData {
                shunt_current: Some(measurement.shunt_current),
                state_of_charge: counter.state_of_charge().and_then(crate::units::Ratio::from_ratio),
                drift: counter.drift(),
            });

            #[cfg(defmt_monitor)]
            {
                defmt_monitor::monitor!("Soc/Measurement/StateOfCharge", desc = "State of charge 0..1. NaN until an open-circuit voltage has seeded the count -- the estimate does not exist yet, rather than being zero.", "{=f32}", counter.state_of_charge().unwrap_or(f32::NAN));
                defmt_monitor::monitor!("Soc/Measurement/CC", desc = "The datasheet's CC: net charge moved since boot, in microcoulombs. Positive is discharge. Never reset -- not by a chip restart, not by a reseed.", "{=i64}", counter.cc_microcoulombs());
                defmt_monitor::monitor!("Soc/Measurement/Drift", desc = "Gap the last accepted OCV found against the coulomb count, in state of charge. Positive means the OCV read higher. Holds until the next reference rather than being wiped by one taken moments later.", "{=f32}", counter.drift());
                defmt_monitor::monitor!("Soc/Measurement/SecsSinceSeed", desc = "Seconds of plant time since the count last referenced itself off an OCV. Sits between 0 and SEED_MIN_INTERVAL while the pack rests, and climbs without bound once it does not. NaN until the first reference, and whenever the plant clock cannot be read.", "{=f32}", counter.secs_since_seed(plant).unwrap_or(f32::NAN));
            }
        }
    }
}

pub use task::soc_task;
