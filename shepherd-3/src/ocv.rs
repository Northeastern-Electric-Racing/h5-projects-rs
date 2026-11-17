//! Open-circuit voltage: when the pack is at rest, and what the cells read at that moment.
//!
//! Latched here rather than in the analyzer because it waits on a current threshold, judged at the shunt's cadence rather than the segments'.
#![cfg_attr(not(defmt_monitor), allow(dead_code))]

use crate::helpers::time::PlantTime;
use embassy_time::{Duration, Instant};

use crate::helpers::deadline::Deadline;
use crate::helpers::stamped::Stamped;
use crate::helpers::startup_cell::{StartupCell, UnwriteableStartupCell};
use crate::pack::analyzer::CriticalCellValue;
use crate::segments::{CellId, ChipId, IndexByCell, IndexByChip, IndexBySegment, NUM_CELLS_PER_SEGMENT, NUM_CELLS_TOTAL};
use crate::units::{Current, Voltage, volt};

/// Below this the pack counts as unloaded.
const OCV_CURR_THRESH: Current = Current::from_amps(0.5_f32);

/// How long the pack has to stay unloaded before its cell voltages count as open-circuit. Plant time: it is the cells' relaxation being waited out.
const OCV_TIMER_DURATION: Duration = Duration::from_millis(1500);

/// The snapshot's statistics, kept apart from the per-cell array so a reader after one number does not copy the whole thing.
#[derive(Copy, Clone)]
pub struct OcvStats {
    pub max: CriticalCellValue<Voltage>,
    pub min: CriticalCellValue<Voltage>,
    pub avg: Voltage,
    pub delta: Voltage,
    pub pack: Voltage,
    /// Per-segment sums of the snapshot, and those sums over the cells in a segment.
    pub segment_total_volts: IndexBySegment<Voltage>,
    pub segment_average_volts: IndexBySegment<Voltage>,
}

impl OcvStats {
    /// The sentinels the analyzer used to start from, for a reader that runs before the first snapshot.
    pub fn empty() -> Self {
        Self {
            max: CriticalCellValue::new(Voltage::new::<volt>(f32::MIN), ChipId::Chip0, CellId::Cell1),
            min: CriticalCellValue::new(Voltage::new::<volt>(f32::MAX), ChipId::Chip0, CellId::Cell1),
            avg: Voltage::new::<volt>(f32::MIN),
            delta: Voltage::new::<volt>(f32::MIN),
            pack: Voltage::new::<volt>(f32::MIN),
            segment_total_volts: IndexBySegment::from_fn(|_| Voltage::new::<volt>(f32::MIN)),
            segment_average_volts: IndexBySegment::from_fn(|_| Voltage::new::<volt>(f32::MIN)),
        }
    }
}

/// What the tracker publishes, stamped with the moment the pack was judged settled.
#[derive(Copy, Clone)]
pub struct OcvData {
    /// Every cell's voltage as of the stamp.
    pub open_cell_voltage: IndexByChip<IndexByCell<Voltage>>,
    pub stats: OcvStats,
}

/// The most recently published snapshot.
static OCV: StartupCell<Stamped<OcvData>> = StartupCell::new();

/// Gets a handle to the most recently published snapshot. The stamp stops advancing once the pack leaves rest, so a reader using it as evidence should go through [`Stamped::fresh`].
pub fn ocv() -> &'static UnwriteableStartupCell<Stamped<OcvData>> {
    OCV.to_unwriteable()
}

/// Publishes a snapshot for other tasks to read. Called only by [`task`].
fn store(data: OcvData, settled_at: Instant) {
    OCV.set(Stamped::new(data, settled_at))
}

/// Holds the snapshot and the rest-detection state between wakes.
struct Tracker {
    open_cell_voltage: IndexByChip<IndexByCell<Voltage>>,
    stats: OcvStats,
    /// Whether the last look found the pack unloaded. Reported, not acted on.
    rest_allowed: bool,
    /// Deadline after which the pack has been unloaded long enough to snapshot.
    rest_timer: Option<Deadline>,
    /// Whether we are still waiting for the first plausible cell voltage to seed the snapshot.
    is_first_reading: bool,
}

impl Tracker {
    fn new() -> Self {
        Self {
            open_cell_voltage: IndexByChip::from_fn(|_| IndexByCell::from_fn(|_| Voltage::new::<volt>(f32::MIN))),
            stats: OcvStats::empty(),
            rest_allowed: false,
            rest_timer: None,
            is_first_reading: true,
        }
    }

    /// Judges whether the pack is at rest and, if it is, latches a snapshot and publishes it. `None` holds the rest timer cleared; the first-reading seed bypasses it.
    fn observe(&mut self, now: Option<PlantTime>) {
        use crate::pack::analyzer;

        // Whichever instrument is currently believed. A CAN reporter measures its own branch, not the pack.
        let Some(sourced) = crate::pack::pack_current() else {
            return;
        };
        let mut pack_current = sourced.get();

        let ocv_update_allowed: bool = { pack_current.abs() < OCV_CURR_THRESH && crate::charging::charger_output_disabled() && !crate::charging::are_we_actively_balancing().as_bool() };
        self.rest_allowed = ocv_update_allowed;

        let mut update_ocv = false;

        if self.is_first_reading {
            let last_cell: Option<Voltage> = analyzer::analyzer().try_partial_get(|holder| holder.data.corrected_cell_voltages[ChipId::last()][CellId::last()]);

            if let Some(last_cell) = last_cell
                && last_cell > Voltage::from_volts(1.0_f32)
                && last_cell < Voltage::from_volts(5.0_f32)
            {
                self.is_first_reading = false;
                update_ocv = true;
            }
        }

        if ocv_update_allowed && let Some(now) = now {
            match self.rest_timer {
                // Timer is expired so we should update OCV.
                Some(deadline) if deadline.past_at(now) => update_ocv = true,

                // Timer is still active so keep waiting.
                Some(_) => {},

                // No timer, but update is allowed now, so we should start it
                None => self.rest_timer = Some(Deadline::expire_after(now, OCV_TIMER_DURATION)),
            }
        } else {
            self.rest_timer = None;
        }

        if !update_ocv {
            return;
        }

        let Some(corrected) = analyzer::analyzer().try_partial_get(|holder| holder.data.corrected_cell_voltages) else {
            return;
        };
        self.open_cell_voltage = corrected;
        self.recalculate();

        // Snapshot and stamp taken together. The MCU's clock: this answers "is this still being published", not "how long has the pack sat".
        store(OcvData { open_cell_voltage: self.open_cell_voltage, stats: self.stats }, Instant::now());
    }

    /// Recomputes every statistic from the snapshot just latched.
    fn recalculate(&mut self) {
        let mut max = CriticalCellValue::new(Voltage::new::<volt>(f32::MIN), ChipId::Chip0, CellId::Cell1);
        let mut min = CriticalCellValue::new(Voltage::new::<volt>(f32::MAX), ChipId::Chip0, CellId::Cell1);

        let mut total: Voltage = Voltage::new::<volt>(0_f32);
        let mut total_seg: Voltage = Voltage::new::<volt>(0_f32);

        for chip in ChipId::iter() {
            for cell in CellId::iter() {
                let ocv = self.open_cell_voltage[chip][cell];

                if ocv > max.value() {
                    max = CriticalCellValue::new(ocv, chip, cell)
                }

                if ocv < min.value() {
                    min = CriticalCellValue::new(ocv, chip, cell)
                }

                total += ocv;
                total_seg += ocv;
            }

            if chip.is_beta() {
                self.stats.segment_average_volts[chip.segment()] = total_seg / (NUM_CELLS_PER_SEGMENT as f32);
                self.stats.segment_total_volts[chip.segment()] = total_seg;
                total_seg = Voltage::new::<volt>(0_f32);
            }
        }

        self.stats.max = max;
        self.stats.min = min;
        self.stats.avg = total / (NUM_CELLS_TOTAL as f32);
        self.stats.pack = total;
        self.stats.delta = max.value() - min.value();
    }

    /// Reports the two OCV-derived cell faults.
    async fn report_faults(&self) {
        use crate::faults::{self, FaultCommand, PassFailAction};

        if self.is_first_reading {
            return;
        }

        // Update cell voltage too low fault
        if self.stats.min.value() < Voltage::from_volts(2.5) {
            faults::queue(FaultCommand::CellVoltageTooLow(PassFailAction::NotifyBad)).await;
        } else {
            faults::queue(FaultCommand::CellVoltageTooLow(PassFailAction::NotifyOkay)).await;
        }

        // u_TODO - (note: the separate CellChargeVoltageTooHigh fault from TSECU-Shepherd is combined with CellVoltageTooHigh now, they just check different values depending on state)
        let max = if crate::state_machine::state().is_charging() { Voltage::from_volts(4.2) } else { Voltage::from_volts(4.2) };
        if self.stats.max.value() > max {
            faults::queue(FaultCommand::CellVoltageTooHigh(PassFailAction::NotifyBad)).await;
        } else {
            faults::queue(FaultCommand::CellVoltageTooHigh(PassFailAction::NotifyOkay)).await;
        }
    }
}

mod task {
    use super::*;
    use embassy_futures::select::{Either, select};
    use embassy_time::Ticker;

    use crate::can::{
        self,
        types::{CellVoltage, SegmentAverageVoltages, SegmentTotalVoltages},
    };
    use crate::segments::{SEGMENTS_FRESH_DATA_SIGNAL, SegmentId};

    /// How often rest is re-judged
    const REST_CHECK_PERIOD: Duration = Duration::from_millis(100);

    /// The OCV telemetry, on the segments' cadence.
    async fn report() {
        let stats = ocv().try_partial_get(|snapshot| snapshot.get().stats).unwrap_or_else(OcvStats::empty);

        can::send(
            CellVoltage {
                high_val: stats.max.value().get::<volt>(),
                high_cell: stats.max.cell().as_u8(),
                high_chip: stats.max.chip().as_u8(),
                low_val: stats.min.value().get::<volt>(),
                low_chip: stats.min.chip().as_u8(),
                low_cell: stats.min.cell().as_u8(),
                avg_val: stats.avg.get::<volt>(),
            }
            .as_frame(),
        )
        .await;

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/high_val", desc = "Value of highest open cell voltage, in volts.", "{=f32}", stats.max.value().get::<volt>());
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/high_cell", desc = "The cell `high_val` was measured from.", "{}", stats.max.cell());
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/high_chip", desc = "The chip `high_val` was measured from.", "{}", stats.max.chip());
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/low_val", desc = "Value of lowest cell voltage, in volts.", "{=f32}", stats.min.value().get::<volt>());
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/low_cell", desc = "The cell `low_val` was measured from.", "{}", stats.min.cell());
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/low_chip", desc = "The chip `low_val` was measured from.", "{}", stats.min.chip());
            defmt_monitor::monitor!("AnalyzerDebug/OpenCellVoltage/avg_val", desc = "The average cell voltage, in volts.", "{=f32}", stats.avg.get::<volt>());
            defmt_monitor::monitor!("AnalyzerDebug/Misc/avg_ocv", desc = "Average Open Cell Voltage, in volts.", "{=f32}", stats.avg.get::<volt>());
            defmt_monitor::monitor!("AnalyzerDebug/Misc/pack_ocv", desc = "Pack Open Cell Voltage, in volts.", "{=f32}", stats.pack.get::<volt>());
            defmt_monitor::monitor!("AnalyzerDebug/Misc/delta_ocv", desc = "Delta Open Cell Voltage, in volts.", "{=f32}", stats.delta.get::<volt>());
        }

        can::send(
            SegmentAverageVoltages {
                seg1: stats.segment_average_volts[SegmentId::Segment0].get::<volt>(),
                seg2: stats.segment_average_volts[SegmentId::Segment1].get::<volt>(),
                seg3: stats.segment_average_volts[SegmentId::Segment2].get::<volt>(),
                seg4: stats.segment_average_volts[SegmentId::Segment3].get::<volt>(),

                seg5: {
                    #[cfg(feature = "four-segments")]
                    {
                        0_f32
                    }

                    #[cfg(not(feature = "four-segments"))]
                    {
                        stats.segment_average_volts[SegmentId::Segment4].get::<volt>()
                    }
                },
            }
            .as_frame(),
        )
        .await;

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            for segment in SegmentId::iter() {
                defmt_monitor::monitor!(["AnalyzerDebug/SegmentAverageVoltages/seg_{=u8}/", segment.as_u8()], desc = "Average voltage for this segment, in volts.", "{=f32}", stats.segment_average_volts[segment].get::<volt>());
            }
        }

        can::send(
            SegmentTotalVoltages {
                seg1: stats.segment_total_volts[SegmentId::Segment0].get::<volt>(),
                seg2: stats.segment_total_volts[SegmentId::Segment1].get::<volt>(),
                seg3: stats.segment_total_volts[SegmentId::Segment2].get::<volt>(),
                seg4: stats.segment_total_volts[SegmentId::Segment3].get::<volt>(),

                seg5: {
                    #[cfg(feature = "four-segments")]
                    {
                        0_f32
                    }

                    #[cfg(not(feature = "four-segments"))]
                    {
                        stats.segment_total_volts[SegmentId::Segment4].get::<volt>()
                    }
                },
            }
            .as_frame(),
        )
        .await;

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            for segment in SegmentId::iter() {
                defmt_monitor::monitor!(["AnalyzerDebug/SegmentTotalVoltages/seg_{=u8}/", segment.as_u8()], desc = "Total voltage for this segment, in volts.", "{=f32}", stats.segment_total_volts[segment].get::<volt>());
            }
        }

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            let cells = ocv().try_partial_get(|snapshot| snapshot.get().open_cell_voltage);
            for chip in ChipId::iter() {
                for cell in CellId::iter() {
                    defmt_monitor::monitor!(["AnalyzerDebug/Cells/chip_{=u8}/cell_{=u8}/open_cell_voltage", chip.as_u8(), cell.as_u8()], desc = "Open cell voltage, in volts.", "{=f32}", cells.map_or(f32::NAN, |c| c[chip][cell].get::<volt>()));
                }
            }
        }
    }

    #[embassy_executor::task]
    pub async fn ocv_task() {
        let mut tracker = Tracker::new();

        let mut rest_check = Ticker::every(REST_CHECK_PERIOD);
        let mut segments_freshdata_subscription = SEGMENTS_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

        loop {
            match select(rest_check.next(), segments_freshdata_subscription.wait()).await {
                // Plant time: it is the cells relaxing that the rest timer waits out.
                Either::First(()) => tracker.observe(crate::helpers::time::plant_now()),
                Either::Second(()) => {
                    tracker.report_faults().await;
                    report().await;
                },
            }

            #[cfg(defmt_monitor)]
            '_defmt_monitor: {
                defmt_monitor::monitor!("AnalyzerDebug/calc_open_cell_voltage()/ocv_update_allowed", desc = "", "{}", &tracker.rest_allowed);
                defmt_monitor::monitor!("AnalyzerDebug/calc_open_cell_voltage()/ocv_is_first_reading", desc = "", "{}", tracker.is_first_reading);
                defmt_monitor::monitor!("AnalyzerDebug/calc_open_cell_voltage()/ocv_timer", desc = "", "{}", tracker.rest_timer);
                defmt_monitor::monitor!("AnalyzerDebug/Ocv/is_first_reading", desc = "Whether we are still waiting for the first plausible cell voltage to seed the OCV snapshot.", "{=bool}", tracker.is_first_reading);
                defmt_monitor::monitor!("AnalyzerDebug/Ocv/rest_timer_running", desc = "Whether the pack is quiet enough (low current, charger off, not balancing) for the OCV rest timer to be counting.", "{=bool}", tracker.rest_timer.is_some());
                defmt_monitor::monitor!("AnalyzerDebug/Ocv/rest_timer_expired", desc = "Whether the rest timer has run out, so the OCV snapshot is being refreshed every cycle.", "{=bool}", crate::helpers::time::plant_now().is_some_and(|now| tracker.rest_timer.is_some_and(|deadline| deadline.past_at(now))));
                defmt_monitor::monitor!("AnalyzerDebug/Ocv/plant_clock", desc = "Whether the plant clock can be read. False on a rig whose model has not answered RDHILTIME, and while it is false the rest timer is held cleared so no OCV snapshot is taken.", "{=bool}", crate::helpers::time::plant_now().is_some());
                defmt_monitor::monitor!("AnalyzerDebug/Ocv/secs_since_settled", desc = "Seconds since the last OCV snapshot was taken. NaN if none has been taken yet.", "{=f32}", ocv().try_partial_get(|snapshot| snapshot.age().as_millis() as f32 / 1000.0).unwrap_or(f32::NAN));
                defmt_monitor::monitor!("AnalyzerDebug/Ocv/rest_current_source", desc = "Which instrument the current that judged rest came from. None until one has a fresh reading.", "{}", crate::pack::pack_current().map(|current| current.source()));
            }
        }
    }
}

pub use task::ocv_task;
