use crate::can::{self, types::ShutdownAsReadByBms};
use crate::faults::{self, FaultCommand, PassFailAction};
use crate::helpers::{deadline::Deadline, debouncer::Debouncer};
use crate::state_machine::BmsState;
use crate::units::Voltage;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_futures::join::join;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_time::{Duration, Instant, Ticker};

const FAULT_ASSERTED: Level = Level::Low;
const FAULT_RELEASED: Level = Level::High;
const DEBOUNCE: Duration = Duration::from_millis(200);
const PERIOD: Duration = Duration::from_millis(10);
const TELEMETRY_PERIOD: Duration = Duration::from_millis(500);

static HV_PERMITTED: AtomicBool = AtomicBool::new(false);

/// Whether the shutdown loop has been continuously made up long enough to allow HV.
pub fn hv_permitted() -> bool {
    HV_PERMITTED.load(Ordering::Relaxed)
}

/// One pass of the four taps. High means that part of the loop is made up.
#[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
struct Senses {
    ts_minus: bool,
    ts_plus: bool,
    acc: bool,
    tsip: bool,
}

impl Senses {
    const fn all_made(self) -> bool {
        self.ts_minus && self.ts_plus && self.acc && self.tsip
    }
}

/// Whether the BMS should be breaking the loop.
const fn fault_asserted(state: BmsState, critical_faults_active: bool) -> bool {
    critical_faults_active || !matches!(state, BmsState::Ready | BmsState::Charging)
}

/// Welded-contact detection.
mod weld {
    use super::*;

    pub const PERIOD: Duration = Duration::from_millis(50);
    /// How long the bus gets to bleed down before a live TS counts as welded contacts.
    const SETTLE: Duration = Duration::from_secs(1);
    /// TS as a fraction of pack, above which the bus counts as still connected.
    const RATIO: f32 = 0.90;
    /// Below this batt voltage value the ratio means nothing.
    const MIN_PACK: Voltage = Voltage::from_volts(50.0);

    #[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
    pub enum Weld {
        Welded,
        Clear,
        /// The loop is intact, the pack is too low to judge, or the bleed window has not elapsed.
        Unknown,
    }

    pub struct Detector {
        still_made: Debouncer,
    }

    impl Detector {
        pub const fn new() -> Self {
            Self { still_made: Debouncer::new() }
        }

        pub fn tick(&mut self, now: Instant, hv_permitted: bool, ts: Voltage, batt: Voltage) -> Weld {
            // Not a test we can run. Clear the timer so the next real window starts from zero.
            if hv_permitted || batt < MIN_PACK {
                self.still_made.poll(false, SETTLE, now);
                return Weld::Unknown;
            }

            let still_made = ts >= batt * RATIO;

            if self.still_made.poll(still_made, SETTLE, now) {
                Weld::Welded
            } else if still_made {
                Weld::Unknown
            } else {
                Weld::Clear
            }
        }
    }
}
use weld::Weld;

#[embassy_executor::task]
pub async fn shutdown_task(r: crate::ShutdownResources) {
    join(drive_and_sense_shutdown(r), watch_for_weld()).await;
}

async fn drive_and_sense_shutdown(r: crate::ShutdownResources) -> ! {
    let mut fault = Output::new(r.fault, FAULT_ASSERTED, Speed::Low);

    let ts_minus = Input::new(r.ts_minus_sense, Pull::None);
    let ts_plus = Input::new(r.ts_plus_sense, Pull::None);
    let acc = Input::new(r.acc_sense, Pull::None);
    let tsip = Input::new(r.tsip_sense, Pull::None);

    // Two debouncers to: HV permitted and forbidden
    let (mut to_permitted, mut to_forbidden) = (Debouncer::new(), Debouncer::new());
    let mut permitted = false;

    let mut ticker = Ticker::every(PERIOD);
    let mut telemetry = Deadline::expire_at_beginning_of_time();
    let mut last = (false, Senses { ts_minus: false, ts_plus: false, acc: false, tsip: false });

    loop {
        ticker.next().await;

        // Fault output from just statemachine and faults state
        let asserted = fault_asserted(crate::state_machine::bms_state(), faults::are_critical_faults_active());
        fault.set_level(if asserted { FAULT_ASSERTED } else { FAULT_RELEASED });

        // Sense input
        let senses = Senses {
            ts_minus: ts_minus.is_high(),
            ts_plus: ts_plus.is_high(),
            acc: acc.is_high(),
            tsip: tsip.is_high(),
        };
        let now = Instant::now();
        if to_permitted.poll(senses.all_made(), DEBOUNCE, now) {
            permitted = true;
        }
        if to_forbidden.poll(!senses.all_made(), DEBOUNCE, now) {
            permitted = false;
        }
        HV_PERMITTED.store(permitted, Ordering::Relaxed);

        // Report on every edge, and periodically in between
        let changed = last != (permitted, senses);
        if changed || telemetry.past() {
            if changed {
                defmt::info!("Shutdown: hv_permitted={}, {}", permitted, senses);
            }
            last = (permitted, senses);
            telemetry = Deadline::expire_in(TELEMETRY_PERIOD);

            can::send(
                ShutdownAsReadByBms {
                    shutdown_state: permitted,
                    shutdown_ts_minus_sense: senses.ts_minus,
                    shutdown_ts_plus_sense: senses.ts_plus,
                    shutdown_acc_sense: senses.acc,
                    shutdown_tsip_sense: senses.tsip,
                }
                .as_frame(),
            )
            .await;

            defmt_monitor::monitor!("Shutdown/HvPermitted", desc = "Whether the shutdown loop has been made up long enough to allow the AIRs to close.", "{=bool}", permitted);
            defmt_monitor::monitor!("Shutdown/FaultAsserted", desc = "Whether the BMS is holding its own link in the shutdown loop open.", "{=bool}", asserted);
        }
    }
}

async fn watch_for_weld() -> ! {
    let voltages = crate::hv_plate::cache().get_voltages().nice().ready().await;
    let current_voltage = crate::hv_plate::cache().get_current_voltage().nice().ready().await;

    let mut detector = weld::Detector::new();
    let mut ticker = Ticker::every(weld::PERIOD);

    let mut weld_state = Weld::Unknown;

    loop {
        ticker.next().await;

        let ts = voltages.get().data().ts_voltage;
        let batt = current_voltage.get().data().batt_voltage;

        let weld = detector.tick(Instant::now(), hv_permitted(), ts, batt);
        defmt_monitor::monitor!("Shutdown/Weld", desc = "Welded-contact verdict. Unknown means the test could not run, not that the contacts are fine.", "{}", weld);

        if weld == weld_state {
            continue;
        }

        let action = match weld {
            Weld::Welded => PassFailAction::NotifyBad,
            Weld::Clear => PassFailAction::NotifyOkay,
            Weld::Unknown => {
                weld_state = weld;
                continue;
            },
        };

        // Logged where the transition is recorded, not where it is attempted: a full queue retries
        // every tick, and printing there would repeat the whole time it took to drain.
        if faults::try_queue(FaultCommand::ContactorWeldedFault(action)).is_ok() {
            if weld == Weld::Welded {
                defmt::error!("Shutdown: AIR contacts appear welded -- loop open for the full settle window, TS still at pack voltage.");
            }
            weld_state = weld;
        }
    }
}
