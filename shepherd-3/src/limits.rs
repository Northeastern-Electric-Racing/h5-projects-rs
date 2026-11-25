//! Charge and discharge current limits (CCL and DCL), ported from Shepherd's `ccl.c` and `dcl.c`, plus the DTI current
//! commands and the limit-enforcement faults.

use crate::helpers::debouncer::Debouncer;
use crate::helpers::startup_cell::{StartupCell, UnwriteableStartupCell};
use crate::units::Current;
use embassy_time::{Duration, Instant};

// Exported functions
pub use task::limits_task;

/// Converts a duration to a number of limits task cycles, for fault debounce counts. Returns `None` if `duration` isn't a
/// whole multiple of the task period, so call it inside a `const {}` block and `.unwrap()`.
pub const fn duration_to_limits_task_cycles(duration: Duration) -> Option<usize> {
    crate::helpers::duration_to_task_cycles(task::PERIOD, duration)
}

/// The pack's current limits, all positive magnitudes.
#[derive(Copy, Clone, defmt::Format)]
pub struct CurrentLimits {
    /// Instantaneous charge current limit.
    pub inst_ccl: Current,
    /// Instantaneous discharge current limit.
    pub inst_dcl: Current,
    /// Continuous charge current limit, sent to the motor controller.
    pub cont_ccl: Current,
    /// Continuous discharge current limit, sent to the motor controller.
    pub cont_dcl: Current,
}

impl CurrentLimits {
    /// Every limit at its direction's minimum.
    const MINIMUM: Self = Self {
        inst_ccl: Current::from_amps(CHARGE.min_current_a),
        inst_dcl: Current::from_amps(DISCHARGE.min_current_a),
        cont_ccl: Current::from_amps(CHARGE.min_current_a),
        cont_dcl: Current::from_amps(DISCHARGE.min_current_a),
    };
}

/// The most recently published limits.
static LIMITS: StartupCell<CurrentLimits> = StartupCell::new();

/// Gets a handle to the most recently published limits.
// u_TODO - nothing outside this module reads the limits yet.
#[allow(dead_code)]
pub fn limits() -> &'static UnwriteableStartupCell<CurrentLimits> {
    LIMITS.to_unwriteable()
}

/// Settings, from `current_limit_algo_config.h`.
mod config {
    use super::*;

    /// How long current must stay above max before a pulse starts.
    pub const TRIGGER_DEBOUNCE: Duration = Duration::from_millis(100);
    /// How long current must stay below max before a pulse ends early.
    pub const QUIET_DEBOUNCE: Duration = Duration::from_millis(100);
    /// Pulsing is only allowed while `inst` is within this of max.
    pub const PULSE_ENABLE_MARGIN_A: f32 = 0.01;

    /// Settings for one direction.
    // u_TODO - switch these fields to the `units.rs` types once const traits are stabilized, so the configs can still be
    // built in `const` context.
    pub struct LimitConfig {
        pub min_current_a: f32,
        pub max_current_a: f32,
        /// Continuous limit while resting or pulsing.
        pub pulse_current_a: f32,
        /// Continuous limit during a cooldown.
        pub cooldown_current_a: f32,
        /// Temperature curve: minimum at or below `temp_min_c`, ramps up to max by `temp_ramp_up_end_c`.
        pub temp_min_c: f32,
        pub temp_ramp_up_end_c: f32,
        /// Temperature curve: max until `temp_ramp_down_start_c`, ramps down to minimum by `temp_max_c`.
        pub temp_ramp_down_start_c: f32,
        pub temp_max_c: f32,
        /// OCV curve: max at `ocv_full_at_v`, ramps to minimum by `ocv_min_at_v`.
        pub ocv_full_at_v: f32,
        pub ocv_min_at_v: f32,
        pub pulse_duration: Duration,
        pub cooldown_duration: Duration,
    }

    /// Charge settings.
    pub const CHARGE: LimitConfig = {
        const MAX_CURRENT_A: f32 = 60.0;
        const PULSE_PERCENT: f32 = 1.1;
        const COOLDOWN_PERCENT: f32 = 0.9;

        LimitConfig {
            min_current_a: 0.0,
            max_current_a: MAX_CURRENT_A,
            pulse_current_a: MAX_CURRENT_A * PULSE_PERCENT,
            cooldown_current_a: MAX_CURRENT_A * COOLDOWN_PERCENT,
            temp_min_c: 0.0,
            temp_ramp_up_end_c: 10.0,
            temp_ramp_down_start_c: 50.0,
            temp_max_c: 60.0,
            ocv_full_at_v: 4.0,
            ocv_min_at_v: 4.2,
            pulse_duration: Duration::from_millis(2000),
            cooldown_duration: Duration::from_millis(1000),
        }
    };

    /// Discharge settings.
    pub const DISCHARGE: LimitConfig = {
        const MAX_CURRENT_A: f32 = 180.0;
        const PULSE_PERCENT: f32 = 1.1;
        const COOLDOWN_PERCENT: f32 = 0.9;

        LimitConfig {
            min_current_a: 30.0,
            max_current_a: MAX_CURRENT_A,
            pulse_current_a: MAX_CURRENT_A * PULSE_PERCENT,
            cooldown_current_a: MAX_CURRENT_A * COOLDOWN_PERCENT,
            temp_min_c: 0.0,
            temp_ramp_up_end_c: 10.0,
            temp_ramp_down_start_c: 50.0,
            temp_max_c: 60.0,
            ocv_full_at_v: 3.2,
            ocv_min_at_v: 3.0,
            pulse_duration: Duration::from_millis(2000),
            cooldown_duration: Duration::from_millis(1000),
        }
    };

    // Compile-time checks that the settings are well formed.
    const _: () = {
        const fn check(cfg: &LimitConfig) {
            assert!(0.0 <= cfg.min_current_a && cfg.min_current_a < cfg.max_current_a, "current bounds must satisfy 0 <= min < max");
            assert!(cfg.temp_min_c < cfg.temp_ramp_up_end_c, "temperature ramp-up has no width");
            assert!(cfg.temp_ramp_up_end_c <= cfg.temp_ramp_down_start_c, "temperature ramps overlap");
            assert!(cfg.temp_ramp_down_start_c < cfg.temp_max_c, "temperature ramp-down has no width");
            assert!(cfg.ocv_full_at_v != cfg.ocv_min_at_v, "OCV ramp has no width");
        }
        check(&CHARGE);
        check(&DISCHARGE);
    };
}
use config::*;

/// The temperature and OCV curves.
mod curves {
    use super::*;

    /// Linear ramp from `max_a` at `full_at` to `min_a` at `min_at`, flat beyond both ends. NaN gives `min_a`.
    fn ramp(x: f32, full_at: f32, min_at: f32, max_a: f32, min_a: f32) -> f32 {
        let t = (x - full_at) / (min_at - full_at);
        if t.is_nan() {
            return min_a;
        }
        max_a + t.clamp(0.0, 1.0) * (min_a - max_a)
    }

    /// Limit from one cell temperature.
    fn from_temp(cfg: &LimitConfig, temp_c: f32) -> f32 {
        let cold = ramp(temp_c, cfg.temp_ramp_up_end_c, cfg.temp_min_c, cfg.max_current_a, cfg.min_current_a);
        let hot = ramp(temp_c, cfg.temp_ramp_down_start_c, cfg.temp_max_c, cfg.max_current_a, cfg.min_current_a);
        cold.min(hot)
    }

    /// Limit from one cell's open-circuit voltage.
    fn from_ocv(cfg: &LimitConfig, ocv_v: f32) -> f32 {
        ramp(ocv_v, cfg.ocv_full_at_v, cfg.ocv_min_at_v, cfg.max_current_a, cfg.min_current_a)
    }

    /// Lowest of the temperature and OCV curves, clamped to `[min, max]`.
    pub fn inst_limit(cfg: &LimitConfig, cells: &CellInputs, ocv_v: f32) -> f32 {
        let temp = from_temp(cfg, cells.min_temp_c).min(from_temp(cfg, cells.max_temp_c));
        temp.min(from_ocv(cfg, ocv_v)).clamp(cfg.min_current_a, cfg.max_current_a)
    }
}
use curves::*;

/// The Rest / Pulse / Cooldown state machine for the continuous limit.
mod pulse {
    use super::*;

    /// Which state a [`PulseMachine`] is in, for diagnostics.
    #[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
    pub enum PulsePhase {
        Rest,
        Pulse,
        Cooldown,
    }

    enum PulseState {
        /// Offers the pulse current, so current can rise above max and start a pulse.
        Rest { above_ceiling: Debouncer },
        /// Current may stay above max until `ends_at`.
        Pulse { ends_at: Instant, below_ceiling: Debouncer },
        /// Held at the cooldown current until `ends_at`.
        Cooldown { ends_at: Instant },
    }

    impl PulseState {
        const fn rest() -> Self {
            Self::Rest { above_ceiling: Debouncer::new() }
        }
    }

    /// Turns one direction's instantaneous limit and pack current into its continuous limit.
    pub struct PulseMachine {
        cfg: &'static LimitConfig,
        state: PulseState,
    }

    impl PulseMachine {
        pub const fn new(cfg: &'static LimitConfig) -> Self {
            Self { cfg, state: PulseState::rest() }
        }

        pub fn reset(&mut self) {
            self.state = PulseState::rest();
        }

        pub const fn phase(&self) -> PulsePhase {
            match self.state {
                PulseState::Rest { .. } => PulsePhase::Rest,
                PulseState::Pulse { .. } => PulsePhase::Pulse,
                PulseState::Cooldown { .. } => PulsePhase::Cooldown,
            }
        }

        /// Advances one tick and returns the continuous limit. `current_a` is positive in this machine's direction.
        pub fn step(&mut self, now: Instant, inst_a: f32, current_a: f32, inhibited: bool) -> f32 {
            let cfg = self.cfg;

            let pulse_allowed = !inhibited && inst_a >= cfg.max_current_a - PULSE_ENABLE_MARGIN_A;
            if !pulse_allowed {
                return self.current_when_step_not_allowed(now, inst_a);
            }

            match &mut self.state {
                PulseState::Rest { above_ceiling } => {
                    if above_ceiling.calculate(current_a > cfg.max_current_a, TRIGGER_DEBOUNCE, now) {
                        self.state = PulseState::Pulse {
                            ends_at: now.saturating_add(cfg.pulse_duration),
                            below_ceiling: Debouncer::new(),
                        };
                    }
                    cfg.pulse_current_a
                },

                PulseState::Pulse { ends_at, below_ceiling } => {
                    // A pulse that runs its full length or ends early both go to cooldown.
                    if now >= *ends_at || below_ceiling.calculate(current_a < cfg.max_current_a, QUIET_DEBOUNCE, now) { self.enter_cooldown(now) } else { cfg.pulse_current_a }
                },

                PulseState::Cooldown { ends_at } => {
                    if now >= *ends_at {
                        self.reset();
                    }
                    cfg.cooldown_current_a
                },
            }
        }

        /// One tick while pulsing is not allowed. A running pulse or cooldown still finishes its cooldown.
        fn current_when_step_not_allowed(&mut self, now: Instant, inst_a: f32) -> f32 {
            match self.state {
                PulseState::Rest { .. } => {
                    self.reset();
                    inst_a
                },
                PulseState::Pulse { .. } => self.enter_cooldown(now).min(inst_a),
                PulseState::Cooldown { ends_at } if now < ends_at => self.cfg.cooldown_current_a.min(inst_a),
                PulseState::Cooldown { .. } => {
                    self.reset();
                    inst_a
                },
            }
        }

        fn enter_cooldown(&mut self, now: Instant) -> f32 {
            self.state = PulseState::Cooldown { ends_at: now.saturating_add(self.cfg.cooldown_duration) };
            self.cfg.cooldown_current_a
        }
    }
}
use pulse::*;

/// Both directions together.
mod limiter {
    use super::*;

    /// Cell readings from the analyzer.
    #[derive(Copy, Clone)]
    pub struct CellInputs {
        pub min_temp_c: f32,
        pub max_temp_c: f32,
        pub min_ocv_v: f32,
        pub max_ocv_v: f32,
    }

    /// Everything one tick reads.
    pub struct Inputs {
        /// `None` until the analyzer has taken its first OCV snapshot.
        pub cells: Option<CellInputs>,
        /// Positive is discharge. `None` when stale.
        pub pack_current_a: Option<f32>,
        pub pulse_inhibited: bool,
    }

    pub struct Limiter {
        pub charge: PulseMachine,
        pub discharge: PulseMachine,
    }

    impl Limiter {
        pub const fn new() -> Self {
            Self {
                charge: PulseMachine::new(&CHARGE),
                discharge: PulseMachine::new(&DISCHARGE),
            }
        }

        fn reset(&mut self) {
            self.charge.reset();
            self.discharge.reset();
        }

        /// Advances one tick and returns the limits to publish.
        pub fn tick(&mut self, now: Instant, inputs: &Inputs) -> CurrentLimits {
            let Some(cells) = inputs.cells else {
                self.reset();
                return CurrentLimits::MINIMUM;
            };

            // Charge is limited by the fullest cell, discharge by the emptiest.
            let inst_ccl = inst_limit(&CHARGE, &cells, cells.max_ocv_v);
            let inst_dcl = inst_limit(&DISCHARGE, &cells, cells.min_ocv_v);

            let (cont_ccl, cont_dcl) = match inputs.pack_current_a {
                // Positive pack current is discharge, so charge sees it negated.
                Some(pack_current_a) => (self.charge.step(now, inst_ccl, -pack_current_a, inputs.pulse_inhibited), self.discharge.step(now, inst_dcl, pack_current_a, inputs.pulse_inhibited)),
                // Can't track pulses without the current, so hold both at minimum.
                None => {
                    self.reset();
                    (CHARGE.min_current_a, DISCHARGE.min_current_a)
                },
            };

            CurrentLimits {
                inst_ccl: Current::from_amps(inst_ccl),
                inst_dcl: Current::from_amps(inst_dcl),
                cont_ccl: Current::from_amps(cont_ccl),
                cont_dcl: Current::from_amps(cont_dcl),
            }
        }
    }
}
use limiter::*;

mod task {
    use super::*;
    use crate::faults::{self, FaultCommand, FaultId, PassFailAction};
    use crate::helpers::deadline::Deadline;
    use crate::state_machine::{self, BmsState};
    use crate::{can, pack};
    use cangen::ToCanFrame;
    use embassy_time::Ticker;

    pub(super) const PERIOD: Duration = Duration::from_millis(20);
    const DIAGNOSTIC_PERIOD: Duration = Duration::from_millis(200);
    /// Kept long so brief CAN hiccups don't drop the limits.
    const PACK_CURRENT_STALE_AFTER: Duration = Duration::from_secs(5);

    /// `None` until the analyzer has taken its first OCV snapshot.
    fn read_cells() -> Option<CellInputs> {
        pack::analyzer::analyzer()
            .try_partial_get(|holder| {
                let data = &holder.data;
                data.last_settled_at.is_some().then(|| CellInputs {
                    min_temp_c: data.min_temp.value().as_celsius(),
                    max_temp_c: data.max_temp.value().as_celsius(),
                    min_ocv_v: data.min_ocv.value().as_volts(),
                    max_ocv_v: data.max_ocv.value().as_volts(),
                })
            })
            .flatten()
    }

    /// Pack current in amps, positive is discharge. `None` when stale.
    fn read_pack_current(now: Instant) -> Option<f32> {
        let (current, received_at) = pack::analyzer::can_pack_current()?;
        (now.saturating_duration_since(received_at) <= PACK_CURRENT_STALE_AFTER).then(|| current.as_amps())
    }

    /// Pulsing is held off while charging or while a critical or segment comms fault is active.
    fn pulse_inhibited(state: BmsState) -> bool {
        state.is_charging() || faults::are_critical_faults_active() || faults::get_fault(FaultId::SegmentCommsFault).is_active()
    }

    /// Sends the continuous limits to the DTI while `Ready`, and zero in every other state. The VCU also faults if
    /// `MaxDcCurrentCommand` stops arriving.
    fn send_dti_commands(state: BmsState, limits: &CurrentLimits) {
        let (max_dc_a, max_brake_a) = match state {
            // The DTI only accepts a negative brake current.
            BmsState::Ready => (limits.cont_dcl.as_amps(), -limits.cont_ccl.as_amps()),
            _ => (0.0, 0.0),
        };
        // A full queue is retried on the next tick.
        let _ = can::try_send(cangen::MaxDcCurrentCommand::new().with_max_dc_current_target(max_dc_a).to_can_frame());
        let _ = can::try_send(cangen::MaxDcBrakeCurrentCommand::new().with_max_dc_brake_current_target(max_brake_a).to_can_frame());
    }

    /// Reports whether the pack current is over either continuous limit. Skipped while the current is stale.
    async fn check_enforcement(pack_current_a: Option<f32>, limits: &CurrentLimits) {
        let Some(current_a) = pack_current_a else {
            return;
        };
        faults::queue(FaultCommand::DischargeLimitEnforcementFault(if current_a > limits.cont_dcl.as_amps() { PassFailAction::NotifyBad } else { PassFailAction::NotifyOkay })).await;
        faults::queue(FaultCommand::ChargeLimitEnforcement(if current_a < -limits.cont_ccl.as_amps() { PassFailAction::NotifyBad } else { PassFailAction::NotifyOkay })).await;
    }

    #[embassy_executor::task]
    pub async fn limits_task() {
        let mut limiter = Limiter::new();
        let mut ticker = Ticker::every(PERIOD);
        let mut diagnostic_deadline = Deadline::expire_at_beginning_of_time();

        // Only queue `PackCurrentStale` on a change, so the fault queue isn't flooded.
        let mut reported_stale = false;

        loop {
            ticker.next().await;

            let now = Instant::now();
            let state = state_machine::state();
            let inputs = Inputs {
                cells: read_cells(),
                pack_current_a: read_pack_current(now),
                pulse_inhibited: pulse_inhibited(state),
            };
            let limits = limiter.tick(now, &inputs);
            LIMITS.set(limits);

            send_dti_commands(state, &limits);
            check_enforcement(inputs.pack_current_a, &limits).await;

            let stale = inputs.pack_current_a.is_none();
            if stale != reported_stale {
                // If the queue is full, this is retried on the next tick.
                if faults::try_queue(FaultCommand::PackCurrentStale(if stale { PassFailAction::NotifyBad } else { PassFailAction::NotifyOkay })).is_ok() {
                    if stale {
                        defmt::warn!("Limits: pack current is stale; continuous limits held at minimum.");
                    } else {
                        defmt::info!("Limits: pack current is fresh again.");
                    }
                    reported_stale = stale;
                }
            }

            if diagnostic_deadline.past() {
                diagnostic_deadline = Deadline::expire_in(DIAGNOSTIC_PERIOD);
                defmt_monitor::monitor!("Limits/InstCcl", desc = "Instantaneous charge current limit, in amps.", "{=f32}", limits.inst_ccl.as_amps());
                defmt_monitor::monitor!("Limits/InstDcl", desc = "Instantaneous discharge current limit, in amps.", "{=f32}", limits.inst_dcl.as_amps());
                defmt_monitor::monitor!("Limits/ContCcl", desc = "Continuous charge current limit, in amps.", "{=f32}", limits.cont_ccl.as_amps());
                defmt_monitor::monitor!("Limits/ContDcl", desc = "Continuous discharge current limit, in amps.", "{=f32}", limits.cont_dcl.as_amps());
                defmt_monitor::monitor!("Limits/ChargePulse", desc = "Charge pulse machine state.", "{}", limiter.charge.phase());
                defmt_monitor::monitor!("Limits/DischargePulse", desc = "Discharge pulse machine state.", "{}", limiter.discharge.phase());
                defmt_monitor::monitor!("Limits/PackCurrent", desc = "Pack current in amps, positive is discharge. NaN when stale.", "{=f32}", inputs.pack_current_a.unwrap_or(f32::NAN));
                defmt_monitor::monitor!("Limits/CellInputsReady", desc = "Whether the analyzer has taken its first OCV snapshot.", "{=bool}", inputs.cells.is_some());
                defmt_monitor::monitor!("Limits/PulseInhibited", desc = "Pulsing held off by charging or a fault.", "{=bool}", inputs.pulse_inhibited);
            }
        }
    }
}
