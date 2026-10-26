//! isoSPI break detection and recovery

use adbms2950::api::LineId;
use adbms2950::line::Error;
use embassy_time::{Duration, Instant};

use crate::hv_plate::core::alias;

const PEC_ERROR_THRESHOLD: u32 = 16;
/// Errors in a single cycle needed to open an accumulation window.
const PEC_ACCUM_START_THRESH: u32 = 0;
const ACCUM_PERIOD: Duration = Duration::from_millis(1500);
const VALIDATION_THRESHOLD: u32 = 3;
const VERIFICATION_READS: u32 = 5;
/// Maks for  startup noise at the read layer
const STARTUP_MASK_TIME: Duration = Duration::from_millis(1500);

#[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
pub(super) enum State {
    Normal,
    BreakDetected,
    Verifying,
    ServiceSuccess,
    ServiceFailed,
}

/// Work [`Service::evaluate`] could not do itself, because it needs the wire.
pub(super) enum Action {
    None,
    Restart,
}

pub(super) struct Service {
    state: State,
    /// Lifetime PEC-failure count at the last reset. The driver never clears its tally, so
    /// "reset" means re-baselining against it.
    pec_baseline: u32,
    accum_deadline: Option<Instant>,
    startup_mask_until: Instant,
    verification_attempts: u32,
    /// Latches on the first successful recovery. There is only one spare line.
    recovery_successful: bool,
    fault_latched: bool,
    started: bool,
}

impl Service {
    pub(super) fn new(now: Instant) -> Self {
        Self {
            state: State::Normal,
            pec_baseline: 0,
            accum_deadline: None,
            startup_mask_until: now + STARTUP_MASK_TIME,
            verification_attempts: 0,
            recovery_successful: false,
            fault_latched: false,
            started: false,
        }
    }

    /// Keeps the chip configured and the isoSPI link on a working port.
    /// Returns true on restarted
    pub(super) async fn run<F>(&mut self, api: &mut alias::Api, startup_fn: F) -> bool
    where
        F: AsyncFn(&mut alias::Api) -> Result<(), Error<alias::SpiError>>,
    {
        let mut restarted = false;

        if self.needs_startup(api) {
            restarted = true;

            match startup_fn(api).await {
                Ok(()) => self.mark_started(),
                Err(err) => {
                    defmt::error!("HvPlate: isoSPI: startup failed, will retry next cycle. Error: {}", err);
                    return restarted;
                },
            }
        }

        if let Action::Restart = self.evaluate(api, Instant::now()) {
            restarted = true;

            if let Err(err) = startup_fn(api).await {
                defmt::error!("HvPlate: isoSPI: restart after line switch failed. Error: {}", err);
            }
        }

        restarted
    }

    /// Whether the chip needs configuring: either it never has been, or it reset and threw
    /// the configuration away.
    #[cfg_attr(feature = "hil", allow(unused_variables))]
    fn needs_startup(&self, api: &alias::Api) -> bool {
        if !self.started {
            return true;
        }

        // HIL doesn't do command counter
        #[cfg(not(feature = "hil"))]
        if api.device().suspected_reset() {
            defmt::warn!("HvPlate: isoSPI: chip reported command counter 0 while we expected {}; reconfiguring.", api.device().expected_command_counter());
            return true;
        }

        false
    }

    const fn mark_started(&mut self) {
        self.started = true;
    }

    /// Where the recovery machine currently is. For diagnostics.
    pub(super) const fn state(&self) -> State {
        self.state
    }

    /// PEC failures accumulated since the last re-baseline -- the quantity the break detector
    /// thresholds on. For diagnostics.
    pub(super) const fn errors_in_window(&self, api: &alias::Api) -> u32 {
        self.errors_since_reset(api)
    }

    fn evaluate(&mut self, api: &mut alias::Api, now: Instant) -> Action {
        let action = self.step(api, now);

        let report = match self.state {
            State::ServiceFailed => crate::faults::PassFailAction::NotifyBad,
            _ => crate::faults::PassFailAction::NotifyOkay,
        };
        let _ = crate::faults::try_queue(crate::faults::FaultCommand::HvPlateCommsFault(report));

        action
    }

    fn step(&mut self, api: &mut alias::Api, now: Instant) -> Action {
        match self.state {
            State::Normal => {
                self.detect_break(api, now);
                Action::None
            },

            State::BreakDetected => {
                if self.recovery_successful {
                    // The spare is spent, so there is nothing to switch to. Terminal, and the
                    // comms fault stays latched for as long as we keep reporting it.
                    defmt::error!("HvPlate: isoSPI: break on the spare line; no recovery left.");
                    self.state = State::ServiceFailed;
                    Action::None
                } else {
                    defmt::warn!("HvPlate: isoSPI: recovery started, switching comms to line B.");
                    api.set_active_line(LineId::B);
                    self.reset_accumulator(api);
                    self.state = State::Verifying;
                    Action::Restart
                }
            },

            State::Verifying => {
                if self.verification_attempts >= VERIFICATION_READS {
                    defmt::error!("HvPlate: isoSPI: verification failed after {} attempts.", self.verification_attempts);
                    self.state = State::ServiceFailed;
                } else {
                    let errors = self.errors_since_reset(api);
                    if errors <= VALIDATION_THRESHOLD {
                        defmt::info!("HvPlate: isoSPI: line B verification passed.");
                        self.state = State::ServiceSuccess;
                        self.recovery_successful = true;
                    } else {
                        defmt::warn!("HvPlate: isoSPI: line B verification failed ({} PEC errors).", errors);
                    }
                    self.reset_accumulator(api);
                    self.verification_attempts = self.verification_attempts.saturating_add(1);
                }
                Action::None
            },

            State::ServiceSuccess => {
                defmt::info!("HvPlate: isoSPI: recovery complete.");
                self.state = State::Normal;
                Action::None
            },

            State::ServiceFailed => {
                if !self.fault_latched {
                    defmt::error!("HvPlate: isoSPI: recovery failed. Comms fault latched.");
                    self.recovery_successful = false;
                    self.fault_latched = true;
                }
                self.reset_accumulator(api);
                Action::None
            },
        }
    }

    /// Two-stage: a spike arms a window, and a break is declared only if that window closes
    /// over threshold.
    fn detect_break(&mut self, api: &alias::Api, now: Instant) {
        if now < self.startup_mask_until {
            self.reset_accumulator(api);
            return;
        }

        let errors = self.errors_since_reset(api);

        match self.accum_deadline {
            None => {
                if errors > PEC_ACCUM_START_THRESH {
                    self.accum_deadline = Some(now + ACCUM_PERIOD);
                } else {
                    self.reset_accumulator(api);
                }
            },
            Some(deadline) if now >= deadline => {
                if errors > PEC_ERROR_THRESHOLD {
                    defmt::error!("HvPlate: isoSPI: break detected on line {} ({} PEC errors in the window).", api.active_line(), errors);
                    self.state = State::BreakDetected;
                }
                self.accum_deadline = None;
                self.reset_accumulator(api);
            },
            Some(_) => {},
        }
    }

    const fn errors_since_reset(&self, api: &alias::Api) -> u32 {
        api.device().pec_failed_count().saturating_sub(self.pec_baseline)
    }

    const fn reset_accumulator(&mut self, api: &alias::Api) {
        self.pec_baseline = api.device().pec_failed_count();
    }
}
