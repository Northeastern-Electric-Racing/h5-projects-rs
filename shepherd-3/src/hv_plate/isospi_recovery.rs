use adbms2950::api::LineId;
use embassy_time::{Duration, Instant};

use crate::hv_plate::core::alias;

/// PEC failures in a closed window that declare the active line broken.
const BREAK_THRESHOLD: u32 = 16;
/// How long a window stays open once a failure opens it.
const WINDOW: Duration = Duration::from_millis(1500);
/// PEC failures tolerated in one verification tick on the spare.
const VERIFY_THRESHOLD: u32 = 3;
/// Verification ticks allowed before the spare is declared bad too.
const VERIFY_TICKS: u32 = 5;
/// Errors are ignored this long after boot, while the line settles.
const SETTLE_TIME: Duration = Duration::from_millis(1500);

#[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
pub enum State {
    /// Watching the active line for a break.
    Healthy,
    /// Checking the line just switched to.
    Verifying,
    /// Both lines are broken. Terminal.
    Exhausted,
}

impl State {
    /// Whether the supervisor has given up. Nothing after this changes that.
    pub const fn is_exhausted(self) -> bool {
        matches!(self, Self::Exhausted)
    }
}

/// Work the supervisor cannot do itself, because it needs the chip reconfigured.
pub enum Action {
    Nothing,
    /// The active line changed, so the chip on it has to be brought up from scratch.
    Reconfigure,
}

/// Watches the PEC tally for a broken line, and spends the one spare line when it finds one.
pub struct Service {
    state: State,
    /// Lifetime PEC-failure count at the last re-baseline. The driver never clears its tally, so
    /// re-baselining moves this mark rather than zeroing anything.
    pec_baseline: u32,
    /// Set when a failure opens a window. A break is declared only if the window closes over
    /// [`BREAK_THRESHOLD`], so a single spike does not cost the spare line.
    window_closes_at: Option<Instant>,
    settle_until: Instant,
    verify_ticks: u32,
}

impl Service {
    pub fn new(now: Instant) -> Self {
        Self {
            state: State::Healthy,
            pec_baseline: 0,
            window_closes_at: None,
            settle_until: now + SETTLE_TIME,
            verify_ticks: 0,
        }
    }

    /// Where the supervisor is. For diagnostics.
    pub const fn state(&self) -> State {
        self.state
    }

    /// PEC failures since the last re-baseline.
    pub const fn errors_in_window(&self, api: &alias::Api) -> u32 {
        api.device().pec_failed_count().saturating_sub(self.pec_baseline)
    }

    /// Advances the machine one tick.
    pub fn tick(&mut self, api: &mut alias::Api, now: Instant) -> Action {
        match self.state {
            State::Healthy => self.watch(api, now),
            State::Verifying => self.verify(api),
            State::Exhausted => {
                // Keep re-baselining so the diagnostic keeps reporting a per-tick figure rather than a total that only ever grows.
                self.rebaseline(api);
                Action::Nothing
            },
        }
    }

    /// A failure opens a window; a break is declared only if that window closes over threshold.
    fn watch(&mut self, api: &mut alias::Api, now: Instant) -> Action {
        if now < self.settle_until {
            self.rebaseline(api);
            return Action::Nothing;
        }

        let errors = self.errors_in_window(api);

        match self.window_closes_at {
            // Quiet, so hold the baseline against the current tally.
            None if errors == 0 => {
                self.rebaseline(api);
                Action::Nothing
            },
            None => {
                self.window_closes_at = Some(now + WINDOW);
                Action::Nothing
            },
            Some(closes_at) if now < closes_at => Action::Nothing,
            Some(_) => {
                self.window_closes_at = None;

                if errors <= BREAK_THRESHOLD {
                    self.rebaseline(api);
                    return Action::Nothing;
                }

                defmt::error!("HvPlate: link: break on line {} ({} PEC errors in the window).", api.active_line(), errors);
                self.fail_over(api)
            },
        }
    }

    /// Moves to the spare line, or gives up if the spare is already the one that broke.
    fn fail_over(&mut self, api: &mut alias::Api) -> Action {
        if let LineId::B = api.active_line() {
            defmt::error!("HvPlate: link: break on the spare line; no recovery left.");
            self.state = State::Exhausted;
            return Action::Nothing;
        }

        defmt::warn!("HvPlate: link: switching comms to line B.");
        api.set_active_line(LineId::B);
        self.rebaseline(api);
        self.verify_ticks = 0;
        self.state = State::Verifying;
        Action::Reconfigure
    }

    /// Gives the spare a few quiet ticks before trusting it.
    fn verify(&mut self, api: &mut alias::Api) -> Action {
        let errors = self.errors_in_window(api);
        self.rebaseline(api);

        if errors <= VERIFY_THRESHOLD {
            defmt::info!("HvPlate: link: line B verified; recovery complete.");
            self.state = State::Healthy;
            return Action::Nothing;
        }

        self.verify_ticks = self.verify_ticks.saturating_add(1);

        if self.verify_ticks >= VERIFY_TICKS {
            defmt::error!("HvPlate: link: line B still failing after {} ticks; no recovery left.", self.verify_ticks);
            self.state = State::Exhausted;
        } else {
            defmt::warn!("HvPlate: link: line B verification tick failed ({} PEC errors).", errors);
        }

        Action::Nothing
    }

    const fn rebaseline(&mut self, api: &alias::Api) {
        self.pec_baseline = api.device().pec_failed_count();
    }
}
