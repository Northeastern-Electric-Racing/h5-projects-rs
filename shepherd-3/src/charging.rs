//! Stub until it is not

pub mod connected {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Latched by the first charger frame, cleared only by `init_boot`.
    static CHARGER_CONNECTED: AtomicBool = AtomicBool::new(false);

    /// Reports that a charger box frame arrived. Called from CAN Task when charger message is recieved.
    pub fn charger_frame_received() { CHARGER_CONNECTED.store(true, Ordering::Relaxed); }

    /// Whether a charger has announced itself since the last pass through `Boot`.
    pub fn is_charger_connected() -> bool { CHARGER_CONNECTED.load(Ordering::Relaxed) }

    /// Drops the charger latch. Called only by `init_boot`.
    pub fn clear_charger_connected() { CHARGER_CONNECTED.store(false, Ordering::Relaxed); }
}

use embassy_time::{Duration, Instant};
use crate::units::{Voltage, volt};
use crate::helpers::deadline::Deadline;

/// Cell voltage that ends a charge phase.
const MAX_CHARGE_VOLT: Voltage = Voltage::from_volts(4.19);
/// Loaded cell voltage (i.e., voltage read when current is flowing) that aborts charging. If we reach this we need to stop charging and balancing immediately.
const MAX_CHARGE_VOLT_FLT: Voltage = Voltage::from_volts(4.25);

const LONG_CHARGE: Duration = Duration::from_secs(15 * 60);
const SHORT_CHARGE: Duration = Duration::from_secs(20);
const SETTLE: Duration = Duration::from_secs(60);

#[derive(Copy, Clone, PartialEq, defmt::Format)]
pub enum ChargeStage {
    /// Phase 1: bulk charge, up to 15 minutes at a time.
    LongChargeUp(Deadline),
    /// Phase 1 rest, so the cells can settle before their OCV is believed.
    LongSettle(Deadline),
    /// Phase 2: short top-up bursts.
    ShortChargeUp(Deadline),
    /// Phase 2 rest.
    ShortSettle(Deadline),
    /// Settled OCV reached the target. Terminal.
    Done,
    /// Charging was abandoned. Terminal.
    Fault,
}
impl ChargeStage {
    /// Creates a [`LongChargeUp`] with the necessary deadline.
    pub fn long_charge_up() -> Self { Self::LongChargeUp(Deadline::expire_in(LONG_CHARGE)) }
    /// Creates a [`LongSettle`] with the necessary deadline.
    pub fn long_settle() -> Self { Self::LongSettle(Deadline::expire_in(SETTLE)) }
    /// Creates a [`ShortChargeUp`] with the necessary deadline.
    pub fn short_charge_up() -> Self { Self::ShortChargeUp(Deadline::expire_in(SHORT_CHARGE)) }
    /// Creates a [`ShortSettle`] with the necessary deadline.
    pub fn short_settle() -> Self { Self::ShortSettle(Deadline::expire_in(SETTLE)) }
}
impl Default for ChargeStage {
    fn default() -> Self { ChargeStage::long_charge_up() }
}

/// Conditions that serve as inputs for `ChargeStage::next()`.
struct ChargeConditions {
    loaded_full: bool,
    settled_full: bool,
    /// Time at which the tick has occurred.
    tick_time: Instant,
}

impl ChargeStage {
    /// Whether current should be requested from the charger in this stage.
    pub const fn charging_allowed(self) -> bool {
        matches!(self, Self::LongChargeUp(..) | Self::ShortChargeUp(..))
    }

    /// How long the stage runs before it times out, or `None` if it has no timer.
    const fn duration(self) -> Option<Duration> {
        match self {
            Self::LongChargeUp(..) => Some(LONG_CHARGE),
            Self::ShortChargeUp(..) => Some(SHORT_CHARGE),
            Self::LongSettle(..) | Self::ShortSettle(..) => Some(SETTLE),
            Self::Done | Self::Fault => None,
        }
    }

    /// The whole stage graph
    fn next(self, c: ChargeConditions) -> Self {
        match self {
            Self::LongChargeUp(deadline) => {
                if c.loaded_full                       { Self::short_settle() }
                else if deadline.past_at(c.tick_time)  { Self::long_settle() }
                else                                   { self }
            },

            Self::LongSettle(deadline) => {
                if deadline.past_at(c.tick_time) {
                    if c.settled_full { Self::Done }
                    else              { Self::long_charge_up() }
                }
                else                  { self }
            },

            Self::ShortChargeUp(deadline) => {
                if deadline.past_at(c.tick_time) || c.loaded_full { Self::short_settle() }
                else                                              { self }
            },

            Self::ShortSettle(deadline) => {
                if deadline.past_at(c.tick_time) {
                    if c.settled_full { Self::Done }
                    else              { Self::short_charge_up() }
                }
                else                  { self }
            },

            Self::Done => self,

            Self::Fault => self,
        }
    }
}

/// The charge machine's state.
#[derive(Default)]
pub struct Charger {
    stage: ChargeStage,
    deadline: Option<Instant>,
}

impl Charger {
    pub const fn stage(&self) -> ChargeStage {
        self.stage
    }

    /// Starts the algorithm over. Called from `init_charging`, so re-entering `Charging`
    /// always begins a fresh bulk phase rather than resuming a stale one.
    pub fn restart(&mut self, now: Instant) {
        self.stage = ChargeStage::default();
        self.arm(now);
        defmt::info!("Charge stage: starting at {}", self.stage);
    }

    /// Advances one cycle.
    pub fn tick(&mut self, max_cell_voltage: Voltage, max_ocv: Voltage, now: Instant) {
        if max_cell_voltage >= MAX_CHARGE_VOLT_FLT || max_ocv >= MAX_CHARGE_VOLT_FLT {
            self.set_stage(ChargeStage::Fault, now);
            return;
        }

        let expired = self.deadline.is_some_and(|deadline| now >= deadline);

        let next = self.stage.next(ChargeConditions {
            loaded_full: max_cell_voltage >= MAX_CHARGE_VOLT, 
            settled_full: max_ocv >= MAX_CHARGE_VOLT, 
            tick_time: now
        });

        self.set_stage(next, now);
    }

    fn set_stage(&mut self, stage: ChargeStage, now: Instant) {
        if stage == self.stage {
            return;
        }

        defmt::info!("Charge stage: {} -> {}", self.stage, stage);
        self.stage = stage;
        self.arm(now);
    }

    /// Starts the current stage's timer, if it has one.
    fn arm(&mut self, now: Instant) {
        self.deadline = self.stage.duration().map(|d| now + d);
    }
}