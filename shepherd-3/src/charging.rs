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
use crate::units::{Voltage, Current};
use crate::helpers::deadline::Deadline;
use crate::pack::analyzer;

/// Cell voltage that ends a charge phase.
const MAX_CHARGE_VOLT: Voltage = Voltage::from_volts(4.19);
/// Loaded cell voltage (i.e., voltage read when current is flowing) that aborts charging. If we reach this we need to stop charging and balancing immediately.
const MAX_CHARGE_VOLT_FLT: Voltage = Voltage::from_volts(4.25);

const LONG_CHARGE: Duration = Duration::from_secs(15 * 60);
const SHORT_CHARGE: Duration = Duration::from_secs(20);
const SETTLE: Duration = Duration::from_secs(60);

/// Configuration for the different charge stages.
mod stages {
    use super::*;

    /// How long a `LongChargeUp` session should last before expiring.
    const LONG_CHARGE_DURATION: Duration = Duration::from_secs(15 * 60);
    /// How much current we should request from the charger while in `LongChargeUp`.
    const LONG_CHARGE_CURRENT: Current = Current::from_amps(5.0);
    #[derive(Copy, Clone, PartialEq)]
    struct LongChargeUp {
        /// RUNTIME FIELD. When this `LongChargeUp` session should expire (based on LONG_CHARGE_DURATION).
        deadline: Deadline,

        /// CONSTANT FIELD. How much current we should request from the charger while in `LongChargeUp`.
        current: Current 
    }
    impl LongChargeUp {
        fn new() -> Self { Self { deadline: Deadline::expire_in(LONG_CHARGE_DURATION), current: LONG_CHARGE_CURRENT }}
    }

    /// How long a `ShortChargeUp` session should last before expiring.
    const SHORT_CHARGE_DURATION: Duration = Duration::from_secs(20);
    /// The starting current for `ShortChargeUp`.
    const SHORT_CHARGE_START_CURRENT: Current = Current::from_amps(5.0);
    /// The rate at which we step down the current.
    const SHORT_CHARGE_CURRENT_STEP: Current = Current::from_amps(1.0);
    #[derive(Copy, Clone, PartialEq)]
    struct ShortChargeUp {
        /// RUNTIME FIELD. When this `ShortChargeUp` session should expire (based on SHORT_CHARGE_DURATION).
        deadline: Deadline,
        /// RUNTIME FIELD. How much current we should request from the charger while in `ShortChargeUp`.
        /// This varies at runtime. It steps down depending on how filled the cells are.
        current: Current,

        /// CONSTANT FIELD. The step at which we step down `current`.
        current_step: Current, 
    }
    impl ShortChargeUp {
        fn new() -> Self { Self { deadline: Deadline::expire_in(SHORT_CHARGE_DURATION), current: SHORT_CHARGE_START_CURRENT, current_step: SHORT_CHARGE_CURRENT_STEP } }
    }

    /// Balancing period.
    const BALANCING_ACTIVE_DURATION: Duration = Duration::from_secs(60);
    /// Cooldown after balancing for BALANCING_ACTIVE_DURATION.
    const BALANCING_COOLDOWN_DURATION: Duration = Duration::from_secs(120);
    /// Current to request from the charger while in the "active" stage.
    const BALANCING_ACTIVE_CURRENT: Current = Current::from_amps(0.5);

    #[derive(Copy, Clone, PartialEq)]

    enum BalancingDeadline {
        /// We are actively in the "active" stage of balancing, so this is our deadline for that ending.
        Active(Deadline),
        /// We are actively in the "cooldown" stage of balancing, so this is our deadline for that ending.
        Cooldown(Deadline),
    }
    impl BalancingDeadline {
        fn active() -> Self { Self::Active(Deadline::expire_in(BALANCING_ACTIVE_DURATION)) }
        fn cooldown() -> Self { Self::Cooldown(Deadline::expire_in(BALANCING_COOLDOWN_DURATION)) }
    }

    #[derive(Copy, Clone, PartialEq)]
    struct BalanceAndChargeUp {
        /// RUNTIME FIELD. When this `BalanceAndChargeUp` should expire.
        deadline: BalancingDeadline,

        /// CONSTANT FIELD. Current to request from the charger while in the "active" stage.
        current: Current,
    }
    impl BalanceAndChargeUp {
        /// Creates a new `BalanceAndChargeUp` (in the active state, since that is what we start in).
        fn new() -> Self { Self { deadline: BalancingDeadline::active(), current: BALANCING_ACTIVE_CURRENT } }
        /// Transition to "cooldown" state.
        fn to_cooldown(&mut self) { self.deadline = BalancingDeadline::cooldown() }
    }

    #[derive(Copy, Clone, PartialEq)]
    struct BalanceOnly {
        /// RUNTIME FIELD. When this `BalanceOnly` should expire.
        deadline: BalancingDeadline,
    }
    impl BalanceOnly {
        /// Creates a new `BalanceOnly` (in the active state, since that is what we start in).
        fn new() -> Self { Self { deadline: BalancingDeadline::active() } }
        /// Transition to "cooldown" state.
        fn to_cooldown(&mut self) { self.deadline = BalancingDeadline::cooldown() }
    }

    /// How long a `Settle` should last before expiring.
    const SETTLE_DURATION: Duration = Duration::from_secs(60);

    #[derive(Copy, Clone, PartialEq)]
    struct Settle {
        /// RUNTIME FIELD. When this `Settle` should expire.
        deadline: Deadline,
    }
    impl Settle {
        /// Creates a new `Settle`.
        fn new() -> Self { Self { deadline: Deadline::expire_in(SETTLE_DURATION) } }
    }



    #[derive(Copy, Clone, PartialEq)]
    pub enum ChargeStage {
        /// First charging stage, in which we pull full current from the charger. This is what is used for like the first 90%+ percent of charge.
        LongChargeUp(LongChargeUp),
        /// Second charging stage (entered when we approach full charge). We charge with less current, and keep decreasing that current gradually as we keep moving towards full charge.
        ShortChargeUp(ShortChargeUp),
        /// Sidequest stage that can be entered from the other stages. Entered if we need to balance, but a little bit of current can still be pulled from the charger at the same time.
        BalanceAndChargeUp(BalanceAndChargeUp),
        /// Sidequest stage that can be entered from the other stages. Entered if we need to balance, but cannot pull any more current from the charger because at least one cell has reached the max/full voltage. 
        BalanceOnly(BalanceOnly),
        /// Short phase that is transitioned to after the Deadline of the active phase expires. It is used to temporarily turn off charging to observe OCV, and then decide the next steps to take (i.e., if we are DONE, or if we must go back to another stage).
        Settle(Settle),
        /// We are done with charging.
        Done,
        /// Something bad happened.
        Fault,
    }
    impl Default for ChargeStage {
        fn default() -> Self { ChargeStage::LongChargeUp(LongChargeUp::new()) }
    }


    /// If any cell meets or exceeds this voltage, we consider it a fault.
    const MAX_CHARGE_VOLT_FLT: Voltage = Voltage::from_volts(4.25);

    /// Charge statemachine inputs.
    struct Inputs {
        max_ocv: Voltage,
        max_voltage: Voltage,
    }
    impl Inputs {
        /// Based on the input values, checks if we have exceeded the fault conditions.
        fn exceeded_fault_voltages(&self) -> bool {
            (self.max_ocv >= MAX_CHARGE_VOLT_FLT) || (self.max_voltage >= MAX_CHARGE_VOLT_FLT)
        }
    }

    /// Cell voltage that ends a charge phase.
    const MAX_CHARGE_VOLT: Voltage = Voltage::from_volts(4.19);

    impl ChargeStage {
        fn next(self, inputs: Inputs) -> Self {
            match self {
                Self::LongChargeUp(h) => {
                    if inputs.exceeded_fault_voltages()           { Self::Fault }
                    else if inputs.max_voltage >= MAX_CHARGE_VOLT { Self::ShortChargeUp(ShortChargeUp::new()) }
                    // u_TODO if balancing needed && balancing possible, then go into BalanceAndChargeUp if less than max charge volts, or go into BalanceOnly
                    else if h.deadline.past()                     { Self::Settle(Settle::new()) },
                    else                                          { self }

                },

                Self::ShortChargeUp(h) => {
                    if inputs.exceeded_fault_voltages() { Self::Fault }
                },

                Self::BalanceAndChargeUp(h) => {
                    if inputs.exceeded_fault_voltages() { Self::Fault }
                },

                Self::BalanceOnly(h) => {
                    if inputs.exceeded_fault_voltages() { Self::Fault }
                },

                Self::Settle(h) => {
                    if inputs.exceeded_fault_voltages() { Self::Fault }
                },

                Self::Done => self,

                Self::Fault => self,
            }
        }   
    } 
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
}

impl Charger {
    pub const fn stage(&self) -> ChargeStage {
        self.stage
    }

    /// Starts the algorithm over. Called from `init_charging`, so re-entering `Charging`
    /// always begins a fresh bulk phase rather than resuming a stale one.
    pub fn restart(&mut self, now: Instant) {
        self.stage = ChargeStage::default();
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
}