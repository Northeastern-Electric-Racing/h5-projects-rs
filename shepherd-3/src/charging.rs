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

// How long a `LongChargeUp` session should last before expiring.
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

/// How long the `ShortCharge` should run before checking if it should step down voltage again.
const SHORT_CHARGE_CHECK_DURATION: Duration = Duration::from_secs(5);
/// The starting current for `ShortChargeUp`.
const SHORT_CHARGE_START_CURRENT: Current = Current::from_amps(5.0);
/// The rate at which we step down the current.
const SHORT_CHARGE_CURRENT_STEP: Current = Current::from_amps(1.0);
#[derive(Copy, Clone, PartialEq)]
struct ShortChargeUp {
    /// RUNTIME FIELD. When this `ShortChargeUp` should check to step down the voltage next.
    deadline: Deadline,
    /// RUNTIME FIELD. How much current we should request from the charger while in `ShortChargeUp`.
    /// This varies at runtime. It steps down depending on how filled the cells are.
    current: Current,

    /// CONSTANT FIELD. The step at which we step down `current`.
    current_step: Current, 
}
impl ShortChargeUp {
    fn new() -> Self { Self { deadline: Deadline::expire_in(SHORT_CHARGE_CHECK_DURATION), current: SHORT_CHARGE_START_CURRENT, current_step: SHORT_CHARGE_CURRENT_STEP } }
    fn with_current(current: Current) -> Self { Self { deadline: Deadline::expire_in(SHORT_CHARGE_CHECK_DURATION), current, current_step: SHORT_CHARGE_CURRENT_STEP } }

    /// Keeps this the same, but schedules a new deadline after `SHORT_CHARGE_CHECK_DURATION`.
    fn reschedule(&mut self) { 
        self.deadline = Deadline::expire_in(SHORT_CHARGE_CHECK_DURATION);  
    }

    /// Steps down the current, and schedules a new deadline after `SHORT_CHARGE_CHECK_DURATION`.
    fn reschedule_with_step_down(&mut self) { 
        self.deadline = Deadline::expire_in(SHORT_CHARGE_CHECK_DURATION);
        self.current -= self.current_step;
    }
}

/// Data structure to store the state of the most-recently-active charge stage (either LongChargeUp or ShortChargeUp) so it can be restored.
#[derive(Copy, Clone)]
enum ChargeRestoreCache {
    /// The most-recently-active charge stage was LongChargeUp.
    LongChargeUp,
    /// The most-recently-active charge stage was ShortChargeUp. This carries the `Current` being used by the ShortChargeUp from when it was active.
    ShortChargeUp(Current),
}
impl ChargeRestoreCache {
    /// Updates this `ChargeRestoreCache` from `stage`. If `stage` is `LongChargeUp` or `ShortChargeUp`, this will update based on that. Otherwise, this will not update.
    fn update_from_chargestage(&mut self, stage: ChargeStage) {
        *self = match stage {
            ChargeStage::LongChargeUp(_) => Self::LongChargeUp,
            ChargeStage::ShortChargeUp(h) => Self::ShortChargeUp(h.current),
            ChargeStage::Balance(h) => match h.kind {
                BalancingKind::Current(current) => Self::ShortChargeUp(current),
                _ => *self,

            },
            _ => *self,
        }
    }
}
impl Default for ChargeRestoreCache {
    fn default() -> Self { Self::LongChargeUp }
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

    /// Executes `f` if this is currently `Active`.
    fn is_active_and(self, f: impl FnOnce(Deadline) -> bool) -> bool {
        match self {
            Self::Active(d) => f(d),
            _ => false,
        }
    }

    /// Executes `f` if this is currently `Cooldown`.
    fn is_cooldown_and(self, f: impl FnOnce(Deadline) -> bool) -> bool {
        match self {
            Self::Cooldown(d) => f(d),
            _ => false,
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
enum BalancingKind {
    /// Balancing is not allowed to draw current right now. We are just going balancing.
    NoCurrent,
    /// While balancing, we will also draw a bit of current from the charger.
    Current(Current),
}

#[derive(Copy, Clone, PartialEq)]
struct Balance {
    /// RUNTIME FIELD. When this `BalanceAndChargeUp` should expire.
    deadline: BalancingDeadline,

    /// RUNTIME FIELD. Kind of balancing.
    kind: BalancingKind,
}
impl Balance {
    /// Creates a new `Balance` (in the active state, since that is what we start in). Current is not allowed to be drawn.
    fn new_nocurrent() -> Self { Self { deadline: BalancingDeadline::active(), kind: BalancingKind::NoCurrent } }
    /// Creates a new `Balance` (in the active state, since that is what we start in). We will draw current.
    fn new_drawcurrent() -> Self { Self { deadline: BalancingDeadline::active(), kind: BalancingKind::Current(BALANCING_ACTIVE_CURRENT) } }
    /// Transition to "cooldown" state.
    fn to_cooldown(&mut self) { self.deadline = BalancingDeadline::cooldown() }
    /// Turns a `Balance` into a `NoCurrent` `Balance`. This will be a no-op if this `Balance` is already `NoCurrent`. This does not change anything else about the `Balance`.
    fn to_nocurrent(&mut self) { self.kind = BalancingKind::NoCurrent }
    /// Turns a `Balance` into a `Current` `Balance`. This will be a no-op if this `Balance` is already `Current`. This does not change anything else about the `Balance`.
    fn to_drawcurrent(&mut self) { self.kind = BalancingKind::Current(BALANCING_ACTIVE_CURRENT)}
    /// Returns how much current we should draw for this `Balance`. This will return `None` if this `Balance` is `NoCurrent`.
    fn current_draw(&self) -> Option<Current> { 
        match self.kind {
            BalancingKind::Current(curr) => Some(curr),
            BalancingKind::NoCurrent => None,
        }
    }   
    /// Indicates if this `Balancing` has kind `Current` or not.
    fn is_drawcurrent(&self) -> bool { matches!(self.kind, BalancingKind::Current(..)) }
    /// Indicates if this `Balancing` has kind `NoCurrent` or not.
    fn is_nocurrent(&self) -> bool { matches!(self.kind, BalancingKind::NoCurrent) }
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
    /// Sidequest stage that can be entered from the other stages. Entered if we need to balance.
    Balance(Balance),
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
impl ChargeStage {
    /// Creates a ChargeStage, either `LongChargeUp` or `ShortChargeUp`, from a `ChargeRestoreCache`.
    fn from_charge_restore_cache(cache: ChargeRestoreCache) -> Self {
        match cache {
            ChargeRestoreCache::LongChargeUp => Self::LongChargeUp(LongChargeUp::new()),
            ChargeRestoreCache::ShortChargeUp(current) => Self::ShortChargeUp(ShortChargeUp::with_current(current))
        }
    }

    /// Based on the current stage, returns the current we should be requesting from the charger right now. If `None`, we shouldn't be requesting any current.
    fn current_we_should_request(&self) -> Option<Current> {
        match self {
            ChargeStage::LongChargeUp(h) => Some(h.current),
            ChargeStage::ShortChargeUp(h) => Some(h.current),
            ChargeStage::Balance(h) => match (h.kind, h.deadline) {
                (BalancingKind::Current(current), BalancingDeadline::Active(..)) => Some(current),
                _ => None,
            }
            ChargeStage::Settle(_) => None,
            ChargeStage::Done => None,
            ChargeStage::Fault => None,
        }
    }
}


/// If any cell meets or exceeds this voltage, we consider it a fault.
const MAX_CHARGE_VOLT_FLT: Voltage = Voltage::from_volts(4.25);

/// Charge statemachine inputs.
struct Inputs {
    max_ocv: Voltage,
    max_voltage: Voltage,
    min_ocv: Voltage,
    delta_ocv: Voltage,
    charge_restore_cache: ChargeRestoreCache,

}
impl Inputs {
    /// Based on the input values, checks if we have exceeded the fault conditions.
    fn exceeded_fault_voltages(&self) -> bool { (self.max_ocv >= MAX_CHARGE_VOLT_FLT) || (self.max_voltage >= MAX_CHARGE_VOLT_FLT) }
    /// If balancing hasn't already been started, this checks if we should start. 
    fn should_we_start_balancing(&self) -> bool { self.delta_ocv > BALANCE_START_DELTA_V }
    /// If balancing is already active, this checks if we should stop.
    fn should_we_stop_balancing(&self) -> bool { self.delta_ocv <= BALANCE_STOP_DELTA_V }
    
    /// Checks if balancing is allowed right now based on Inputs.
    fn is_balancing_allowed(&self) -> bool {
        self.max_ocv >= BAL_MIN_V 
        && self.max_voltage < MAX_CHARGE_VOLT_FLT 
        // u_TODO && shutdown active 
    }

    /// Returns the appropriate `Balance` phase based on inputs, using `max_voltage` for the comparison.
    fn appropriate_balancing_phase(&self) -> ChargeStage {
        if self.max_voltage >= MAX_CHARGE_VOLT { 
            ChargeStage::Balance(Balance::new_nocurrent()) 
        } else {
            ChargeStage::Balance(Balance::new_drawcurrent())
        }
    }

    /// Returns the appropriate `Balance` phase based on inputs, using `max_ocv` for the comparison.
    fn appropriate_balancing_phase_ocv(&self) -> ChargeStage {
        if self.max_ocv >= MAX_CHARGE_VOLT { 
            ChargeStage::Balance(Balance::new_nocurrent()) 
        } else {
            ChargeStage::Balance(Balance::new_drawcurrent())
        }
    }
}

/// Cell voltage that ends a charge phase.
const MAX_CHARGE_VOLT: Voltage = Voltage::from_volts(4.193);
/// `delta_ocv` at which balancing should start.
const BALANCE_START_DELTA_V: Voltage = Voltage::from_volts(0.020);
/// `delta_ocv` at which balancing should stop.
const BALANCE_STOP_DELTA_V: Voltage = Voltage::from_volts(0.015);
/// Minimum OCV voltage needed to balance.
const BAL_MIN_V: Voltage = Voltage::from_volts(4.05);

impl ChargeStage {
    #[rustfmt::skip]
    fn tick(self, inputs: Inputs) -> Self {
        match self {
            Self::LongChargeUp(h) => {
                if inputs.exceeded_fault_voltages()                                         { Self::Fault }
                else if inputs.max_voltage >= MAX_CHARGE_VOLT                               { Self::ShortChargeUp(ShortChargeUp::new()) }
                else if inputs.should_we_start_balancing() && inputs.is_balancing_allowed() { inputs.appropriate_balancing_phase() /* Self::BalanceAndChargeUp or Self::BalanceOnly */ }
                else if h.deadline.past()                                                   { Self::Settle(Settle::new()) }
                else                                                                        { self }

            },

            Self::ShortChargeUp(mut h) => {
                if inputs.exceeded_fault_voltages()                                                         { Self::Fault }
                else if (inputs.max_voltage >= MAX_CHARGE_VOLT) && (h.current <= SHORT_CHARGE_CURRENT_STEP) { Self::Settle(Settle::new()) }
                else if inputs.should_we_start_balancing() && inputs.is_balancing_allowed()                 { inputs.appropriate_balancing_phase() /* Self::BalanceAndChargeUp or Self::BalanceOnly */ }
                else if h.deadline.past()                                                                   { if inputs.max_voltage >= MAX_CHARGE_VOLT { h.reschedule_with_step_down(); Self::ShortChargeUp(h) } else { h.reschedule(); Self::ShortChargeUp(h)} }
                else                                                                                        { self }
            },

            Self::Balance(mut h) => {
                if inputs.exceeded_fault_voltages()                                 { Self::Fault }
                else if h.is_drawcurrent() && inputs.max_voltage >= MAX_CHARGE_VOLT { h.to_nocurrent(); Self::Balance(h) }
                else if h.deadline.is_active_and(|d| { d.past() })        { h.to_cooldown(); Self::Balance(h) }
                else if h.deadline.is_cooldown_and(|d| { d.past() })      { Self::Settle(Settle::new()) }
                else                                                                { self }
            },

            Self::Settle(h) => {
                if inputs.exceeded_fault_voltages() { Self::Fault }
                else if h.deadline.past() {
                    if inputs.is_balancing_allowed() && inputs.should_we_start_balancing() { inputs.appropriate_balancing_phase_ocv() /* Self::BalanceAndChargeUp or Self::BalanceOnly */ }
                    else if inputs.max_ocv >= MAX_CHARGE_VOLT                              { Self::Done }
                    else                                                                   { Self::from_charge_restore_cache(inputs.charge_restore_cache) /* Self::LongChargeUp or Self::ShortChargeUp */ }
                }
                else                                                                       { self }
            },

            Self::Done => self,

            Self::Fault => self,
        }
    }   

    /// Routines to be called every tick we are in a state. 
    /// 
    /// This is called before the state updates on the tick. So, if a tick results in a state change, this will not be called until the subsequent tick.
    fn on_tick(&mut self) {
        match self {
            Self::LongChargeUp(h) => {
            },

            Self::ShortChargeUp(h) => {
                h.deadline
            },

            Self::Balance(h) => {
                
            },

            Self::Settle(h) => {
                
            },

            Self::Done => (),
            Self::Fault => (),
        }
    }
} 

/// The charge machine's state.
#[derive(Default)]
pub struct Charger {
    stage: ChargeStage,
    cache: ChargeRestoreCache,

    /// Current that we should be requesting from the charger right now. If this is `None`, we shouldn't be requesting any current from the charger.
    current: Option<Current>,
}

impl Charger {
    pub const fn stage(&self) -> ChargeStage {
        self.stage
    }

    /// Starts the algorithm over. Called from `init_charging`, so re-entering `Charging`
    /// always begins a fresh bulk phase rather than resuming a stale one.
    pub fn restart(&mut self) {
        *self = Charger::default()
    }

    /// Advances one cycle.
    pub fn tick(&mut self) {
        let Some((max_ocv, max_voltage, min_ocv, delta_ocv)) = analyzer::analyzer().try_partial_get(|h| (h.data.max_ocv.value(), h.data.max_voltage.value(), h.data.min_ocv.value(), h.data.delta_ocv)) else { return; };
        
        self.stage.on_tick();
        self.stage = self.stage.tick(Inputs {
            max_ocv,
            max_voltage,
            min_ocv,
            delta_ocv,
            charge_restore_cache: self.cache,
        });

        self.cache.update_from_chargestage(self.stage);
        self.current = self.stage.current_we_should_request();
    }
}