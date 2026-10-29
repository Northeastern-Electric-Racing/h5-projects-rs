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
use crate::helpers::snapshot_cell::{SnapshotCell, LazySnapshotCell};

mod state {
    use super::*;

    // How long a `LongChargeUp` session should last before expiring.
    const LONG_CHARGE_DURATION: Duration = Duration::from_secs(15 * 60);
    /// How much current we should request from the charger while in `LongChargeUp`.
    const LONG_CHARGE_CURRENT: Current = Current::from_amps(5.0);
    #[derive(Copy, Clone, PartialEq)]
    pub struct LongChargeUp {
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
    pub struct ShortChargeUp {
        /// RUNTIME FIELD. When this `ShortChargeUp` should check to step down the voltage next.
        deadline: Deadline,
        /// RUNTIME FIELD. How much current we should request from the charger while in `ShortChargeUp`.
        /// This varies at runtime. It steps down depending on how filled the cells are.
        current: Current,

        /// CONSTANT FIELD. The step at which we step down `current`.
        current_step: Current, 
    }
    impl ShortChargeUp {
        fn new() -> Self { Self { deadline: Deadline::expire_at_beginning_of_time(), current: SHORT_CHARGE_START_CURRENT, current_step: SHORT_CHARGE_CURRENT_STEP } }
        fn with_current(current: Current) -> Self { Self { deadline: Deadline::expire_at_beginning_of_time(), current, current_step: SHORT_CHARGE_CURRENT_STEP } }

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
                    BalancingKind::NoCurrent => Self::ShortChargeUp(BALANCING_ACTIVE_CURRENT),
                    BalancingKind::Current(_) => *self,
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
    pub struct Balance {
        /// RUNTIME FIELD. When this `BalanceAndChargeUp` should expire.
        deadline: BalancingDeadline,

        /// RUNTIME FIELD. Kind of balancing.
        kind: BalancingKind,
    }
    
    #[allow(dead_code)]
    impl Balance {
        /// Creates a new `Balance` (in the active state, since that is what we start in). Current is not allowed to be drawn.
        pub(super) fn new_nocurrent() -> Self { Self { deadline: BalancingDeadline::active(), kind: BalancingKind::NoCurrent } }
        /// Creates a new `Balance` (in the active state, since that is what we start in). We will draw current.
        pub(super) fn new_drawcurrent() -> Self { Self { deadline: BalancingDeadline::active(), kind: BalancingKind::Current(BALANCING_ACTIVE_CURRENT) } }
        /// Transition to "cooldown" state.
        pub(super) fn to_cooldown(&mut self) { self.deadline = BalancingDeadline::cooldown() }
        /// Turns a `Balance` into a `NoCurrent` `Balance`. This will be a no-op if this `Balance` is already `NoCurrent`. This does not change anything else about the `Balance`.
        pub(super) fn to_nocurrent(&mut self) { self.kind = BalancingKind::NoCurrent }
        /// Turns a `Balance` into a `Current` `Balance`. This will be a no-op if this `Balance` is already `Current`. This does not change anything else about the `Balance`.
        pub(super) fn to_drawcurrent(&mut self) { self.kind = BalancingKind::Current(BALANCING_ACTIVE_CURRENT)}
        /// Returns how much current we should draw for this `Balance`. This will return `None` if this `Balance` is `NoCurrent`.
        pub(super) fn current_draw(&self) -> Option<Current> { 
            match self.kind {
                BalancingKind::Current(curr) => Some(curr),
                BalancingKind::NoCurrent => None,
            }
        }   
        /// Indicates if this `Balancing` has kind `Current` or not.
        pub(super) fn is_drawcurrent(&self) -> bool { matches!(self.kind, BalancingKind::Current(..)) }
        /// Indicates if this `Balancing` has kind `NoCurrent` or not.
        pub(super) fn is_nocurrent(&self) -> bool { matches!(self.kind, BalancingKind::NoCurrent) }
    }

    /// How long a `Settle` should last before expiring.
    const SETTLE_DURATION: Duration = Duration::from_secs(60);

    #[derive(Copy, Clone, PartialEq)]
    pub struct Settle {
        /// RUNTIME FIELD. When this `Settle` should expire.
        deadline: Deadline,

        /// RUNTIME FIELD. If this `Settle` was entered from `Balancing`.
        were_balancing: bool,
    }
    impl Settle {
        /// Creates a new `Settle`.
        fn new() -> Self { Self { deadline: Deadline::expire_in(SETTLE_DURATION), were_balancing: false } }

        /// Creates a new `Settle` with `were_balancing` = true.
        fn new_from_balancing() -> Self { Self { deadline: Deadline::expire_in(SETTLE_DURATION), were_balancing: true } }
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
        delta_ocv: Voltage,
        charge_restore_cache: ChargeRestoreCache,

    }
    impl Inputs {
        /// Based on the input values, checks if we have exceeded the fault conditions.
        fn exceeded_fault_voltages(&self) -> bool { (self.max_ocv >= MAX_CHARGE_VOLT_FLT) || (self.max_voltage >= MAX_CHARGE_VOLT_FLT) }
        /// If balancing hasn't already been started, this checks if we should start. 
        fn should_we_start_balancing(&self) -> bool { (self.delta_ocv > BALANCE_START_DELTA_V) && (PWM_DUTY_CYCLE_CONFIG.read().inner() != PwmDutyCycleConfig::Disabled) }
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
    pub const MAX_CHARGE_VOLT: Voltage = Voltage::from_volts(4.193);
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
                    if inputs.exceeded_fault_voltages()                                                                              { Self::Fault }
                    else if (inputs.max_voltage >= MAX_CHARGE_VOLT) && (h.current <= SHORT_CHARGE_CURRENT_STEP) && h.deadline.past() { Self::Settle(Settle::new()) }
                    else if inputs.should_we_start_balancing() && inputs.is_balancing_allowed()                                      { inputs.appropriate_balancing_phase() /* Self::BalanceAndChargeUp or Self::BalanceOnly */ }
                    else if (inputs.max_voltage >= MAX_CHARGE_VOLT) && h.deadline.past()                                             { h.reschedule_with_step_down(); Self::ShortChargeUp(h) }
                    else                                                                                                             { self }
                },

                Self::Balance(mut h) => {
                    if inputs.exceeded_fault_voltages()                                 { Self::Fault }
                    else if h.is_drawcurrent() && inputs.max_voltage >= MAX_CHARGE_VOLT { h.to_nocurrent(); Self::Balance(h) }
                    else if h.deadline.is_active_and(|d| { d.past() })        { h.to_cooldown(); Self::Balance(h) }
                    else if h.deadline.is_cooldown_and(|d| { d.past() })      { Self::Settle(Settle::new_from_balancing()) }
                    else                                                                { self }
                },

                Self::Settle(h) => {
                    let wants_balancing = if h.were_balancing { !inputs.should_we_stop_balancing() } else { inputs.should_we_start_balancing() };

                    if inputs.exceeded_fault_voltages() { Self::Fault }
                    else if h.deadline.past() {
                        if inputs.is_balancing_allowed() && wants_balancing { inputs.appropriate_balancing_phase_ocv() /* Self::BalanceAndChargeUp or Self::BalanceOnly */ }
                        else if inputs.max_ocv >= MAX_CHARGE_VOLT           { Self::Done }
                        else                                                { Self::from_charge_restore_cache(inputs.charge_restore_cache) /* Self::LongChargeUp or Self::ShortChargeUp */ }
                    }
                    else                                                    { self }
                },

                Self::Done => self,

                Self::Fault => self,
            }
        }   
    } 

    /// The charge machine's state.
    #[derive(Copy, Clone)]
    pub struct Charger {
        stage: ChargeStage,
        cache: ChargeRestoreCache,

        /// Current that we should be requesting from the charger right now. If this is `None`, we shouldn't be requesting any current from the charger.
        current: Option<Current>,
    }
    impl Default for Charger {
        fn default() -> Self {
            Self {
                stage: ChargeStage::LongChargeUp(LongChargeUp::new()),
                cache: ChargeRestoreCache::LongChargeUp,
                current: None,
            }
        }
    }

    impl Charger {
        pub const fn stage(&self) -> ChargeStage { self.stage }

        pub const fn current(&self) -> Option<Current> { self.current }

        /// Resets charging to the first charge stage.
        pub fn reset(&mut self) {
            *self = Charger::default()
        }

        /// Advances one cycle.
        pub fn tick(&mut self) {
            let Some((max_ocv, max_voltage, delta_ocv)) = analyzer::analyzer().try_partial_get(|h| (h.data.max_ocv.value(), h.data.max_voltage.value(), h.data.delta_ocv)) else { return; };
            
            self.stage = self.stage.tick(Inputs {
                max_ocv,
                max_voltage,
                delta_ocv,
                charge_restore_cache: self.cache,
            });

            self.cache.update_from_chargestage(self.stage);
            self.current = self.stage.current_we_should_request();
        }
    }
}
pub use state::*;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ChargerConnected {
    /// We have not received a ping from the charger in a bit, so we consider it disconnected.
    Disconnected,
    /// We have received a ping from the charger somewhat recently, so we consider it
    /// connected. The inner `Instant` indicates the time at which that ping was received.
    Connected(Instant),
}
impl ChargerConnected {
    /// Check if charger connected heartbeat has expired.
    fn tick(self) -> Self {
        /// If we don't recieve the charger message within this timeout we assume it is disconnected.
        const CHARGER_SILENCE_TIMEOUT: Duration = Duration::from_secs(5);

        match self {
            ChargerConnected::Disconnected => { self },
            ChargerConnected::Connected(last_ping) => {
                if Instant::now().saturating_duration_since(last_ping) >= CHARGER_SILENCE_TIMEOUT { ChargerConnected::Disconnected }
                else { self }
            },
        }
    }

    /// To be called when a charger message comes in.
    fn alert_ping() -> Self { ChargerConnected::Connected(Instant::now()) }

    const fn is_connected(&self) -> bool { matches!(*self, ChargerConnected::Connected(..)) }
}

use adbms6830b::chip::registers::pwm::types::PwmDutyCycleConfig;
use cangen::ShepherdBmsPwmPercent;

#[derive(Copy, Clone)]
pub struct RequestedPwm { inner: PwmDutyCycleConfig }
impl RequestedPwm {
    pub const fn default() -> Self { Self { inner: PwmDutyCycleConfig::Disabled } }
    pub const fn from_frame(frame: &ShepherdBmsPwmPercent) -> Self {
        let config = frame.into_bits();

        Self {
            inner: match config {
                1..=6 => PwmDutyCycleConfig::Pct06_6,
                7..=13 => PwmDutyCycleConfig::Pct13_2,
                14..=19 => PwmDutyCycleConfig::Pct19_8,
                20..=26 => PwmDutyCycleConfig::Pct26_4,
                27..=33 => PwmDutyCycleConfig::Pct33_0,
                34..=39 => PwmDutyCycleConfig::Pct39_6,
                40..=46 => PwmDutyCycleConfig::Pct46_2,
                47..=52 => PwmDutyCycleConfig::Pct52_8,
                53..=59 => PwmDutyCycleConfig::Pct59_4,
                60..=66 => PwmDutyCycleConfig::Pct66_0,
                67..=72 => PwmDutyCycleConfig::Pct72_6,
                73..=79 => PwmDutyCycleConfig::Pct79_2,
                80..=85 => PwmDutyCycleConfig::Pct85_8,
                86..=92 => PwmDutyCycleConfig::Pct92_4,
                93..=100 => PwmDutyCycleConfig::Pct100_0,
                _ => PwmDutyCycleConfig::Disabled,
            }
        }
    }
    pub const fn inner(&self) -> PwmDutyCycleConfig { self.inner }
}

static PWM_DUTY_CYCLE_CONFIG: SnapshotCell<RequestedPwm> = SnapshotCell::new(RequestedPwm::default());
static CHARGER_CONNECTED: SnapshotCell<ChargerConnected> = SnapshotCell::new(ChargerConnected::Disconnected);
static CHARGE_STATE: LazySnapshotCell<Charger> = LazySnapshotCell::new(Charger::default);

/// Gets current PWM duty cycle config.
pub fn get_pwm_duty_cycle() -> PwmDutyCycleConfig { PWM_DUTY_CYCLE_CONFIG.read().inner() }
/// To be called when we recieve a PWM config from Calypso.
pub fn update_pwm_duty_cycle(frame: &ShepherdBmsPwmPercent) { PWM_DUTY_CYCLE_CONFIG.set(RequestedPwm::from_frame(frame)) }
/// Indicates if the charger is connected or not.
pub fn is_charger_connected() -> bool { CHARGER_CONNECTED.read().is_connected() }
/// To be called when a charger message comes in.
pub fn alert_charger_received() { CHARGER_CONNECTED.set(ChargerConnected::alert_ping()); }
/// Resets the charge state. To be called by the state machine when charging is entered.
pub fn reset_charge_state() { CHARGE_STATE.partial_set(|h| h.reset() );}

/// Charger task.
#[embassy_executor::task]
pub async fn charger_task() -> ! {
    use embassy_time::Ticker;
    use crate::state_machine;
    use crate::segments::NUM_CELLS_TOTAL;
    use crate::can::types::BmsChargeMessageSend;
    use crate::can;
    use crate::faults;

    /// How often the charger task should run.
    const CHARGER_TASK_FREQUENCY: Duration = Duration::from_millis(20);
    /// How often we should send out the charger current request CAN message.
    const CHARGER_COMMAND_SEND_FREQUENCY: Duration = Duration::from_secs(1);

    let mut ticker = Ticker::every(CHARGER_TASK_FREQUENCY);
    let mut current_request_deadline = Deadline::expire_now();

    loop {
        /// Voltage to request from the charger.
        const VOLTAGE_TO_REQUEST: Voltage = state::MAX_CHARGE_VOLT.scale(NUM_CELLS_TOTAL as f32);

        CHARGER_CONNECTED.partial_set(|h| *h = h.tick() );

        // Only advance the charging stage statemachine if the overall BMS state is `Charging`.
        if state_machine::state().is_charging() && !faults::are_critical_faults_active() {
            let old_current = CHARGE_STATE.partial_read(|h| h.current());
            CHARGE_STATE.partial_set(|h| h.tick() );
            let new_current = CHARGE_STATE.partial_read(|h| h.current());

            if (old_current != new_current) || (current_request_deadline.past()) {
                let message = match new_current {
                    Some(c) => {
                        BmsChargeMessageSend {
                            charge_volts: VOLTAGE_TO_REQUEST.as_volts(),
                            charge_current: c.as_amps(),
                            enable_charging: 1,
                        }.as_frame()
                    },
                    None => {
                        BmsChargeMessageSend {
                            charge_volts: VOLTAGE_TO_REQUEST.as_volts(),
                            charge_current: 0_f32,
                            enable_charging: 0,
                        }.as_frame()
                    },
                };
                can::send(message).await;

                current_request_deadline = Deadline::expire_in(CHARGER_COMMAND_SEND_FREQUENCY);
            }

            // u_TODO maybe send motor controller zero commands every charger tick (look at C code)
        }

        ticker.next().await;
    }
}