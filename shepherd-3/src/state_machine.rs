mod sm {
    use strum::FromRepr;
    use embassy_time::Instant;

    use crate::units::Voltage;
    use super::api;
    use super::charging::Charger;

    pub struct StateMachine {
        current: BmsState,
    }

    impl StateMachine {
        /// Starts the machine in `initial`. Does **not** run `initial`'s entry action.
        pub const fn new(initial: BmsState) -> Self {
            Self { current: initial }
        }

        /// Runs the initial state's entry action and publishes it.
        pub fn start(&mut self, inputs: &Inputs, ctx: &mut Ctx) {
            api::publish_state(self.current);
            self.current.on_enter(inputs, ctx);
        }

        /// Runs this tick's handler, and transitions if it asked for one.
        pub fn tick(&mut self, inputs: &Inputs, ctx: &mut Ctx) {
            let next = self.current.on_tick(inputs, ctx);
            self.transition_to(next, inputs, ctx);
        }

        /// The only path that changes `self.current`, whether the request came from a handler's
        /// return value or from the task forcing one.
        pub fn transition_to(&mut self, next: BmsState, inputs: &Inputs, ctx: &mut Ctx) {
            if next == self.current {
                return;
            }

            if !self.current.transition_allowed(next) {
                defmt::error!("State machine: illegal transition {} -> {}, refused.", self.current, next);
                return;
            }

            defmt::info!("State machine: {} -> {}", self.current, next);

            self.current = next;
            api::publish_state(next);
            next.on_enter(inputs, ctx);
        }
    }

    /// The operating state of the pack.
    #[derive(Copy, Clone, PartialEq, Eq, FromRepr, defmt::Format)]
    #[repr(u8)]
    pub enum BmsState {
        /// Bringing the pack up. Startup has not finished.
        Boot = 0,
        /// Normal operation.
        Ready,
        /// The charger is connected and charging is being managed.
        Charging,
        /// A critical fault is active. The shutdown circuit is asserted.
        Faulted,
    }

    impl BmsState {
        /// Whether moving from `self` to `next` is legal.
        #[rustfmt::skip]
        pub const fn transition_allowed(self, next: Self) -> bool {
            use BmsState::{Boot, Charging, Faulted, Ready};

            // Exhaust all states to make sure we have all valid transitions listed
            match self {
                Boot     => matches!(next, Ready | Charging | Faulted),
                Ready    => matches!(next, Charging | Faulted),
                Charging => matches!(next, Ready | Faulted),
                Faulted  => matches!(next, Boot),
            }
        }
    }

    /// What the state machine reads, snapshotted once per tick by value.
    pub struct Inputs {
        pub data_ready: bool,
        pub now: Instant,
        pub critical_fault_active: bool,
        pub charger_connected: bool,
        pub max_cell_voltage: Voltage,
        pub max_ocv: Voltage,
    }

    /// What the state machine owns and mutates.
    #[derive(Default)]
    pub struct Ctx {
        /// The charge algorithm.
        pub charger: Charger,
        /// Limits the charge frame to 1 Hz. `None` means not sent yet
        pub charger_message_deadline: Option<Instant>,
    }

    impl BmsState {
        pub fn on_enter(self, inputs: &Inputs, ctx: &mut Ctx) {
            match self {
                BmsState::Boot => handlers::enter_boot(inputs, ctx),
                BmsState::Ready => handlers::enter_ready(inputs, ctx),
                BmsState::Charging => handlers::enter_charging(inputs, ctx),
                BmsState::Faulted => handlers::enter_faulted(inputs, ctx),
            }
        }

        pub fn on_tick(self, inputs: &Inputs, ctx: &mut Ctx) -> Self {
            match self {
                BmsState::Boot => handlers::tick_boot(inputs, ctx),
                BmsState::Ready => handlers::tick_ready(inputs, ctx),
                BmsState::Charging => handlers::tick_charging(inputs, ctx),
                BmsState::Faulted => handlers::tick_faulted(inputs, ctx),
            }
        }
    }

    // u_TODO: Complete all of these (actually fully port from C code)
    /// Per-state entry actions and tick handlers.
    mod handlers {
        use embassy_time::Duration;

        use super::*;
        use crate::state_machine::api;
        use crate::state_machine::charging::{CHARGE_TARGET_VOLTS, CHARGING_CURRENT, CONTROL_CHARGE, CONTROL_STOP};

        /// How often the charge frame goes out while charging.
        const CHARGE_FRAME_PERIOD: Duration = Duration::from_secs(1);

        /// Drives the BMS side of the shutdown circuit
        fn set_fault(_faulted: bool) {}

        /// Queues a `BMS Charge Message Send` frame.
        fn send_charge_frame(charge_volts: f32, charge_current: f32, enable_charging: u8) {
            let frame = crate::can::types::BmsChargeMessageSend { charge_volts, charge_current, enable_charging }.as_frame();

            if crate::can::try_send(frame).is_err() {
                defmt::warn!("Charge frame dropped: the outgoing CAN channel was full.");
            }
        }

        pub fn enter_boot(_inputs: &Inputs, ctx: &mut Ctx) {
            api::set_charger_connected(false);
            ctx.charger_message_deadline = None;
        }

        pub fn enter_ready(_inputs: &Inputs, _ctx: &mut Ctx) {
            set_fault(false);
        }

        pub fn enter_charging(inputs: &Inputs, ctx: &mut Ctx) {
            ctx.charger.restart(inputs.now);
        }

        pub fn enter_faulted(_inputs: &Inputs, _ctx: &mut Ctx) {
            set_fault(true);
            send_charge_frame(0.0, 0.0, CONTROL_STOP);
        }

        pub fn tick_boot(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            if inputs.data_ready { BmsState::Ready } else { BmsState::Boot }
        }

        pub fn tick_ready(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            if inputs.charger_connected { BmsState::Charging } else { BmsState::Ready }
        }

        pub fn tick_charging(inputs: &Inputs, ctx: &mut Ctx) -> BmsState {
            let now = inputs.now;

            ctx.charger.tick(inputs.max_cell_voltage, inputs.max_ocv, now);

            if ctx.charger.stage().charging_allowed() {
                if ctx.charger_message_deadline.is_none_or(|deadline| now >= deadline) {
                    send_charge_frame(CHARGE_TARGET_VOLTS, CHARGING_CURRENT, CONTROL_CHARGE);
                    ctx.charger_message_deadline = Some(now + CHARGE_FRAME_PERIOD);
                }
            } else {
                send_charge_frame(0.0, 0.0, CONTROL_STOP);
            }

            BmsState::Charging
        }

        pub fn tick_faulted(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            set_fault(true);

            if inputs.critical_fault_active { BmsState::Faulted } else { BmsState::Boot }
        }
    }
}
pub use sm::BmsState;

/// The charge algorithm that runs while the pack is in `Charging`.
mod charging {
    use embassy_time::{Duration, Instant};

    use crate::segments::{ADBMS6830B_NUM_CELLS_PER_CHIP, ADBMS6830B_NUM_CHIPS};
    use crate::units::{Voltage, volt};

    /// Cell voltage that ends a charge phase.
    const MAX_CHARGE_VOLT: f32 = 4.19;
    /// Loaded cell voltage that aborts charging outright.
    const MAX_CHARGE_VOLT_FLT: f32 = 4.25;

    const LONG_CHARGE: Duration = Duration::from_secs(15 * 60);
    const SHORT_CHARGE: Duration = Duration::from_secs(20);
    const SETTLE: Duration = Duration::from_secs(60);

    pub const CHARGING_CURRENT: f32 = 5.0;
    pub const CONTROL_CHARGE: u8 = 0x00;
    pub const CONTROL_STOP: u8 = 0xFF;

    pub const CHARGE_TARGET_VOLTS: f32 = MAX_CHARGE_VOLT * (ADBMS6830B_NUM_CHIPS * ADBMS6830B_NUM_CELLS_PER_CHIP) as f32;

    #[derive(Copy, Clone, PartialEq, Eq, Default, defmt::Format)]
    pub enum ChargeStage {
        /// Phase 1: bulk charge, up to 15 minutes at a time.
        #[default]
        LongChargeUp,
        /// Phase 1 rest, so the cells can settle before their OCV is believed.
        LongSettle,
        /// Phase 2: short top-up bursts.
        ShortChargeUp,
        /// Phase 2 rest.
        ShortSettle,
        /// Settled OCV reached the target. Terminal.
        Done,
        /// Charging was abandoned. Terminal.
        Fault,
    }

    impl ChargeStage {
        /// Whether current should be requested from the charger in this stage.
        pub const fn charging_allowed(self) -> bool {
            matches!(self, Self::LongChargeUp | Self::ShortChargeUp)
        }

        /// How long the stage runs before it times out, or `None` if it has no timer.
        const fn duration(self) -> Option<Duration> {
            match self {
                Self::LongChargeUp => Some(LONG_CHARGE),
                Self::ShortChargeUp => Some(SHORT_CHARGE),
                Self::LongSettle | Self::ShortSettle => Some(SETTLE),
                Self::Done | Self::Fault => None,
            }
        }

        /// The whole stage graph
        const fn next(self, loaded_full: bool, settled_full: bool, expired: bool) -> Self {
            match self {
                Self::LongChargeUp if loaded_full => Self::ShortSettle,
                Self::LongChargeUp if expired => Self::LongSettle,
                Self::LongSettle if expired => {
                    if settled_full {
                        Self::Done
                    } else {
                        Self::LongChargeUp
                    }
                },
                Self::ShortChargeUp if loaded_full || expired => Self::ShortSettle,
                Self::ShortSettle if expired => {
                    if settled_full {
                        Self::Done
                    } else {
                        Self::ShortChargeUp
                    }
                },
                other => other,
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

        /// Starts the algorithm over. Called from `enter_charging`, so re-entering `Charging`
        /// always begins a fresh bulk phase rather than resuming a stale one.
        pub fn restart(&mut self, now: Instant) {
            self.stage = ChargeStage::default();
            self.arm(now);
            defmt::info!("Charge stage: starting at {}", self.stage);
        }

        /// Advances one cycle.
        pub fn tick(&mut self, max_cell_voltage: Voltage, max_ocv: Voltage, now: Instant) {
            if max_cell_voltage.get::<volt>() >= MAX_CHARGE_VOLT_FLT || max_ocv.get::<volt>() >= MAX_CHARGE_VOLT_FLT {
                self.set_stage(ChargeStage::Fault, now);
                return;
            }

            let expired = self.deadline.is_some_and(|deadline| now >= deadline);

            let next = self.stage.next(max_cell_voltage.get::<volt>() >= MAX_CHARGE_VOLT, max_ocv.get::<volt>() >= MAX_CHARGE_VOLT, expired);

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
}

// u_TODO: complete all of these too
/// Public api for state machine and also internal api
mod api {
    use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
    use super::sm::BmsState;

    static STATE: AtomicU8 = AtomicU8::new(BmsState::Boot as u8);

    const TRANSITION_QUEUE_DEPTH: usize = 4;
    static TRANSITION_REQUESTS: Channel<ThreadModeRawMutex, BmsState, TRANSITION_QUEUE_DEPTH> = Channel::new();

    // Called by other tasks so they are exported assuch

    /// The current BMS state.
    pub fn bms_state() -> BmsState {
        BmsState::from_repr(STATE.load(Ordering::Relaxed)).unwrap_or(BmsState::Faulted)
    }

    /// How many transition requests are queued but not yet applied (only really useful for diagnostics).
    pub fn pending_transition_requests() -> usize {
        TRANSITION_REQUESTS.len()
    }

    /// Whether a charger box has announced itself.
    static CHARGER_CONNECTED: AtomicBool = AtomicBool::new(false);

    /// Reports that a charger box frame arrived.
    pub fn charger_message_received() {
        set_charger_connected(true);
        request_transition(BmsState::Charging);
    }

    /// Asks the state machine to move to `next`. Callable from any task.
    pub fn request_transition(next: BmsState) {
        if TRANSITION_REQUESTS.try_send(next).is_err() {
            defmt::warn!("State machine: transition request to {} dropped, queue full.", next);
        }
    }

    // The state machine reaching its own storage. `pub(super)` rather than `pub` on purpose

    /// Publishes `state` for [`bms_state`]. Called only by the driver on an accepted transition.
    pub(super) fn publish_state(state: BmsState) {
        STATE.store(state as u8, Ordering::Relaxed);
    }

    /// Takes one queued request, if there is one. Called only by the task loop.
    pub(super) fn take_transition_request() -> Option<BmsState> {
        TRANSITION_REQUESTS.try_receive().ok()
    }

    pub(super) fn charger_connected() -> bool {
        CHARGER_CONNECTED.load(Ordering::Relaxed)
    }

    pub(super) fn set_charger_connected(connected: bool) {
        CHARGER_CONNECTED.store(connected, Ordering::Relaxed);
    }
}
pub use api::*;

/// The task that runs the state machine.
mod task {
    use embassy_time::{Duration, Instant, Ticker};
    use crate::units::{Voltage, volt};
    use super::*;
    use super::sm::*;

    /// Reads every published value the state machine depends on, once and saves it to an `Inputs`.
    fn snapshot() -> Inputs {
        Inputs {
            // u_TODO: true once every producer below has published at least once
            data_ready: false,
            now: Instant::now(),

            // u_TODO: crate::faults::are_critical_faults_active(), once faults.rs merges.
            critical_fault_active: false,

            charger_connected: api::charger_connected(),

            // u_TODO: read from somewhere
            max_cell_voltage: Voltage::new::<volt>(0.0),
            max_ocv: Voltage::new::<volt>(0.0),
        }
    }

    fn run_cycle(sm: &mut StateMachine, ctx: &mut Ctx, inputs: Inputs) {
        let requested = api::take_transition_request();
        let target = if inputs.critical_fault_active { Some(BmsState::Faulted) } else { requested };

        if let Some(target) = target {
            sm.transition_to(target, &inputs, ctx);
        }
        sm.tick(&inputs, ctx);
    }

    /// Main state machine task.
    #[embassy_executor::task]
    pub async fn state_machine_task() -> ! {
        const TICK_PERIOD_MS: u64 = 20;

        let mut ticker = Ticker::every(Duration::from_millis(TICK_PERIOD_MS));
        let mut sm = StateMachine::new(BmsState::Boot);

        let mut ctx = Ctx::default();
        sm.start(&snapshot(), &mut ctx);

        loop {
            // Cycle the statemachine
            let inputs = snapshot();
            run_cycle(&mut sm, &mut ctx, inputs);

            ticker.next().await;
        }
    }
}
pub use task::state_machine_task;
