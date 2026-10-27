mod sm {
    use strum::FromRepr;
    use embassy_time::Instant;

    use crate::units::Voltage;
    use super::api;
    use super::charging::Charger;

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

        /// Whether the state is `BmsState::Charging` or not.
        pub const fn is_charging(self) -> bool {
            matches!(self, BmsState::Charging)
        }
    }

    /// What the state machine reads, snapshotted once per tick by value.
    pub struct Inputs {
        pub now: Instant,
        pub critical_fault_active: bool,
        pub charger_connected: bool,
        pub max_cell_voltage: Voltage,
        pub max_ocv: Voltage,
    }

    /// The state a handler is allowed to mutate.
    #[derive(Default)]
    pub struct Ctx {
        /// The charge algorithm.
        pub charger: Charger,
    }

    impl BmsState {
        /// Dispatch to each states enter handlers
        pub fn on_init(&self, inputs: &Inputs, ctx: &mut Ctx) {
            match self {
                Self::Boot => handlers::init_boot(inputs, ctx),
                Self::Ready => handlers::init_ready(inputs, ctx),
                Self::Charging => handlers::init_charging(inputs, ctx),
                Self::Faulted => handlers::init_faulted(inputs, ctx),
            }
        }

        /// Dispatch to each states tick handlers
        pub fn on_tick(&self, inputs: &Inputs, ctx: &mut Ctx) -> Self {
            match self {
                Self::Boot => handlers::tick_boot(inputs, ctx),
                Self::Ready => handlers::tick_ready(inputs, ctx),
                Self::Charging => handlers::tick_charging(inputs, ctx),
                Self::Faulted => handlers::tick_faulted(inputs, ctx),
            }
        }
    }

    /// What a transition request did. Returned by every path that can move the machine.
    #[must_use]
    #[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
    pub enum Transition {
        NoChange,
        Changed { from: BmsState, to: BmsState },
        Refused { from: BmsState, to: BmsState },
    }

    pub struct BmsStateMachine {
        ctx: Ctx,
        current: BmsState,
    }

    impl BmsStateMachine {
        /// Starts the machine in `initial`. Does **not** run `initial`'s entry action.
        pub fn new(initial: BmsState) -> Self {
            Self { ctx: Ctx::default(), current: initial }
        }

        /// Store the state and run the `on_init` fn for the initial state
        pub fn start(&mut self, inputs: Inputs) {
            api::store_state(self.current);
            self.current.on_init(&inputs, &mut self.ctx);
        }

        /// One cycle: exactly one tick handler, then at most one entry action.
        pub fn tick(&mut self, inputs: Inputs) -> Transition {
            // The current state runs and proposes what it wants to be next.
            let next = self.current.on_tick(&inputs, &mut self.ctx);

            // A critical fault outranks whatever the handler wanted.
            let next = if inputs.critical_fault_active { BmsState::Faulted } else { next };

            self.transition_to(next, &inputs)
        }

        /// The only path that changes `self.current`.
        fn transition_to(&mut self, next: BmsState, inputs: &Inputs) -> Transition {
            let prev = self.current;

            if next == prev {
                return Transition::NoChange;
            }

            if !prev.transition_allowed(next) {
                return Transition::Refused { from: prev, to: next };
            }

            self.current = next;
            api::store_state(next);
            next.on_init(inputs, &mut self.ctx);

            Transition::Changed { from: prev, to: next }
        }
    }

    // u_TODO: Complete all of these (actually fully port from C code)
    /// Per-state entry actions and tick handlers
    mod handlers {
        use super::*;
        use crate::state_machine::api;

        /// Helper to drive the BMS side of the shutdown circuit
        fn set_fault(_faulted: bool) {}

        // Boot

        pub fn init_boot(_inputs: &Inputs, _ctx: &mut Ctx) {
            api::clear_charger_connected();
        }

        pub fn tick_boot(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Ready
        }

        // Ready

        pub fn init_ready(_inputs: &Inputs, _ctx: &mut Ctx) {
            set_fault(false);
        }

        pub fn tick_ready(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            if inputs.charger_connected { BmsState::Charging } else { BmsState::Ready }
        }

        // Charging

        pub fn init_charging(inputs: &Inputs, ctx: &mut Ctx) {
            ctx.charger.restart(inputs.now);
        }

        pub fn tick_charging(inputs: &Inputs, ctx: &mut Ctx) -> BmsState {
            ctx.charger.tick(inputs.max_cell_voltage, inputs.max_ocv, inputs.now);

            BmsState::Charging
        }

        // Faulted

        pub fn init_faulted(_inputs: &Inputs, _ctx: &mut Ctx) {
            set_fault(true);
        }

        pub fn tick_faulted(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            set_fault(true);

            if inputs.critical_fault_active { BmsState::Faulted } else { BmsState::Boot }
        }
    }
}
pub use sm::{BmsState, Transition};

/// The charge algorithm that runs while the pack is in `Charging`.
mod charging {
    use embassy_time::{Duration, Instant};

    use crate::units::{Voltage, volt};

    /// Cell voltage that ends a charge phase.
    const MAX_CHARGE_VOLT: f32 = 4.19;
    /// Loaded cell voltage that aborts charging outright.
    const MAX_CHARGE_VOLT_FLT: f32 = 4.25;

    const LONG_CHARGE: Duration = Duration::from_secs(15 * 60);
    const SHORT_CHARGE: Duration = Duration::from_secs(20);
    const SETTLE: Duration = Duration::from_secs(60);

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

        /// Starts the algorithm over. Called from `init_charging`, so re-entering `Charging`
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
///
/// Other tasks report conditions, but never force the machine to transition,
/// instead the machine will request and handle transitions.
mod api {
    use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
    use super::sm::BmsState;

    static STATE: AtomicU8 = AtomicU8::new(BmsState::Boot as u8);

    // Called by other tasks, so they are exported as such.

    /// The current BMS state.
    pub fn bms_state() -> BmsState {
        BmsState::from_repr(STATE.load(Ordering::Relaxed)).unwrap_or(BmsState::Faulted)
    }

    /// Latched by the first charger frame, cleared only by `init_boot`.
    static CHARGER_CONNECTED: AtomicBool = AtomicBool::new(false);

    /// Reports that a charger box frame arrived. Called from CAN Task when charger message is recieved.
    pub fn charger_frame_received() {
        CHARGER_CONNECTED.store(true, Ordering::Relaxed);
    }

    // Helpers for the state machine to reach its own storage.

    /// Publishes `state` for [`bms_state`]. Called only on an accepted transition.
    pub(super) fn store_state(state: BmsState) {
        STATE.store(state as u8, Ordering::Relaxed);
    }

    /// Whether a charger has announced itself since the last pass through `Boot`.
    pub fn charger_connected() -> bool {
        CHARGER_CONNECTED.load(Ordering::Relaxed)
    }

    /// Drops the charger latch. Called only by `init_boot`.
    pub(super) fn clear_charger_connected() {
        CHARGER_CONNECTED.store(false, Ordering::Relaxed);
    }
}
pub use api::*;

/// The task that runs the state machine.
mod task {
    use embassy_time::{Duration, Instant, Ticker};
    use crate::units::{Voltage, volt};
    use super::*;
    use super::sm::*;

    /// Reads every value the state machine depends on, once, into an [`Inputs`].
    fn snapshot() -> Inputs {
        Inputs {
            now: Instant::now(),
            critical_fault_active: crate::faults::are_critical_faults_active(),
            charger_connected: api::charger_connected(),
            // u_TODO: read from somewhere
            max_cell_voltage: Voltage::new::<volt>(0.0),
            max_ocv: Voltage::new::<volt>(0.0),
        }
    }

    /// Main state machine task.
    #[embassy_executor::task]
    pub async fn state_machine_task() -> ! {
        const TICK_PERIOD_MS: u64 = 20;

        let mut ticker = Ticker::every(Duration::from_millis(TICK_PERIOD_MS));
        let mut sm = BmsStateMachine::new(BmsState::Boot);

        sm.start(snapshot());
        loop {
            match sm.tick(snapshot()) {
                Transition::NoChange => {},
                Transition::Changed { from, to } => defmt::info!("State machine: {} -> {}", from, to),
                Transition::Refused { from, to } => defmt::warn!("State machine: illegal transition refused {} -> {}!", from, to),
            }

            ticker.next().await;
        }
    }
}
pub use task::state_machine_task;
