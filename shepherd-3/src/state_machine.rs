use core::sync::atomic::AtomicU8;

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};

static STATE: AtomicU8 = AtomicU8::new(state::BmsState::Boot as u8);

const TRANSITION_QUEUE_DEPTH: usize = 4;
static TRANSITION_REQUESTS: Channel<ThreadModeRawMutex, state::BmsState, TRANSITION_QUEUE_DEPTH> = Channel::new();

/// Generic state machine driver.
pub mod fsm {
    pub trait State: Copy + PartialEq + defmt::Format {
        /// What the machine owns and mutates
        type Ctx;
        /// What the machine reads from to handle its own state. Built for itself each tick (no holding of data)
        type Inputs;

        /// Whether moving from `self` to `next` is legal
        fn can_transition_to(self, next: Self) -> bool;

        /// Runs once, on entry, after the transition has been accepted.
        fn on_enter(self, ctx: &mut Self::Ctx);

        /// Runs every tick. Returns the next state, which may be `self`.
        fn on_tick(self, inputs: &Self::Inputs, ctx: &mut Self::Ctx) -> Self;

        /// Publish the current state to cache here.
        fn publish(self) {}
    }

    /// Drives a [`State`], guaranteeing entry actions run exactly once per accepted transition.
    pub struct Fsm<S: State> {
        current: S,
    }

    impl<S: State> Fsm<S> {
        /// Starts the machine in `initial`. Does **not** run `initial`'s entry action.
        pub fn new(initial: S) -> Self {
            Self { current: initial }
        }

        /// The current state.
        pub const fn state(&self) -> S {
            self.current
        }

        /// Runs this tick's handler, and transitions if it asked for one.
        pub fn tick(&mut self, inputs: &S::Inputs, ctx: &mut S::Ctx) {
            let next = self.current.on_tick(inputs, ctx);
            self.transition(next, ctx);
        }

        /// Forces a transition from outside the handler.
        pub fn transition_to(&mut self, next: S, ctx: &mut S::Ctx) {
            self.transition(next, ctx);
        }

        /// The only path that changes `current`.
        fn transition(&mut self, next: S, ctx: &mut S::Ctx) {
            if next == self.current {
                return;
            }

            if !self.current.can_transition_to(next) {
                defmt::error!("State machine: illegal transition {} -> {}, refused.", self.current, next);
                return;
            }

            defmt::info!("State machine: {} -> {}", self.current, next);

            self.current = next;
            next.publish();
            next.on_enter(ctx);
        }
    }
}
pub use fsm::*;

/// `BmsState`, its transition graph and handlers, and the data they operate on.
pub mod state {
    use core::sync::atomic::Ordering;
    use strum::FromRepr;
    use super::fsm::State;

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

            matches!(
                (self, next),
                (Boot,     Ready)    | (Boot,     Faulted) |
                (Ready,    Charging) | (Ready,    Faulted) |
                (Charging, Ready)    | (Charging, Faulted) |
                (Faulted,  Boot)
            )
        }
    }

    /// What the state machine reads, snapshotted once per tick by value.
    #[derive(Default)]
    pub struct Inputs {
        pub critical_fault_active: bool,
        // u_TODO: add one field per producer as it lands.
        // voltages:        PackVoltageStats,
        // sanitized_temps: SanitizedTemps,
        // ocv:             OcvStats,
        // limits:          CurrentLimits,
        // shutdown:        ShutdownState,
    }

    /// What the state machine owns and mutates.
    #[derive(Default)]
    pub struct Ctx {
        /// Whether the charger has announced itself.
        pub charger_connected: bool,
    }

    impl State for BmsState {
        type Ctx = Ctx;
        type Inputs = Inputs;

        fn can_transition_to(self, next: Self) -> bool {
            self.transition_allowed(next)
        }

        fn publish(self) {
            super::STATE.store(self as u8, Ordering::Relaxed);
        }

        fn on_enter(self, ctx: &mut Ctx) {
            match self {
                BmsState::Boot => handlers::init_boot(ctx),
                BmsState::Ready => handlers::init_ready(ctx),
                BmsState::Charging => handlers::init_charging(ctx),
                BmsState::Faulted => handlers::init_faulted(ctx),
            }
        }

        fn on_tick(self, inputs: &Inputs, ctx: &mut Ctx) -> Self {
            match self {
                BmsState::Boot => handlers::handle_boot(inputs, ctx),
                BmsState::Ready => handlers::handle_ready(inputs, ctx),
                BmsState::Charging => handlers::handle_charging(inputs, ctx),
                BmsState::Faulted => handlers::handle_faulted(inputs, ctx),
            }
        }
    }

    /// Per-state entry actions and tick handlers.
    mod handlers {
        use super::{BmsState, Ctx, Inputs};

        // u_TODO: Complete all of these
        pub fn init_boot(_ctx: &mut Ctx) {}

        pub fn init_ready(_ctx: &mut Ctx) {}

        pub fn init_charging(_ctx: &mut Ctx) {}

        pub fn init_faulted(_ctx: &mut Ctx) {}

        pub fn handle_boot(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Boot
        }

        pub fn handle_ready(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Ready
        }

        pub fn handle_charging(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Charging
        }

        pub fn handle_faulted(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            if inputs.critical_fault_active { BmsState::Faulted } else { BmsState::Boot }
        }
    }
}
pub use state::{BmsState, Ctx, Inputs};

pub mod api {
    use core::sync::atomic::Ordering;
    use super::state::BmsState;

    /// The current BMS state.
    pub fn bms_state() -> BmsState {
        BmsState::from_repr(super::STATE.load(Ordering::Relaxed)).unwrap_or(BmsState::Faulted)
    }

    /// How many transition requests are queued but not yet applied (only really useful for diagnostics).
    pub fn pending_transition_requests() -> usize {
        super::TRANSITION_REQUESTS.len()
    }

    /// Asks the state machine to move to `next`. Callable from any task.
    pub fn request_transition(next: BmsState) {
        if super::TRANSITION_REQUESTS.try_send(next).is_err() {
            defmt::warn!("State machine: transition request to {} dropped, queue full.", next);
        }
    }
}
pub use api::*;

/// The task that runs the state machine.
pub mod task {
    use embassy_time::{Duration, Ticker};
    use super::fsm::Fsm;
    use super::state::{BmsState, Ctx, Inputs};

    /// Reads every published value the state machine depends on.
    fn snapshot() -> Inputs {
        Inputs {
            // u_TODO: critical_fault_active: crate::faults::are_critical_faults_active(),
            // u_TODO: one line per producer as it lands
            ..Default::default()
        }
    }

    /// Main state machine task.
    #[embassy_executor::task]
    pub async fn state_machine_task() -> ! {
        const TICK_PERIOD_MS: u64 = 20;

        let mut ticker = Ticker::every(Duration::from_millis(TICK_PERIOD_MS));
        let mut fsm = Fsm::new(BmsState::Boot);
        let mut ctx = Ctx::default();

        loop {
            let inputs = snapshot();

            if inputs.critical_fault_active {
                fsm.transition_to(BmsState::Faulted, &mut ctx);
            }

            while let Ok(next) = super::TRANSITION_REQUESTS.try_receive() {
                fsm.transition_to(next, &mut ctx);
            }

            fsm.tick(&inputs, &mut ctx);

            ticker.next().await;
        }
    }
}
pub use task::state_machine_task;
