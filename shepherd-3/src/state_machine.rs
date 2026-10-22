mod sm {
    use strum::FromRepr;

    use super::api;

    /// The operating state of the pack.
    #[derive(Copy, Clone, PartialEq, Eq, FromRepr, defmt::Format)]
    #[repr(u8)]
    pub enum BmsState {
        /// Bringing the pack up. Startup has not finished.
        Boot = 0,
        /// Normal operation.
        Ready,
        /// The charger is connected.
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
        pub critical_fault_active: bool,
    }

    /// The state a handler is allowed to mutate. Empty until a handler needs to carry something.
    #[derive(Default)]
    pub struct Ctx {}

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

    // u_TODO: Complete all of these
    /// Per-state entry actions and tick handlers
    mod handlers {
        use super::*;

        // Boot

        pub fn init_boot(_inputs: &Inputs, _ctx: &mut Ctx) {}

        pub fn tick_boot(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Ready
        }

        // Ready

        pub fn init_ready(_inputs: &Inputs, _ctx: &mut Ctx) {}

        pub fn tick_ready(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Ready
        }

        // Charging

        pub fn init_charging(_inputs: &Inputs, _ctx: &mut Ctx) {}

        pub fn tick_charging(_inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            BmsState::Charging
        }

        // Faulted

        pub fn init_faulted(_inputs: &Inputs, _ctx: &mut Ctx) {}

        pub fn tick_faulted(inputs: &Inputs, _ctx: &mut Ctx) -> BmsState {
            if inputs.critical_fault_active { BmsState::Faulted } else { BmsState::Boot }
        }
    }
}
pub use sm::{BmsState, Transition};

// u_TODO: complete all of these too
/// Public api for state machine and also internal api
///
/// Other tasks report conditions, but never force the machine to transition,
/// instead the machine will request and handle transitions.
mod api {
    use core::sync::atomic::{AtomicU8, Ordering};
    use super::sm::BmsState;

    static STATE: AtomicU8 = AtomicU8::new(BmsState::Boot as u8);

    // Called by other tasks, so they are exported as such.

    /// The current BMS state.
    pub fn bms_state() -> BmsState {
        BmsState::from_repr(STATE.load(Ordering::Relaxed)).unwrap_or(BmsState::Faulted)
    }

    // Helpers for the state machine to reach its own storage.

    /// Publishes `state` for [`bms_state`]. Called only on an accepted transition.
    pub(super) fn store_state(state: BmsState) {
        STATE.store(state as u8, Ordering::Relaxed);
    }
}
pub use api::*;

/// The task that runs the state machine.
mod task {
    use embassy_time::{Duration, Ticker};
    use super::sm::*;

    /// Reads every value the state machine depends on, once, into an [`Inputs`].
    fn snapshot() -> Inputs {
        Inputs { critical_fault_active: crate::faults::are_critical_faults_active() }
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
