use strum::FromRepr;
use crate::{charging, faults, faults::{FaultPin}};
use core::cell::Cell;
use embassy_sync::blocking_mutex::ThreadModeMutex;

/// The operating state of the pack.
#[derive(Copy, Clone, PartialEq, Eq, FromRepr, defmt::Format)]
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

/// INTERNAL API FOR STATE ADVANCEMENT.
impl BmsState {
    /// Updates the state. We either stay in the current state, or transition to another state.
    #[rustfmt::skip]
    fn evaluate(self) -> Self {

        match self {
            BmsState::Boot => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else if charging::is_charger_connected() { BmsState::Charging } 
                else                                     { BmsState::Ready }
            },

            BmsState::Ready => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else if charging::is_charger_connected() { BmsState::Charging } 
                else                                     { BmsState::Ready }
            },

            BmsState::Charging => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else if charging::is_charger_connected() { BmsState::Charging } 
                else                                     { BmsState::Charging } // we shouldn't go back to READY after in charging even if the charger isn't connected
            },

            BmsState::Faulted => {
                if faults::are_critical_faults_active()  { BmsState::Faulted } 
                else                                     { BmsState::Boot } 
            },
        }
    }

    /// Routines to be called every tick we are in a state.
    /// 
    /// In some cases, this can be called twice in a single tick. This is because there are technically two
    /// states for each tick (the state before we call `evaluate()`, and the state after we call `evaluate()`). Most of the time,
    /// the state before and after will be the same. But if there is a state transition, the state before and after will be different.
    /// In that case, `on_tick()` will be called for the "before" state, and then once `evaluate()` runs, it will be called for the
    /// "after" state.
    fn on_tick(self) {
        match self {
            BmsState::Boot => {
                
            },

            BmsState::Ready => {
                
            },

            BmsState::Charging => {
                
            },

            BmsState::Faulted => {
                
            },
        }
    }

    /// Routines to execute once whenever a state is entered.
    fn on_enter(self) {
        match self {
            BmsState::Boot => {
                
            },

            BmsState::Ready => {
                
            },

            BmsState::Charging => {
                charging::reset_charge_state();
            },

            BmsState::Faulted => {
                faults::set_fault(FaultPin::Active);
            },
        }
    }

    /// Routines to execute once whenever a state is exited.
    fn on_exit(self) {
        match self {
            BmsState::Boot => {
                
            },

            BmsState::Ready => {
                
            },

            BmsState::Charging => {
                
            },

            BmsState::Faulted => {
                faults::set_fault(FaultPin::Inactive);
            },
        }
    }
}

/// PUBLIC API.
impl BmsState {
    /// Indicates if the current state is `BmsState::Charging` or not.
    pub const fn is_charging(&self) -> bool { matches!(*self, BmsState::Charging) }
}

struct State { state: ThreadModeMutex<Cell<BmsState>> }
impl State {
    const fn new() -> Self { Self { state: ThreadModeMutex::new(Cell::new(BmsState::Boot)) } }
    fn get(&self) -> BmsState { self.state.lock(|i| i.get()) }
    fn tick(&self) {
        self.state.lock(|i| {
            let old = i.get();
            old.on_tick();
            i.update(|state| state.evaluate());
            let new = i.get();
            if old != new {
                old.on_exit();
                new.on_enter();
                new.on_tick();
            }
        });
    }
}

static STATE: State = State::new();

/// Gets the current BMS State.
pub fn state() -> BmsState { STATE.get() }

#[embassy_executor::task]
pub async fn state_machine_task() -> ! {
    use embassy_time::{Ticker, Duration};

    /// How often the state machine should tick.
    const TICK_PERIOD_MS: u64 = 20;

    let mut ticker = Ticker::every(Duration::from_millis(TICK_PERIOD_MS));
    loop {
        STATE.tick();
        ticker.next().await;
    }
}