use core::fmt::{Debug};
use cangen::ToCanFrame;
use defmt::{bitflags, debug, error, info, warn};
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::Timer;
use ner_can::NerCan;
use variant_count::VariantCount;
use crate::can::OUTGOING;
use crate::rtds::sound_rtds;
use crate::statemachine::FuncState::Ready;

#[derive(Copy, Clone, VariantCount, Eq, PartialEq, Debug, defmt::Format)]
pub enum FuncState {
    Ready,
    Pit,
    Reverse,
    Performance,
    Efficiency,
    Faulted
} // Must stay below 8 for CAN size limits

#[derive(Copy, Clone, VariantCount, Ord, PartialOrd, Eq, PartialEq, Debug, defmt::Format)]
pub enum NeroMenu {
    Off,
    Pit,
    Reverse,
    Performance,
    Efficiency,
    Games,
    Themes,
    Exit
} // Must stay below 15 for CAN size limits

impl NeroMenu {
    fn next(&self) -> NeroMenu {
        match self {
            NeroMenu::Off => NeroMenu::Pit,
            NeroMenu::Pit => NeroMenu::Reverse,
            NeroMenu::Reverse => NeroMenu::Performance,
            NeroMenu::Performance => NeroMenu::Efficiency,
            NeroMenu::Efficiency => NeroMenu::Games,
            NeroMenu::Games => NeroMenu::Games,
            NeroMenu::Themes => NeroMenu::Exit,
            NeroMenu::Exit => NeroMenu::Off,
        }
    }

    fn prev(&self) -> NeroMenu {
        match self {
            NeroMenu::Off => NeroMenu::Exit,
            NeroMenu::Pit => NeroMenu::Off,
            NeroMenu::Reverse => NeroMenu::Pit,
            NeroMenu::Performance => NeroMenu::Reverse,
            NeroMenu::Efficiency => NeroMenu::Performance,
            NeroMenu::Games => NeroMenu::Efficiency,
            NeroMenu::Themes => NeroMenu::Games,
            NeroMenu::Exit => NeroMenu::Themes
        }
    }
}

struct NeroState {
    menu: NeroMenu,
    home_mode: bool,
}

enum TransitionCommands {
    INCREMENT,
    DECREMENT,
    SELECT,
    HOME,
    READY,
    FAULT
}

bitflags!(
    struct TransitionError: u8 {
        const NONE = 0;
        const REVERSE_DISABLED = 1 << 0;
        const DRIVE_FROM_FAULT = 1 << 1;
        const SHUTDOWN_OPEN = 1 << 2;
        const BRAKES_NOT_ENGAGED = 1 << 3;
        const ENTER_GAMES_SHUTDOWN_CLOSED = 1 << 4;
        const ENTER_GAMES_WHILE_MOVING = 1 << 5;
        const ACCEL_PRESSED = 1 << 6;
    }
);

#[derive(Copy, Clone, Debug)]
struct CerberusState {
    home_mode: bool,
    state: FuncState,
    nero_menu: NeroMenu,
    transition_error: TransitionError
}

static STATE_MUTEX: OnceLock<Mutex<ThreadModeRawMutex, CerberusState>> = OnceLock::new();

const STATE_TRANSITION_QUEUE_SIZE: usize = 4;
const STATE_TRANSITION_ERROR_CLEAR_TIMEOUT: i32 = 3000; // ms

static TRANSITION_QUEUE: Channel<ThreadModeRawMutex, TransitionCommands, STATE_TRANSITION_QUEUE_SIZE> = Channel::new();

pub async fn get_func_state() -> FuncState {
    STATE_MUTEX.get().await.lock().await.state
}

pub async fn get_nero_menu() -> NeroMenu {
    STATE_MUTEX.get().await.lock().await.nero_menu
}

pub async fn get_active() -> bool {
    match STATE_MUTEX.get().await.lock().await.state {
        FuncState::Pit => {true}
        FuncState::Reverse => {true}
        FuncState::Performance => {true}
        FuncState::Efficiency => {true}
        _ => false
    }
}

async fn transition_functional_state(new_state: FuncState) -> Result<(), ()> {
    let mut state: CerberusState = STATE_MUTEX.get().await.lock().await.clone();

    state.transition_error = TransitionError::NONE;

    if false { // TODO: if accelerator pressed
        warn!("Accelerator should not be pressed when transitioning state");
        state.transition_error |= TransitionError::ACCEL_PRESSED;
        STATE_MUTEX.get().await.lock().await.clone_from(&state);
        return Err(());
    }

    if state.state == FuncState::Reverse {
        // TODO: stop reverse sound
    }

    match new_state {
        FuncState::Ready => {
            // TODO: call MCU fault with true
            state.state = FuncState::Ready;
            STATE_MUTEX.get().await.lock().await.clone_from(&state);
            Ok(())
        }
        FuncState::Faulted => {
            state.nero_menu = NeroMenu::Off;
            state.home_mode = true;
            // TODO: call MCU fault with true
            STATE_MUTEX.get().await.lock().await.clone_from(&state);
            Ok(())
        }
        _ => {
            if new_state == FuncState::Reverse && cfg!(feature = "disable_reverse") {
                state.transition_error |= TransitionError::REVERSE_DISABLED;
                STATE_MUTEX.get().await.lock().await.clone_from(&state);
                return Err(());
            }

            if state.state == FuncState::Faulted {
                state.transition_error |= TransitionError::DRIVE_FROM_FAULT;
                STATE_MUTEX.get().await.lock().await.clone_from(&state);
                return Err(());
            }

            if false { // TODO: brakes not open
                state.transition_error |= TransitionError::BRAKES_NOT_ENGAGED;
                STATE_MUTEX.get().await.lock().await.clone_from(&state);
                return Err(());
            }

            if false { // TODO: shutdown loop open
                state.transition_error |= TransitionError::SHUTDOWN_OPEN;
                STATE_MUTEX.get().await.lock().await.clone_from(&state);
                return Err(());
            }

            if new_state == FuncState::Reverse {
                // TODO sound reverse sound
            } else {
                sound_rtds().expect("Failed to sound RTDS");
            }

            info!("ACTIVE STATE: {:?}", new_state);

            if state.state == FuncState::Reverse {
                todo!("Stop reversing sound");
            }

            state.state = new_state;
            STATE_MUTEX.get().await.lock().await.clone_from(&state);
            Ok(())
        }
    }
}

async fn transition_nero_state(new_state: &mut NeroState) -> Result<(), ()> {
    let mut state: CerberusState = STATE_MUTEX.get().await.lock().await.clone();

    if get_active().await && !state.home_mode && new_state.home_mode {
        transition_functional_state(Ready).await.expect("Failed to transition state");
        return Ok(());
    }

    if !state.home_mode {
        debug!("Cannot transition current state as not in home mode");
        return Err(())
    }

    if new_state.menu < NeroMenu::Games && new_state.menu > NeroMenu::Off {
        let funcState: FuncState = match new_state.menu {
            NeroMenu::Pit => {FuncState::Pit}
            NeroMenu::Reverse => {FuncState::Reverse}
            NeroMenu::Performance => {FuncState::Performance}
            NeroMenu::Efficiency => {FuncState::Efficiency}
            _ => return Err(())
        };

        transition_functional_state(funcState).await.expect("Failed to transition state");
    } else if new_state.menu == NeroMenu::Games {
        if false { // TODO: if shutdown not closed
            state.transition_error |= TransitionError::ENTER_GAMES_SHUTDOWN_CLOSED;
            STATE_MUTEX.get().await.lock().await.clone_from(&state);
            return Err(());
        }

        if false { // TODO: if mph > 1
            state.transition_error |= TransitionError::ENTER_GAMES_WHILE_MOVING;
            STATE_MUTEX.get().await.lock().await.clone_from(&state);
            return Err(());
        }

        state.home_mode = false;
        STATE_MUTEX.get().await.lock().await.clone_from(&state);
        return Ok(());
    }

    state.nero_menu = new_state.menu;
    debug!("Transitioned NERO state");
    STATE_MUTEX.get().await.lock().await.clone_from(&state);
    Ok(())
}

fn handle_command(command: TransitionCommands) {

}

pub(crate) async fn increment_nero_index() {
    debug!("Increment nero index");

    TRANSITION_QUEUE.send(TransitionCommands::INCREMENT).await;
}

pub(crate) async fn decrement_nero_index() {
    debug!("Decrement nero index");

    TRANSITION_QUEUE.send(TransitionCommands::DECREMENT).await;
}

pub(crate) async fn select_nero_index() {
    debug!("Select nero index");
    TRANSITION_QUEUE.send(TransitionCommands::SELECT).await;
}

pub(crate) async fn set_home_mode() {
    debug!("Set mode mode");
    TRANSITION_QUEUE.send(TransitionCommands::HOME).await;
}

pub(crate) async fn set_ready_mode() {
    debug!("Set ready mode");
    TRANSITION_QUEUE.send(TransitionCommands::READY).await;
}

pub(crate) async fn fault() {
    debug!("Set fault mode");
    TRANSITION_QUEUE.send(TransitionCommands::FAULT).await;
}

#[embassy_executor::task]
pub async fn statemachine_task() {
    loop {
        let event: Either<TransitionCommands, ()> = select(TRANSITION_QUEUE.receive(), Timer::after_millis(500)).await;

        match event {
            Either::First(command) => {handle_command(command)}
            Either::Second(_) => {}
        }

        let state = STATE_MUTEX.get().await.lock().await;

        let frame = cangen::CarState::new()
            .with_home_mode(u8::from(state.home_mode))
            .with_nero_index(state.nero_menu as u8)
            .with_car_speed(0f32) // TODO: call dti mph
            .with_tsms(false)
            .with_torque_limit_percentage(0f32) // TODO call pedals torque limit
            .with_reverse(state.state != FuncState::Reverse)
            .with_launch_control(false)
            .with_functional_state(state.state as u8)
            .with_traction_control(false)
            .with_state_transition_error(state.transition_error.bits());

        OUTGOING.send(frame.to_can_frame()).await;
    }
}
