//! Cerberus-3 vehicle state machine.
//!
//! Owns the car's functional state (`FuncState`: Ready, Pit, Reverse,
//! Performance, Efficiency, Faulted) and the NERO dash menu position
//! (`NeroMenu`), and arbitrates transitions between them.

use crate::rtds;
use crate::shutdown::{self, BmsShutdownState};
use cangen::ToCanFrame;
use defmt::{bitflags, debug, info, warn};
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use variant_count::VariantCount;

const STATE_TRANSITION_QUEUE_SIZE: usize = 4;
const STATE_TRANSITION_ERROR_CLEAR_TIMEOUT: Duration = Duration::from_millis(3000);
const SEND_NERO_TIMEOUT: Duration = Duration::from_millis(200);

static STATE_MUTEX: Mutex<ThreadModeRawMutex, CerberusState> = Mutex::new(CerberusState::INIT);
static TRANSITION_QUEUE: Channel<
    ThreadModeRawMutex,
    TransitionCommands,
    STATE_TRANSITION_QUEUE_SIZE,
> = Channel::new();

#[derive(Copy, Clone, VariantCount, Eq, PartialEq, Debug, defmt::Format)]
pub enum FuncState {
    Ready,
    Pit,
    Reverse,
    Performance,
    Efficiency,
    Faulted,
} // Must stay below 8 for CAN size limits

impl FuncState {
    /// If this is a Drive state
    fn is_active(&self) -> bool {
        matches!(
            self,
            FuncState::Pit | FuncState::Reverse | FuncState::Performance | FuncState::Efficiency
        )
    }
}

#[derive(Copy, Clone, VariantCount, Eq, PartialEq, Debug, defmt::Format)]
pub enum NeroMenu {
    Off,
    Pit,
    Reverse,
    Performance,
    Efficiency,
    Games,
    Themes,
    Exit,
} // Must stay below 15 for CAN size limits

impl NeroMenu {
    fn next(&self) -> NeroMenu {
        match self {
            NeroMenu::Off => NeroMenu::Pit,
            NeroMenu::Pit => NeroMenu::Reverse,
            NeroMenu::Reverse => NeroMenu::Performance,
            NeroMenu::Performance => NeroMenu::Efficiency,
            NeroMenu::Efficiency => NeroMenu::Games,
            NeroMenu::Games => NeroMenu::Themes,
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
            NeroMenu::Exit => NeroMenu::Themes,
        }
    }
}

enum TransitionCommands {
    INCREMENT,
    DECREMENT,
    SELECT,
    HOME,
    READY,
    FAULT,
}

bitflags!(
    pub struct TransitionError: u8 {
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

// ---------------------------------------------------------------------------
// TEMP functions, replace later
// ---------------------------------------------------------------------------

fn accel_pressed() -> bool {
    false
}

fn pedals_get_brake_state() -> bool {
    true
}

fn get_launch_control() -> bool {
    false
}

fn get_traction_control() -> bool {
    false
}

fn get_mph() -> f32 {
    0.0f32
}

fn get_torque_limit_percent() -> f32 {
    100.0f32
}

fn set_mcu_fault(_faulted: bool) {}

// ---------------------------------------------------------------------------
// END TEMP functions
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, Debug, defmt::Format)]
struct CerberusState {
    home_mode: bool,
    state: FuncState,
    nero_menu: NeroMenu,
    transition_error: TransitionError,
    /// Instant::MAX if no deadline
    error_deadline: Instant,
}

impl CerberusState {
    const INIT: Self = Self {
        home_mode: true,
        state: FuncState::Ready,
        nero_menu: NeroMenu::Off,
        transition_error: TransitionError::NONE,
        error_deadline: Instant::MAX, // never expires
    };

    fn transition_functional_state(&mut self, new_state: FuncState) -> Result<(), ()> {
        if self.state == new_state {
            return Ok(());
        }

        self.clear_error();

        if accel_pressed() && new_state != FuncState::Faulted {
            debug!("Accelerator should not be pressed when transitioning state");
            self.set_error(TransitionError::ACCEL_PRESSED);
            return Err(());
        }

        match new_state {
            FuncState::Ready => {
                set_mcu_fault(false);
                self.set_func_state(FuncState::Ready);
                Ok(())
            }
            FuncState::Faulted => {
                set_mcu_fault(true);
                self.nero_menu = NeroMenu::Off;
                self.set_func_state(FuncState::Faulted);
                self.home_mode = true;
                Ok(())
            }
            FuncState::Pit
            | FuncState::Reverse
            | FuncState::Performance
            | FuncState::Efficiency => {
                if new_state == FuncState::Reverse && cfg!(feature = "disable_reverse") {
                    self.set_error(TransitionError::REVERSE_DISABLED);
                    return Err(());
                }

                if self.state == FuncState::Faulted {
                    self.set_error(TransitionError::DRIVE_FROM_FAULT);
                    return Err(());
                }

                if !pedals_get_brake_state() {
                    self.set_error(TransitionError::BRAKES_NOT_ENGAGED);
                    return Err(());
                }

                if shutdown::get_shutdown_state() == BmsShutdownState::Open {
                    self.set_error(TransitionError::SHUTDOWN_OPEN);
                    return Err(());
                }

                // rtds switches straight to the new sound, even if coming from another active state
                if new_state == FuncState::Reverse {
                    if rtds::start_reverse_rtds().is_err() {
                        warn!("Failed to start Reverse Sound")
                    }
                } else if rtds::sound_rtds().is_err() {
                    warn!("Failed to start Ready to Drive Sound")
                }

                info!("ACTIVE STATE: {:?}", new_state);
                self.set_func_state(new_state);
                self.home_mode = false;
                Ok(())
            }
        }
    }

    fn transition_nero_state(&mut self, new_state: NeroMenu) -> Result<(), ()> {
        if !self.home_mode {
            debug!("Cannot transition current state as not in home mode");
            return Err(());
        }

        self.nero_menu = new_state;
        debug!("Transitioned NERO state");
        Ok(())
    }

    /// Sets the func state, stops reversing sound
    fn set_func_state(&mut self, new: FuncState) {
        if self.state == FuncState::Reverse
            && new != FuncState::Reverse
            && rtds::cancel_sounds().is_err()
        {
            warn!("Failed to stop Reverse Sound")
        }
        self.state = new;
    }

    /// Add the error flag and set the deadline for STATE_TRANSITION_ERROR_CLEAR_TIMEOUT from now
    fn set_error(&mut self, error: TransitionError) {
        self.transition_error |= error;
        self.error_deadline = Instant::now() + STATE_TRANSITION_ERROR_CLEAR_TIMEOUT;
    }

    /// Clear the errors and disable deadline
    fn clear_error(&mut self) {
        self.transition_error = TransitionError::NONE;
        self.error_deadline = Instant::MAX;
    }

    /// Select the current NERO state
    fn select(&mut self) -> Result<(), ()> {
        if !self.home_mode {
            debug!("Cannot select current state as not in home mode");
            return Err(());
        }

        match self.nero_menu {
            NeroMenu::Off => Err(()),
            NeroMenu::Pit => self.transition_functional_state(FuncState::Pit),
            NeroMenu::Reverse => self.transition_functional_state(FuncState::Reverse),
            NeroMenu::Performance => self.transition_functional_state(FuncState::Performance),
            NeroMenu::Efficiency => self.transition_functional_state(FuncState::Efficiency),
            NeroMenu::Games => {
                if shutdown::get_shutdown_state() == BmsShutdownState::Closed {
                    self.set_error(TransitionError::ENTER_GAMES_SHUTDOWN_CLOSED);
                    return Err(());
                }

                if get_mph() >= 1.0f32 {
                    self.set_error(TransitionError::ENTER_GAMES_WHILE_MOVING);
                    return Err(());
                }

                self.home_mode = false;
                Ok(())
            }
            NeroMenu::Themes => {
                self.home_mode = false;
                Ok(())
            }
            NeroMenu::Exit => Err(()),
        }
    }

    fn handle_command(&mut self, command: TransitionCommands) -> Result<(), ()> {
        match command {
            TransitionCommands::INCREMENT => self.transition_nero_state(self.nero_menu.next()),
            TransitionCommands::DECREMENT => self.transition_nero_state(self.nero_menu.prev()),
            TransitionCommands::SELECT => self.select(),
            TransitionCommands::HOME => {
                if get_mph() > 0.1f32 {
                    return Err(());
                }

                if self.state.is_active() {
                    self.transition_functional_state(FuncState::Ready)?;
                }
                self.home_mode = true;
                Ok(())
            }
            TransitionCommands::READY => self.transition_functional_state(FuncState::Ready),
            TransitionCommands::FAULT => self.transition_functional_state(FuncState::Faulted),
        }
    }

    /// Construct the CarState CAN struct
    fn to_frame(self) -> cangen::CarState {
        cangen::CarState::new()
            .with_home_mode(u8::from(self.home_mode))
            .with_nero_index(self.nero_menu as u8)
            .with_car_speed(get_mph())
            .with_tsms(shutdown::get_shutdown_state() == BmsShutdownState::Closed)
            .with_torque_limit_percentage(get_torque_limit_percent())
            // Same expression as the C (`functional != F_REVERSE`). Verify against
            // the CAN definition if the dash shows reverse inverted.
            .with_reverse(self.state != FuncState::Reverse)
            .with_launch_control(get_launch_control())
            .with_functional_state(self.state as u8)
            .with_traction_control(get_traction_control())
            .with_state_transition_error(self.transition_error.bits())
    }
}

pub(crate) async fn get_func_state() -> FuncState {
    STATE_MUTEX.lock().await.state
}

pub(crate) async fn get_nero_menu() -> NeroMenu {
    STATE_MUTEX.lock().await.nero_menu
}

pub(crate) async fn get_active() -> bool {
    STATE_MUTEX.lock().await.state.is_active()
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

pub(crate) async fn set_home_state() {
    debug!("Set home state");
    TRANSITION_QUEUE.send(TransitionCommands::HOME).await;
}

pub(crate) async fn set_ready_state() {
    debug!("Set ready state");
    TRANSITION_QUEUE.send(TransitionCommands::READY).await;
}

pub(crate) async fn set_fault_state() {
    debug!("Set fault state");
    TRANSITION_QUEUE.send(TransitionCommands::FAULT).await;
}

#[embassy_executor::task]
pub async fn statemachine_task() -> ! {
    let mut state = CerberusState::INIT;

    loop {
        match select3(
            TRANSITION_QUEUE.receive(),      // On a new command, handle it
            Timer::at(state.error_deadline), // Clear errors after deadline
            Timer::after(SEND_NERO_TIMEOUT), // Periodically send CAN
        )
        .await
        {
            Either3::First(command) => {
                if state.handle_command(command).is_err() {
                    debug!("Error handling NERO command");
                } else {
                    debug!("Now state: {}", state);
                }
            }
            Either3::Second(_) => {
                // clear the transition errors and disable deadline
                state.clear_error()
            }
            Either3::Third(_) => {}
        }

        *STATE_MUTEX.lock().await = state; // Copy the edited state into the readable mutex
        crate::can::OUTGOING
            .send(state.to_frame().to_can_frame())
            .await;
    }
}
