use crate::{pedals, statemachine, traction};
use defmt::debug;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

const BUTTON_QUEUE_SIZE: usize = 32;
static PRESSED: Channel<ThreadModeRawMutex, Button, BUTTON_QUEUE_SIZE> = Channel::new();

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) enum Button {
    Escape,
    Left,
    Right,
    Enter,
    UpRegen,
    DownRegen,
    UpTorque,
    DownTorque,
    ToggleLaunchControl,
    ToggleTractionControl,
}

impl TryFrom<u8> for Button {
    type Error = u8;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Button::Escape),
            1 => Ok(Button::Left),
            2 => Ok(Button::ToggleLaunchControl),
            3 => Ok(Button::UpRegen),
            4 => Ok(Button::DownRegen),
            5 => Ok(Button::Enter),
            6 => Ok(Button::Right),
            7 => Ok(Button::ToggleTractionControl),
            8 => Ok(Button::UpTorque),
            9 => Ok(Button::DownTorque),
            _ => Err(value),
        }
    }
}

pub fn register_press(button: Button) {
    debug!("Pressed: {}", button);
    if PRESSED.try_send(button).is_err() {
        debug!("Button queue full, dropping press {:?}", button);
    }
}

#[embassy_executor::task]
pub async fn button_task() -> ! {
    loop {
        match PRESSED.receive().await {
            Button::Escape => statemachine::set_home_state().await,
            Button::Left => statemachine::decrement_nero_index().await,
            Button::Right => statemachine::increment_nero_index().await,
            Button::Enter => statemachine::select_nero_index().await,

            Button::UpRegen => pedals::increase_regen_limit(statemachine::get_func_state().await),
            Button::DownRegen => pedals::decrease_regen_limit(statemachine::get_func_state().await),
            Button::UpTorque => pedals::increase_torque_limit(),
            Button::DownTorque => pedals::decrease_torque_limit(),

            Button::ToggleLaunchControl => pedals::toggle_launch_control(),
            Button::ToggleTractionControl => traction::toggle_tc(),
            Button::Unknown => {}
        }
    }
}
