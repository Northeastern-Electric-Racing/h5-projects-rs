use crate::{pedals, statemachine};
use defmt::debug;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

const BUTTON_QUEUE_SIZE: usize = 12;
static PRESSED: Channel<ThreadModeRawMutex, Button, BUTTON_QUEUE_SIZE> = Channel::new();

enum Button {
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
    Unknown,
}

impl Button {
    fn from_id(id: u8) -> Button {
        match id {
            0 => Button::Escape,
            1 => Button::Left,
            2 => Button::Right,
            3 => Button::Enter,
            4 => Button::UpRegen,
            5 => Button::DownRegen,
            6 => Button::UpTorque,
            7 => Button::DownTorque,
            8 => Button::ToggleLaunchControl,
            9 => Button::ToggleTractionControl,
            _ => {
                debug!("Unexpected id: {}", id);
                Button::Unknown
            }
        }
    }
}

pub async fn register_press(button: u8) {
    debug!("Pressed: {}", button);
    PRESSED.send(Button::from_id(button)).await;
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
            Button::ToggleTractionControl => {},
            Button::Unknown => {}
        }
    }
}
