use defmt::{debug, info, warn};
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    Peri, bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    interrupt, peripherals,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::Timer;

bind_interrupts!(
    struct Irqs {
        EXTI2 => embassy_stm32::exti::InterruptHandler<interrupt::typelevel::EXTI2>;
    }
);

const LED_COMMAND_QUEUE_SIZE: usize = 4;

#[derive(Clone, Copy)]
enum LedCommand {
    RedOn,
    RedOff,
    GreenOn,
    GreenOff,
}
static LED_COMMANDS: Channel<ThreadModeRawMutex, LedCommand, LED_COMMAND_QUEUE_SIZE> =
    Channel::new();

pub struct DebugResources {
    pub red_led: Peri<'static, peripherals::PE3>,
    pub green_led: Peri<'static, peripherals::PE4>,
    pub button: Peri<'static, peripherals::PE2>,
    pub button_exti: Peri<'static, peripherals::EXTI2>,
}

fn send_led_command(command: LedCommand) {
    if LED_COMMANDS.try_send(command).is_err() {
        warn!("LED command queue full, dropping command");
    }
}

pub fn enable_red_led() {
    debug!("debug_enableRedLED called");
    send_led_command(LedCommand::RedOn);
}

pub fn enable_green_led() {
    debug!("debug_enableGreenLED called");
    send_led_command(LedCommand::GreenOn);
}

pub fn disable_red_led() {
    debug!("debug_disableRedLED called");
    send_led_command(LedCommand::RedOff);
}

pub fn disable_green_led() {
    debug!("debug_disableGreenLED called");
    send_led_command(LedCommand::GreenOff);
}

pub fn user_button_pressed() {
    info!("User button pressed.");
}

#[embassy_executor::task]
pub async fn debug_task(resources: DebugResources) {
    let mut red_led = Output::new(resources.red_led, Level::Low, Speed::Low);
    let mut green_led = Output::new(resources.green_led, Level::Low, Speed::Low);
    let mut button = ExtiInput::new(resources.button, resources.button_exti, Pull::Up, Irqs);

    loop {
        match select(LED_COMMANDS.receive(), button.wait_for_falling_edge()).await {
            Either::First(command) => match command {
                LedCommand::RedOn => red_led.set_high(),
                LedCommand::RedOff => red_led.set_low(),
                LedCommand::GreenOn => green_led.set_high(),
                LedCommand::GreenOff => green_led.set_low(),
            },
            Either::Second(()) => {
                user_button_pressed();
                Timer::after_millis(50).await; // debounce
            }
        }
    }
}
