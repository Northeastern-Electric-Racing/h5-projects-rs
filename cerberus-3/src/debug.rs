use core::sync::atomic::{AtomicBool, Ordering};
use defmt::debug;
use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};

use defmt::info;
use embassy_stm32::{exti::ExtiInput, mode::Async};
use embassy_time::Timer;

#[derive(Clone, Copy)]
struct LedState {
    red: bool, 
    green: bool,
}
static LED_SIGNAL: Signal<ThreadModeRawMutex, LedState> = Signal::new();
static RED_ON: AtomicBool = AtomicBool::new(false);
static GREEN_ON: AtomicBool = AtomicBool::new(false);

fn push_state() {
    LED_SIGNAL.signal(LedState {
        red: RED_ON.load(Ordering::Relaxed),
        green: GREEN_ON.load(Ordering::Relaxed),
    });
}

pub fn enable_red_led() {
    debug!("debug_enableRedLED called"); 
    RED_ON.store(true, Ordering::Relaxed);
    push_state();
}

pub fn enable_green_led() {
    debug!("debug_enableGreenLED called"); 
    GREEN_ON.store(true, Ordering::Relaxed);
    push_state();
}

pub fn disable_red_led() {
    debug!("debug_disableRedLED called"); 
    RED_ON.store(false, Ordering::Relaxed);
    push_state();
}

pub fn disable_green_led() {
    debug!("debug_disableGreenLED called"); 
    GREEN_ON.store(false, Ordering::Relaxed);
    push_state();
}

pub fn user_button_pressed() {
    info!("User button pressed.");
}

#[embassy_executor::task]
pub async fn led_task(mut red: Output<'static>, mut green: Output<'static>) {
    red.set_low();
    green.set_low();
    loop{
        let state = LED_SIGNAL.wait().await;
            if state.red {
                red.set_high();
            } else {
                red.set_low();
            }
            if state.green {
                green.set_high();
            } else {
                green.set_low();
            }
    }
}

#[embassy_executor::task]
pub async fn user_button_task(mut button: ExtiInput<'static, Async>) {
    loop {
        button.wait_for_falling_edge().await;
        user_button_pressed(); 
        Timer::after_millis(50).await;
    }
}