#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod boot;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    info!("Initializing project...");

    let mut config = Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: embassy_stm32::rcc::HseMode::Oscillator,
    });
    let p = embassy_stm32::init(config);

    boot::start(spawner, p.FLASH, p.FDCAN2, p.PD9, p.PB13)
        .expect("Failed to confirm application startup");

    let mut watchdog = IndependentWatchdog::new(p.IWDG, 1000000);
    watchdog.unleash();
    loop {
        debug!("Status: Alive");
        Timer::after_millis(500).await;
        watchdog.pet();
    }
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset() // <- you could do something other than reset
}
