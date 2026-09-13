#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::{time::Hertz, wdg};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Initializing project...");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::mux::*;
        use embassy_stm32::rcc::*;

        config.rcc.hsi48 = Some(Default::default());
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::Hse,
            prediv: PllPreDiv::Div2,
            mul: PllMul::Mul28,
            divp: Some(PllDiv::Div2),
            divq: Some(PllDiv::Div2),
            divr: None,
        });
        config.rcc.sys = Sysclk::Pll1P;
        config.rcc.ahb_pre = AHBPrescaler::Div1;
        config.rcc.apb1_pre = APBPrescaler::Div2;

        config.rcc.pll2 = Some(Pll {
            source: PllSource::Hse,
            prediv: PllPreDiv::Div5,
            mul: PllMul::Mul64,
            divp: Some(PllDiv::Div5),
            divq: Some(PllDiv::Div5),
            divr: None,
        });

        config.rcc.mux.lpuart1sel = Lpusartsel::Pclk3;
        config.rcc.mux.uart4sel = Usartsel::Pclk1;

        config.rcc.mux.spi1sel = Spi1sel::Pll2P;
        config.rcc.mux.spi2sel = Spi2sel::Pll2P;
        config.rcc.mux.spi3sel = Spi3sel::Pll2P;

        config.rcc.mux.fdcan12sel = Fdcansel::Pll2Q;

        config.rcc.voltage_scale = VoltageScale::Scale1;
    }

    let p = embassy_stm32::init(config);

    let mut watchdog = wdg::IndependentWatchdog::new(p.IWDG, 1000000);
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
