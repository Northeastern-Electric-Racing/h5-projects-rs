#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::eth;
use embassy_stm32::gpio::Level;
use embassy_stm32::gpio::Output;
use embassy_stm32::gpio::Speed;
use embassy_stm32::peripherals;
use embassy_stm32::rng;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::i2c;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct IrqsEth {
    ETH => eth::InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

bind_interrupts!(struct IrqsI2c {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<
    'static,
    eth::Ethernet<
    'static,
    peripherals::ETH,
    eth::GenericPhy<eth::Sma<'static, peripherals::ETH_SMA>>,
    >,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Initializing project...");

    // Clock
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
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL28,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;

        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL64,
            divp: Some(PllDiv::DIV5),
            divq: Some(PllDiv::DIV5),
            divr: None,
        });

        config.rcc.mux.lpuart1sel = Lpusartsel::PCLK3;
        config.rcc.mux.uart4sel = Usartsel::PCLK1;

        config.rcc.mux.spi1sel = Spi1sel::PLL2_P;
        config.rcc.mux.spi2sel = Spi2sel::PLL2_P;
        config.rcc.mux.spi3sel = Spi3sel::PLL2_P;

        config.rcc.mux.fdcan12sel = Fdcansel::PLL2_Q;

        config.rcc.voltage_scale = VoltageScale::Scale1;
    }

    let p = embassy_stm32::init(config);
    
    // initials Debug LEDs
    let mut red_led = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut green_led = Output::new(p.PE4, Level::Low, Speed::Low);

    // Watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 1000000);
    watchdog.unleash();

    loop {
        debug!("Status: Alive");
        red_led.set_high();
        green_led.set_low();
        Timer::after_millis(500).await;
        watchdog.pet();
        red_led.set_low();
        green_led.set_high();
        Timer::after_millis(500).await;
    }
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset() // <- you could do something other than reset
}
