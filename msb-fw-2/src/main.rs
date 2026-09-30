#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::adc::{Adc, Config as AdcConfig};
use embassy_stm32::{time::Hertz, wdg};
use embassy_time::Timer;
use msb_fw_2::multiplexor_handler::{MuxHandler, MuxPins, SharedMux};
use msb_fw_2::shock_pot;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static MUX: StaticCell<SharedMux> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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

    let adc1 = Adc::new_blocking(p.ADC1, AdcConfig::default());
    let adc2 = Adc::new_blocking(p.ADC2, AdcConfig::default());
    let mux = MUX.init(SharedMux::new(MuxHandler::new(
        adc1,
        adc2,
        MuxPins {
            u18_sel1: p.PC6,
            u18_sel2: p.PC7,
            u18_sel3: p.PC8,
            u18_sel4: p.PC9,
            u19_sel1: p.PF6,
            u19_sel2: p.PF7,
            u19_sel3: p.PF8,
            u19_sel4: p.PF9,
            u18_d1: p.PC0,
            u18_d2: p.PC2,
            u18_d3: p.PC3,
            u18_d4: p.PA0,
            u19_d1: p.PA3,
            u19_d2: p.PF13,
            u19_d3: p.PF14,
            u19_d4: p.PF12,
        },
    )));

    spawner.spawn(shock_pot::shock_pot_task(mux).expect("Failed to spawn shock_pot::shock_pot_task()."));

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
