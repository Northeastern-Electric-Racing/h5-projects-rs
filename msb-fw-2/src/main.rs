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
use msb_fw_2::multiplexor_handler::{self, MuxResources};
use msb_fw_2::shock_pot;
use msb_fw_2::steering_angle;
use msb_fw_2::strain_guage;
use msb_fw_2::wheel_speed::{self, WheelSpeedResources};
use {defmt_rtt as _, panic_probe as _};

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

    let mux_resources = MuxResources {
        adc1: p.ADC1,
        adc2: p.ADC2,
        dma1: p.GPDMA1_CH0,
        dma2: p.GPDMA1_CH1,
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
    };

    spawner.spawn(multiplexor_handler::mux_scan_task(mux_resources).expect("Failed to spawn multiplexor_handler::mux_scan_task()."));
    spawner.spawn(shock_pot::shock_pot_task().expect("Failed to spawn shock_pot::shock_pot_task()."));
    spawner.spawn(steering_angle::steering_angle_task().expect("Failed to spawn steering_angle::steering_angle_task()."));
    spawner.spawn(strain_guage::strain_gauge_task().expect("Failed to spawn strain_guage::strain_gauge_task()."));

    let wheel_speed_resources = WheelSpeedResources {
        left_timer: p.TIM1,
        left_pin: p.PE9,
        right_timer: p.TIM15,
        right_pin: p.PC12,
    };
    spawner.spawn(wheel_speed::wheel_speed_task(wheel_speed_resources).expect("Failed to spawn wheel_speed::wheel_speed_task()."));

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
