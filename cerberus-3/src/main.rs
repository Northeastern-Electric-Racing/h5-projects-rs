#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::info;
use defmt::{debug};
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::gpio::Level;
use embassy_stm32::gpio::Output;
use embassy_stm32::gpio::Speed;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod adc;
mod can;
mod efuses;
mod rtds;

use adc::{Adc1Resources, Adc2Resources, adc1_task, adc2_task};
use can::CanPins;
use efuses::{EfusePins, efuse_task};
use ner_can::{can_rx, can_tx};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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

    // initials Debug LEDs
    let mut red_led = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut green_led = Output::new(p.PE4, Level::Low, Speed::Low);

    // CAN
    let (can_tx_half, can_rx_half) = can::init(CanPins {
        can: p.FDCAN2,
        rx: p.PB5,
        tx: p.PB6,
    });
    spawner.spawn(
        can_tx(
            can_tx_half,
            can::OUTGOING.dyn_receiver(),
            can::OUTGOING.dyn_sender(),
        )
        .expect("Failed to spawn ner_can::can_tx()."),
    );
    spawner.spawn(
        can_rx(can_rx_half, can::INCOMING.dyn_sender())
            .expect("Failed to spawn ner_can::can_rx()."),
    );

    // ADC1 + analog mux
    spawner.spawn(
        adc1_task(Adc1Resources {
            adc: p.ADC1,
            dma: p.GPDMA1_CH0,
            dash: p.PA6,
            mux1: p.PA0,
            mux3: p.PB1,
            mux4: p.PB0,
            fanbatt: p.PF12,
            pump1: p.PF11,
            pump2: p.PC3,
            mc: p.PA4,
            mux2: p.PA3,
            sel1: p.PC6,
            sel2: p.PC7,
            sel3: p.PC8,
            sel4: p.PC9,
        })
        .expect("Failed to spawn adc::adc1_task()."),
    );

    spawner.spawn(
        adc2_task(Adc2Resources {
            adc: p.ADC2,
            dma: p.GPDMA1_CH4,
            apps1: p.PC2,
            apps2: p.PC0,
            bse1: p.PF13,
            bse2: p.PF14,
        })
        .expect("Failed to spawn adc2_task()."),
    );

    // eFuses
    spawner.spawn(
        efuse_task(EfusePins {
            dashboard_en: p.PD0,
            dashboard_er: p.PD1,
            brake_en: p.PD8,
            brake_er: p.PD9,
            shutdown_en: p.PG6,
            shutdown_er: p.PG7,
            lv_en: p.PD6,
            lv_er: p.PD7,
            radfan_en: p.PG4,
            radfan_er: p.PG5,
            fanbatt_en: p.PD10,
            fanbatt_er: p.PD11,
            pump1_en: p.PD12,
            pump1_er: p.PD13,
            pump2_en: p.PD14,
            pump2_er: p.PD15,
            battbox_en: p.PF2,
            battbox_er: p.PF3,
            mc_en: p.PF4,
            mc_er: p.PF5,
            spare_en: p.PG10,
            spare_er: p.PG11,
        })
        .expect("Failed to spawn efuses::efuse_task()."),
    );

    // RTDS
    // shutdown isn't ported so is_shutdown_closed_placeholder just returns false for now
    let rtds_pin = Output::new(p.PD2, Level::Low, Speed::Low);
    spawner.spawn(
        rtds::rtds_task(rtds_pin, rtds::is_shutdown_closed_placeholder)
            .expect("Failed to spawn rtds::rtds_task()."),
    );

    // Watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 1_000_000);
    watchdog.unleash();

    loop {
        debug!("Status: Alive");
        watchdog.pet();
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
