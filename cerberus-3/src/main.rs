#![no_std]
#![no_main]

#[macro_use]
extern crate uom;

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::Peri;
use embassy_stm32::gpio::Level;
use embassy_stm32::gpio::Output;
use embassy_stm32::gpio::Speed;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, peripherals as stm32_peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod adc;
mod can;
mod efuses;
mod peripherals;
mod rtds;

use adc::{Adc1Resources, adc1_task};
use can::CanPins;
use efuses::{EfusePins, efuse_task};
use ner_can::{can_rx, can_tx};

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<stm32_peripherals::GPDMA1_CH1>;
    GPDMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<stm32_peripherals::GPDMA1_CH2>;
});

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

    // IMU
    // wiring matches the C project: SPI2 at 2 MHz (64 MHz PLL2P / prescaler 32), CS on PB9
    let mut imu_spi_config = spi::Config::default();
    imu_spi_config.frequency = Hertz::mhz(2);
    let imu_spi_bus = Spi::new(
        p.SPI2,
        p.PA12,
        p.PG1,
        p.PB14,
        p.GPDMA1_CH1,
        p.GPDMA1_CH2,
        Irqs,
        imu_spi_config,
    );
    static IMU_SPI_BUS: StaticCell<
        Mutex<ThreadModeRawMutex, Spi<'static, Async, spi::mode::Master>>,
    > = StaticCell::new();
    let imu_spi_bus = IMU_SPI_BUS.init(Mutex::new(imu_spi_bus));
    let imu_cs = Output::new(p.PB9, Level::High, Speed::High);
    let spi: peripherals::ImuSpi = SpiDevice::new(imu_spi_bus, imu_cs);
    spawner.spawn(
        peripherals::imu_task(spi, can::OUTGOING.dyn_sender())
            .expect("Failed to spawn peripherals::imu_task()"),
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
