#![no_std]
#![no_main]

mod hardware;
mod inbox;
mod state;
use core::fmt::Write;
use core::num::{NonZeroU8, NonZeroU16};
use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::{Config, can, dma, peripherals, usart};
use embassy_stm32::{bind_interrupts, wdg::IndependentWatchdog};
use embassy_time::{Duration, Ticker, Timer};
use embedded_can::ExtendedId;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct IrqsCan {
    FDCAN2_IT0 => can::IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<peripherals::FDCAN2>;
});

bind_interrupts!(struct IrqsUsart {
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Initializing wheel...");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::mux::*;
        use embassy_stm32::rcc::*;
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

    // initialize the project, ensure we can debug during sleep
    let p = embassy_stm32::init(config);

    let mut can = can::CanConfigurator::new(p.FDCAN2, p.PB12, p.PB13, IrqsCan);
    {
        use embassy_stm32::can::config::*;
        use embassy_stm32::can::filter::*;
        use embedded_can::StandardId;
        let can_config = FdCanConfig::default()
            .set_automatic_bus_off_recovery(true)
            .set_automatic_retransmit(false)
            .set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly)
            .set_clock_divider(ClockDivider::_1)
            .set_data_bit_timing(DataBitTiming {
                transceiver_delay_compensation: false,
                prescaler: NonZeroU16::new(8).unwrap(),
                seg1: NonZeroU8::new(8).unwrap(),
                seg2: NonZeroU8::new(4).unwrap(),
                sync_jump_width: NonZeroU8::new(1).unwrap(),
            })
            .set_transmit_pause(true)
            .set_global_filter(GlobalFilter::reject_all());
        can.set_config(can_config);

        let mut std1 = StandardFilter::default();
        std1.filter = FilterType::DedicatedDual(
            StandardId::new(0x37).unwrap(),
            StandardId::new(0x01E).unwrap(),
        ); // IMD and BMS LIGHTNING
        std1.action = Action::StoreInFifo0;

        let mut ext1 = ExtendedFilter::default();
        ext1.filter = FilterType::DedicatedSingle(ExtendedId::new(0x0CA).unwrap()); // Cerb lightning
        ext1.action = Action::StoreInFifo0;

        can.properties()
            .set_standard_filter(StandardFilterSlot::_0, std1);
        can.properties()
            .set_extended_filter(ExtendedFilterSlot::_0, ext1);
    }
    let mut can = can.into_normal_mode();

    let mut usart_config = usart::Config::default();
    usart_config.swap_rx_tx = true;
    let mut usart = Uart::new(
        p.LPUART1,
        p.PA10,
        p.PA9,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        IrqsUsart,
        usart_config,
    )
    .unwrap();

    let mut s: String<128> = String::new();
    core::write!(&mut s, "MSB-FW.rs prints in RTT, not UART!\r\n",).unwrap();
    unwrap!(usart.write(s.as_bytes()).await);

    let mut watchdog = IndependentWatchdog::new(p.IWDG, 1000000);
    watchdog.unleash();
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        debug!("Status: Alive");
        Timer::after_millis(500).await;
        ticker.next().await;
        watchdog.pet();
    }
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset() // <- you could do something other than reset
}
