#![no_std]
#![no_main]

mod hardware;
mod inbox;
mod state;
use crate::hardware::Leds;
use crate::inbox::FaultframeState;
use crate::inbox::inbox::{BMS_CAN_ID, IMD_CAN_ID, LATCHING_CAN_ID};
use can_handler::{NerCan, can_handler};
use core::fmt::Write;
use core::num::{NonZeroU8, NonZeroU16};
use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::{Config, can, dma, peripherals, usart};
use embassy_stm32::{bind_interrupts, wdg::IndependentWatchdog};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
use embedded_can::ExtendedId;
use heapless::String;
use heapless::mpmc::Queue;

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

    let mut ner_can = NerCan::init(can::CanConfigurator::new(p.FDCAN2, p.PB12, p.PB13, IrqsCan));
    ner_can = ner_can
        .add_standard_filter(
            can::filter::StandardFilterSlot::_0,
            LATCHING_CAN_ID,
            Some(IMD_CAN_ID),
        )
        .add_extended_filter(can::filter::ExtendedFilterSlot::_0, BMS_CAN_ID, None);
    // There used to be some configuration here, but I removed it s.t I wouldn't step on NerCan's toes
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
    #[expect(deprecated)]
    static FFS_QUEUE: Queue<Option<FaultframeState>, 32> = Queue::new();
    // A mutex that isn't a mutex. Contains a mpmc queue that is neither multi producer nor multi
    // consumer
    static QUEUTEX: Mutex<ThreadModeRawMutex, &'static Queue<Option<FaultframeState>, 32>> =
        Mutex::new(&FFS_QUEUE);

    static RX_CHANNEL: Channel<ThreadModeRawMutex, Frame, 16> = Channel::new();
    static TX_CHANNEL: Channel<ThreadModeRawMutex, Frame, 16> = Channel::new();

    _spawner.spawn(
        can_handler(
            ner_can.can_configurator,
            TX_CHANNEL.sender(),
            RX_CHANNEL.receiver(),
        )
        .expect("Failed to init candler"),
    );

    let mut s: String<128> = String::new();
    core::write!(&mut s, "MSB-FW.rs prints in RTT, not UART!\r\n",).unwrap();
    unwrap!(usart.write(s.as_bytes()).await);

    let mut watchdog = IndependentWatchdog::new(p.IWDG, 5000000);
    watchdog.unleash();
    let mut ticker = Ticker::every(Duration::from_millis(500));

    _spawner.spawn(
        inbox::inbox::populate_queue(RX_CHANNEL.receiver(), &QUEUTEX)
            .expect("Failed to spawn inbox queue populator"),
    );

    let leds: Leds = Leds::new(p.PE4, p.PE5);
    _spawner.spawn(
        state::state_machine::state_machine(&QUEUTEX, leds).expect("Failed to spawn state machine"),
    );
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
