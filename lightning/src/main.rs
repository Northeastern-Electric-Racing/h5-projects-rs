#![no_std]
#![no_main]

mod can;
mod hardware;
mod state;

use crate::hardware::Leds;
use cangen::ToCanFrame;
use core::fmt::Write;
use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::{Config, dma, peripherals, usart};
use embassy_stm32::{bind_interrupts, wdg::IndependentWatchdog};
use embassy_time::{Duration, Ticker};
use heapless::String;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct IrqsUsart {
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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

    // initialize the project, ensure we can debug during sleep
    let p = embassy_stm32::init(config);

    // There used to be some configuration here, but I removed it s.t I wouldn't step on NerCan's toes
    let mut usart_config = usart::Config::default();
    usart_config.swap_rx_tx = true;
    let mut usart = Uart::new(
        p.LPUART1,
        p.PA9,
        p.PA10,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        IrqsUsart,
        usart_config,
    )
    .unwrap();

    let (can_tx_half, can_rx_half) = can::init(can::CanPins {
        can: p.FDCAN2,
        rx: p.PB5,
        tx: p.PB6,
    });
    spawner.spawn(
        ner_can::can_tx(
            can_tx_half,
            can::OUTGOING.dyn_receiver(),
            can::OUTGOING.dyn_sender(),
        )
        .expect("Failed to spawn ner_can::can_tx()."),
    );
    spawner.spawn(
        ner_can::can_rx(can_rx_half, can::INCOMING.dyn_sender())
            .expect("Failed to spawn ner_can::can_rx()."),
    );
    spawner.spawn(can::can_inbox_task().expect("Failed to spawn can::can_inbox_task()."));

    let mut s: String<128> = String::new();
    core::write!(&mut s, "MSB-FW.rs prints in RTT, not UART!\r\n",).unwrap();
    unwrap!(usart.write(s.as_bytes()).await);

    let mut watchdog = IndependentWatchdog::new(p.IWDG, 5000000);
    watchdog.unleash();
    let mut ticker = Ticker::every(Duration::from_millis(500));

    spawner.spawn(
        state::state_machine::state_machine(Leds::new(p.PE4, p.PE5))
            .expect("Failed to spawn state machine"),
    );

    let mut count: u32 = 0;
    let msg = cangen::LightningPulseMessage::new().with_count(count);

    loop {
        debug!("I'm not dead yet");
        count += 1;
        msg.with_count(count);
        can::OUTGOING.send(msg.to_can_frame()).await;
        ticker.next().await;
        watchdog.pet();
    }
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset() // <- you could do something other than reset
}
