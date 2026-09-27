#![no_std]
#![no_main]

use core::num::NonZeroU8;
use core::num::NonZeroU16;
use core::str::FromStr;

use cangen::SecondVcuTestMessage;
use cangen::TemperatureSensor;
use cangen::ToCanFrame;
use cangen::VcuTestMessage;
use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::debug;
use defmt::info;
use defmt::trace;
use defmt::unwrap;
use defmt::warn;
use embassy_executor::Spawner;

use embassy_stm32::Config;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can;
use embassy_stm32::dma;
use embassy_stm32::eth::StationManagement;
use embassy_stm32::gpio::Level;
use embassy_stm32::gpio::Output;
use embassy_stm32::gpio::Speed;
use embassy_stm32::i2c;
use embassy_stm32::peripherals;

use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::Delay;
use embassy_time::Timer;
use embedded_can::ExtendedId;
use nereth::ServerData;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct IrqsCan {
    FDCAN2_IT0 => can::IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<peripherals::FDCAN2>;
});

bind_interrupts!(struct IrqsI2c {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

fn setup_plca(sm: &mut impl StationManagement, reg: u16, val: u16) {
    // enable vendor specific access and address write
    sm.smi_write(0, 0x0D, 0x1F);

    // write address
    sm.smi_write(0, 0x0E, reg);

    // write normal data, keep vendor specific location
    sm.smi_write(0, 0x0D, 0x1F | 1 << 14);

    // write payload
    sm.smi_write(0, 0x0E, val);
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

    let mut can = can::CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, IrqsCan);
    {
        use embassy_stm32::can::config::*;
        use embassy_stm32::can::filter::*;
        use embedded_can::StandardId;
        let can_config = FdCanConfig::default()
            .set_automatic_bus_off_recovery(true)
            .set_automatic_retransmit(false)
            .set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly)
            .set_clock_divider(ClockDivider::_1)
            .set_nominal_bit_timing(NominalBitTiming {
                prescaler: NonZeroU16::new(8).unwrap(),
                seg1: NonZeroU8::new(11).unwrap(),
                seg2: NonZeroU8::new(4).unwrap(),
                sync_jump_width: NonZeroU8::new(1).unwrap(),
            })
            .set_transmit_pause(true)
            .set_global_filter(GlobalFilter::reject_all());
        can.set_config(can_config);

        let std1 = Filter::<embedded_can::StandardId, u16> {
            filter: FilterType::DedicatedDual(
                StandardId::new(0x37).unwrap(),
                StandardId::new(0x01E).unwrap(),
            ),
            action: Action::StoreInFifo0,
        }; // IMD and BMS LIGHTNING

        let ext1 = Filter::<embedded_can::ExtendedId, u32> {
            filter: FilterType::DedicatedSingle(ExtendedId::new(0x0CA).unwrap()),
            action: Action::StoreInFifo0,
        }; // Cerb lightning
        can.properties()
            .set_standard_filter(StandardFilterSlot::_0, std1);
        can.properties()
            .set_extended_filter(ExtendedFilterSlot::_0, ext1);
    }
    let mut can = can.into_normal_mode();

    // ETH
    let mut phy_reset = Output::new(p.PE10, Level::Low, Speed::Low);
    phy_reset.set_low();
    Timer::after_millis(500).await;
    phy_reset.set_high();

    let nereth = nereth::NerEth::<2>::new(
        _spawner, p.RNG, p.ETH, p.PA1, p.PA7, p.PC4, p.PC5, p.PB12, p.PB15, p.PA5, p.ETH_SMA,
        p.PA2, p.PC1,
    )
    .await;

    let mut publish = unwrap!(nereth.get_publisher("HELLO/FROM/NERETH", Some("Z")).await);

    // LEDs
    let mut red_led = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut green_led = Output::new(p.PE4, Level::Low, Speed::Low);
    //let mut dash = Output::new(p.PD0, Level::Low, Speed::Low);
    let mut dti = Output::new(p.PF4, Level::Low, Speed::Low);
    dti.set_high();

    // i2C
    let mut i2c2 = i2c::I2c::new(
        p.I2C2,
        p.PF1,
        p.PF0,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        IrqsI2c,
        Default::default(),
    );
    let mut sht3x = sht3x_ner::Sht3x::new(i2c2, sht3x_ner::Address::Low);

    // Watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 3000000);
    watchdog.unleash();

    // dash.set_high();

    loop {
        debug!("Status: Alive");
        red_led.set_high();
        green_led.set_low();
        Timer::after_millis(500).await;
        watchdog.pet();
        red_led.set_low();
        green_led.set_high();
        Timer::after_millis(500).await;
        match sht3x
            .measure(
                sht3x_ner::ClockStretch::Disabled,
                sht3x_ner::Repeatability::Low,
                &mut Delay,
            )
            .await
        {
            Ok(res) => {
                trace!("Got temperature {}", res.temperature);
                //let topic = TopicName::new(MqttString::from_str("demo/topic").unwrap()).unwrap();

                // let packet_identifier = client
                //     .publish(
                //         &PublicationOptions::new(TopicReference::Name(topic)).exactly_once(),
                //         "Hello World!".into(),
                //     )
                //     .await
                //     .unwrap()
                //     .unwrap();
                //
                let frame: can::Frame = TemperatureSensor::new()
                    .with_vcu_temperature((res.temperature as f32) / 100f32)
                    .with_vcu_humidity((res.humidity as f32) / 100f32)
                    .to_can_frame();
                can.write(&frame).await;

                let frame: can::Frame = VcuTestMessage::new()
                    .with_five_bits(0x4)
                    .with_float_value(69.454)
                    .with_signed_8_bits(-3)
                    .with_sixteen_bits(0b1111111111111110)
                    .with_three_bits(0x2)
                    .to_can_frame();
                can.write(&frame).await;

                let frame: can::Frame = SecondVcuTestMessage::new()
                    .with_one(0x3)
                    .with_two(0x3)
                    .with_three(0x3)
                    .with_four(true)
                    .with_five(0x3)
                    .with_six(0x18)
                    .to_can_frame();
                can.write(&frame).await;

                let _ = publish
                    .send_values(
                        [res.temperature as f32],
                    )
                    .await;
            }
            Err(_) => warn!("Error reading SHT3X"),
        }
    }
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset() // <- you could do something other than reset
}
