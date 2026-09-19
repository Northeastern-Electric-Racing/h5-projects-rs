#![no_std]
#![no_main]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::time::Hertz;
use embassy_stm32::adc::{self, Adc, AdcChannel, BasicAdcRegs, RxDma, SampleTime};
use embassy_stm32::peripherals::{ADC1, GPDMA1_CH0, PA0, PA1, PA6, PA7, PB0, PB1};
use embassy_stm32::{Config, Peri, bind_interrupts, dma, interrupt};
use embassy_time::{Duration, Instant, Ticker};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Watch, Sender, Receiver};

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL0 => dma::InterruptHandler<GPDMA1_CH0>;
});

#[derive(Clone, Copy, Debug, Default)]
pub struct Voltages{
    pub apps1: f32,
    pub apps2: f32,
    pub bspd: f32,
    pub bspd_thresh: f32,
    pub bspd_max: f32,
    pub bspd_min:f32,
    pub seq: u32
}

const RECEIVERS: usize = 2;

static VOLTAGES: Watch<CriticalSectionRawMutex, Voltages, RECEIVERS> = Watch::new();

type VoltageSender = Sender<'static, CriticalSectionRawMutex, Voltages, RECEIVERS>;
type VoltageReceiver = Receiver<'static, CriticalSectionRawMutex, Voltages, RECEIVERS>;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    info!("Initializing project...");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;

        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: HseMode::Oscillator
        });

        config.rcc.pll1 = Some(Pll {
            source: PllSource::Hse,
            prediv: PllPreDiv::Div5,
            mul: PllMul::Mul80,
            divp: Some(PllDiv::Div2), // 200 Mhz
            divq: Some(PllDiv::Div2), // 200 Mhz
            divr: None,
        });

        config.rcc.pll2 = Some(Pll {
            source: PllSource::Hse,
            prediv: PllPreDiv::Div5,
            mul: PllMul::Mul60,
            divp: Some(PllDiv::Div10), // 30 Mhz
            divq: None,
            divr: Some(PllDiv::Div3), // 100 MHz
        });

        config.rcc.sys = Sysclk::Pll1P;
        config.rcc.ahb_pre = AHBPrescaler::Div1; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::Div1; // 200 Mhz
        config.rcc.apb2_pre = APBPrescaler::Div1; // 200 Mhz
        config.rcc.apb3_pre = APBPrescaler::Div1; // 200 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.adcdacsel = mux::Adcdacsel::Pll2R;
    }

    let p = embassy_stm32::init(config);

    let sender = VOLTAGES.sender();

    spawner.spawn(unwrap!(adc1_task(p.ADC1, p.GPDMA1_CH0, sender, p.PA0, p.PA1, p.PA6, p.PA7, p.PB0, p.PB1)));

    let mut watchdog = IndependentWatchdog::new(p.IWDG, 1000000);
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

#[embassy_executor::task]
async fn adc1_task(
    adc: Peri<'static, ADC1>,
    dma: Peri<'static, GPDMA1_CH0>,
    sender: VoltageSender,
    pin1: Peri<'static, PA0>,
    pin2: Peri<'static, PA1>,
    pin3: Peri<'static, PA6>,
    pin4: Peri<'static, PA7>,
    pin5: Peri<'static, PB0>,
    pin6: Peri<'static, PB1>,
){
    adc_task(adc, dma, Irqs, sender, pin1, pin2, pin3, pin4, pin5, pin6).await;
}

async fn adc_task<'a, T, D, I>(
    adc: Peri<'a, T>,
    mut dma: Peri<'a, D>,
    irq: I,
    sender: VoltageSender,
    mut pin1: impl AdcChannel<'_, T>,
    mut pin2: impl AdcChannel<'_, T>,
    mut pin3: impl AdcChannel<'_, T>,
    mut pin4: impl AdcChannel<'_, T>,
    mut pin5: impl AdcChannel<'_, T>,
    mut pin6: impl AdcChannel<'_, T>,
) where 
    T: adc::Instance,
    T::Regs: BasicAdcRegs<SampleTime = SampleTime>,
    D: RxDma<T>,
    I: interrupt::typelevel::Binding<D::Interrupt, dma::InterruptHandler<D>> + Copy,
{
    let mut adc = Adc::new_blocking(adc, adc::Config::default());

    info!("adc init");

    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut tic = Instant::now();
    let mut buffer = [0u16; 512];
    let mut seq = 0u32;
    loop{
        // This is not a true continuous read as there is downtime between each
        // call to Adc::read where the ADC is sitting idle
        adc.read_sequence(
            dma.reborrow(), 
            irq,
            [
                (pin1.reborrow_adc(), SampleTime::Cycles6405),
                (pin2.reborrow_adc(), SampleTime::Cycles6405),
                (pin3.reborrow_adc(), SampleTime::Cycles6405),
                (pin4.reborrow_adc(), SampleTime::Cycles6405),
                (pin5.reborrow_adc(), SampleTime::Cycles6405),
                (pin6.reborrow_adc(), SampleTime::Cycles6405),
                // 640.5 cycles * 6 channels * 50 MHz = 768 us per sample, or 1.3 kHz
            ].into_iter(),
            None,
            &mut buffer[0..6],
        )
        .await;
        let toc = Instant::now();
        info!("\n adc: {} dt = {}",buffer[0..6], (toc - tic).as_micros());
        tic = toc;

        let sample = Voltages {
            apps1: (buffer[0] as f32) * 5.0 / 4096.0,
            apps2: (buffer[1] as f32) * 5.0 / 4096.0,
            bspd: (buffer[2] as f32) * 5.0 / 4096.0,
            bspd_thresh: (buffer[3] as f32) * 5.0 / 4096.0,
            bspd_max: (buffer[4] as f32) * 5.0 / 4096.0,
            bspd_min: (buffer[5] as f32) * 5.0 / 4096.0,
            seq,
        };

        sender.send(sample);

        seq = seq.wrapping_add(1);
        ticker.next().await;
    }
}