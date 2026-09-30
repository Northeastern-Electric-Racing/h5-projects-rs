//! ADC sampling and analog multiplexer handling.
//!
//! For mux'd inputs, SELx=HIGH selects the A input and SELx=LOW the B input.

use embassy_stm32::adc::{Adc, AdcChannel, Clock, Config, Prescaler, Resolution, SampleTime};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::Timer;
use variant_count::VariantCount;

use crate::efuses::EfuseId;

bind_interrupts!(struct Irqs {
    ADC1 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>;
    GPDMA1_CHANNEL0 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH0>;
});

/// How long the external mux needs to settle after the SEL lines change.
const MUX_SETTLE_MS: u64 = 10;

/// Sample time for every ADC1 channel except the LFIU mux input.
const SAMPLE_TIME: SampleTime = SampleTime::Cycles475;
/// Sample time for the LFIU mux input, which needs longer to change.
const SAMPLE_TIME_MUX2: SampleTime = SampleTime::Cycles2475;

#[derive(PartialEq, Eq)]
enum AdcMuxSel {
    SelLow,
    SelHigh,
}

#[derive(VariantCount)]
pub enum Adc1Channels {
    Adc1Channel3,  // EF_DASH_ADC
    Adc1Channel0,  // Mux 1
    Adc1Channel5,  // Mux 3
    Adc1Channel9,  // Mux 4
    Adc1Channel6,  // EF_FANBATT_ADC
    Adc1Channel2,  // EF_PUMP1_ADC
    Adc1Channel13, // EF_PUMP2_ADC
    Adc1Channel18, // EF_MC_ADC
    Adc1Channel15, // Mux 2
}

#[derive(VariantCount)]
pub enum Adc2Channels {
    Adc2Channel12, // APPS_1_ADC
    Adc2Channel10, // APPS_2_ADC
    Adc2Channel2,  // BSE_1_ADC
    Adc2Channel6,  // BSE_2_ADC
}

#[derive(VariantCount)]
enum MuxDataIndex {
    Sel1High, // BREAKLIGHT_ADC
    Sel1Low,  // BATTBOX_ADC

    Sel2High, // LFIU_CURRENT_1
    Sel2Low,  // LFIU_CURRENT_2

    Sel3High, // LV_ADC
    Sel3Low,  // SHUTDOWN_ADC

    Sel4High, // RADFAN_ADC
    Sel4Low,  // LV_BATT_ADC
}

/// Number of SEL lines on the analog mux.
const MUX_SEL_COUNT: usize = MuxDataIndex::VARIANT_COUNT / 2;

/// A copy of the latest readings, taken out from under the [`AdcMux`] lock.
///
/// Holding one of these does not block [`adc1_task`], so consumers should take a
/// snapshot once and then read as many channels off it as they need.
#[derive(Clone, Copy)]
pub struct AdcMuxData {
    adc1: [u16; Adc1Channels::VARIANT_COUNT],
    mux: [u16; MuxDataIndex::VARIANT_COUNT],
}

impl AdcMuxData {
    pub fn get_efuse_data(&self, efuse_id: EfuseId) -> Option<u16> {
        match efuse_id {
            EfuseId::EfuseDashboard => Some(self.adc1[Adc1Channels::Adc1Channel3 as usize]),
            EfuseId::EfuseBrake => Some(self.mux[MuxDataIndex::Sel1High as usize]),
            EfuseId::EfuseShutdown => Some(self.mux[MuxDataIndex::Sel3Low as usize]),
            EfuseId::EfuseLV => Some(self.mux[MuxDataIndex::Sel3High as usize]),
            EfuseId::EfuseRadfan => Some(self.mux[MuxDataIndex::Sel4High as usize]),
            EfuseId::EfuseFanbatt => Some(self.adc1[Adc1Channels::Adc1Channel6 as usize]),
            EfuseId::EfusePump1 => Some(self.adc1[Adc1Channels::Adc1Channel2 as usize]),
            EfuseId::EfusePump2 => Some(self.adc1[Adc1Channels::Adc1Channel13 as usize]),
            EfuseId::EfuseBattbox => Some(self.mux[MuxDataIndex::Sel1Low as usize]),
            EfuseId::EfuseMC => Some(self.adc1[Adc1Channels::Adc1Channel18 as usize]),
            EfuseId::EfuseSpare => None,
        }
    }
}

pub struct AdcMux {
    mux_sel: AdcMuxSel,
    mux_buffer: [u16; MuxDataIndex::VARIANT_COUNT],
    sel_pins: [Output<'static>; MUX_SEL_COUNT],
    adc_buffer: [u16; Adc1Channels::VARIANT_COUNT],
}

impl AdcMux {
    /// The SEL lines start low, matching the reset state CubeMX writes in
    /// `MX_GPIO_Init` (`Core/Src/main.c:1024`).
    fn new(sel_pins: [Output<'static>; MUX_SEL_COUNT]) -> Self {
        Self {
            mux_sel: AdcMuxSel::SelLow,
            mux_buffer: [0; MuxDataIndex::VARIANT_COUNT],
            sel_pins,
            adc_buffer: [0; Adc1Channels::VARIANT_COUNT],
        }
    }

    /// Stores a fresh sweep of the ADC1 sequence.
    fn update(&mut self, adc_buffer: &[u16; Adc1Channels::VARIANT_COUNT]) {
        self.adc_buffer = *adc_buffer;
    }

    /// Latches the side the mux is currently parked on, then flips it.
    ///
    /// The latch has to happen before the SEL lines move: the readings in
    /// `adc_buffer` belong to the side that was selected while they were taken.
    /// The caller must wait [`MUX_SETTLE_MS`] before the next [`update`] so the
    /// new side has settled.
    fn flip(&mut self) {
        match self.mux_sel {
            AdcMuxSel::SelLow => {
                self.mux_buffer[MuxDataIndex::Sel1Low as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel0 as usize];
                self.mux_buffer[MuxDataIndex::Sel2Low as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel15 as usize];
                self.mux_buffer[MuxDataIndex::Sel3Low as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel5 as usize];
                self.mux_buffer[MuxDataIndex::Sel4Low as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel9 as usize];

                self.sel_pins
                    .iter_mut()
                    .for_each(|output| output.set_high());
                self.mux_sel = AdcMuxSel::SelHigh;
            }
            AdcMuxSel::SelHigh => {
                self.mux_buffer[MuxDataIndex::Sel1High as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel0 as usize];
                self.mux_buffer[MuxDataIndex::Sel2High as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel15 as usize];
                self.mux_buffer[MuxDataIndex::Sel3High as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel5 as usize];
                self.mux_buffer[MuxDataIndex::Sel4High as usize] =
                    self.adc_buffer[Adc1Channels::Adc1Channel9 as usize];

                self.sel_pins.iter_mut().for_each(|output| output.set_low());
                self.mux_sel = AdcMuxSel::SelLow;
            }
        }
    }

    /// Copies the current readings out.
    fn data(&self) -> AdcMuxData {
        AdcMuxData {
            adc1: self.adc_buffer,
            mux: self.mux_buffer,
        }
    }
}

/// The one [`AdcMux`], populated by [`adc1_task`] when it starts.
static ADC_MUX: OnceLock<Mutex<ThreadModeRawMutex, AdcMux>> = OnceLock::new();

/// Takes a snapshot of the latest readings.
///
/// Waits for [`adc1_task`] to have started. The lock is held only long enough to
/// copy the buffers out.
pub async fn data() -> AdcMuxData {
    ADC_MUX.get().await.lock().await.data()
}

/// Peripherals ADC1 and the analog mux are wired to.
///
/// Channel pins follow the CubeMX rank order; see `Core/Inc/main.h:90-117` and
/// the raw GPIOF writes in `Core/Src/stm32h5xx_hal_msp.c:155-158`.
pub struct Adc1Resources {
    pub adc: Peri<'static, peripherals::ADC1>,
    pub dma: Peri<'static, peripherals::GPDMA1_CH0>,

    pub dash: Peri<'static, peripherals::PA6>, // INP3,  EF_DASH_ADC
    /// High: BREALIGHT, Low: BATTBOX
    pub mux1: Peri<'static, peripherals::PA0>, // INP0,  Mux 1
    /// High: LV, Low: SHUTDOWN
    pub mux3: Peri<'static, peripherals::PB1>, // INP5,  Mux 3
    /// High: RADFAN, Low: LV_BAT
    pub mux4: Peri<'static, peripherals::PB0>, // INP9,  Mux 4
    pub fanbatt: Peri<'static, peripherals::PF12>, // INP6,  EF_FANBATT_ADC
    pub pump1: Peri<'static, peripherals::PF11>, // INP2,  EF_PUMP1_ADC
    pub pump2: Peri<'static, peripherals::PC3>, // INP13, EF_PUMP2_ADC
    pub mc: Peri<'static, peripherals::PA4>,   // INP18, EF_MC_ADC
    /// High: LFIU_CURRENT_1, Low: LFIU_CURRENT_2
    pub mux2: Peri<'static, peripherals::PA3>, // INP15, Mux 2

    pub sel1: Peri<'static, peripherals::PC6>,
    pub sel2: Peri<'static, peripherals::PC7>,
    pub sel3: Peri<'static, peripherals::PC8>,
    pub sel4: Peri<'static, peripherals::PC9>,
}

/// Sweeps the ADC1 sequence and drives the analog mux.
#[embassy_executor::task]
pub async fn adc1_task(r: Adc1Resources) {
    let sel_pins = [
        Output::new(r.sel1, Level::Low, Speed::Low),
        Output::new(r.sel2, Level::Low, Speed::Low),
        Output::new(r.sel3, Level::Low, Speed::Low),
        Output::new(r.sel4, Level::Low, Speed::Low),
    ];
    let _ = ADC_MUX.init(Mutex::new(AdcMux::new(sel_pins)));

    let mut config = Config::default();
    config.resolution = Some(Resolution::Bits12);
    config.clock = Clock::Async(Prescaler::Div64);

    let mut adc = Adc::new(r.adc, Irqs, config);

    // Order must match `Adc1Channels`.
    let sequence = [
        (r.dash.degrade_adc(), SAMPLE_TIME),
        (r.mux1.degrade_adc(), SAMPLE_TIME),
        (r.mux3.degrade_adc(), SAMPLE_TIME),
        (r.mux4.degrade_adc(), SAMPLE_TIME),
        (r.fanbatt.degrade_adc(), SAMPLE_TIME),
        (r.pump1.degrade_adc(), SAMPLE_TIME),
        (r.pump2.degrade_adc(), SAMPLE_TIME),
        (r.mc.degrade_adc(), SAMPLE_TIME),
        (r.mux2.degrade_adc(), SAMPLE_TIME_MUX2),
    ];
    let mut sequence = adc.configure_sequence(r.dma, sequence.into_iter(), Irqs);

    let mut buffer = [0u16; Adc1Channels::VARIANT_COUNT];

    loop {
        sequence.read(&mut buffer).await;

        {
            let mut mux = ADC_MUX.get().await.lock().await;
            mux.update(&buffer);
            mux.flip();
        }

        // Settle with the lock released, so readers are not blocked for 10ms.
        Timer::after_millis(MUX_SETTLE_MS).await;
    }
}

pub struct PedalBuffer {
    adc_buffer: [u16; Adc2Channels::VARIANT_COUNT],
}

pub struct RawPedalData {
    accel1: u16,
    accel2: u16,
    brake1: u16,
    brake2: u16,
}

impl PedalBuffer {
    pub fn new(adc_buffer: [u16; Adc2Channels::VARIANT_COUNT]) -> Self {
        PedalBuffer { adc_buffer }
    }

    pub fn get_pedal_data(self) -> RawPedalData {
        RawPedalData {
            accel1: self.adc_buffer[Adc2Channels::Adc2Channel12 as usize],
            accel2: self.adc_buffer[Adc2Channels::Adc2Channel10 as usize],
            brake1: self.adc_buffer[Adc2Channels::Adc2Channel2 as usize],
            brake2: self.adc_buffer[Adc2Channels::Adc2Channel6 as usize],
        }
    }
}
