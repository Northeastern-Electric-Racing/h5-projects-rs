use defmt::Format;
use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, BorrowedAdcChannel, Config as AdcConfig, SampleTime};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{ADC1, PC0, PC2, PC6, PC7};
use embassy_time::Timer;

/// ADC reference voltage (VDDA), in volts.
const VREF_V: f32 = 3.3;

/// Full stroke of the shock pots (50 mm), in inches.
const STROKE_IN: f32 = 50.0 / 25.4;

/// Sample time for each conversion. The inputs sit behind an RC filter, so use a long one.
const SAMPLE_TIME: SampleTime = SampleTime::Cycles2475;

/// How often `shock_pot_task` samples the pots, in ms.
const SAMPLE_PERIOD_MS: u64 = 100;

/// Per-pot calibration.
pub struct Calibration {
    /// Subtracted from the measured voltage before scaling, in volts.
    pub zero_offset_v: f32,
    /// Multiplied into the offset-corrected voltage.
    pub scale: f32,
    /// Calibrated voltage at full extension (zero travel), in volts.
    pub full_extension_v: f32,
    /// Subtracted from the computed travel so the pot reads zero at ride height, in inches.
    pub travel_trim_in: f32,
}

/// Shock 1 (left) calibration.
/// u_TODO - measure `full_extension_v` on the car and replace the placeholder.
pub const SHOCK1_CALIBRATION: Calibration = Calibration {
    zero_offset_v: 0.0,
    scale: 1.0,
    full_extension_v: 3.3,
    travel_trim_in: 0.195,
};

/// Shock 2 (right) calibration.
/// u_TODO - measure `full_extension_v` on the car and replace the placeholder.
pub const SHOCK2_CALIBRATION: Calibration = Calibration {
    zero_offset_v: 0.0,
    scale: 1.0,
    full_extension_v: 3.3,
    travel_trim_in: 0.140,
};

/// One reading from a shock pot.
#[derive(Clone, Copy, Format)]
pub struct Reading {
    /// Raw ADC count.
    pub raw: u16,
    /// Calibrated voltage, in volts.
    pub volts: f32,
    /// Travel from full extension, in inches.
    pub travel_in: f32,
}

impl Reading {
    fn new(raw: u16, max_count: u32, cal: &Calibration) -> Self {
        let measured_v = f32::from(raw) * VREF_V / max_count as f32;
        let volts = (measured_v - cal.zero_offset_v) * cal.scale;
        // Voltage falls as the shock compresses, so travel is measured down from full extension.
        let travel_in = (cal.full_extension_v - volts) * (STROKE_IN / VREF_V) - cal.travel_trim_in;

        Self { raw, volts, travel_in }
    }
}

/// Readings from both shock pots.
#[derive(Clone, Copy, Format)]
pub struct ShockPotData {
    pub shock1: Reading,
    pub shock2: Reading,
}

/// Both shock pots, their mux select lines, and the ADC they are read on.
pub struct ShockPots<'d> {
    adc: Adc<'d, ADC1, Blocking>,
    shock1: BorrowedAdcChannel<'d, ADC1>,
    shock2: BorrowedAdcChannel<'d, ADC1>,
    // Held low for as long as this struct lives, so the mux stays on the shock pots.
    _shock1_sel: Output<'d>,
    _shock2_sel: Output<'d>,
}

impl<'d> ShockPots<'d> {
    pub fn new(adc: Peri<'d, ADC1>, shock1_adc: Peri<'d, PC0>, shock2_adc: Peri<'d, PC2>, shock1_sel: Peri<'d, PC6>, shock2_sel: Peri<'d, PC7>) -> Self {
        Self {
            adc: Adc::new_blocking(adc, AdcConfig::default()),
            shock1: shock1_adc.degrade_adc(),
            shock2: shock2_adc.degrade_adc(),
            _shock1_sel: Output::new(shock1_sel, Level::Low, Speed::Low),
            _shock2_sel: Output::new(shock2_sel, Level::Low, Speed::Low),
        }
    }

    /// Reads both shock pots.
    pub fn read(&mut self) -> ShockPotData {
        let max_count = self.adc.resolution().max_count();
        let shock1_raw = self.adc.blocking_read(&mut self.shock1, SAMPLE_TIME);
        let shock2_raw = self.adc.blocking_read(&mut self.shock2, SAMPLE_TIME);

        ShockPotData {
            shock1: Reading::new(shock1_raw, max_count, &SHOCK1_CALIBRATION),
            shock2: Reading::new(shock2_raw, max_count, &SHOCK2_CALIBRATION),
        }
    }
}

/// Periodically reads the shock pots and logs them.
#[embassy_executor::task]
pub async fn shock_pot_task(mut shock_pots: ShockPots<'static>) -> ! {
    loop {
        let data = shock_pots.read();
        defmt::info!("Shock pots: {}", data);
        Timer::after_millis(SAMPLE_PERIOD_MS).await;
    }
}
