//! Strain gauge polling. Ported from `u_strain_gauge.c` in the C firmware.

use crate::analog_sensor::{self, AnalogSensor, SensorData};
use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, MuxSource};
use embassy_sync::watch::Watch;
use embassy_time::Duration;

pub const NUM_STRAIN_GAUGES: usize = 4;

/// Calibration from the C firmware (placeholders: no offset, unity scale). MEASURE AND REPLACE.
const ZERO_OFFSET: [f32; NUM_STRAIN_GAUGES] = [0.0, 0.0, 0.0, 0.0];
const SCALE_FACTOR: [f32; NUM_STRAIN_GAUGES] = [1.0, 1.0, 1.0, 1.0];

/// Latest strain gauge readings, indexed by [`StrainGauge`].
pub static STRAIN_GAUGE_DATA: SensorData<StrainGaugeReading, NUM_STRAIN_GAUGES> = Watch::new();

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StrainGauge {
    /// `STRAIN_GAUGE1` in the C firmware.
    Sg1 = 0,
    /// `STRAIN_GAUGE2` in the C firmware.
    Sg2 = 1,
    /// `STRAIN_GAUGE3` in the C firmware.
    Sg3 = 2,
    /// `STRAIN_GAUGE4` in the C firmware.
    Sg4 = 3,
}

impl StrainGauge {
    pub const ALL: [StrainGauge; NUM_STRAIN_GAUGES] = [StrainGauge::Sg1, StrainGauge::Sg2, StrainGauge::Sg3, StrainGauge::Sg4];

    const fn name(self) -> &'static str {
        match self {
            StrainGauge::Sg1 => "1",
            StrainGauge::Sg2 => "2",
            StrainGauge::Sg3 => "3",
            StrainGauge::Sg4 => "4",
        }
    }

    /// Where this gauge is wired on the muxes (STRAIN_OUT1..4 on the U18 `A` inputs).
    const fn source(self) -> MuxSource {
        match self {
            StrainGauge::Sg1 => (MuxId::U18, MuxChannel::Ch1, MuxInput::A), // PC0, ADC1 ch10
            StrainGauge::Sg2 => (MuxId::U18, MuxChannel::Ch2, MuxInput::A), // PC2, ADC1 ch12
            StrainGauge::Sg3 => (MuxId::U18, MuxChannel::Ch3, MuxInput::A), // PC3, ADC1 ch13
            StrainGauge::Sg4 => (MuxId::U18, MuxChannel::Ch4, MuxInput::A), // PA0, ADC1 ch0
        }
    }
}

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct StrainGaugeReading {
    pub raw: u16,
    pub volts: f32,
    /// Calibrated value. Equal to `volts` until real offsets and scales are measured.
    pub strain: f32,
}

/// All four strain gauges, as one [`AnalogSensor`].
pub struct StrainGauges;

impl AnalogSensor<NUM_STRAIN_GAUGES> for StrainGauges {
    const POLL_PERIOD: Duration = Duration::from_millis(20);
    const SOURCES: [MuxSource; NUM_STRAIN_GAUGES] = [
        StrainGauge::Sg1.source(),
        StrainGauge::Sg2.source(),
        StrainGauge::Sg3.source(),
        StrainGauge::Sg4.source(),
    ];
    const NAMES: [&'static str; NUM_STRAIN_GAUGES] =
        [StrainGauge::Sg1.name(), StrainGauge::Sg2.name(), StrainGauge::Sg3.name(), StrainGauge::Sg4.name()];

    type Reading = StrainGaugeReading;

    fn convert(i: usize, raw: u16, volts: f32) -> StrainGaugeReading {
        let strain = (volts - ZERO_OFFSET[i]) * SCALE_FACTOR[i];

        StrainGaugeReading { raw, volts, strain }
    }

    fn log(name: &'static str, r: &StrainGaugeReading) {
        defmt::info!("Strain gauge {=str}: raw={=u16} volts={=f32} strain={=f32}", name, r.raw, r.volts, r.strain);
    }

    fn data() -> &'static SensorData<StrainGaugeReading, NUM_STRAIN_GAUGES> {
        &STRAIN_GAUGE_DATA
    }
}

#[embassy_executor::task]
pub async fn strain_gauge_task() {
    analog_sensor::run::<StrainGauges, NUM_STRAIN_GAUGES>().await
}
