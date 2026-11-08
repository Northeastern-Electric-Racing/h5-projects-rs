use crate::analog_sensor::{self, AnalogSensor, SensorData};
use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, MuxSource};
use embassy_sync::watch::Watch;
use embassy_time::Duration;

pub const NUM_MISC_ADCS: usize = 3;

const ZERO_OFFSET: [f32; NUM_MISC_ADCS] = [0.0, 0.0, 0.0];
const SCALE_FACTOR: [f32; NUM_MISC_ADCS] = [1.0, 1.0, 1.0];

pub static MISC_ADC_DATA: SensorData<MiscAdcReading, NUM_MISC_ADCS> = Watch::new();

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MiscAdc {
    Ext1 = 0,
    Ext2 = 1,
    Ext3 = 2,
}

impl MiscAdc {
    const fn name(self) -> &'static str {
        match self {
            MiscAdc::Ext1 => "1", 
            MiscAdc::Ext2 => "2", 
            MiscAdc::Ext2 => "2", 
        }
    }

const fn source(self) -> MuxSource {
    match self {
        MiscAdc::Ext1 => (MuxId::U19, MuxChannel::Ch3, MuxInput::B),
        MiscAdc::Ext2 => (MuxId::U19, MuxChannel::Ch4, MuxInput::A),
        MiscAdc::Ext3 => (MuxId::U19, MuxChannel::Ch4, MuxInput::B),
    }
}
}

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct MiscAdcReading {
    pub raw: u16,
    pub volts: f32,
    pub value: f32
}

pub struct MiscAdcs;

impl AnalogSensor<NUM_MISC_ADCS> for MiscAdcs {
    const POLL_PERIOD: Duration = Duration::from_millis(20);
    const SOURCES: [MuxSource; NUM_MISC_ADCS] = [
        MiscAdc::Ext1.source(),
        MiscAdc::Ext2.source(),
        MiscAdc::Ext3.source(),
    ];
    const NAMES: [&'static str; NUM_MISC_ADCS] = [
        MiscAdc::Ext1.name(),
        MiscAdc::Ext2.name(),
        MiscAdc::Ext3.name(),
    ];

    type Reading = MiscAdcReading;

    fn convert(i: usize, raw: u16, volts: f32) -> MiscAdcreading {
        let value = (volts - ZERO_OFFSET[i]) * SCALE_FACTOR[i];
        MiscAdcReading { raw, volts, value }
    }
    fn log(name: &'static str, r: &MiscAdcReading) {
        defmt::info!("Misc ADC {=str}: raw={=u16} volts={=f32} value={=f32}", name, r.raw, r.volts, r.value);
    }
    fn data() -> &'static SensorData<MiscAdcReading, NUM_MISC_ADCS> { &MISC_ADC_DATA}

}

#[embassy_executor::task] 
pub async fn misc_adc_task() { analog_sensor::run::<MiscAdcs, NUM_MISC_ADCS>().await }
