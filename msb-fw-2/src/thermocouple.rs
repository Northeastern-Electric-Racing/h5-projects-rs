//! Thermocouple polling, similar to `u_thermocouple.c` in the C code
 
use crate::analog_sensor::{self, AnalogSensor, SensorData};
use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, MuxSource};
use embassy_sync::watch::Watch;
use embassy_time::Duration;
 
/// Amplifier output at `REF_TEMP_C`, in volts
const REF_V: f32 = 1.25;

/// Temperature at which the amplifier outputs `REF_V`, in °C

const REF_TEMP_C: f32 = 25.0;
/// Amplifier output slope, in volts per °C (5 mV/°C)
const VOLTS_PER_C: f32 = 0.005;
 
/// CAN ID for the thermocouple message.
/// u_TODO - copy THERMOCOUPLE_CAN_ID from the C u_can.h.
pub const CAN_ID: u32 = 0x633;
 
pub const NUM_THERMOCOUPLES: usize = 1;
 
/// Latest thermocouple reading.
pub static THERMOCOUPLE_DATA: SensorData<ThermocoupleReading, NUM_THERMOCOUPLES> = Watch::new();
 
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct ThermocoupleReading {
    pub raw: u16,
    pub volts: f32,
    /// temp in °C
    pub temp_c: f32,
}
 
impl ThermocoupleReading {
    /// 4-byte CAN payload: temperature as a little-endian i32, matching the C
    pub fn can_payload(&self) -> [u8; 4] {
        (self.temp_c as i32).to_le_bytes()
    }
}
 
/// The thermocouple, as an [`AnalogSensor`].
pub struct Thermocouple;
 
impl AnalogSensor<NUM_THERMOCOUPLES> for Thermocouple {
    /// Temperature changes slowly
    const POLL_PERIOD: Duration = Duration::from_millis(1000);
    /// U19 S1A (PA3, ADC2 ch15). Shares U19 Ch1 with load cell 1 (S1B).
    const SOURCES: [MuxSource; NUM_THERMOCOUPLES] = [(MuxId::U19, MuxChannel::Ch1, MuxInput::A)];
    const NAMES: [&'static str; NUM_THERMOCOUPLES] = ["thermocouple"];
 
    type Reading = ThermocoupleReading;
 
    /// Same as `_voltage_to_temp` in C: `REF_V` -> `REF_TEMP_C`, +5 mV per °C
    fn convert(_i: usize, raw: u16, volts: f32) -> ThermocoupleReading {
        let temp_c = (volts - REF_V) / VOLTS_PER_C + REF_TEMP_C;
 
        ThermocoupleReading { raw, volts, temp_c }
    }
 
    fn log(_name: &'static str, r: &ThermocoupleReading) {
        defmt::info!("Thermocouple: raw={=u16} volts={=f32} temp={=f32}C", r.raw, r.volts, r.temp_c);
    }
 
    fn data() -> &'static SensorData<ThermocoupleReading, NUM_THERMOCOUPLES> {
        &THERMOCOUPLE_DATA
    }
}
 
#[embassy_executor::task]
pub async fn thermocouple_task() {
    analog_sensor::run::<Thermocouple, NUM_THERMOCOUPLES>().await
}