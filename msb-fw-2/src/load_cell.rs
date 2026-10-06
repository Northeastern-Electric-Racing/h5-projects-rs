//! Load cell polling, similar to `u_load_cell.c` in the C firmware
 
use crate::analog_sensor::{self, AnalogSensor, SensorData};
use crate::multiplexor_handler::{MuxChannel, MuxId, MuxInput, MuxSource};
use embassy_sync::watch::Watch;
use embassy_time::Duration;
 
pub const NUM_LOAD_CELLS: usize = 2;
 
/// Subtracted from the measured voltage (output with no load), indexed by [`LoadCell`].
/// C: LOAD_CELL1/2_ZERO_OFFSET. u_TODO - measure each cell's unloaded voltage.
const ZERO_OFFSET: [f32; NUM_LOAD_CELLS] = [0.0, 0.0];
/// Multiplied into the offset-corrected voltage to get force, indexed by [`LoadCell`].
/// C: LOAD_CELL1/2_SCALE_FACTOR. u_TODO - derive from a known load.
const SCALE_FACTOR: [f32; NUM_LOAD_CELLS] = [1.0, 1.0];
 
/// CAN ID for the load cell message (both cells in one frame).
/// `convert_can_id`, so check whether it ends up standard or extended).
pub const CAN_ID: u32 = 0x630;
 
/// Latest load cell readings, indexed by [`LoadCell`].
pub static LOAD_CELL_DATA: SensorData<LoadCellReading, NUM_LOAD_CELLS> = Watch::new();
 
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LoadCell {
    /// `LOAD_CELL1` in the C code
    Lc1 = 0,
    /// `LOAD_CELL2` in the C code
    Lc2 = 1,
}
 
impl LoadCell {
    pub const ALL: [LoadCell; NUM_LOAD_CELLS] = [LoadCell::Lc1, LoadCell::Lc2];
 
    const fn name(self) -> &'static str {
        match self {
            LoadCell::Lc1 => "1",
            LoadCell::Lc2 => "2",
        }
    }
 
    /// Where this cell is wired on the muxes.
    const fn source(self) -> MuxSource {
        match self {
            LoadCell::Lc1 => (MuxId::U19, MuxChannel::Ch1, MuxInput::B), // LPF5, PA3, ADC2 ch15
            LoadCell::Lc2 => (MuxId::U19, MuxChannel::Ch2, MuxInput::A), // LPF6, PF13, ADC2 ch2
        }
    }
}
 
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct LoadCellReading {
    pub raw: u16,
    pub volts: f32,
    /// Calibrated force. Equal to `volts` until real offsets and scales are measured.
    pub force: f32,
}
 
/// 8-byte CAN payload with both cells as little-endian i32s, matching the C
/// `send_load_cell_data`: bytes 0..4 = load cell 1, bytes 4..8 = load cell 2.

pub fn can_payload(readings: &[LoadCellReading; NUM_LOAD_CELLS]) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[..4].copy_from_slice(&(readings[0].force as i32).to_le_bytes());
    out[4..].copy_from_slice(&(readings[1].force as i32).to_le_bytes());
    out
}
 
/// Both load cells, as one [`AnalogSensor`].
pub struct LoadCells;
 
impl AnalogSensor<NUM_LOAD_CELLS> for LoadCells {
    /// u_TODO - check what rate the C firmware used and match it.
    const POLL_PERIOD: Duration = Duration::from_millis(20);
    const SOURCES: [MuxSource; NUM_LOAD_CELLS] = [LoadCell::Lc1.source(), LoadCell::Lc2.source()];
    const NAMES: [&'static str; NUM_LOAD_CELLS] = [LoadCell::Lc1.name(), LoadCell::Lc2.name()];
 
    type Reading = LoadCellReading;
 
    fn convert(i: usize, raw: u16, volts: f32) -> LoadCellReading {
        let force = (volts - ZERO_OFFSET[i]) * SCALE_FACTOR[i];
 
        LoadCellReading { raw, volts, force }
    }
 
    fn log(name: &'static str, r: &LoadCellReading) {
        defmt::info!("Load cell {=str}: raw={=u16} volts={=f32} force={=f32}", name, r.raw, r.volts, r.force);
    }
 
    fn data() -> &'static SensorData<LoadCellReading, NUM_LOAD_CELLS> {
        &LOAD_CELL_DATA
    }
}
 
#[embassy_executor::task]
pub async fn load_cell_task() {
    analog_sensor::run::<LoadCells, NUM_LOAD_CELLS>().await
}