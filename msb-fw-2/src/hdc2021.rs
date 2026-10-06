//! HDC2021 temperature/humidity sensor polling, similar to the C code
//! The register-level work lives in the `hdc2021-ner` driver (firmware-rs/drivers); this file
//! just runs it and publishes readings. The I2C bus is set up in main.rs and handed to
//! [`hdc2021_task`].

use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Duration, Ticker};
use hdc2021_ner::{Address, Hdc2021, MANUFACTURER_ID, Measurement};

pub type Hdc2021I2c = I2c<'static, Async>;
 
/// how often to measure
/// temperature and humidity change slowly

const POLL_PERIOD: Duration = Duration::from_millis(1000);
 
/// max number of tasks 
const MAX_RECEIVERS: usize = 2;
 
/// latest temperature/humidity reading for any task that wants it 
pub static HDC2021_DATA: Watch<ThreadModeRawMutex, Measurement, MAX_RECEIVERS> = Watch::new();
 
#[embassy_executor::task]
pub async fn hdc2021_task(i2c: Hdc2021I2c) {
    // ADDR is tied to GND on the board: the C firmware uses HDC2021_I2C_ADDR = 0x40
    let mut sensor = Hdc2021::new(i2c, Address::Low);
    let mut delay = Delay;
 
    // Presence check: confirms wiring, address and byte order before trusting readings.
    match sensor.manufacturer_id().await {
        Ok(id) if id == MANUFACTURER_ID => defmt::info!("HDC2021 found (manufacturer ID {=u16:#x})", id),
        Ok(id) => defmt::warn!("HDC2021: unexpected manufacturer ID {=u16:#x} (expected {=u16:#x})", id, MANUFACTURER_ID),
        Err(e) => defmt::error!("HDC2021: ID read failed: {} (check I2C wiring/address)", e),
    }
 
    let sender = HDC2021_DATA.sender();
    let mut ticker = Ticker::every(POLL_PERIOD);
 
    loop {
        // One-shot measurement: trigger, wait for data-ready, read.
        match sensor.measure(&mut delay).await {
            Ok(m) => {
                defmt::info!("HDC2021: {=f32} C, {=f32} %RH", m.temperature, m.humidity);
                sender.send(m);
            }
            Err(e) => defmt::warn!("HDC2021: measurement failed: {}", e),
        }
        ticker.next().await;
    }
}