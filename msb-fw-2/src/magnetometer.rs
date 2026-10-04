//! LIS2MDL magnetometer polling, similar to the C code
//! The driver is still in development
//! The sensor setup lives in the `lis2mdl-ner` driver (firmware-rs/drivers); this file just
//! runs it and publishes readings

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use lis2mdl_ner::{Lis2mdl, MagneticField};

/// PI2 plus the magnetometer's chip select pin. `ExclusiveDevice` pulls CS low around every
/// transaction, which the C firmware never did
/// if this doesn't compile, check the `Spi` type parameters for your embassy-stm32 version
pub type MagSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
 
/// How often to check for new data. Matches the sensor's 50 Hz output data rate.
const POLL_PERIOD: Duration = Duration::from_millis(20);
 
/// Max number of tasks 
const MAX_RECEIVERS: usize = 2;
 
/// Latest magnetometer reading, for any task that wants it (CAN, sensor fusion, ...).
pub static MAG_DATA: Watch<ThreadModeRawMutex, MagneticField, MAX_RECEIVERS> = Watch::new();
 
#[embassy_executor::task]
pub async fn magnetometer_task(spi: MagSpi) {
    let mut mag = Lis2mdl::new(spi, Delay);
    if let Err(e) = mag.init().await {
        defmt::error!("LIS2MDL init failed: {} (check SPI wiring/mode)", e);
        return;
    }
    defmt::info!("LIS2MDL initialised");
 
    let sender = MAG_DATA.sender();
    let mut ticker = Ticker::every(POLL_PERIOD);
 
    loop {
        match mag.read().await {
            Ok(Some(field)) => {
                defmt::info!("Magnetometer: {}", field);
                sender.send(field);
            }
            Ok(None) => {} // no new sample yet
            Err(e) => defmt::warn!("LIS2MDL read failed: {}", e),
        }
        ticker.next().await;
    }
}
 