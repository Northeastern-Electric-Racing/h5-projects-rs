//! LSM6DSV IMU polling, similar to `u_sensors.c`.
//!
//! The sensor setup lives in the `lsm6dsv-ner` driver (firmware-rs/drivers); this file just
//! runs it and publishes readings. note: that driver is still WIP
//!

//! CS PA4 (`SPI_1_NSS`). The SPI bus is set up in main.rs and handed to [`imu_task`] as an
//! [`ImuSpi`].
 
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsv_ner::{AccelRange, Config, DataRate, GyroRange, ImuReading, Lsm6dsv};
 
/// SPI1 plus the IMU's chip select pin
/// if this doesn't compile, check the `Spi` type parameters for your embassy-stm32 version.
pub type ImuSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
 
/// how often to read
/// the C firmware read the IMU and magnetometer together every 20 ms
const POLL_PERIOD: Duration = Duration::from_millis(20);
 
/// same as the C firmware

const CONFIG: Config = Config {
    accel_range: AccelRange::G2, /// let's see if this is good enough
    gyro_range: GyroRange::Dps2000,
    data_rate: DataRate::Hz120,
};
 
/// max number of tasks 
const MAX_RECEIVERS: usize = 2;
 
/// latest IMU reading, for any task that wants it 
pub static IMU_DATA: Watch<ThreadModeRawMutex, ImuReading, MAX_RECEIVERS> = Watch::new();
 
#[embassy_executor::task]
pub async fn imu_task(spi: ImuSpi) {
    let mut imu = Lsm6dsv::new(spi, Delay);
    if let Err(e) = imu.init(CONFIG).await {
        defmt::error!("LSM6DSV init failed: {} (check SPI wiring/mode)", e);
        return;
    }
    defmt::info!("LSM6DSV initialised: {}", CONFIG);
 
    let sender = IMU_DATA.sender();
    let mut ticker = Ticker::every(POLL_PERIOD);
 
    loop {
        match imu.read().await {
            Ok(reading) => {
                defmt::info!("IMU: {}", reading);
                sender.send(reading);
            }
            Err(e) => defmt::warn!("LSM6DSV read failed: {}", e),
        }
        ticker.next().await;
    }
}