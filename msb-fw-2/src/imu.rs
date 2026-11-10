//! LSM6DSV IMU polling, similar to `u_sensors.c`.
//!
//! The sensor setup lives in the `lsm6dsv-ner` driver (firmware-rs/drivers); this file just
//! runs it and publishes readings. note: that driver is still WIP
//!

//! CS PA4 (`SPI_1_NSS`). The SPI bus is set up in main.rs and handed to [`imu_task`] as an
//! [`ImuSpi`].

use defmt::{Debug2Format, debug, error, info};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsv16x_ner::mdps::millidegree_per_second;
use lsm6dsv16x_ner::{NerLsm6dsv16x, milligravity};

/// SPI1 plus the IMU's chip select pin
/// if this doesn't compile, check the `Spi` type parameters for your embassy-stm32 version.
pub type ImuSpi = ExclusiveDevice<Spi<'static, Async, Master>, Output<'static>, Delay>;

/// how often to read
/// the C firmware read the IMU and magnetometer together every 20 ms
const POLL_PERIOD: Duration = Duration::from_millis(20);

/// max number of tasks
const MAX_RECEIVERS: usize = 2;

/// a value on each of the thrree axes
#[derive(Debug, Clone, Copy, Default, defmt::Format)]
pub struct Axes {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// one imu reading in milli-g for accel and millidegrees/sec for angular rate
#[derive(Debug, Clone, Copy, Default, defmt::Format)]
pub struct ImuReading {
    /// accel
    pub accel_mg: Axes,
    /// angular rate
    pub gyro_mdps: Axes,
}

/// latest IMU reading, for any task that wants it
pub static IMU_DATA: Watch<ThreadModeRawMutex, ImuReading, MAX_RECEIVERS> = Watch::new();

#[embassy_executor::task]
pub async fn imu_task(spi: ImuSpi) {
    Timer::after_millis(20).await; // let the IMU finish booting after power-up
    let mut imu = NerLsm6dsv16x::new(spi, Delay);

    // keep retrying instead of giving up, so the bus stays alive for debugging
    loop {
        match imu.init().await {
            Ok(()) => break,
            Err(e) => {
                error!("LSM6DSV init failed: {} (retrying)", Debug2Format(&e));
                Timer::after_millis(100).await;
            }
        }
    }
    info!("LSM6DSV initialised");

    

    let sender = IMU_DATA.sender();

    // since accel and gyro can have new data at diff times, keep the latest of each
    let mut reading = ImuReading::default();
    let mut ticker = Ticker::every(POLL_PERIOD);
    loop {
        let mut updated = false;

        if let Some(a) = imu.get_accel().await {
            reading.accel_mg = Axes {
                x: a.x.get::<milligravity>(),
                y: a.y.get::<milligravity>(),
                z: a.z.get::<milligravity>(),
            };
            updated = true;
        }

        if let Some(g) = imu.get_anguar_vel().await {
            reading.gyro_mdps = Axes {
                x: g.roll_rate.get::<millidegree_per_second>(),
                y: g.pitch_rate.get::<millidegree_per_second>(),
                z: g.yaw_rate.get::<millidegree_per_second>(),
            };
            updated = true;
        }

        if updated {
            debug!("IMU: {}", reading);
            sender.send(reading);
        }
        ticker.next().await;
    }
}
