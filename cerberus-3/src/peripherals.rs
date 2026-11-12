//! Contains temperature sensor and imu
//! and sends their data to the CAN bus.

use core::cell::Cell;

use cangen::ToCanFrame;
use defmt::{debug, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, bind_interrupts, dma, peripherals};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Ticker};
use embedded_sht3x::{Repeatability::Medium, Sht3x, Status, Temperature};
use lsm6dsv16x_ner::NerLsm6dsv16x;
use uom::si::f32::{Ratio, ThermodynamicTemperature};
use uom::si::ratio::percent;
use uom::si::thermodynamic_temperature::degree_celsius;

use crate::can;

const SHT30_ADDRESS: u8 = 0x44;
const TEMP_POLL_INTERVAL: Duration = Duration::from_millis(100);
const IMU_POLL_INTERVAL: Duration = Duration::from_millis(100);
const IMU_SPI_FREQUENCY: Hertz = Hertz::mhz(2);

/// Latest IMU accelerometer reading, in milligravity
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct AccelSample {
    pub x_mg: f32,
    pub y_mg: f32,
    pub z_mg: f32,
}

pub static LATEST_ACCEL: BlockingMutex<CriticalSectionRawMutex, Cell<Option<AccelSample>>> =
    BlockingMutex::new(Cell::new(None));

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA1_CH2>;
    GPDMA1_CHANNEL3 => dma::InterruptHandler<peripherals::GPDMA1_CH3>;
});

pub struct I2C2Resources {
    pub i2c: Peri<'static, peripherals::I2C2>,
    pub scl: Peri<'static, peripherals::PF1>,
    pub sda: Peri<'static, peripherals::PF0>,
}

/// LSM6DSV16X IMU on SPI2 (pinout from Cerberus-2.0.ioc).
pub struct SPI2Resources {
    pub spi: Peri<'static, peripherals::SPI2>,
    pub sck: Peri<'static, peripherals::PA12>,
    pub mosi: Peri<'static, peripherals::PG1>,
    pub miso: Peri<'static, peripherals::PB14>,
    pub cs: Peri<'static, peripherals::PB9>,
    pub tx_dma: Peri<'static, peripherals::GPDMA1_CH2>,
    pub rx_dma: Peri<'static, peripherals::GPDMA1_CH3>,
}

pub struct TempSensor<'a> {
    sensor: embedded_sht3x::Sht3x<&'a mut I2c<'static, Blocking, Master>, Delay>,
    temperature: ThermodynamicTemperature,
    humidity: Ratio,
}

impl TempSensor<'_> {
    fn sht30_init(&mut self) {
        self.sensor.repeatability = Medium;
        self.poll_temperature_and_humidity(); //get initial reading
    }

    fn toggle_heater(&mut self, enable: bool) {
        if let Ok(status) = self.sensor.get_status() {
            if !status.intersects(Status::HEATER) && enable && self.sensor.enable_heater().is_err()
            {
                debug!("Sht30 heater enable failed");
            } else if status.intersects(Status::HEATER)
                && !enable
                && self.sensor.disable_heater().is_err()
            {
                debug!("Sht30 heater disable failed");
            }
        }
    }

    fn poll_temperature_and_humidity(&mut self) {
        if let Ok(value) = self.sensor.single_measurement() {
            self.temperature = ThermodynamicTemperature::new::<degree_celsius>(
                value.temperature.celsius().value(),
            );
            self.humidity = Ratio::new::<percent>(value.relative_humidity.value());
        }
    }
}

#[embassy_executor::task]
pub async fn i2c_peripherals_task(r: I2C2Resources) {
    let mut ticker = Ticker::every(TEMP_POLL_INTERVAL);

    let mut i2c = I2c::new_blocking(r.i2c, r.scl, r.sda, embassy_stm32::i2c::Config::default());

    //initialization will loop until successful
    let sensor_init = loop {
        if let Ok(s) = Sht3x::new(&mut i2c, SHT30_ADDRESS, Delay) {
            break s;
        } else {
            warn!("Sht30 init failed, retrying");
        }
        ticker.next().await;
    };

    let mut temp_sensor: TempSensor = TempSensor {
        sensor: sensor_init,
        temperature: ThermodynamicTemperature::new::<degree_celsius>(0.0),
        humidity: Ratio::new::<percent>(0.0),
    };
    temp_sensor.sht30_init();

    loop {
        ticker.next().await;

        temp_sensor.poll_temperature_and_humidity();
        let temperature = temp_sensor.temperature.get::<degree_celsius>();
        let humidity = temp_sensor.humidity.get::<percent>();
        if (temperature, humidity) == (0.0, 0.0) {
            continue;
        }

        can::OUTGOING
            .send(
                cangen::TemperatureSensor::new()
                    .with_vcu_temperature(temperature)
                    .with_vcu_humidity(humidity)
                    .to_can_frame(),
            )
            .await;

        let pwm = (temperature - 20f32) / (40f32 - 20f32) * 100f32; // 20-40C maps to 0-100% PWM

        can::OUTGOING
            .send(
                cangen::LvBoxFanPwm::new()
                    .with_fan_pwm_percentage(pwm.clamp(0.0, 100.0) as u8)
                    .to_can_frame(),
            )
            .await;
    }
}

/// Brings up SPI2 and runs the IMU driver, which sends accelerometer and gyro
/// frames to [`can::OUTGOING`].
#[embassy_executor::task]
pub async fn spi_peripherals_task(r: SPI2Resources) -> ! {
    let mut config = spi::Config::default();
    config.mode = spi::MODE_0;
    config.frequency = IMU_SPI_FREQUENCY;

    let spi = Spi::new(
        r.spi, r.sck, r.mosi, r.miso, r.tx_dma, r.rx_dma, Irqs, config,
    );
    let bus: Mutex<ThreadModeRawMutex, Spi<'static, Async, spi::mode::Master>> = Mutex::new(spi);

    let cs = Output::new(r.cs, Level::High, Speed::VeryHigh);
    let delay: Delay = Delay {};
    let mut imu = NerLsm6dsv16x::new(SpiDevice::new(&bus, cs), delay);

    imu.init().await.expect("Creating IMU failed");

    let mut ticker = Ticker::every(IMU_POLL_INTERVAL);
    loop {
        if let Some(accel) = imu.get_accel().await {
            let sample = AccelSample {
                x_mg: accel.x.get::<lsm6dsv16x_ner::milligravity>(),
                y_mg: accel.y.get::<lsm6dsv16x_ner::milligravity>(),
                z_mg: accel.z.get::<lsm6dsv16x_ner::milligravity>(),
            };
            LATEST_ACCEL.lock(|c| c.set(Some(sample)));

            let frame = cangen::ImuAccelerometer::new()
                .with_imu_accelerometer_x(accel.x.get::<lsm6dsv16x_ner::milligravity>())
                .with_imu_accelerometer_y(accel.y.get::<lsm6dsv16x_ner::milligravity>())
                .with_imu_accelerometer_z(accel.z.get::<lsm6dsv16x_ner::milligravity>());
            can::OUTGOING.send(frame.to_can_frame()).await;

            // TODO: send over ethernet as well
        }

        if let Some(ang_vel) = imu.get_angular_vel().await {
            let frame = cangen::ImuGyro::new()
                .with_imu_gyro_x(
                    ang_vel
                        .roll_rate
                        .get::<lsm6dsv16x_ner::mdps::millidegree_per_second>(),
                )
                .with_imu_gyro_y(
                    ang_vel
                        .pitch_rate
                        .get::<lsm6dsv16x_ner::mdps::millidegree_per_second>(),
                )
                .with_imu_gyro_z(
                    ang_vel
                        .yaw_rate
                        .get::<lsm6dsv16x_ner::mdps::millidegree_per_second>(),
                );
            can::OUTGOING.send(frame.to_can_frame()).await;

            // TODO: send over ethernet as well
        }

        ticker.next().await;
    }
}
