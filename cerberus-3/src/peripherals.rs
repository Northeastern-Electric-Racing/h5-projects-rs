//! Contains temperature sensor and imu

use cangen::ToCanFrame;
use defmt::{debug, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, bind_interrupts, dma, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Ticker};
use embedded_sht3x::{Repeatability::Medium, Sht3x, Status, Temperature};
use static_cell::StaticCell;
use uom::si::f32::{Ratio, ThermodynamicTemperature};
use uom::si::ratio::percent;
use uom::si::thermodynamic_temperature::degree_celsius;

use crate::can;

const SHT30_ADDRESS: u8 = 0x44;
const TEMP_POLL_INTERVAL: Duration = Duration::from_millis(100);
/// Same SPI2 clock as the Cerberus-2.0 C firmware.
const IMU_SPI_FREQUENCY: Hertz = Hertz::mhz(2);

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA1_CH2>;
    GPDMA1_CHANNEL3 => dma::InterruptHandler<peripherals::GPDMA1_CH3>;
});

// Needlessly verbose and overengineered, but hey maximum flexibility I suppose
static IMU_SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, spi::mode::Master>>> =
    StaticCell::new();

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
pub async fn peripherals_task(i2c_resources: I2C2Resources) {
    let mut ticker = Ticker::every(TEMP_POLL_INTERVAL);

    let mut i2c = I2c::new_blocking(
        i2c_resources.i2c,
        i2c_resources.scl,
        i2c_resources.sda,
        embassy_stm32::i2c::Config::default(),
    );

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

        crate::can::OUTGOING
            .send(
                cangen::TemperatureSensor::new()
                    .with_vcu_temperature(temperature)
                    .with_vcu_humidity(humidity)
                    .to_can_frame(),
            )
            .await;
    }
}

/// Brings up SPI2 and runs the IMU driver, which sends accelerometer and gyro
/// frames to [`can::OUTGOING`].
#[embassy_executor::task]
pub async fn imu_task(r: SPI2Resources) -> ! {
    let mut config = spi::Config::default();
    config.mode = spi::MODE_0;
    config.frequency = IMU_SPI_FREQUENCY;

    let spi = Spi::new(
        r.spi, r.sck, r.mosi, r.miso, r.tx_dma, r.rx_dma, Irqs, config,
    );
    let bus = IMU_SPI_BUS.init(Mutex::new(spi));
    let cs = Output::new(r.cs, Level::High, Speed::VeryHigh);

    lsm6dsv16x_ner::imu_loop(SpiDevice::new(bus, cs), can::OUTGOING.dyn_sender()).await
}
