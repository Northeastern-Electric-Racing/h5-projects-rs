//! Peripheral device structs
//!
//! Contains temperature sensor and imu

use cangen::ToCanFrame;
use defmt::{debug, warn};
use embassy_stm32::Peri;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_time::{Delay, Duration, Ticker};
use embedded_sht3x::{Repeatability::Medium, Sht3x, Status, Temperature};
use uom::si::f32::{Ratio, ThermodynamicTemperature};
use uom::si::ratio::percent;
use uom::si::thermodynamic_temperature::degree_celsius;

const SHT30_ADDRESS: u8 = 0x44;
const TEMP_POLL_INTERVAL: Duration = Duration::from_millis(100);
static TEMP_SENSOR: CriticalSectionRawMutex = CriticalSectionRawMutex::new();

pub struct I2C2Resources {
    pub i2c: Peri<'static, peripherals::I2C2>,
    pub scl: Peri<'static, peripherals::PF1>,
    pub sda: Peri<'static, peripherals::PF0>,
}

pub struct TempSensor<'a> {
    sensor: embedded_sht3x::Sht3x<&'a mut I2c<'static, Blocking, Master>, Delay>,
    i2c_address: u8,
    temperature: ThermodynamicTemperature,
    humidity: Ratio,
}

impl TempSensor<'_> {
    pub fn sht30_init(&mut self) {
        TEMP_SENSOR.lock(|| {
            self.sensor.repeatability = Medium;
        });
        self.poll_temperature_and_humidity(); //get initial reading
    }

    pub fn toggle_heater(&mut self, enable: bool) {
        TEMP_SENSOR.lock(|| {
            if let Ok(status) = self.sensor.get_status() {
                if !status.intersects(Status::HEATER)
                    && enable
                    && self.sensor.enable_heater().is_err()
                {
                    debug!("Sht30 heater enable failed");
                } else if status.intersects(Status::HEATER)
                    && !enable
                    && self.sensor.disable_heater().is_err()
                {
                    debug!("Sht30 heater disable failed");
                }
            }
        });
    }

    pub fn poll_temperature_and_humidity(&mut self) {
        TEMP_SENSOR.lock(|| {
            if let Ok(value) = self.sensor.single_measurement() {
                self.temperature = ThermodynamicTemperature::new::<degree_celsius>(
                    value.temperature.celsius().value(),
                );
                self.humidity = Ratio::new::<percent>(value.relative_humidity.value());
            }
        });
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
        match Sht3x::new(&mut i2c, SHT30_ADDRESS, Delay) {
            Ok(s) => break s,
            Err(_) => {
                warn!("Sht30 init failed, retrying");
            }
        }
        ticker.next().await;
    };

    let mut temp_sensor: TempSensor = TempSensor {
        sensor: sensor_init,
        i2c_address: SHT30_ADDRESS,
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
