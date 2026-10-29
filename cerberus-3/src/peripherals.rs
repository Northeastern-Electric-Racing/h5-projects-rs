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
    pub scl: Peri<'static, peripherals::PF1>, // need to update
    pub sda: Peri<'static, peripherals::PF0>, // need to update
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
        self.tempsensor_get_temperature_and_humidity(); //get initial reading
    }

    pub fn tempsensor_toggle_heater(&mut self, enable: bool) {
        TEMP_SENSOR.lock(|| {
            match self.sensor.get_status() {
                Ok(status) => {
                    if !status.intersects(Status::HEATER) && enable {
                        match self.sensor.enable_heater() {
                            Ok(_) => {}
                            Err(_) => {
                                debug!("Sht30 heater enable failed");
                            }
                        };
                    } else if status.intersects(Status::HEATER) && !enable {
                        match self.sensor.disable_heater() {
                            Ok(_) => {}
                            Err(_) => {
                                debug!("Sht30 heater disable failed");
                            }
                        };
                    }
                }
                Err(_) => {
                    debug!("Sht30 status error occurred");
                }
            };
        });
    }

    pub fn tempsensor_get_temperature_and_humidity(&mut self) -> (f32, f32) {
        TEMP_SENSOR.lock(|| {
            match self.sensor.single_measurement() {
                Ok(value) => {
                    self.temperature = ThermodynamicTemperature::new::<degree_celsius>(
                        value.temperature.celsius().value(),
                    );
                    self.humidity = Ratio::new::<percent>(value.relative_humidity.value());
                }
                Err(_) => {
                    debug!("Sht30 measurement error occured");
                }
            };
        });
        (
            self.temperature.get::<degree_celsius>(),
            self.humidity.get::<percent>(),
        )
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

        let temp_humidity_reading = temp_sensor.tempsensor_get_temperature_and_humidity();
        if temp_humidity_reading == (0.0, 0.0) {
            continue;
        }

        crate::can::OUTGOING
            .send(
                cangen::TemperatureSensor::new()
                    .with_vcu_temperature(temp_humidity_reading.0)
                    .with_vcu_humidity(temp_humidity_reading.1)
                    .to_can_frame(),
            )
            .await;
    }
}
