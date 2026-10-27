//! Periphal device structs

use defmt::{debug, warn};
use embassy_stm32::{
    Peripherals, bind_interrupts, dma,
    i2c::{I2c, Master},
    mode::Async,
    peripherals,
};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_time::Delay;
use embedded_sht3x::{Repeatability::High, Sht3x, Status, Temperature};
use uom::si::{
    f32::{Ratio, ThermodynamicTemperature},
    ratio::percent,
    thermodynamic_temperature::degree_celsius,
};

const SHT30_ADDRESS: u8 = 0x44;
static TEMP_SENSOR: CriticalSectionRawMutex = CriticalSectionRawMutex::new();

bind_interrupts!(struct I2cIrqs {
    GPDMA1_CHANNEL5 => dma::InterruptHandler<peripherals::GPDMA1_CH5>; //need to update
    GPDMA1_CHANNEL6 => dma::InterruptHandler<peripherals::GPDMA1_CH6>; //need to update
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

pub struct TempSensor {
    sensor: embedded_sht3x::Sht3x<I2c<'static, Async, Master>, Delay>,
    i2c_address: u8,
    temperature: ThermodynamicTemperature,
    humidity: Ratio,
}

impl TempSensor {
    pub fn sht30_init(&mut self, p: Peripherals, address: u8) {
        TEMP_SENSOR.lock(|| {
            let i2c = I2c::new(
                p.I2C1,
                p.PB6,        //need to update
                p.PB7,        //need to update
                p.GPDMA1_CH6, //need to update
                p.GPDMA1_CH5, //need to update
                I2cIrqs,
                embassy_stm32::i2c::Config::default(),
            );

            match Sht3x::new(i2c, SHT30_ADDRESS, Delay) {
                Ok(sensor_init) => {
                    self.sensor = sensor_init;
                    self.sensor.repeatability = High;
                }
                Err(_) => {
                    warn!("Sht30 init error occured");
                }
            }
            self.i2c_address = address;
            self.temperature = ThermodynamicTemperature::new::<degree_celsius>(0.0);
            self.humidity = Ratio::new::<percent>(0.0);
        });
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
                    debug!("Sht30 status error occured");
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
