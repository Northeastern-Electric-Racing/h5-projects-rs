use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Delay;
use lsm6dso::Lsm6dso;
use lsm6dsox::{Lsm6dsox, SlaveAddress};
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_acceleration;
use uom::si::f32::*;
use uom::si::length::meter;

/// lsm6dsox is a blocking (embedded-hal 1.0) driver, so it needs the blocking shared bus.
pub type ImuI2c = I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Blocking, Master>>;

pub struct IMU {
    imu: Lsm6dsox<ImuI2c, Delay>,
}

pub struct AccelVec {
    x: Acceleration,
    y: Acceleration,
    z: Acceleration,
}

pub struct AngularAccelVec {
    x: AngularAcceleration,
    y: AngularAcceleration,
    z: AngularAcceleration,
}

impl AccelVec {
    fn from_mps(x: f32, y: f32, z: f32) -> Self {
        AccelVec {
            x: Acceleration::new::<meter_per_second_squared>(x),
            y: Acceleration::new::<meter_per_second_squared>(y),
            z: Acceleration::new::<meter_per_second_squared>(z),
        }
    }
    fn norm(&self) -> meter_per_second_squared {
        todo!("lazy bum")
    }
}

impl IMU {
    pub fn new(i2c: ImuI2c, address: SlaveAddress) -> Self {
        IMU {
            imu: Lsm6dsox::new(i2c, address, Delay),
        }
    }
    fn get_accel(&self) -> AccelVec {
        todo!("do this")
    }
    fn get_anguar_accel(&self) -> AngularAccelVec {
        todo!("you should do this")
    }
}
