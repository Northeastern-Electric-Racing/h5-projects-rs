use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use lsm6dso::Lsm6dso;
use lsm6dsox::Lsm6dsox;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_acceleration;
use uom::si::f32::*;
use uom::si::length::meter;

pub struct IMU {
    // TODO: Set up I2C Generic
    imu: Lsm6dsox<...>
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
    fn get_accel(&self) -> AccelVec {
        todo!("do this")
    }
    fn get_anguar_accel(&self) -> AngularAccelVec {
        todo!("you should do this")
    }
}
