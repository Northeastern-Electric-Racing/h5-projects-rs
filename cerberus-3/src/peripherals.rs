use cangen::ToCanFrame;
use defmt::{error, info, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::can::Frame;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::DynamicSender;
use embassy_time::{Delay, Timer};
use lsm6dsv16x_rs::asynchronous::Lsm6dsv16x;
use lsm6dsv16x_rs::asynchronous::register::MainBank;
use lsm6dsv16x_rs::asynchronous::register::main::{
    FiltGyLp1Bandwidth, FiltSettlingMask, FiltXlLp2Bandwidth, GyFullScale, Odr, Reset, XlFullScale,
};
use st_mems_bus::asynchronous::SpiBus;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_velocity::degree_per_second;
use uom::si::f32::*;

unit! {
    system: uom::si;
    quantity: uom::si::acceleration;
    @milligravity: 9.80665e-3; "mG", "milligravity", "milligravities"; // Singular abbreviation, singular name, plural name
}

// each unit! expansion emits its own `__system`/`__quantity`/`Conversion`/`Unit`
// imports and `mod f32`, so a second invocation needs its own module
mod mdps {
    unit! {
        system: uom::si;
        quantity: uom::si::angular_velocity;
        @millidegree_per_second: 1.745_329_251_994_329_5_E-5; "mdps", "millidegree per second", "millidegrees per second"; // Singular abbreviation, singular name, plural name
        // coefficient is relative to the base unit rad/s: (pi/180)/1000
    }
}
use mdps::millidegree_per_second;

pub type ImuSpi =
    SpiDevice<'static, ThreadModeRawMutex, Spi<'static, Async, spi::mode::Master>, Output<'static>>;
const ID: u8 = 0x70;
pub struct IMU {
    imu: Lsm6dsv16x<SpiBus<ImuSpi>, Delay, MainBank>,
    // imu: Lsm6dsox<ImuI2c, Delay>,
    can_tx: DynamicSender<'static, Frame>,
}

pub struct AccelVec {
    x: Acceleration,
    y: Acceleration,
    z: Acceleration,
}

#[derive(Debug)]
pub enum ImuInitError {
    WrongId,
}

pub struct AngularVelVec {
    roll_rate: AngularVelocity,
    pitch_rate: AngularVelocity,
    yaw_rate: AngularVelocity,
}

impl AngularVelVec {
    #[allow(dead_code)] // This might come in handy later
    fn from_rads_per_s(p: f32, q: f32, r: f32) -> Self {
        AngularVelVec {
            roll_rate: AngularVelocity::new::<degree_per_second>(p),
            pitch_rate: AngularVelocity::new::<degree_per_second>(q),
            yaw_rate: AngularVelocity::new::<degree_per_second>(r),
        }
    }
    fn from_raws(raws: [i16; 3]) -> Self {
        AngularVelVec {
            roll_rate: AngularVelocity::new::<millidegree_per_second>(raws[0] as f32),
            pitch_rate: AngularVelocity::new::<millidegree_per_second>(raws[1] as f32),
            yaw_rate: AngularVelocity::new::<millidegree_per_second>(raws[2] as f32),
        }
    }
}

impl AccelVec {
    #[allow(dead_code)] // This might come in handy later
    fn from_mps_sq(x: f32, y: f32, z: f32) -> Self {
        AccelVec {
            x: Acceleration::new::<meter_per_second_squared>(x),
            y: Acceleration::new::<meter_per_second_squared>(y),
            z: Acceleration::new::<meter_per_second_squared>(z),
        }
    }
    fn from_raws(raw: [i16; 3]) -> Self {
        use lsm6dsv16x_rs::asynchronous::from_fs2_to_mg;
        AccelVec {
            x: Acceleration::new::<milligravity>(from_fs2_to_mg(raw[0])),
            y: Acceleration::new::<milligravity>(from_fs2_to_mg(raw[1])),
            z: Acceleration::new::<milligravity>(from_fs2_to_mg(raw[2])),
        }
    }
}

impl IMU {
    pub fn new(spi: ImuSpi, tim: Delay, can_tx: DynamicSender<'static, Frame>) -> Self {
        IMU {
            imu: Lsm6dsv16x::new_spi(spi, tim),
            can_tx: can_tx,
        }
    }
    async fn get_accel(&mut self) -> Option<AccelVec> {
        match self.imu.flag_data_ready_get().await {
            Ok(f) => {
                let raw_accel = self.imu.acceleration_raw_get().await;
                if f.drdy_xl == 1 && raw_accel.is_ok() {
                    Some(AccelVec::from_raws(raw_accel.unwrap()))
                } else {
                    None
                }
            }
            Err(_) => {
                warn!("Reading accelerometer failed!");
                None
            }
        }
    }

    async fn send_accel(&self, accel: AccelVec) {
        let frame = cangen::ImuAccelerometer::new()
            .with_imu_accelerometer_x(accel.x.get::<milligravity>())
            .with_imu_accelerometer_y(accel.y.get::<milligravity>())
            .with_imu_accelerometer_z(accel.z.get::<milligravity>());
        self.can_tx.send(frame.to_can_frame()).await;
    }

    async fn send_angular_vel(&self, vel: AngularVelVec) {
        let frame = cangen::ImuGyro::new()
            .with_imu_gyro_x(vel.roll_rate.get::<mdps::millidegree_per_second>())
            .with_imu_gyro_y(vel.pitch_rate.get::<mdps::millidegree_per_second>())
            .with_imu_gyro_z(vel.yaw_rate.get::<mdps::millidegree_per_second>());
        self.can_tx.send(frame.to_can_frame()).await;
    }
    async fn get_anguar_vel(&mut self) -> Option<AngularVelVec> {
        match self.imu.flag_data_ready_get().await {
            Ok(f) => {
                let raw_rate = self.imu.angular_rate_raw_get().await;
                if f.drdy_gy == 1 && raw_rate.is_ok() {
                    Some(AngularVelVec::from_raws(raw_rate.unwrap()))
                } else {
                    None
                }
            }
            Err(_) => {
                warn!("Reading accelerometer failed!");
                None
            }
        }
    }
    async fn init(&mut self) -> Result<(), ImuInitError> {
        // All of this is ripped from the example code
        Timer::after_millis(5).await;

        // Check device ID
        let id = self.imu.device_id_get().await.unwrap();
        info!("Device ID: {:x}", id);
        if id != ID {
            error!("Unexpected device ID: {:x}", id);
            return Err(ImuInitError::WrongId);
        }

        // Restore default configuration
        self.imu.reset_set(Reset::RestoreCtrlRegs).await.unwrap();
        let mut rst: Reset = Reset::RestoreCtrlRegs;
        while rst != Reset::Ready {
            rst = self.imu.reset_get().await.unwrap();
        }

        // Enable Block Data Update
        self.imu.block_data_update_set(1).await.unwrap();

        // Set Output Data Rate for accelerometer and gyroscope
        self.imu.xl_data_rate_set(Odr::_7_5hz).await.unwrap();
        self.imu.gy_data_rate_set(Odr::_15hz).await.unwrap();

        // Set full scale for accelerometer and gyroscope
        self.imu.xl_full_scale_set(XlFullScale::_2g).await.unwrap();
        self.imu
            .gy_full_scale_set(GyFullScale::_2000dps)
            .await
            .unwrap();

        // Configure filtering chain
        let filt_settling_mask = FiltSettlingMask {
            drdy: 1,
            ois_drdy: 1,
            irq_xl: 1,
            irq_g: 1,
        };
        self.imu
            .filt_settling_mask_set(filt_settling_mask)
            .await
            .unwrap();
        self.imu.filt_gy_lp1_set(1).await.unwrap();
        self.imu
            .filt_gy_lp1_bandwidth_set(FiltGyLp1Bandwidth::UltraLight)
            .await
            .unwrap();
        self.imu.filt_xl_lp2_set(1).await.unwrap();
        self.imu
            .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
            .await
            .unwrap();

        info!("Configuration ended, check the output on the UART channel");
        Ok(())
    }
}
#[embassy_executor::task]
pub async fn imu_task(spi: ImuSpi, can_tx: DynamicSender<'static, Frame>) -> ! {
    let mut imu: IMU = IMU::new(spi, Delay, can_tx);
    imu.init().await.expect("Failed to init IMU");
    loop {
        let latest_accel = imu.get_accel().await;
        if latest_accel.is_some() {
            imu.send_accel(latest_accel.unwrap()).await;
        }
        let latest_angular_vel = imu.get_anguar_vel().await;
        if latest_angular_vel.is_some() {
            imu.send_angular_vel(latest_angular_vel.unwrap()).await;
        }

        Timer::after_millis(50).await;
    }
}
