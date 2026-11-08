//! 9D orientation estimate (VQF) from the IMU and magnetometer.
//! Runs at the same 50 Hz as both sensors. Axis alignment between the two sensors and
//! magnetometer calibration are NOT handled yet, so treat heading as unverified.

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};
use vqf_rs::VQF;

use crate::imu::IMU_DATA;
use crate::magnetometer::MAG_DATA;

const SAMPLE_PERIOD: Duration = Duration::from_millis(20);
const SAMPLE_TS_S: f32 = 0.02;
const MAX_RECEIVERS: usize = 2;
/// ignore the magnetometer if no new sample arrived within this long
const MAG_STALE_AFTER: Duration = Duration::from_millis(200);

/// mdps -> rad/s
const MDPS_TO_RADPS: f32 = core::f32::consts::PI / 180.0 / 1000.0;
/// mg -> m/s^2
const MG_TO_MPS2: f32 = 9.80665 / 1000.0;
/// milligauss -> microtesla
const MGAUSS_TO_UT: f32 = 0.1;

/// orientation quaternion (w, x, y, z)
#[derive(Debug, Clone, Copy, Default, defmt::Format)]
pub struct Orientation {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// true when the magnetometer is being rejected as disturbed
    pub mag_disturbed: bool,
}

/// latest orientation
pub static ORIENTATION: Watch<ThreadModeRawMutex, Orientation, MAX_RECEIVERS> = Watch::new();

#[embassy_executor::task]
pub async fn orientation_task() {
    let mut imu_rx = IMU_DATA.receiver().expect("no free IMU_DATA receiver");
    let mut mag_rx = MAG_DATA.receiver().expect("no free MAG_DATA receiver");
    let sender = ORIENTATION.sender();
    let mut vqf = VQF::new(SAMPLE_TS_S, None, None, None);
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    let mut last_mag: Option<([f32; 3], Instant)> = None;

    loop {
        ticker.next().await;

        // need an IMU sample to run at all; the magnetometer is optional
        let Some(imu) = imu_rx.try_get() else {
            continue;
        };
        if let Some(m) = mag_rx.try_changed() {
            last_mag = Some((
                [
                    m.x_mgauss * MGAUSS_TO_UT,
                    m.y_mgauss * MGAUSS_TO_UT,
                    m.z_mgauss * MGAUSS_TO_UT,
                ],
                Instant::now(),
            ));
        }
        // only feed VQF a magnetometer sample that is still fresh
        let mag = last_mag.and_then(|(v, t)| (t.elapsed() < MAG_STALE_AFTER).then_some(v));

        vqf.update(
            [
                imu.gyro_mdps.x * MDPS_TO_RADPS,
                imu.gyro_mdps.y * MDPS_TO_RADPS,
                imu.gyro_mdps.z * MDPS_TO_RADPS,
            ],
            [
                imu.accel_mg.x * MG_TO_MPS2,
                imu.accel_mg.y * MG_TO_MPS2,
                imu.accel_mg.z * MG_TO_MPS2,
            ],
            mag,
        );

        let q = if mag.is_some() {
            vqf.quat_9d()
        } else {
            vqf.quat_6d()
        };
        sender.send(Orientation {
            w: q.0,
            x: q.1,
            y: q.2,
            z: q.3,
            mag_disturbed: vqf.mag_dist_detected(),
        });
    }
}
