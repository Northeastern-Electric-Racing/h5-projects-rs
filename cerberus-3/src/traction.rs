use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;

use uom::ConstZero;
use uom::si::acceleration::standard_gravity;
use uom::si::angular_velocity::revolution_per_minute;
use uom::si::f32::{Acceleration, AngularVelocity, Length, Velocity};
use uom::si::length::inch;
use uom::si::ratio::ratio;
use uom::si::velocity::mile_per_hour;

use crate::peripherals::{self, AccelSample};

const CURVE_MAGIC: u32 = 0x004E_4552;
const CURVE_VERSION: u8 = 1;
const CURVE_POINTS_MAX: usize = 200;
const CALIB_SAMPLES: u16 = 50;
const MIN_VX: f32 = 0.5; // mph
const VEL_ALPHA: f32 = 0.98;
const INTEGRAL_LIMIT: f32 = 1.0;
const KP_DEFAULT: f32 = 1.5;
const KI_DEFAULT: f32 = 0.2;
const TIRE_DIAMETER: f32 = 16.0;
const GEAR_RATIO: f32 = 39.0 / 13.0;
const TIRE_CURVE_HEADER_SIZE: usize = 27;
const TIRE_CURVE_POINT_SIZE: usize = 8;
const TIRE_CURVE_DATA_SIZE: usize =
    TIRE_CURVE_HEADER_SIZE + CURVE_POINTS_MAX * TIRE_CURVE_POINT_SIZE;

// TODO
pub fn imu_get_acceleration() -> Option<AccelSample> {
    peripherals::LATEST_ACCEL.lock(|c| c.get())
}

//TODO
pub fn dti_get_rpm() -> AngularVelocity {
    AngularVelocity::new::<revolution_per_minute>(0.0)
}

#[derive(Clone, Copy)]
struct TireCurvePoint {
    slip_ratio: f32,
    fx_norm: f32,
}

#[derive(Clone, Copy)]
struct TireCurve {
    magic: u32,
    version: u8,
    num_points: u16,
    surface_id: [u8; 16],
    peak_lambda: f32,
    points: [TireCurvePoint; CURVE_POINTS_MAX],
}

#[derive(Clone, Copy)]
struct VelEstimator {
    v_x: Velocity,
    ax_bias: f32,
    alpha: f32,
    init: bool,
}

impl VelEstimator {
    const fn new() -> Self {
        Self {
            v_x: Velocity::ZERO,
            ax_bias: 0.0,
            alpha: VEL_ALPHA,
            init: false,
        }
    }
}

#[derive(Clone, Copy)]
struct TcPi {
    kp: f32,
    ki: f32,
    integral: f32,
}

impl TcPi {
    const fn new() -> Self {
        Self {
            kp: KP_DEFAULT,
            ki: KI_DEFAULT,
            integral: 0.0,
        }
    }
}

pub struct TcState {
    tire_curve: Option<TireCurve>,
    vel_estimator: VelEstimator,
    pi: TcPi,
    prev_time: Option<Instant>,
    ax_sum: f32,
    sample_count: u16,
}

impl TcState {
    pub const fn new() -> Self {
        Self {
            tire_curve: None,
            vel_estimator: VelEstimator::new(),
            pi: TcPi::new(),
            prev_time: None,
            ax_sum: 0.0,
            sample_count: 0,
        }
    }

    fn update_dt(&mut self) -> f32 {
        let now = Instant::now();
        let dt = match self.prev_time {
            Some(prev) => (now - prev).as_micros() as f32 / 1_000_000.0,
            None => 0.0,
        };
        self.prev_time = Some(now);
        dt
    }
}

static ENABLED: AtomicBool = AtomicBool::new(false);

pub fn is_enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

pub fn enable_tc() {
    ENABLED.store(true, Ordering::Relaxed);
}

pub fn disable_tc() {
    ENABLED.store(false, Ordering::Relaxed);
}

pub fn toggle_tc() {
    let curr_state = ENABLED.load(Ordering::Relaxed);
    ENABLED.store(!curr_state, Ordering::Relaxed);
}

static FRONT_WHEEL_SPEEDS: Mutex<
    CriticalSectionRawMutex,
    Cell<(AngularVelocity, AngularVelocity)>,
> = Mutex::new(Cell::new((AngularVelocity::ZERO, AngularVelocity::ZERO)));

pub fn set_front_wheel_speeds(omega_fl: AngularVelocity, omega_fr: AngularVelocity) {
    FRONT_WHEEL_SPEEDS.lock(|c| c.set((omega_fl, omega_fr)));
}

fn get_front_wheel_speeds() -> (AngularVelocity, AngularVelocity) {
    FRONT_WHEEL_SPEEDS.lock(|c| c.get())
}

fn read_u16(data: &[u8], offset: usize) -> Option<u16> {
    Some(u16::from_le_bytes(
        data.get(offset..offset + 2)?.try_into().ok()?,
    ))
}

fn read_u32(data: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_le_bytes(
        data.get(offset..offset + 4)?.try_into().ok()?,
    ))
}

fn read_f32(data: &[u8], offset: usize) -> Option<f32> {
    Some(f32::from_le_bytes(
        data.get(offset..offset + 4)?.try_into().ok()?,
    ))
}

fn load_tire_curve(data: &[u8]) -> Option<TireCurve> {
    if data.len() < TIRE_CURVE_DATA_SIZE {
        defmt::error!(
            "TC: curve data too small ({} < {} bytes)",
            data.len(),
            TIRE_CURVE_DATA_SIZE
        );
        return None;
    }

    let magic = read_u32(data, 0)?;
    let version = data[4];
    let num_points = read_u16(data, 5)?;

    if magic != CURVE_MAGIC {
        defmt::error!("TC: bad curve magic 0x{:08x}", magic);
        return None;
    }

    if version != CURVE_VERSION {
        defmt::error!("TC: unsupported curve version {}", version);
        return None;
    }

    if num_points < 2 || num_points as usize > CURVE_POINTS_MAX {
        defmt::error!("TC: invalid num_points {}", num_points);
        return None;
    }

    let mut surface_id = [0u8; 16];
    surface_id.copy_from_slice(&data[7..23]);

    let peak_lambda = read_f32(data, 23)?;

    let mut points = [TireCurvePoint {
        slip_ratio: 0.0,
        fx_norm: 0.0,
    }; CURVE_POINTS_MAX];

    for (i, point) in points.iter_mut().enumerate() {
        let offset = TIRE_CURVE_HEADER_SIZE + i * TIRE_CURVE_POINT_SIZE;

        *point = TireCurvePoint {
            slip_ratio: read_f32(data, offset)?,
            fx_norm: read_f32(data, offset + 4)?,
        };
    }

    Some(TireCurve {
        magic,
        version,
        num_points,
        surface_id,
        peak_lambda,
        points,
    })
}

fn wheel_speed(omega: AngularVelocity) -> Velocity {
    Length::new::<inch>(TIRE_DIAMETER / 2.0) * omega
}

fn calc_slip(motor_speed: AngularVelocity, vx_car: Velocity) -> f32 {
    let v_rear = wheel_speed(motor_speed / GEAR_RATIO);
    let denom = vx_car.abs().max(v_rear.abs());

    if denom < Velocity::new::<mile_per_hour>(MIN_VX) {
        return 0.0;
    }

    ((v_rear - vx_car) / denom).get::<ratio>()
}

fn lookup_fx(curve: &TireCurve, slip: f32) -> f32 {
    let n = (curve.num_points as usize).min(CURVE_POINTS_MAX);
    if n == 0 {
        return 0.0;
    }
    let pts = &curve.points;

    let first = pts[0];
    let last = pts[n - 1];
    let (first_slip, first_fx) = (first.slip_ratio, first.fx_norm);
    let (last_slip, last_fx) = (last.slip_ratio, last.fx_norm);

    if slip <= first_slip {
        return first_fx;
    }
    if slip >= last_slip {
        return last_fx;
    }

    let mut lo = 0;
    let mut hi = n - 1;
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        let mid_slip = pts[mid].slip_ratio;
        if mid_slip <= slip {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let (lo_slip, lo_fx) = (pts[lo].slip_ratio, pts[lo].fx_norm);
    let (hi_slip, hi_fx) = (pts[hi].slip_ratio, pts[hi].fx_norm);

    let span = hi_slip - lo_slip;
    if span <= 0.0 {
        return lo_fx;
    }
    let t = (slip - lo_slip) / span;
    lo_fx + t * (hi_fx - lo_fx)
}

fn init_vel_estimator(est: &mut VelEstimator, avg_ax_stationary: f32) {
    est.ax_bias = avg_ax_stationary;
    est.v_x = Velocity::ZERO;
    est.alpha = VEL_ALPHA;
    est.init = true;
    defmt::info!("TC: ax bias = {} mg", est.ax_bias);
}

fn estimate_velocity(
    est: &mut VelEstimator,
    avg_front_rads: AngularVelocity,
    ax: f32,
    dt: f32,
) -> Velocity {
    let vx_wheel = wheel_speed(avg_front_rads);
    let ax_corrected = Acceleration::new::<standard_gravity>((ax - est.ax_bias) / 1000.0);
    let dv = ax_corrected * uom::si::f32::Time::new::<uom::si::time::second>(dt);
    let vx_imu = est.v_x + dv;
    let v = est.alpha * vx_wheel + (1.0 - est.alpha) * vx_imu;

    est.v_x = v.max(Velocity::ZERO);
    est.v_x
}

fn update_pi(pi: &mut TcPi, curve: &TireCurve, slip: f32, dt: f32) -> f32 {
    let slip_ref = curve.peak_lambda * 0.98;
    let error = slip_ref - slip;

    pi.integral = (pi.integral + error * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    let fx_ff = lookup_fx(curve, slip_ref);
    (fx_ff + pi.kp * error + pi.ki * pi.integral).clamp(0.0, 1.0)
}

/// Loads the tire curve into the state owned by the pedals task.
pub fn init(state: &mut TcState, tire_curve_data: &[u8]) -> Result<(), ()> {
    state.tire_curve = load_tire_curve(tire_curve_data);
    let loaded = state.tire_curve.is_some();
    defmt::info!("TC: init done (curve loaded: {})", loaded);
    Ok(())
}

pub fn process(state: &mut TcState) -> f32 {
    let dt = state.update_dt();

    // 1. Disabled -> pass-through.
    if !is_enabled() {
        return 1.0;
    }

    // 2. No valid curve -> pass-through.
    let Some(_) = state.tire_curve else {
        return 1.0;
    };

    // 3. Calibration.
    if !state.vel_estimator.init {
        if let Some(accel) = imu_get_acceleration() {
            state.ax_sum += accel.x_mg;
            state.sample_count += 1;
        }

        if state.sample_count >= CALIB_SAMPLES {
            let avg = state.ax_sum / state.sample_count as f32;
            init_vel_estimator(&mut state.vel_estimator, avg);
        }

        return 1.0;
    }

    // 4. Control step.
    let accel = imu_get_acceleration();
    let (omega_fl, omega_fr) = get_front_wheel_speeds();
    let motor_speed = dti_get_rpm();

    let scale = {
        let TcState {
            tire_curve,
            vel_estimator,
            pi,
            ..
        } = state;

        let Some(curve) = tire_curve.as_ref() else {
            return 1.0;
        };

        let ax = match accel {
            Some(a) => {
                vel_estimator.alpha = VEL_ALPHA;
                a.x_mg
            }
            None => {
                defmt::error!("TC: IMU read failed, using wheel speed only");
                // alpha = 1.0 -> velocity entirely from wheel speeds this cycle.
                vel_estimator.alpha = 1.0;
                0.0
            }
        };

        let avg_front_rads = (omega_fl + omega_fr) / 2.0;
        let vx_car = estimate_velocity(vel_estimator, avg_front_rads, ax, dt);
        let slip = calc_slip(motor_speed, vx_car);
        update_pi(pi, curve, slip, dt)
    };

    if scale.is_finite() {
        scale.clamp(0.0, 1.0)
    } else {
        defmt::error!("TC: non-finite torque scale.");
        1.0
    }
}
