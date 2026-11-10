use core::cell::{Cell, RefCell};
use core::fmt::Error;
use core::mem::size_of;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use uom::ConstZero;
use uom::si::acceleration::standard_gravity;
use uom::si::angular_velocity::{radian_per_second, revolution_per_minute};
use uom::si::f32::{Acceleration, AngularVelocity, Length, Time, Velocity};
use uom::si::length::{inch, meter};
use uom::si::ratio::ratio;
use uom::si::time::second;
use uom::si::velocity::{meter_per_second, mile_per_hour};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;

const CURVE_MAGIC: u32 = 0x004E_4552;
const CURVE_VERSION: u8 = 1;
const CURVE_POINTS_MAX: usize = 200;
const CALIB_SAMPLES: u16 = 50;
const MIN_VX: f32 = 0.5;
const VEL_ALPHA: f32 = 0.98;
const INTEGRAL_LIMIT: f32 = 1.0;
const KP_DEFAULT: f32 = 1.5;
const KI_DEFAULT: f32 = 0.2;
const TIRE_DIAMETER: f32 = 16.0;
const GEAR_RATIO: f32 = 39.0 / 13.0;

//TODO
#[derive(Clone, Copy, Debug, Default)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// TODO
pub fn imu_get_acceleration() -> Result<Vector3, Error> {
    Ok(Vector3::default())
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
    v_x: f32,
    ax_bias: f32,
    alpha: f32,
    init: bool,
}

impl VelEstimator {
    const fn new() -> Self {
        Self {
            v_x: 0.0,
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
    const fn new() -> Self {
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

static TORQUE_SCALE: AtomicU32 = AtomicU32::new(0x3F80_0000);

pub fn tc_get_torque_scale() -> f32 {
    f32::from_bits(TORQUE_SCALE.load(Ordering::Relaxed))
}

pub fn set_torque_scale(scale: f32) {
    let mut safe = 1.0;
    if scale.is_finite() {
        safe = scale.clamp(0.0, 1.0)
    } else {
        defmt::error!("TC: non-finite torque scale.")
    };
    TORQUE_SCALE.store(safe.to_bits(), Ordering::Relaxed);
}

static TC_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn tc_is_enabled() -> bool {
    TC_ENABLED.load(Ordering::Relaxed)
}

pub fn enable_tc() {
    TC_ENABLED.store(true, Ordering::Relaxed);
}

pub fn disable_tc() {
    TC_ENABLED.store(false, Ordering::Relaxed);
}

pub fn toggle_tc() {
    let curr_tc_state = TC_ENABLED.load(Ordering::Relaxed);
    TC_ENABLED.store(!curr_tc_state, Ordering::Relaxed);
}

static TC_STATE: Mutex<CriticalSectionRawMutex, RefCell<TcState>> =
    Mutex::new(RefCell::new(TcState::new()));

pub fn set_tc_state<R>(f: impl FnOnce(&mut TcState) -> R) -> R {
    TC_STATE.lock(|cell| f(&mut cell.borrow_mut()))
}

/// (omega_fl, omega_fr)
static FRONT_WHEEL_SPEEDS: Mutex<CriticalSectionRawMutex, Cell<(AngularVelocity, AngularVelocity)>> =
    Mutex::new(Cell::new((AngularVelocity::ZERO, AngularVelocity::ZERO)));

pub fn set_front_wheel_speeds(omega_fl: AngularVelocity, omega_fr: AngularVelocity) {
    FRONT_WHEEL_SPEEDS.lock(|c| c.set((omega_fl, omega_fr)));
}

pub fn get_front_wheel_speeds() -> (AngularVelocity, AngularVelocity) {
    FRONT_WHEEL_SPEEDS.lock(|c| c.get())
}

fn load_tire_curve(data: &[u8]) -> Option<TireCurve> {
    if data.len() < size_of::<TireCurve>() {
        defmt::error!(
            "TC: curve data too small ({} < {} bytes)",
            data.len(),
            size_of::<TireCurve>()
        );
        return None;
    }

    let curve: TireCurve = unsafe { ptr::read_unaligned(data.as_ptr() as *const TireCurve) };
    let magic = curve.magic;
    let version = curve.version;
    let num_points = curve.num_points;

    if magic != CURVE_MAGIC {
        defmt::error!("TC: bad curve magic 0x{:08x}", magic);
        return None;
    }
    if version != CURVE_VERSION {
        defmt::error!("TC: unsupported curve version {}", version);
        return None;
    }
    if (num_points as usize) > CURVE_POINTS_MAX || num_points < 2 {
        defmt::error!("TC: invalid num_points {}", num_points);
        return None;
    }

    Some(curve)
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
    est.v_x = 0.0;
    est.alpha = VEL_ALPHA;
    est.init = true;
    defmt::info!("TC: ax bias = {} mg", est.ax_bias);
}

fn estimate_velocity(est: &mut VelEstimator, avg_front_rads: AngularVelocity, ax: f32, dt: f32) -> Velocity {
    let vx_wheel = wheel_speed(avg_front_rads).get::<mile_per_hour>();
    let ax_corrected = Acceleration::new::<standard_gravity>((ax - est.ax_bias) / 1000.0);
    let dv = (ax_corrected * Time::new::<second>(dt)).get::<mile_per_hour>();
    let vx_imu = est.v_x + dv;
    let v = est.alpha * vx_wheel + (1.0 - est.alpha) * vx_imu;
    est.v_x = if v > 0.0 { v } else { 0.0 };
    Velocity::new::<mile_per_hour>(est.v_x)
}

fn update_pi(pi: &mut TcPi, curve: &TireCurve, slip: f32, dt: f32) -> f32 {
    let slip_ref = curve.peak_lambda * 0.98;
    let error = slip_ref - slip;

    pi.integral = (pi.integral + error * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    let fx_ff = lookup_fx(curve, slip_ref);
    (fx_ff + pi.kp * error + pi.ki * pi.integral).clamp(0.0, 1.0)
}

pub fn tc_init(tire_curve_data: &[u8]) -> Result<(), ()> {
    let curve = load_tire_curve(tire_curve_data);
    let loaded = curve.is_some();
    set_tc_state(|s| s.tire_curve = curve);
    defmt::info!("TC: init done (curve loaded: {})", loaded);
    Ok(())
}

pub fn tc_record_front_rpm(data: &[u8]) {
    if data.len() < 4 {
        defmt::error!("TC: front RPM message too short ({} bytes)", data.len());
        return;
    }
    let fl_rpm = i16::from_be_bytes([data[0], data[1]]);
    let fr_rpm = i16::from_be_bytes([data[2], data[3]]);
    set_front_wheel_speeds(
        AngularVelocity::new::<revolution_per_minute>(fl_rpm as f32),
        AngularVelocity::new::<revolution_per_minute>(fr_rpm as f32),
    );
}

pub fn tc_process() {
    let dt = set_tc_state(|s| s.update_dt());

    // 1. Disabled -> pass-through.
    if !tc_is_enabled() {
        set_torque_scale(1.0);
        return;
    }

    let (curve_loaded, calibrated) =
        set_tc_state(|s| (s.tire_curve.is_some(), s.vel_estimator.init));

    // 2. No valid curve -> pass-through.
    if !curve_loaded {
        set_torque_scale(1.0);
        return;
    }

    // 3. Calibration
    if !calibrated {
        let accel = imu_get_acceleration();
        set_tc_state(|s| {
            if let Ok(a) = accel {
                s.ax_sum += a.x;
                s.sample_count += 1;
            }
            if s.sample_count >= CALIB_SAMPLES {
                let avg = s.ax_sum / s.sample_count as f32;
                init_vel_estimator(&mut s.vel_estimator, avg);
            }
        });
        set_torque_scale(1.0);
        return;
    }

    // 4. Control step.
    let accel = imu_get_acceleration();
    let (omega_fl, omega_fr) = get_front_wheel_speeds();
    let motor_rpm = dti_get_rpm();

    let scale = set_tc_state(|s| {
        let TcState {
            tire_curve,
            vel_estimator,
            pi,
            ..
        } = s;
        let Some(curve) = tire_curve.as_ref() else {
            return 1.0;
        };

        let ax = match accel {
            Ok(a) => {
                vel_estimator.alpha = VEL_ALPHA;
                a.x
            }
            Err(_) => {
                defmt::error!("TC: IMU read failed, using wheel speed only");
                // alpha = 1.0 -> velocity entirely from wheel speeds this cycle.
                vel_estimator.alpha = 1.0;
                0.0
            }
        };

        let avg_front_rads = (omega_fl + omega_fr) / 2.0;
        let vx_car = estimate_velocity(vel_estimator, avg_front_rads, ax, dt);
        let slip = calc_slip(motor_rpm, vx_car);
        update_pi(pi, curve, slip, dt)
    });

    set_torque_scale(scale);
}
