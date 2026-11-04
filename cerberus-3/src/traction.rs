use core::cell::{Cell, RefCell};
use core::f32::consts::PI;
use core::mem::size_of;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU32};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;

const TC_CURVE_MAGIC: u32 = 0x004E_4552;
const TC_CURVE_VERSION: u8 = 1;
const TC_CURVE_POINTS_MAX: usize = 200;
const TC_CALIB_SAMPLES: u16 = 50;
const TC_MIN_VX: f32 = 0.5;
const VEL_ALPHA: f32 = 0.98;
const TC_INTEGRAL_LIMIT: f32 = 1.0;
const INCHES_TO_MILES: f32 = 63360.0;
const SECONDS_TO_HOURS: f32 = 3600.0;
const G_IN_PER_S2: f32 = 386.09;
const TC_KP_DEFAULT: f32 = 1.5;
const TC_KI_DEFAULT: f32 = 0.2;
const TIRE_DIAMETER: f32 = 0.0; //TODO
const GEAR_RATIO: f32 = 0.0; //TODO

// ---------------------------------------------------------------------------
// Tire curve
// ---------------------------------------------------------------------------

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
    points: [TireCurvePoint; TC_CURVE_POINTS_MAX],
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
            kp: TC_KP_DEFAULT,
            ki: TC_KI_DEFAULT,
            integral: 0.0,
        }
    }
}

struct TcState {
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

static TC_STATE: Mutex<CriticalSectionRawMutex, RefCell<TcState>> =
    Mutex::new(RefCell::new(TcState::new()));
/// f32 bits of the torque scale (1.0 = 0x3F80_0000). Written only by `tc_process`.
static TORQUE_SCALE: AtomicU32 = AtomicU32::new(0x3F80_0000);
/// Set by the state machine CAN message; read by the pedal task and `tc_process`.
static TC_ENABLED: AtomicBool = AtomicBool::new(false);
/// (omega_fl, omega_fr) in rad/s
static FRONT_WHEEL_SPEEDS: Mutex<CriticalSectionRawMutex, Cell<(f32, f32)>> =
    Mutex::new(Cell::new((0.0, 0.0)));

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

    if magic != TC_CURVE_MAGIC {
        defmt::error!("TC: bad curve magic 0x{:08x}", magic);
        return None;
    }
    if version != TC_CURVE_VERSION {
        defmt::error!("TC: unsupported curve version {}", version);
        return None;
    }
    if (num_points as usize) > TC_CURVE_POINTS_MAX || num_points < 2 {
        defmt::error!("TC: invalid num_points {}", num_points);
        return None;
    }

    Some(curve)
}

fn calc_slip(motor_rpm: f32, vx_car: f32) -> f32 {
    let wheel_rps = motor_rpm / GEAR_RATIO / 60.0;
    let v_rear = wheel_rps * PI * TIRE_DIAMETER * SECONDS_TO_HOURS / INCHES_TO_MILES;

    let denom = vx_car.abs().max(v_rear.abs());
    if denom < TC_MIN_VX {
        return 0.0;
    }
    (v_rear - vx_car) / denom
}

fn rpm_to_rads(rpm: i16) -> f32 {
    rpm as f32 * 2.0 * PI / 60.0
}

fn lookup_fx(curve: &TireCurve, slip: f32) -> f32 {
    let n = (curve.num_points as usize).min(TC_CURVE_POINTS_MAX);
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

/// Velocity in mph
fn estimate_velocity(est: &mut VelEstimator, avg_front_rads: f32, ax: f32, dt: f32) -> f32 {
    let vx_wheel = avg_front_rads * (TIRE_DIAMETER / 2.0) * SECONDS_TO_HOURS / INCHES_TO_MILES;
    let ax_corrected = ax - est.ax_bias;
    let vx_imu =
        est.v_x + ax_corrected * (G_IN_PER_S2 / 1000.0) * dt * SECONDS_TO_HOURS / INCHES_TO_MILES;
    let v = est.alpha * vx_wheel + (1.0 - est.alpha) * vx_imu;
    est.v_x = if v > 0.0 { v } else { 0.0 };
    est.v_x
}

fn update_pi(pi: &mut TcPi, curve: &TireCurve, slip: f32, dt: f32) -> f32 {
    let slip_ref = curve.peak_lambda * 0.98;
    let error = slip_ref - slip;

    pi.integral = (pi.integral + error * dt).clamp(-TC_INTEGRAL_LIMIT, TC_INTEGRAL_LIMIT);

    let fx_ff = lookup_fx(curve, slip_ref);
    (fx_ff + pi.kp * error + pi.ki * pi.integral).clamp(0.0, 1.0)
}
