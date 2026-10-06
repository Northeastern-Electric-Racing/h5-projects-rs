//! Wheel speed sensing. Ported from `u_wheel_speed.c` in the C firmware.
//!
//! Each wheel's hall effect sensor clocks a hardware timer in external clock mode 1, so every
//! pulse increments the timer's counter with no CPU involvement (no interrupts, no DMA). The task
//! reads both counters every [`SAMPLE_PERIOD`], averages over a rolling window, and zeroes the speed
//! if no pulse arrives within [`ZERO_TIMEOUT`].
//!
//! | Wheel | Timer | Pin                     |
//! |-------|-------|-------------------------|
//! | Left  | TIM1  | PE9 (TIM1_CH1, AF1)     |
//! | Right | TIM15 | PC12 (TIM15_CH1, AF2)   |

use crate::analog_sensor::SensorData;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{AfType, Flex, Pin, Pull};
use embassy_stm32::pac::timer::vals::CcmrInputCcs;
use embassy_stm32::peripherals::{PC12, PE9, TIM1, TIM15};
use embassy_stm32::timer::GeneralInstance2Channel;
use embassy_stm32::timer::low_level::{FilterValue, SlaveMode, Timer, TriggerSource};
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};

/// 50 Hz, matching the analog sensors.
const SAMPLE_PERIOD: Duration = Duration::from_millis(20);
/// Speed is forced to zero after this long without a pulse.
const ZERO_TIMEOUT: Duration = Duration::from_millis(150);
/// Number of samples in the rolling average (~100 ms at [`SAMPLE_PERIOD`]).
const WINDOW_SAMPLES: usize = 5;

const PULSES_PER_ROTATION: f32 = 60.0;
/// Wheel radius from the C firmware ("8-inch wheel radius"). Check against the actual tyre.
const WHEEL_RADIUS_M: f32 = 0.2032;
const WHEEL_CIRCUMFERENCE_M: f32 = 2.0 * core::f32::consts::PI * WHEEL_RADIUS_M;
/// RPM × circumference (m) × this = MPH (60 min/h, 1609.344 m/mile).
const RPM_TO_MPH: f32 = 60.0 / 1609.344;

/// Alternate function numbers, from the C firmware's `HAL_TIM_Base_MspInit`.
const LEFT_PIN_AF: u8 = 1; // PE9  -> TIM1_CH1
const RIGHT_PIN_AF: u8 = 2; // PC12 -> TIM15_CH1

pub const NUM_WHEELS: usize = 2;

/// Latest wheel speeds, indexed by [`Wheel`].
pub static WHEEL_SPEED_DATA: SensorData<WheelSpeedReading, NUM_WHEELS> = Watch::new();

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Wheel {
    Left = 0,
    Right = 1,
}

impl Wheel {
    pub const ALL: [Wheel; NUM_WHEELS] = [Wheel::Left, Wheel::Right];

    /// Fixed-width name so log rows line up.
    const fn name(self) -> &'static str {
        match self {
            Wheel::Left => "left ",
            Wheel::Right => "right",
        }
    }
}

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct WheelSpeedReading {
    pub rpm: f32,
    pub mph: f32,
}

/// Peripherals owned by [`wheel_speed_task`].
pub struct WheelSpeedResources {
    pub left_timer: Peri<'static, TIM1>,
    pub left_pin: Peri<'static, PE9>,
    pub right_timer: Peri<'static, TIM15>,
    pub right_pin: Peri<'static, PC12>,
}

/// A timer counting rising edges on its channel 1 input.
struct PulseCounter<'d, T: GeneralInstance2Channel> {
    timer: Timer<'d, T>,
    _pin: Flex<'d>,
    previous: u16,
}

impl<'d, T: GeneralInstance2Channel> PulseCounter<'d, T> {
    /// Sets the timer up like the C firmware's `MX_TIMx_Init`: external clock mode 1 from TI1FP1,
    /// rising edge, input filter fDTS/2 N=6, free-running 16-bit counter.
    fn new(tim: Peri<'d, T>, pin: Peri<'d, impl Pin>, af_num: u8) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_af_unchecked(af_num, AfType::input(Pull::None));

        let timer = Timer::new(tim);
        let regs = timer.regs_2ch();

        // CH1 as input from TI1, filtered; rising edge.
        regs.ccmr_input(0).modify(|w| {
            w.set_ccs(0, CcmrInputCcs::Ti4);
            w.set_icf(0, FilterValue::FdtsDiv2N6);
        });
        regs.ccer().modify(|w| {
            w.set_ccp(0, false);
            w.set_ccnp(0, false);
        });

        // Count edges of TI1FP1 instead of the internal clock. Trigger source before mode.
        regs.smcr().modify(|w| w.set_ts(TriggerSource::Ti1fp1));
        regs.smcr().modify(|w| w.set_sms(SlaveMode::ExtClockMode));

        regs.arr().write(|w| w.set_arr(u16::MAX));
        timer.reset();
        timer.start();

        Self { timer, _pin: pin, previous: 0 }
    }

    /// Pulses counted since the last call. The 16-bit counter wraps, so subtract with wrap-around.
    fn take_pulses(&mut self) -> u16 {
        let current = self.timer.regs_2ch().cnt().read().cnt();
        let pulses = current.wrapping_sub(self.previous);
        self.previous = current;
        pulses
    }
}

/// Rolling pulse-count window for one wheel.
struct PulseWindow {
    pulse_counts: [u16; WINDOW_SAMPLES],
    sample_times_ms: [u32; WINDOW_SAMPLES],
    total_pulses: u32,
    total_time_ms: u32,
    next_index: usize,
    sample_count: usize,
}

impl PulseWindow {
    const EMPTY: Self = Self {
        pulse_counts: [0; WINDOW_SAMPLES],
        sample_times_ms: [0; WINDOW_SAMPLES],
        total_pulses: 0,
        total_time_ms: 0,
        next_index: 0,
        sample_count: 0,
    };

    /// Adds a sample, dropping the oldest once the window is full.
    fn push(&mut self, pulses: u16, elapsed_ms: u32) {
        let i = self.next_index;
        if self.sample_count >= WINDOW_SAMPLES {
            self.total_pulses -= u32::from(self.pulse_counts[i]);
            self.total_time_ms -= self.sample_times_ms[i];
        } else {
            self.sample_count += 1;
        }

        self.pulse_counts[i] = pulses;
        self.sample_times_ms[i] = elapsed_ms;
        self.total_pulses += u32::from(pulses);
        self.total_time_ms += elapsed_ms;

        self.next_index = (i + 1) % WINDOW_SAMPLES;
    }
}

/// Per-wheel state: the rolling window, when a pulse was last seen, and the latest speed.
struct WheelState {
    window: PulseWindow,
    last_pulse: Instant,
    reading: WheelSpeedReading,
}

impl WheelState {
    fn new(now: Instant) -> Self {
        Self { window: PulseWindow::EMPTY, last_pulse: now, reading: WheelSpeedReading::default() }
    }

    /// Port of the C `process_wheel_sample`.
    fn update(&mut self, pulses: u16, elapsed_ms: u32, now: Instant) {
        self.window.push(pulses, elapsed_ms);

        if pulses > 0 {
            self.last_pulse = now;
        }

        if self.window.total_pulses > 0 && self.window.total_time_ms > 0 {
            let frequency_hz = self.window.total_pulses as f32 * 1000.0 / self.window.total_time_ms as f32;
            let rpm = frequency_hz * 60.0 / PULSES_PER_ROTATION;
            self.reading = WheelSpeedReading { rpm, mph: rpm * WHEEL_CIRCUMFERENCE_M * RPM_TO_MPH };
        }

        if now - self.last_pulse >= ZERO_TIMEOUT && (self.reading.rpm > 0.0 || self.window.total_pulses > 0) {
            self.reading = WheelSpeedReading::default();
            // Discard old samples before the wheel starts again.
            self.window = PulseWindow::EMPTY;
        }
    }
}

#[embassy_executor::task]
pub async fn wheel_speed_task(r: WheelSpeedResources) {
    let mut left = PulseCounter::new(r.left_timer, r.left_pin, LEFT_PIN_AF);
    let mut right = PulseCounter::new(r.right_timer, r.right_pin, RIGHT_PIN_AF);

    let sender = WHEEL_SPEED_DATA.sender();
    let mut previous_sample = Instant::now();
    let mut wheels = [WheelState::new(previous_sample), WheelState::new(previous_sample)];
    let mut ticker = Ticker::every(SAMPLE_PERIOD);

    loop {
        ticker.next().await;

        let now = Instant::now();
        let elapsed_ms = (now - previous_sample).as_millis() as u32;
        previous_sample = now;

        let pulses = [left.take_pulses(), right.take_pulses()];
        for wheel in Wheel::ALL {
            wheels[wheel as usize].update(pulses[wheel as usize], elapsed_ms, now);
        }

        let readings = Wheel::ALL.map(|wheel| wheels[wheel as usize].reading);
        for wheel in Wheel::ALL {
            let r = readings[wheel as usize];
            defmt::info!("Wheel speed {=str}: rpm={=f32} mph={=f32}", wheel.name(), r.rpm, r.mph);
        }
        sender.send(readings);
    }
}
