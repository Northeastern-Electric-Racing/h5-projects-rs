//! Free-running counter for the microsecond waits the ADBMS parts need between transactions.
//!
//! `embassy_time` runs at 32.768 kHz here, so its finest wait is 30.5 us. This is C's
//! `delay_us_2950` -- spin on a free-running timer -- with one shared counter instead of C's four,
//! and the rate read from the clock tree instead of a hardcoded prescaler.

use embassy_stm32::Peri;
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::timer::low_level::Timer;
use static_cell::StaticCell;

static TIMER: StaticCell<Timer<'static, TIM2>> = StaticCell::new();

/// A blocking delay backed by the counter [`init`] starts.
///
/// `Copy`, and it only reads the counter, so one value serves every line and SPI device.
#[derive(Copy, Clone)]
pub struct TimerDelay {
    timer: &'static Timer<'static, TIM2>,
    ticks_per_us: u32,
}

/// Claims TIM2 and starts it free-running. Call once; panics if called twice.
///
/// TIM2 rather than C's TIM5, which embassy's time driver already took. Unprescaled, so the
/// unavoidable +/-1 tick sampling error is nanoseconds rather than a microsecond.
pub fn init(tim: Peri<'static, TIM2>) -> TimerDelay {
    let timer = TIMER.init(Timer::new(tim));

    timer.regs_gp32().psc().write_value(0);
    timer.regs_gp32().arr().write_value(u32::MAX);
    // PSC and ARR are preloaded; without this the timer keeps its old values.
    timer.generate_update_event();
    timer.start();

    let ticks_per_us = timer.get_clock_frequency().0 / 1_000_000;

    TimerDelay { timer, ticks_per_us }
}

impl TimerDelay {
    /// Spins for at least `ns`. Rounds up, and the wrapping subtraction stays correct across the
    /// counter's ~25 s lap -- which no delay this can be asked for comes close to.
    fn spin_ns(&self, ns: u32) {
        let ticks = (u64::from(ns) * u64::from(self.ticks_per_us)).div_ceil(1_000) as u32;

        let start = self.timer.get_counter();
        while self.timer.get_counter().wrapping_sub(start) < ticks {}
    }
}

/// Blocking: what the ADBMS drivers' `BusIdle` is built on.
impl embedded_hal::delay::DelayNs for TimerDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.spin_ns(ns);
    }
}

/// Longest wait still worth spinning: one tick of the async timer, below which `embassy_time`
/// cannot represent the wait anyway.
const SPIN_LIMIT_NS: u32 = (1_000_000_000 / embassy_time::TICK_HZ) as u32;

/// Async, for `ExclusiveDevice`'s `Operation::DelayNs`.
///
/// Yields past [`SPIN_LIMIT_NS`]: the 6830 pulses 500 us *per chip*, which would otherwise hold
/// the executor for milliseconds.
impl embedded_hal_async::delay::DelayNs for TimerDelay {
    async fn delay_ns(&mut self, ns: u32) {
        if ns < SPIN_LIMIT_NS {
            self.spin_ns(ns);
        } else {
            embassy_time::Timer::after_nanos(u64::from(ns)).await;
        }
    }
}
