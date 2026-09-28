//! Tracks how much time the thread-mode executor spends asleep in `wfe`,
//! using embassy_executor's `trace` hooks, to get a rough sleep/wfe duty cycle.

use embassy_executor::ExecutorId;
use embassy_executor::raw::TaskRef;
use embassy_executor::raw::trace::Trace;
use embassy_time::Instant;
use portable_atomic::{AtomicU64, Ordering};

const NOT_IDLE: u64 = u64::MAX;

static IDLE_SINCE: AtomicU64 = AtomicU64::new(NOT_IDLE);
static SLEEP_TICKS: AtomicU64 = AtomicU64::new(0);

pub struct IdleTracker;

impl Trace for IdleTracker {
    fn idle() {
        IDLE_SINCE.store(Instant::now().as_ticks(), Ordering::Relaxed);
    }

    fn poll_start(_executor: ExecutorId) {
        let idle_since = IDLE_SINCE.swap(NOT_IDLE, Ordering::Relaxed);
        if idle_since != NOT_IDLE {
            let now = Instant::now().as_ticks();
            SLEEP_TICKS.fetch_add(now.saturating_sub(idle_since), Ordering::Relaxed);
        }
    }

    fn task_new(_executor: ExecutorId, _task: TaskRef) {}
    fn task_end(_executor: ExecutorId, _task: TaskRef) {}
    fn task_exec_begin(_executor: ExecutorId, _task: TaskRef) {}
    fn task_exec_end(_executor: ExecutorId, _task: TaskRef) {}
    fn task_ready_begin(_executor: ExecutorId, _task: TaskRef) {}
    fn executor_idle(_executor: ExecutorId) {}
    fn task_name_set(_task: TaskRef, _name: &'static str) {}
    fn task_priority_set(_task: TaskRef, _priority: u8) {}
    fn task_deadline_set(_task: TaskRef, _deadline: u64) {}
}

embassy_executor::trace_impl!(IdleTracker);

/// Rough percentage of wall-clock time since boot spent asleep in `wfe`.
pub fn sleep_percent() -> f32 {
    let sleep = SLEEP_TICKS.load(Ordering::Relaxed) as f32;
    let total = Instant::now().as_ticks() as f32;
    if total == 0.0 { 0.0 } else { 100.0 * sleep / total }
}
