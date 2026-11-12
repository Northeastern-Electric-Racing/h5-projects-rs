/// Various helper types that can be used throughout this project.

pub mod deadline;
pub mod debouncer;
pub mod moving_average;
pub mod snapshot_cell;
pub mod startup_cell;
pub mod hysteresis;

/// Helpers that aren't cool enough for their own module.
mod misc {
    use embassy_time::Duration;

    /// Helper that converts `duration` to a "lapses of `task_frequency`" value. Will return `None` if `duration` isn't an exact
    /// multiple of `task_frequency` or if either isn't a whole number of milliseconds. So, it is recommended to `.unwrap()` or `.expect()` this
    /// call inside a `const{}` block for compile-time error checking purposes.
    pub const fn duration_to_task_cycles(task_frequency: Duration, duration: Duration) -> Option<usize> {
        // Converts `duration` into a whole number of milliseconds. If `duration` cannot be represented in a whole number of milliseconds, this returns `None`.
        const fn whole_millis(duration: Duration) -> Option<u64> {
            let millis = duration.as_millis();
            if Duration::from_millis(millis).as_ticks() == duration.as_ticks() { Some(millis) } else { None }
        }

        let (Some(duration_ms), Some(period_ms)) = (whole_millis(duration), whole_millis(task_frequency)) else {
            return None;
        };

        match duration_ms.checked_rem(period_ms) {
            Some(0) => Some((duration_ms / period_ms) as usize),
            _ => None,
        }
    }
}
pub use misc::*;
