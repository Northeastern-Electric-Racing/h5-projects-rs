//! Shared plumbing for subsystem "jobs" -- the units of work that trigger conversions and pull
//! register data into a cache.

use embassy_sync::blocking_mutex::{Mutex, raw::ThreadModeRawMutex};
use embassy_time::{Duration, Instant};

/// Diagnostic data for a job, mainly just used to track timing stuff (i.e., how long SPI reads and cache updates take).
#[derive(Copy, Clone)]
pub struct JobDiagnostics {
    /// How long the job took to run the last time it was ran.
    last_job_duration: Duration,
    /// The maximum time it took the job to run recorded so far.
    max_job_duration: Duration,
    /// The minimum time it took the job to run recorded so far.
    min_job_duration: Duration,
    /// Times this job was unable to run to completion due to an error.
    error_count: usize,
}
impl JobDiagnostics {
    /// Initializes the JobDiagnostics to its defaults. Meant to be called only once at init time.
    pub(crate) const fn new() -> Self {
        Self {
            last_job_duration: Duration::MIN,
            max_job_duration: Duration::MIN,
            min_job_duration: Duration::MAX,
            error_count: 0,
        }
    }

    /// How long the job took to run the last time it was ran.
    pub const fn last_job_duration(&self) -> Duration {
        self.last_job_duration
    }
    /// The maximum time it took the job to run recorded so far.
    pub const fn max_job_duration(&self) -> Duration {
        self.max_job_duration
    }
    /// The minimum time it took the job to run recorded so far.
    pub const fn min_job_duration(&self) -> Duration {
        self.min_job_duration
    }
    /// Times this job was unable to run to completion due to an error.
    pub const fn error_count(&self) -> usize {
        self.error_count
    }
}

/// Holds a job's [`JobDiagnostics`] across runs.
///
/// Meant to live in a function-local `static` inside the job it belongs to. Call
/// [`start`](Self::start) at the top of the job to begin a run.
pub struct JobDiagnosticsContainer {
    inner: Mutex<ThreadModeRawMutex, JobDiagnostics>,
}
impl JobDiagnosticsContainer {
    /// Initializes the JobDiagnostics to its defaults. Meant to be called only once at init time.
    pub(crate) const fn new() -> Self {
        Self { inner: Mutex::new(JobDiagnostics::new()) }
    }

    /// Updates the JobDiagnostics with new data after a successful job run has completed.
    /// This should be called right at the end of the job when you are about to return (i.e., where it is no longer possible for errors to occur and the job is known to have been successful).
    ///
    /// ### Parameters
    /// - `start_time`: The instant at which the job started. The caller should save this at the top of their job function. This function will then grab the current `Instant::now()` as `end_time` and calculate the duration the job took.
    pub(crate) fn update_with_successful_run(&self, start_time: Instant) {
        let end_time = Instant::now();
        let job_duration: Duration = end_time.saturating_duration_since(start_time);

        // SAFETY: this call doesn't nest calls to another lock or lock_mut closure. The closure
        // below only touches plain `Duration`/`usize` fields and calls nothing, so it cannot
        // re-enter this mutex. Keep it that way: never let caller-supplied code inside here, or
        // the non-reentrancy obligation moves to whoever writes that code.
        unsafe {
            self.inner.lock_mut(|data| {
                data.last_job_duration = job_duration;

                if data.last_job_duration > data.max_job_duration {
                    data.max_job_duration = data.last_job_duration;
                }

                if data.last_job_duration < data.min_job_duration {
                    data.min_job_duration = data.last_job_duration;
                }
            });
        }
    }

    /// Updates the JobDiagnostics after a failure has occured.
    /// This should be called whenever a job has to return early due to a failure. This doesn't record any duration metrics, and just increments the error count.
    /// This doesn't record the specific error or anything, so it is up to the job to do any more specific logging via defmt and such. JobDiagnostics mainly just keeps a log
    /// to ensure the history isn't lost.
    pub(crate) fn update_with_failure(&self) {
        // SAFETY: see `update_with_successful_run`. Same reasoning: a leaf closure over plain
        // fields, so no reentrancy is possible.
        unsafe {
            self.inner.lock_mut(|data| {
                data.error_count += 1;
            })
        }
    }

    /// Starts a run of this job, returning a guard that records the outcome.
    ///
    /// Hold it for the body of the job. Ending with [`JobRun::finish`] records the duration;
    /// dropping it any other way -- which is what `?` does on an early return -- records a
    /// failure instead.
    pub fn start(&self) -> JobRun<'_> {
        JobRun { container: self, start: Instant::now(), finished: false }
    }

    /// Copies out the current inner `JobDiagnostics`.
    pub(crate) fn copy_inner(&self) -> JobDiagnostics {
        self.inner.lock(|inner| *inner)
    }
}

/// A job in progress. Records the outcome when it goes out of scope.
///
/// This is how a job gets timed without a macro wrapping its body: take one at the top, end with
/// [`JobRun::finish`], and let `?` handle the failure paths. An early return drops the guard,
/// which counts the failure.
#[must_use = "a JobRun that is dropped without finish() records the job as failed"]
pub struct JobRun<'a> {
    container: &'a JobDiagnosticsContainer,
    start: Instant,
    finished: bool,
}

impl JobRun<'_> {
    /// Records a successful run and returns the job's diagnostics.
    pub fn finish(mut self) -> JobDiagnostics {
        self.container.update_with_successful_run(self.start);
        self.finished = true;
        self.container.copy_inner()
    }
}

impl Drop for JobRun<'_> {
    fn drop(&mut self) {
        // `finish` already recorded a success, so only an early exit reaches this.
        if !self.finished {
            self.container.update_with_failure();
        }
    }
}

/// Publishes a job's [`JobDiagnostics`] to `defmt_monitor` under `"<Subsystem>/JobDiagnostics/<job>()/<field>"`.
macro_rules! log_job_diagnostics {
    ($subsystem:literal, $job:literal, $diagnostics:expr) => {{
        let diagnostics = &$diagnostics;
        ::defmt_monitor::monitor!([$subsystem, "/JobDiagnostics/", $job, "()/last_job_duration"], desc = "Time (in milliseconds) it took for this job to run the last time it ran.", "{=u64}", diagnostics.last_job_duration().as_millis());
        ::defmt_monitor::monitor!([$subsystem, "/JobDiagnostics/", $job, "()/max_job_duration"], desc = "Maximum time (in milliseconds) we have observed this job taking to run so far.", "{=u64}", diagnostics.max_job_duration().as_millis());
        ::defmt_monitor::monitor!([$subsystem, "/JobDiagnostics/", $job, "()/min_job_duration"], desc = "Minimum time (in milliseconds) we have observed this job taking to run so far.", "{=u64}", diagnostics.min_job_duration().as_millis());
        ::defmt_monitor::monitor!([$subsystem, "/JobDiagnostics/", $job, "()/error_count"], desc = "Number of times this job has had to return early due to an error.", "{=usize}", diagnostics.error_count());
    }};
}

pub(crate) use log_job_diagnostics;
