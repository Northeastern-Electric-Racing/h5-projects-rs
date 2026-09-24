use strum::{EnumCount, VariantArray, EnumIter, EnumIs};
use embassy_time::{Duration, Instant, Timer};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::Channel,
};

mod ids {
    use super::*;

    #[derive(EnumIs)]
    #[derive(Copy, Clone)]
    pub enum FaultSeverity {
        Critical,
        NonCritical,
    }

    /// Const config metadata for a fault.
    pub struct FaultConfig {
        timeout: Duration,
        severity: FaultSeverity,
    }
    impl FaultConfig {
        /// How long a fault should stay active before expiring.
        pub const fn timeout(&self) -> Duration { self.timeout }
        /// The severity of a fault.
        pub const fn severity(&self) -> FaultSeverity { self.severity }
    }
    
    #[derive(EnumCount, VariantArray, EnumIter)]
    #[derive(Copy, Clone)]
    #[repr(u32)]
    pub enum FaultId {
        DischargeLimitEnforcementFault,
        ChargeLimitEnforcement,
        CellVoltageTooLow,
        CellVoltageTooHigh,
        CellChargeVoltageTooHigh,
        PackTooHot,
        DieTempMaximumFault,
        HvPlateCommsFault,
        SegmentCommsFault,
        CellOpenWireFault,
    }
    impl FaultId {
        /// Returns this FaultId's config settings.
        #[rustfmt::skip]
        pub const fn config(self) -> FaultConfig {
            // This function body is for defining the config settings for each fault.

            // using a match statement instead of a lookup table here because rust doesnt have designated initializers for arrays
            // but this should probably (?) compile into a lookup table anyway since there doesn't seem to be a reason not to
            match self {
                Self::DischargeLimitEnforcementFault => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::ChargeLimitEnforcement         => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::CellVoltageTooLow              => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::CellVoltageTooHigh             => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::CellChargeVoltageTooHigh       => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::PackTooHot                     => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::DieTempMaximumFault            => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
                Self::HvPlateCommsFault              => FaultConfig { timeout: Duration::from_millis(20_000), severity: FaultSeverity::Critical },
                Self::SegmentCommsFault              => FaultConfig { timeout: Duration::from_millis(20_000), severity: FaultSeverity::NonCritical },
                Self::CellOpenWireFault              => FaultConfig { timeout: Duration::from_millis(40_000), severity: FaultSeverity::Critical },
            }
        }

        /// Checks if this particular fault is configured to be critical or not.
        pub const fn is_critical(&self) -> bool { *&self.config().severity().is_critical() }

        /// Mask of all fault flags that are configured as critical.
        /// 
        /// This is computed at compile time and mainly exists so we can easily check if any critical faults are active via a simple bitwise &.
        pub const CRITICAL_MASK: u32 = {
            let mut mask = 0;
            let mut i = 0;

            // can't use iterators here because Rust doesn't support them in consts yet. sad!
            while i < FaultId::COUNT {
                if FaultId::VARIANTS[i].is_critical() {
                    mask |= 1 << i;
                }
                i += 1;
            }
            mask
        };
    }

    /// Lets you index by Fault ID.
    #[derive(Copy, Clone, Debug)]
    pub struct IndexByFaultId<T> {
        data: [T; FaultId::COUNT],
    }
    pub type FaultIds = core::iter::Copied<core::slice::Iter<'static, FaultId>>;
    pub type Iter<'borrow, T> = core::iter::Zip<FaultIds, core::slice::Iter<'borrow, T>>;
    pub type IterMut<'borrow, T> = core::iter::Zip<FaultIds, core::slice::IterMut<'borrow, T>>;
    pub type IntoIter<T> = core::iter::Zip<FaultIds, core::array::IntoIter<T, { FaultId::COUNT }>>;

    impl<T> IndexByFaultId<T> {
        /// Creates a new `IndexByFaultId` directly from an array.
        pub const fn new(data: [T; FaultId::COUNT]) -> Self {
            Self { data }
        }

        /// Retrives the data for `fault`.
        pub const fn get(&self, fault: FaultId) -> &T {
            let i: usize = fault as usize;
            &self.data[i]
        }

        /// Retrives the data for `fault`.
        ///
        /// This is literally just an alias for `.get()`. It may be more readable in large method chains.
        pub const fn fault(&self, fault: FaultId) -> &T {
            self.get(fault)
        }

        /// Allows you to mutate the inner value located at `fault`.
        pub fn set(&mut self, fault: FaultId, value: T) {
            let i: usize = fault as usize;
            self.data[i] = value;
        }

        /// Gets a mutable reference to the inner data at `fault`.
        pub fn get_mut(&mut self, fault: FaultId) -> &mut T {
            let i: usize = fault as usize;
            &mut self.data[i]
        }

        pub fn from_fn(mut f: impl FnMut(FaultId) -> T) -> Self {
            Self { data: core::array::from_fn(|i| f(FaultId::VARIANTS[i])) }
        }

        pub fn iter(&self) -> Iter<'_, T> {
            FaultId::VARIANTS.iter().copied().zip(self.data.iter())
        }

        pub fn iter_mut(&mut self) -> IterMut<'_, T> {
            FaultId::VARIANTS.iter().copied().zip(self.data.iter_mut())
        }

        /// Converts this back into its inner array.
        pub fn into_array(self) -> [T; FaultId::COUNT] {
            self.data
        }
    }
}
pub use ids::*;

/// Wrapper around an AtomicU32 that stores the fault flags.
struct FaultFlags {
    flags: AtomicU32,
}
impl FaultFlags {
    /// Creates a new fault flags, where all flags are unset.
    pub const fn new() -> Self { Self { flags: AtomicU32::new(0) } }

    /// Gets the status of all faults.
    pub fn get_all(&self) -> IndexByFaultId<bool> {
        let flags = self.flags.load(Ordering::Relaxed);

        IndexByFaultId::from_fn(|fault| {
            flags & (1 << fault as u32) != 0
        })
    }

    /// Checks whether or not a particular fault flag is set.
    pub fn is_set(&self, fault: FaultId) -> bool { self.flags.load(Ordering::Relaxed) & (1 << fault as u32) != 0 }

    /// Sets the flag for a fault.
    pub fn set_fault(&self, fault: FaultId) { self.flags.fetch_or(1 << fault as u32, Ordering::Relaxed); }

    /// Clears the flag for a fault.
    pub fn clear_fault(&self, fault: FaultId) { self.flags.fetch_and(!(1 << fault as u32), Ordering::Relaxed); }

    /// Checks if any critical faults are currently active.
    pub fn are_critical_faults_active(&self) -> bool { self.flags.load(Ordering::Relaxed) & FaultId::CRITICAL_MASK != 0 }
}

mod timers {
    use super::*;

    /// Current activation/expiration state for a fault and its timer.
    /// 
    /// PRIVATE! This tracks the internal 
    #[derive(Copy, Clone)]
    enum FaultTimerState {
        Inactive,
        Active { deadline: Instant, }
    }

    pub enum EvaluationResult {
        /// Timer has expired.
        Expired,
        /// Timer is not active.
        Inactive,
        /// Timer is actively counting down, but has not expired yet. `deadline` is the time when it is planned to expire.
        Active { deadline: Instant },
    }

    /// "Timer" for faults stuff.
    pub struct FaultTimer { state: FaultTimerState }
    impl FaultTimer {
        /// Creates a new inactive timer. Meant to be called at init time.
        pub const fn new() -> Self {
            Self { state: FaultTimerState::Inactive }
        }

        /// Evaluates the current state of the timer against `now`. `now` should be the current time (get via `Instant::now()`)
        pub fn evaluate(&self, now: Instant) -> EvaluationResult {
            match self.state {
                FaultTimerState::Inactive => EvaluationResult::Inactive,

                // If we are past the deadline, then the timer is expired.
                FaultTimerState::Active { deadline } if now >= deadline => EvaluationResult::Expired,

                // If we aren't past the deadline, the timer is still active.
                FaultTimerState::Active { deadline } => EvaluationResult::Active { deadline },
            }
        }

        /// Makes this timer inactive.
        pub fn set_inactive(&mut self) {
            self.state = FaultTimerState::Inactive;
        }

        /// Restarts this timer with a new `duration`. This timer will drop whatever its current state
        /// is, and enter the `Active` state with a expiration deadline of `now + duration`.
        pub fn restart(&mut self, duration: Duration) {
            let deadline= Instant::now().saturating_add(duration);
            self.state = FaultTimerState::Active { deadline };
        }
    }
}
use timers::*;

static FAULT_QUEUE: Channel<ThreadModeRawMutex, FaultId, 10> = Channel::new();
static FLAGS: FaultFlags = FaultFlags::new();

/// !!!! PUBLIC API !!!!
/// 
/// (This is the public API of the faults module).
mod api {
    use super::*;

    #[repr(u8)]
    pub enum FaultState {
        /// This fault is not current active (i.e., everything is normal for this fault).
        Inactive = 0,
        /// This fault is currently active (i.e., something bad happened that triggered this fault).
        Active = 1,
    }
    impl FaultState {
        /// Returns `true` if this is `FaultState::Active`.
        pub const fn is_active(&self) -> bool { matches!(self, FaultState::Active) }

        /// PRIVATE! Creates a `FaultState` based on a fault flag `bool`.
        const fn from_bool(flag: bool) -> Self { if flag { Self::Active } else { Self::Inactive } }
    }

    /// Gets the state of a particular fault.
    pub fn get_fault(fault: FaultId) -> FaultState {
        FaultState::from_bool(FLAGS.is_set(fault))
    }

    /// Gets the state of all faults.
    pub fn get_all_faults() -> IndexByFaultId<FaultState> {
        let faults = FLAGS.get_all();
        IndexByFaultId::from_fn(|fault| { FaultState::from_bool(*faults.fault(fault)) })
    }

    /// Checks if any critical faults are currently active.
    pub fn are_critical_faults_active() -> bool {
        FLAGS.are_critical_faults_active()
    }

    /// Adds a fault to the fault queue.
    /// 
    /// This is used when you want to trigger a fault.
    pub async fn queue(fault: FaultId) {
        match FAULT_QUEUE.try_send(fault) {
            Ok(()) => {},
            Err(_) => {
                defmt::warn!("Faults: Tried to queue a fault, but the faults queue was full. This is not an error, since this function will now .await until the queue is open. However, you should probably increase the size of the faults queue if this is getting printed a lot.");
                FAULT_QUEUE.send(fault).await;
            }
        }
    }

    /// Tries to add a fault to the fault queue.
    /// 
    /// If this returns `Err(_)`, then the faults queue was full and the fault couldn't be added. That is a sign to increase the capacity of the faults queue.
    pub fn try_queue(fault: FaultId) -> Result<(), ()> {
        // Throwing away the specific error in map_err is fine here since Err always means "channel was full". This is also what can.rs does
        FAULT_QUEUE.try_send(fault).map_err(|_| {()})
    }
}
pub use api::*;

pub mod task {
    use super::*;

    /// Guy in charge of the faults.
    struct FaultManager {
        timers: IndexByFaultId<FaultTimer>,
    }
    impl FaultManager {
        /// Initializes the faults manager.
        pub fn new() -> Self {
            Self {
                timers: IndexByFaultId::from_fn(|_| { FaultTimer::new() }),
            }
        }

        /// Triggers a fault.
        pub fn trigger_fault(&mut self, fault: FaultId) {
            FLAGS.set_fault(fault);
            self.timers.get_mut(fault).restart(fault.config().timeout());
        }
    }

    #[embassy_executor::task]
    pub async fn faults_task() -> ! {
        use embassy_futures::select::select;

        let mut manager = FaultManager::new();

        loop {
            // Dequeue faults from the faults queue and trigger them.
            while let Ok(fault) = FAULT_QUEUE.try_receive() {
                manager.trigger_fault(fault);
            }

            let now = Instant::now();

            // Stores the deadline of the soonest expiration.
            let mut soonest_expiration: Option<Instant> = None;

            // Check the state of each fault timer.
            for (fault, timer) in manager.timers.iter_mut() {
                match timer.evaluate(now) {
                    // This timer is inactive so we don't need to do anything.
                    EvaluationResult::Inactive => {},

                    // This timer has expired, so we can set it to Inactive and clear the associated fault.
                    EvaluationResult::Expired => {
                        timer.set_inactive();
                        FLAGS.clear_fault(fault);
                    },

                    // This timer is active, so we use it as part of our "soonest deadline" calculation (to see how long this task should sleep).
                    EvaluationResult::Active { deadline } => {
                        soonest_expiration = Some(match soonest_expiration {
                            // If no soonest_expiration exists yet, just use this timer's deadline.
                            None => deadline,

                            // If a soonest_expiration does exist, compare it to this timer's deadline and keep the sooner of the two
                            Some(soonest) => soonest.min(deadline),
                        });
                    }
                }
            }

            // u_TODO do stuff here probably:
            if FLAGS.are_critical_faults_active() {

            }

            // Sleep until more faults are queued, or a timer is ready to expire (whichever happens sooner). 
            // If there are no timers counting down, then just sleep until more faults are queued.
            match soonest_expiration {
                Some(deadline) => {
                    select(FAULT_QUEUE.ready_to_receive(), Timer::at(deadline)).await;
                },
                None => {
                    FAULT_QUEUE.ready_to_receive().await;
                }
            }
        }
    }
}
// u_TODO - we should probably have a separate task that reads the fault values for reporting over CAN and such, since we don't want any of that stuff to interfere with the timing and deadline stuff from the current faults task. maybe we also want a broadcast belonging to this internal faults task that can signal every time it runs