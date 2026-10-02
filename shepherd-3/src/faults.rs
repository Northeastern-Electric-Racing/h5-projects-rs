use strum::{EnumCount, VariantArray, EnumIter, EnumIs, IntoEnumIterator};
use embassy_time::{Duration, Instant, Timer};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};

mod system {
    use super::*;

    #[derive(EnumIs)]
    #[derive(Copy, Clone)]
    pub enum FaultSeverity {
        Critical,
        NonCritical,
    }

    /// Const config metadata for a fault.
    pub struct FaultConfig {
        severity: FaultSeverity,
        kind: FaultKind,
    }
    impl FaultConfig {
        /// If this fault is critical or noncritical.
        pub const fn severity(&self) -> FaultSeverity { self.severity }
        /// The kind of fault this is.
        pub const fn kind(&self) -> &FaultKind { &self.kind }
    }

    mod kind {
        use embassy_time::Duration;

        #[derive(Copy, Clone, defmt::Format)]
        pub enum FaultKind {
            Automatic{
                /// How long it should take for the fault to
                /// automatically clear after being triggered.
                timeout: Duration 
            },

            /// This is for faults that are triggered based on a periodic test or check.
            /// Basically, every time the fault caller runs the check, it will update the
            PassFail{
                /// The number of consecutive `bads` required to trigger
                /// a fault. Think of this like a debounce for triggering
                /// the fault (i.e., "we need to make sure this test fails two times in a row
                /// before we actually latch the fault, just to make sure the first time wasn't a fluke").
                /// 
                /// If you set this to zero, the fault will trigger immediately upon the first `bad`.
                /// If you set this to 1, the fault will trigger after two `bads` in a row.
                consecutive_bads: usize,
                /// The number of consecutive `okays` required to
                /// clear an active fault. Basically just `consecutive_bads`, but for clearing
                /// a fault. If you want the fault to clear immediately upon a `okay`, set this to zero.
                consecutive_okays: usize,
            }
        }

        pub trait Kind { type Action; }
        pub struct Automatic;
        pub struct PassFail;
        impl Kind for Automatic { type Action = super::AutomaticAction; }
        impl Kind for PassFail  { type Action = super::PassFailAction; }
    }
    pub use kind::*;

    mod actions {
        /// Actions that the user can tell the fault manager to take.
        #[derive(Copy, Clone, defmt::Format)]
        pub enum FaultAction { AutomaticAction(AutomaticAction), PassFailAction(PassFailAction) }
        
        /// Actions for a `PassFail` fault.
        #[derive(Copy, Clone, defmt::Format)]
        pub enum PassFailAction {
            /// Tell the faults manager that the check passed/the fault condition is okay.
            NotifyOkay,
            /// Tell the faults manager that the check is "bad"/the fault condition is past the "not good" threshold.
            NotifyBad,
        }
        impl From<PassFailAction> for FaultAction { fn from(action: PassFailAction) -> Self { Self::PassFailAction(action) } }

        /// Actions for a `Automatic` fault.
        #[derive(Copy, Clone, defmt::Format)]
        pub enum AutomaticAction {
            /// Tell the fault manager to trigger the fault. This will
            /// start that fault's timer.
            Trigger,
        }
        impl From<AutomaticAction> for FaultAction { fn from(action: AutomaticAction) -> Self { Self::AutomaticAction(action) } }
    }
    pub use actions::*;

    /// Macro to define faults.
    macro_rules! define_faults {
        ($( $name:ident => $severity:ident, $kind:ident { $($field:ident: $value:expr),* $(,)? } ),* $(,)?) => {
            
            #[derive(EnumCount, VariantArray, EnumIter, defmt::Format, Copy, Clone)]
            #[repr(u32)]
            pub enum FaultId { $($name,)* }

            /// Command used to specify your request to the faults manager.
            /// 
            /// This enum maintains the same variants as `FaultId`, but with inners that let you
            /// describe a command or action on behalf of the `FaultId`.
            #[derive(Copy, Clone, defmt::Format)]
            pub enum FaultCommand { $(
                #[doc = concat!("Command for the `", stringify!($name), "` fault.")]
                $name(<kind::$kind as kind::Kind>::Action),
            )* }

            impl FaultCommand {
                /// The fault this message is for.
                pub const fn id(self) -> FaultId { match self { $(Self::$name(_) => FaultId::$name,)* } }

                /// The action this message is requesting.
                pub fn action(self) -> FaultAction { match self { $(Self::$name(action) => action.into(),)* }}
            }

            impl FaultId {
                /// Returns this FaultId's config settings.
                pub const fn config(self) -> FaultConfig {
                    match self {
                        $(Self::$name => FaultConfig {
                            severity: FaultSeverity::$severity,
                            kind: FaultKind::$kind { $($field: $value),* },
                        },)*
                    }
                }

                /// Checks if this particular fault is configured to be critical or not.
                pub const fn is_critical(&self) -> bool {
                    *&self.config().severity().is_critical()
                }

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
        };
    }

    mod index {
        use super::*;
        use strum::{EnumCount, VariantArray};

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
            pub fn get(&self, fault: FaultId) -> &T {
                let i: usize = fault as usize;
                &self.data[i]
            }

            /// Retrives the data for `fault`.
            ///
            /// This is literally just an alias for `.get()`. It may be more readable in large method chains.
            pub fn fault(&self, fault: FaultId) -> &T {
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

            /// Converts this back into its inner array.
            pub fn into_array(self) -> [T; FaultId::COUNT] {
                self.data
            }

            pub fn iter(&self) -> Iter<'_, T> {
                FaultId::VARIANTS.iter().copied().zip(self.data.iter())
            }

            pub fn iter_mut(&mut self) -> IterMut<'_, T> {
                FaultId::VARIANTS.iter().copied().zip(self.data.iter_mut())
            }
        }

        impl<T> core::ops::Index<FaultId> for IndexByFaultId<T> {
            type Output = T;

            fn index(&self, index: FaultId) -> &Self::Output {
                &self.get(index)
            }
        }

        impl<T> core::ops::IndexMut<FaultId> for IndexByFaultId<T> {
            fn index_mut(&mut self, index: FaultId) -> &mut Self::Output {
                let i: usize = index as usize;
                &mut self.data[i]
            }
        }
    }
    pub use index::*;

    // CONFIGURE FAULTS HERE!!!
    #[rustfmt::skip]
    define_faults! {
        DischargeLimitEnforcementFault => Critical, Automatic { timeout: Duration::from_secs(5) },
        ChargeLimitEnforcement         => Critical, Automatic { timeout: Duration::from_secs(5) },
        CellVoltageTooLow              => Critical, Automatic { timeout: Duration::from_secs(5) },
        CellVoltageTooHigh             => Critical, Automatic { timeout: Duration::from_secs(5) },
        CellChargeVoltageTooHigh       => Critical, Automatic { timeout: Duration::from_secs(5) },
        PackTooHot                     => Critical, Automatic { timeout: Duration::from_secs(5) },
        DieTempMaximumFault            => Critical, Automatic { timeout: Duration::from_secs(5) },
        HvPlateCommsFault              => Critical, Automatic { timeout: Duration::from_secs(5) },
        SegmentCommsFault              => Critical, Automatic { timeout: Duration::from_secs(5) },
        CellOpenWireFault              => Critical, PassFail  { consecutive_bads: 1, consecutive_okays: 0 },
        FakeFault1                     => Critical, Automatic { timeout: Duration::from_millis(1000) },
        FakeFault2                     => Critical, Automatic { timeout: Duration::from_millis(2500) },
    }
}
pub use system::*;

/// Wrapper around an AtomicU32 that stores the fault flags.
struct FaultFlags {
    flags: AtomicU32,
}
impl FaultFlags {
    /// Creates a new fault flags, where all flags are unset.
    pub const fn new() -> Self {
        Self { flags: AtomicU32::new(0) }
    }

    /// Gets the status of all faults.
    pub fn get_all(&self) -> IndexByFaultId<bool> {
        let flags = self.flags.load(Ordering::Relaxed);

        IndexByFaultId::from_fn(|fault| flags & (1 << fault as u32) != 0)
    }

    /// Checks whether or not a particular fault flag is set.
    pub fn is_set(&self, fault: FaultId) -> bool {
        self.flags.load(Ordering::Relaxed) & (1 << fault as u32) != 0
    }

    /// Sets the flag for a fault.
    pub fn set_fault(&self, fault: FaultId) {
        self.flags.fetch_or(1 << fault as u32, Ordering::Relaxed);
    }

    /// Clears the flag for a fault.
    pub fn clear_fault(&self, fault: FaultId) {
        self.flags.fetch_and(!(1 << fault as u32), Ordering::Relaxed);
    }

    /// Checks if any critical faults are currently active.
    pub fn are_critical_faults_active(&self) -> bool {
        self.flags.load(Ordering::Relaxed) & FaultId::CRITICAL_MASK != 0
    }
}

mod timers {
    use super::*;

    /// Current activation/expiration state for a fault and its timer.
    ///
    /// PRIVATE! This tracks the internal
    #[derive(Copy, Clone, defmt::Format)]
    enum FaultTimerState {
        Inactive,
        Active { deadline: Instant },
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
    #[derive(Copy, Clone, defmt::Format)]
    pub struct FaultTimer {
        state: FaultTimerState,
    }
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
            let deadline = Instant::now().saturating_add(duration);
            self.state = FaultTimerState::Active { deadline };
        }
    }
}
use timers::*;

static FAULT_QUEUE: Channel<ThreadModeRawMutex, FaultCommand, 10> = Channel::new();
static FLAGS: FaultFlags = FaultFlags::new();

/// !!!! PUBLIC API !!!!
///
/// (This is the public API of the faults module).
mod api {
    use super::*;

    #[repr(u8)]
    #[derive(defmt::Format)]
    pub enum FaultState {
        /// This fault is not current active (i.e., everything is normal for this fault).
        Inactive = 0,
        /// This fault is currently active (i.e., something bad happened that triggered this fault).
        Active = 1,
    }
    impl FaultState {
        /// Returns `true` if this is `FaultState::Active`.
        pub const fn is_active(&self) -> bool {
            matches!(self, FaultState::Active)
        }

        /// PRIVATE! Creates a `FaultState` based on a fault flag `bool`.
        const fn from_bool(flag: bool) -> Self {
            if flag { Self::Active } else { Self::Inactive }
        }
    }

    /// Gets the state of a particular fault.
    pub fn get_fault(fault: FaultId) -> FaultState {
        FaultState::from_bool(FLAGS.is_set(fault))
    }

    /// Gets the state of all faults.
    pub fn get_all_faults() -> IndexByFaultId<FaultState> {
        let faults = FLAGS.get_all();
        IndexByFaultId::from_fn(|fault| FaultState::from_bool(*faults.fault(fault)))
    }

    /// Checks if any critical faults are currently active.
    pub fn are_critical_faults_active() -> bool {
        FLAGS.are_critical_faults_active()
    }

    /// Adds a fault to the fault queue.
    pub async fn queue(message: FaultCommand) {
        match FAULT_QUEUE.try_send(message) {
            Ok(()) => {},
            Err(_) => {
                defmt::warn!("Faults: Tried to queue a fault, but the faults queue was full. This is not an error, since this function will now .await until the queue is open. However, you should probably increase the size of the faults queue if this is getting printed a lot.");
                FAULT_QUEUE.send(message).await;
            },
        }
    }

    /// Tries to add a fault to the fault queue.
    ///
    /// If this returns `Err(())`, then the faults queue was full and the fault couldn't be added. That is a sign to increase the capacity of the faults queue.
    pub fn try_queue(message: FaultCommand) -> Result<(), ()> {
        // Throwing away the specific error in map_err is fine here since Err always means "channel was full". This is also what can.rs does
        FAULT_QUEUE.try_send(message).map_err(|_| ())
    }
}
pub use api::*;

pub mod task {
    use super::*;

    /// Small private helper to set/clear faults in terms of `FaultState`.
    fn set_fault(fault: FaultId, state: FaultState) {
        match state {
            FaultState::Active => FLAGS.set_fault(fault),
            FaultState::Inactive => FLAGS.clear_fault(fault),
        }
    }

    #[derive(Copy, Clone, defmt::Format)]
    enum PassFailState {
        /// Fault is currently cleared, we are counting consecutive `bads` to see if we need to activate it.
        CountingBads{ count: usize },
        /// Fault is currently active, we are counting consecutive `okays` to see if we can clear it.
        CountingOkays{ count: usize },
    }
    impl PassFailState { 
        const fn new() -> Self {
            // default state on boot: fault is cleared, so we are counting any `bads` starting from 0 
            Self::CountingBads { count: 0 }
        }

        /// Increments the current variant's `count` by 1.
        const fn increment(&mut self) {
            match self {
                Self::CountingBads { count } => *count = count.saturating_add(1),
                Self::CountingOkays { count } => *count = count.saturating_add(1),
            }
        }
        const fn set(&mut self, new_state: PassFailState) { *self = new_state }

        /// Resets the current variant's `count` to zero.
        const fn reset(&mut self) {
            match self {
                Self::CountingBads { count } => *count = 0,
                Self::CountingOkays { count } => *count = 0,
            }
        }
    }

    #[derive(Copy, Clone, defmt::Format)]
    enum RuntimeData {
        Automatic{ timer: FaultTimer},
        PassFail{ state: PassFailState },
    }
    impl RuntimeData {
        pub fn new(fault: FaultId) -> Self {
            match fault.config().kind() {
                FaultKind::Automatic{..} => Self::Automatic{ timer: FaultTimer::new() },
                FaultKind::PassFail {..} => Self::PassFail { state: PassFailState::new() },
            }
        }
    }

    /// Guy in charge of the faults.
    struct FaultManager {
        runtime_data: IndexByFaultId<RuntimeData>,

        /// Debug counter tracking how many times we reach the unreachable
        /// code inside `handle()`. This should always be zero unless there has been a programmer error.
        unreachable_count: usize,
    }
    impl FaultManager {
        /// Initializes the faults manager.
        pub fn new() -> Self {
            Self { 
                runtime_data: IndexByFaultId::from_fn(|fault| RuntimeData::new(fault)),
                unreachable_count: 0,
            }
        }

        /// Handles commands/updates/requests queued from the app.
        fn handle(&mut self, message: FaultCommand) {
            let id = message.id();
            match(message.action(), *id.config().kind(), &mut self.runtime_data[id]) {
                // Kind == Automatic
                (FaultAction::AutomaticAction(action), FaultKind::Automatic{timeout}, RuntimeData::Automatic{timer}) => {
                    match action {
                        AutomaticAction::Trigger => {
                            FLAGS.set_fault(id);
                            timer.restart(timeout);
                        }
                    }
                },

                // Kind == PassFail
                (FaultAction::PassFailAction(action), FaultKind::PassFail{consecutive_bads, consecutive_okays}, RuntimeData::PassFail{state}) => {
                    
                    // While a fault is active, it uses CountingOkays to count consecutive `okays`. If it gets enough `okays` in a row, it has redeemed itself and can clear itself.
                    // While a fault is inactive, it uses CountingBads to count consecutive `bads`. If it gets enough `bads` in a row, it neeeds to activate itself.
                    match (get_fault(id), action) {
                        // NOTE: When `FaultState::Active`, the PassFailState is `PassFailState::CountingOkays`.
                        //       And when `FaultState::Inactive`, the PassFailState is `PassFailState::CountingBads`.
                        //       
                        // ^^ The above is enforced by the `match state` block below. The ONLY time `state.set()` is called is when a fault flips.
                        // That is literally the only time there is a state transition for `state`.
                        //
                        // If anyone ever modifies this code pls update this comment so it stays accurate.

                        // Fault is already active, and we just recieved another `bad`, so reset CountingBads to zero.
                        (FaultState::Active, PassFailAction::NotifyBad) => state.reset(),

                        // Fault is already inactive, and we just recieved another `okay`, so reset CountingOkays to zero.
                        (FaultState::Inactive, PassFailAction::NotifyOkay) => state.reset(),
                        
                        // Fault is already inactive, but we just recieved a `bad`. Maybe the condition is starting to fail? We need to increment the `bads` counter.
                        (FaultState::Inactive, PassFailAction::NotifyBad) => state.increment(),

                        // Fault is already active, but we just recieved a `okay`. Maybe the condition is no longer failing? We need to increment the `okays` counter.
                        (FaultState::Active, PassFailAction::NotifyOkay) => state.increment(),
                    }

                    // Check if `count` has exceeded the amount needed for a state change
                    match state {
                        PassFailState::CountingBads{count} => {
                            // `bads` count has exceeded the configured `consecutive_bads` needed to activate a fault, so we need to acivate it.
                            if *count > consecutive_bads {
                                set_fault(id, FaultState::Active);

                                // Now that the fault is active, it needs to start counting `okays` to see if it can clear itself.
                                state.set(PassFailState::CountingOkays{ count: 0 })
                            }
                        },
                        PassFailState::CountingOkays{count} => {
                            // `okays` count has exceeded the configured `consecutive_okays` needed to clear an active fault, so we need to clear it.
                            if *count > consecutive_okays {
                                set_fault(id, FaultState::Inactive);

                                // Now that the fault is inactive, it needs to start counting `bads` to see if it has to activate again.
                                state.set(PassFailState::CountingBads{ count: 0 })
                            }
                        }
                    }
                },

                // This block is not possible to reach due to the public API.
                (_, k, d) => {
                    defmt::error!("faults: in FaultManager::handle(): Reached an unreachable block in the match statement. This should not be possible, so if this message is shown, it indicates a programmer error. FaultCommand: {}, FaultKind: {}, RuntimeData: {}", message, k, d);
                    self.unreachable_count += 1;
                },
            }

            defmt_monitor::monitor!(["FaultsDebug/unreachable_count"], desc = "How many times we have reached the unreachable block in the `handle()` match statement. Should always be zero unless we made a mistake.", "{=usize}", self.unreachable_count);
        }
    }

    #[embassy_executor::task]
    pub async fn faults_task() -> ! {
        use embassy_futures::select::select;

        let mut manager = FaultManager::new();

        loop {
            // Dequeue faults from the faults queue and trigger them.
            while let Ok(message) = FAULT_QUEUE.try_receive() {
                manager.handle(message);
            }

            let now = Instant::now();

            // Stores the deadline of the soonest expiration.
            let mut soonest_expiration: Option<Instant> = None;

            // Check the state of each fault timer.
            for fault in FaultId::iter() {
                if let RuntimeData::Automatic{ timer } = &mut manager.runtime_data[fault] {
                    match timer.evaluate(now) {
                        // This timer is inactive so we don't need to do anything.
                        EvaluationResult::Inactive => {},

                        // This timer has expired, so we can set it to Inactive and clear the associated fault.
                        EvaluationResult::Expired => {
                            timer.set_inactive();
                            set_fault(fault, FaultState::Inactive);
                        },

                        // This timer is active, so we use it as part of our "soonest deadline" calculation (to see how long this task should sleep).
                        EvaluationResult::Active { deadline } => {
                            soonest_expiration = Some(match soonest_expiration {
                                // If no soonest_expiration exists yet, just use this timer's deadline.
                                None => deadline,

                                // If a soonest_expiration does exist, compare it to this timer's deadline and keep the sooner of the two
                                Some(soonest) => soonest.min(deadline),
                            });
                        },
                    }
                }
            }

            // u_TODO do stuff here probably:
            if FLAGS.are_critical_faults_active() {}

            // Sleep until more faults are queued, or a timer is ready to expire (whichever happens sooner).
            // If there are no timers counting down, then just sleep until more faults are queued.
            match soonest_expiration {
                Some(deadline) => {
                    select(FAULT_QUEUE.ready_to_receive(), Timer::at(deadline)).await;
                },
                None => {
                    FAULT_QUEUE.ready_to_receive().await;
                },
            }
        }
    }
}
// u_TODO - we should probably have a separate task that reads the fault values for reporting over CAN and such, since we don't want any of that stuff to interfere with the timing and deadline stuff from the current faults task. maybe we also want a broadcast belonging to this internal faults task that can signal every time it runs
