//! Which instrument's word the firmware acts on, where more than one speaks about the same thing.
//!
//! The plate measures pack current, BATT and TS directly; other nodes claim some of the same
//! quantities over CAN, and the charger announces its presence the same way. This module keeps the
//! claims as they arrive, decides which source to believe, and says which one it picked.

use embassy_time::{Duration, Instant};

use crate::faults::{self, FaultId};
use crate::helpers::stamped::Stamped;
use crate::hv_plate;
use crate::units::{Current, Voltage};

/// What other nodes on the bus claim about the accumulator, as last heard.
pub mod reported {
    use core::cell::Cell;
    use embassy_sync::blocking_mutex::ThreadModeMutex;
    use embassy_time::Instant;

    use crate::helpers::stamped::Stamped;
    use crate::units::{Current, Voltage};

    /// All the possible sources of a reported value
    #[derive(Copy, Clone, PartialEq, Eq)]
    #[derive(defmt::Format)]
    pub enum Reporter {
        Charger,
        Dti,
    }

    /// A value and who claimed it.
    #[derive(Copy, Clone)]
    pub struct Claim<T> {
        pub value: Stamped<T>,
        pub from: Reporter,
    }

    struct Inbox {
        pack_current: Cell<Option<Claim<Current>>>,
        ts_voltage: Cell<Option<Claim<Voltage>>>,
        /// The charger has no value to claim beyond having spoken at all.
        charger_ping: Cell<Option<Stamped<()>>>,
    }

    static INBOX: ThreadModeMutex<Inbox> = ThreadModeMutex::new(Inbox {
        pack_current: Cell::new(None),
        ts_voltage: Cell::new(None),
        charger_ping: Cell::new(None),
    });

    /// Pack current as last reported, with when the frame arrived.
    pub fn pack_current() -> Option<Claim<Current>> {
        INBOX.lock(|inbox| inbox.pack_current.get())
    }

    /// TS voltage as last reported, with when the frame arrived.
    pub fn ts_voltage() -> Option<Claim<Voltage>> {
        INBOX.lock(|inbox| inbox.ts_voltage.get())
    }

    /// When the charger was last heard from. `None` until it has spoken once since boot.
    pub fn charger_ping() -> Option<Stamped<()>> {
        INBOX.lock(|inbox| inbox.charger_ping.get())
    }

    // API: called from `can.rs`

    pub(crate) fn store_pack_current(current: Current, from: Reporter) {
        INBOX.lock(|inbox| inbox.pack_current.set(Some(Claim { value: Stamped::new(current, Instant::now()), from })));
    }

    pub(crate) fn store_ts_voltage(voltage: Voltage, from: Reporter) {
        INBOX.lock(|inbox| inbox.ts_voltage.set(Some(Claim { value: Stamped::new(voltage, Instant::now()), from })));
    }

    pub(crate) fn store_charger_ping() {
        INBOX.lock(|inbox| inbox.charger_ping.set(Some(Stamped::new((), Instant::now()))));
    }
}

pub use reported::Reporter;

/// Several passes of the plate loop. Past this the link is not merely slow.
#[cfg(not(feature = "hil"))]
const PLATE_MAX_AGE: Duration = Duration::from_millis(250);
// The Simulink model responds slower than the chip
#[cfg(feature = "hil")]
const PLATE_MAX_AGE: Duration = Duration::from_millis(1_000);

/// Generous next to any sender's frame rate, so a single dropped frame does not flap the source.
const REPORTED_MAX_AGE: Duration = Duration::from_millis(1_000);

/// If we don't hear the charger box within this timeout we assume it has gone away.
const CHARGER_SILENCE_TIMEOUT: Duration = Duration::from_secs(5);

/// Where a reading came from.
// u_TODO: bring in Segments as source here
#[derive(Copy, Clone, PartialEq, Eq)]
#[derive(defmt::Format)]
pub enum Source {
    HvPlate,
    /// Reported over CAN by another node.
    Reported(Reporter),
}

/// A reading, when it was measured, and which instrument measured it.
#[derive(Copy, Clone)]
pub struct Sourced<T> {
    value: Stamped<T>,
    source: Source,
}

impl<T: Copy> Sourced<T> {
    /// The value.
    pub const fn get(&self) -> T {
        self.value.get()
    }

    /// Which instrument measured it.
    pub const fn source(&self) -> Source {
        self.source
    }

    /// When it was measured.
    pub const fn at(&self) -> Instant {
        self.value.at()
    }

    /// How old it is from [`Instant::now()`].
    pub fn age(&self) -> Duration {
        self.value.age()
    }

    /// The value, but only if it is newer than `within`. For a caller stricter than this module.
    pub fn fresh(&self, within: Duration) -> Option<T> {
        self.value.fresh(within)
    }
}

/// Whether the plate's isoSPI link is believed to be working at all.
fn hv_plate_link_up() -> bool {
    !faults::get_fault(FaultId::HvPlateCommsFault).is_active()
}

/// Takes a plate reading if the link is up and the reading is still current.
fn from_plate<T: Copy>(reading: Option<Stamped<T>>) -> Option<Sourced<T>> {
    reading.filter(|reading| hv_plate_link_up() && reading.is_fresh(PLATE_MAX_AGE)).map(|value| Sourced { value, source: Source::HvPlate })
}

/// Takes a reported claim if the frame that carried it is still current.
fn from_reporter<T: Copy>(claim: Option<reported::Claim<T>>) -> Option<Sourced<T>> {
    claim.filter(|claim| claim.value.is_fresh(REPORTED_MAX_AGE)).map(|claim| Sourced { value: claim.value, source: Source::Reported(claim.from) })
}

/// Pack current, discharge positive.
pub fn pack_current() -> Option<Sourced<Current>> {
    from_plate(hv_plate::instant_current()).or_else(|| from_reporter(reported::pack_current()))
}

/// BATT-side voltage, off the plate's own divider. No node reports this one, so there is no failover.
pub fn batt_voltage() -> Option<Sourced<Voltage>> {
    from_plate(hv_plate::instant_batt_voltage())
}

/// Tractive-system voltage, off the 2950's divider behind GPO2 or whoever else is talking.
pub fn ts_voltage() -> Option<Sourced<Voltage>> {
    from_plate(hv_plate::ts_voltage()).or_else(|| from_reporter(reported::ts_voltage()))
}

/// Whether a charger has spoken recently enough to count as plugged in.
pub fn charger_present() -> bool {
    reported::charger_ping().is_some_and(|ping| ping.is_fresh(CHARGER_SILENCE_TIMEOUT))
}
