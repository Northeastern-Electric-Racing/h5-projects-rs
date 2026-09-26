//! Module for caching SPI reads to the HV plate's ADBMS2950B chip.
//!
//! A failed read leaves the cache untouched, so staleness shows up as `last_successful_read`
//! not advancing. Device health (command counter, PEC tallies) lives in
//! [`adbms2950::api::DeviceState`], reachable via `HvPlate::device()`.
//!
//! Uses `Cell` rather than `RefCell` because the cache can be read by any task at any time.

use adbms2950::chip::registers::{
    ReadableGroup,
    aux::{AuxA, AuxB, AuxC},
    flag::Flag,
    results::{AccumulatedCurrentAndBatteryVoltage, CurrentAndBatteryVoltage, OverCurrentResults},
    status::Status,
    voltage::{Voltages1A, Voltages1C},
};
use adbms2950::line::Error;
use core::cell::Cell;

use super::core::alias;

/// Cache to hold read data.
pub(super) static CACHE: CacheData = CacheData::new();

/// A driver error, carrying the concrete SPI error type.
pub type LineError = Error<alias::SpiError>;

/// Errors that may occur when trying to update a value in the cache.
#[derive(Clone, Copy, Debug)]
#[derive(defmt::Format)]
pub enum UpdateError {
    /// A register read failed. Inner contains the driver error, including a PEC mismatch.
    ReadFailed(LineError),
    /// Error occurred while trying to run the SNAP command.
    SnapError(LineError),
    /// Error occurred while trying to run the UNSNAP command.
    UnsnapError(LineError),
    /// Error occurred while triggering or polling a conversion.
    ConversionError(LineError),
}

/// Actual register cache data (held inside blocking mutex)
#[derive(Copy, Clone)]
pub struct RegisterCacheData<R: ReadableGroup> {
    /// The read data. Starts out as `None` if this register hasn't been cached yet.
    data: Option<R>,
    /// Last instant this register cache was successfully read over SPI and updated.
    /// If no read has been made yet, this is None.
    last_successful_read: Option<embassy_time::Instant>,
}

impl<R: ReadableGroup> RegisterCacheData<R> {
    /// Last instant this register cache was successfully read over SPI and updated.
    pub const fn last_successful_read(&self) -> Option<embassy_time::Instant> {
        self.last_successful_read
    }

    /// The register reading. `None` if no read has succeeded yet.
    pub const fn data(&self) -> &Option<R> {
        &self.data
    }
}

pub struct RegisterCache<R: ReadableGroup> {
    inner: embassy_sync::blocking_mutex::ThreadModeMutex<Cell<RegisterCacheData<R>>>,
}

impl<R: ReadableGroup> RegisterCache<R> {
    /// New uninitialized register cache.
    pub const fn new() -> Self {
        Self {
            inner: embassy_sync::blocking_mutex::ThreadModeMutex::new(Cell::new(RegisterCacheData { data: None, last_successful_read: None })),
        }
    }

    /// Copies out Register Cache data.
    pub fn data(&self) -> RegisterCacheData<R> {
        self.inner.lock(|inner| inner.get())
    }

    /// Reads the register and updates the cache.
    ///
    /// On any driver error -- including a failed PEC -- the cache keeps whatever it had and
    /// `last_successful_read` does not advance, so consumers see the reading go stale rather
    /// than seeing corrupt data.
    pub async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
        let data = match api.read::<R>().await {
            Ok(data) => data,
            Err(err) => {
                defmt::error!("HvPlate: cache: In `RegisterCache::update()`: SPI read of `{=str}` failed. Error: {}", core::any::type_name::<R>(), err);
                return Err(UpdateError::ReadFailed(err));
            },
        };

        self.inner.lock(|inner| {
            inner.set(RegisterCacheData {
                data: Some(data),
                last_successful_read: Some(embassy_time::Instant::now()),
            });
        });

        Ok(())
    }
}

/// All of the HV plate's register caches.
pub struct CacheData {
    ivb1: RegisterCache<CurrentAndBatteryVoltage>,
    ivb1acc: RegisterCache<AccumulatedCurrentAndBatteryVoltage>,
    flag: RegisterCache<Flag>,
    v1a: RegisterCache<Voltages1A>,
    v1c: RegisterCache<Voltages1C>,
    auxa: RegisterCache<AuxA>,
    auxb: RegisterCache<AuxB>,
    auxc: RegisterCache<AuxC>,
    status: RegisterCache<Status>,
    oc: RegisterCache<OverCurrentResults>,
}

impl CacheData {
    /// New uninitialized cache. Meant to be called once, to build the static.
    pub const fn new() -> Self {
        Self {
            ivb1: RegisterCache::new(),
            ivb1acc: RegisterCache::new(),
            flag: RegisterCache::new(),
            v1a: RegisterCache::new(),
            v1c: RegisterCache::new(),
            auxa: RegisterCache::new(),
            auxb: RegisterCache::new(),
            auxc: RegisterCache::new(),
            status: RegisterCache::new(),
            oc: RegisterCache::new(),
        }
    }
}

/// Pack current and BATT voltage, read coherently inside one SNAP window.
pub mod current_voltage {
    use super::*;
    use crate::hv_plate::board;
    use crate::units::{Current, Voltage};

    /// Raw readings.
    pub struct Raw {
        pub ivb1: RegisterCacheData<CurrentAndBatteryVoltage>,
    }
    impl Raw {
        /// Tries to make it nice.
        ///
        /// Returns `Err(())` if any register this view needs has never been read
        /// successfully, since there is nothing to decode in that case.
        pub fn try_nice(&self) -> Result<NiceData, ()> {
            NiceData::try_from(self)
        }
    }

    /// The raw readings with the board's shunt resistance and divider applied.
    pub struct NiceData {
        /// Current through the shunt. Positive is into the pack.
        pub pack_current: Current,
        /// BATT-side voltage, after the 3.6 MOhm / 9.1 kOhm divider.
        pub batt_voltage: Voltage,
    }

    impl TryFrom<&Raw> for NiceData {
        type Error = ();
        fn try_from(raw: &Raw) -> Result<Self, Self::Error> {
            let Some(group) = raw.ivb1.data() else {
                return Err(());
            };
            Ok(Self {
                pack_current: board::pack_current(group.i1().as_microvolts()),
                batt_voltage: board::batt_voltage(group.vb1().as_microvolts()),
            })
        }
    }

    impl CacheData {
        /// Reads IVB1 into the cache.
        pub(in crate::hv_plate) async fn update_current_voltage(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.ivb1.update(api).await
        }

        /// Gets the current cached pack current and BATT voltage.
        pub fn get_current_voltage(&self) -> Raw {
            Raw { ivb1: self.ivb1.data() }
        }
    }
}

/// Accumulated current and BATT voltage, for coulomb counting.
pub mod accumulated {
    use super::*;

    /// Raw readings.
    pub struct Raw {
        pub ivb1acc: RegisterCacheData<AccumulatedCurrentAndBatteryVoltage>,
    }
    impl Raw {
        /// Tries to make it nice.
        ///
        /// Returns `Err(())` if any register this view needs has never been read
        /// successfully, since there is nothing to decode in that case.
        pub fn try_nice(&self) -> Result<NiceData, ()> {
            NiceData::try_from(self)
        }
    }

    /// The accumulator registers, as raw sums.
    ///
    /// These are sums of `ACCN` conversions, not averages, and are never cleared here. Divide
    /// by `ACCN` (from CFGA's `acci`) for an average; diff against your own previous value to
    /// integrate.
    pub struct NiceData {
        /// Summed shunt voltage, in microvolts.
        pub current_sum_microvolts: i32,
        /// Summed BATT-tap voltage, in microvolts.
        pub batt_sum_microvolts: i32,
    }

    impl TryFrom<&Raw> for NiceData {
        type Error = ();
        fn try_from(raw: &Raw) -> Result<Self, Self::Error> {
            let Some(group) = raw.ivb1acc.data() else {
                return Err(());
            };
            Ok(Self {
                current_sum_microvolts: group.i1acc().as_microvolts(),
                batt_sum_microvolts: group.vb1acc().as_microvolts(),
            })
        }
    }

    impl CacheData {
        /// Reads IVB1ACC into the cache.
        pub(in crate::hv_plate) async fn update_accumulated(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.ivb1acc.update(api).await
        }

        /// Gets the cached accumulator readings.
        pub fn get_accumulated(&self) -> Raw {
            Raw { ivb1acc: self.ivb1acc.data() }
        }
    }
}

/// The FLAG register: fault latches plus the conversion counters.
pub mod flag {
    use super::*;

    /// Raw readings.
    pub struct Raw {
        pub flag: RegisterCacheData<Flag>,
    }
    impl Raw {
        /// Tries to make it nice.
        ///
        /// Returns `Err(())` if any register this view needs has never been read
        /// successfully, since there is nothing to decode in that case.
        pub fn try_nice(&self) -> Result<NiceData, ()> {
            NiceData::try_from(self)
        }
    }

    /// Decoded FLAG contents.
    pub struct NiceData {
        /// The whole register, for inspecting individual fault latches.
        ///
        /// Remember these reset to `1`, not `0`: a freshly reset chip reports essentially every
        /// fault until a `CLRFLAG`.
        pub flags: Flag,
        /// 11-bit I1ADC conversion counter. Rolls over; resets on an `ADI1`.
        pub i1cnt: u16,
        /// 13-bit `[I1CNT, I1PHA]` counter, for sub-sample resolution.
        pub i1cntpha: u16,
        /// 3-bit I2ADC conversion counter.
        pub i2cnt: u8,
    }

    impl TryFrom<&Raw> for NiceData {
        type Error = ();
        fn try_from(raw: &Raw) -> Result<Self, Self::Error> {
            let Some(group) = raw.flag.data() else {
                return Err(());
            };
            Ok(Self {
                flags: *group,
                i1cnt: group.i1cnt(),
                i1cntpha: group.i1cntpha(),
                i2cnt: group.i2cnt(),
            })
        }
    }

    impl CacheData {
        /// Reads FLAG into the cache.
        pub(in crate::hv_plate) async fn update_flag(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.flag.update(api).await
        }

        /// Gets the cached FLAG reading.
        pub fn get_flag(&self) -> Raw {
            Raw { flag: self.flag.data() }
        }
    }
}

/// The V1ADC voltage channels this board actually uses: TS voltage and the shunt thermistor.
pub mod voltages {
    use super::*;
    use crate::hv_plate::board;
    use crate::units::{Temperature, Voltage};

    /// Raw readings.
    pub struct Raw {
        pub v1a: RegisterCacheData<Voltages1A>,
        pub v1c: RegisterCacheData<Voltages1C>,
    }
    impl Raw {
        /// Tries to make it nice.
        ///
        /// Returns `Err(())` if any register this view needs has never been read
        /// successfully, since there is nothing to decode in that case.
        pub fn try_nice(&self) -> Result<NiceData, ()> {
            NiceData::try_from(self)
        }
    }

    /// TS voltage and shunt temperature, with the board networks applied.
    pub struct NiceData {
        /// Tractive-system voltage, from channel V2 after the 3.6 MOhm / 4.53 kOhm divider.
        pub ts_voltage: Voltage,
        /// Shunt temperature, from the thermistor on channel V7.
        pub shunt_temperature: Temperature,
    }

    impl TryFrom<&Raw> for NiceData {
        type Error = ();
        fn try_from(raw: &Raw) -> Result<Self, Self::Error> {
            let Some(v1a) = raw.v1a.data() else {
                return Err(());
            };
            let Some(v1c) = raw.v1c.data() else {
                return Err(());
            };
            Ok(Self {
                ts_voltage: board::ts_voltage(v1a.v2a().as_microvolts()),
                shunt_temperature: board::shunt_temperature(v1c.v7a().as_microvolts()),
            })
        }
    }

    impl CacheData {
        /// Reads V1A and V1C into the cache.
        ///
        /// This doesn't trigger the conversion -- the caller runs `adv_autoconvert` first.
        pub(in crate::hv_plate) async fn update_voltages(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.v1a.update(api).await?;
            self.v1c.update(api).await?;

            Ok(())
        }

        /// Gets the cached voltage-channel readings.
        pub fn get_voltages(&self) -> Raw {
            Raw { v1a: self.v1a.data(), v1c: self.v1c.data() }
        }
    }
}

/// The AUX ADC rails and on-chip temperature sensors.
pub mod aux {
    use super::*;
    use crate::units::{Temperature, Voltage};

    /// Raw readings.
    pub struct Raw {
        pub auxa: RegisterCacheData<AuxA>,
        pub auxb: RegisterCacheData<AuxB>,
        pub auxc: RegisterCacheData<AuxC>,
    }
    impl Raw {
        /// Tries to make it nice.
        ///
        /// Returns `Err(())` if any register this view needs has never been read
        /// successfully, since there is nothing to decode in that case.
        pub fn try_nice(&self) -> Result<NiceData, ()> {
            NiceData::try_from(self)
        }
    }

    /// The chip's internal rails and temperatures.
    pub struct NiceData {
        /// The 1.25 V reference, which the TS divider and shunt thermistor both depend on.
        pub vref1p25: Voltage,
        /// Regulator output.
        pub vreg: Voltage,
        /// Supply.
        pub vdd: Voltage,
        /// Digital rail.
        pub vdig: Voltage,
        /// Exposed-pad voltage.
        pub epad: Voltage,
        /// Divided reference.
        pub vdiv: Voltage,
        /// Die temperature (`TMP1`). Not the shunt temperature -- see `voltages::NiceData`.
        pub die_temperature: Temperature,
        /// Second on-chip temperature sensor (`TMP2`).
        pub secondary_temperature: Temperature,
        /// Oscillator counter. Outside 0x34..=0x47 the chip asserts `OSCFLT`.
        pub osccnt: u8,
    }

    impl TryFrom<&Raw> for NiceData {
        type Error = ();
        fn try_from(raw: &Raw) -> Result<Self, Self::Error> {
            use crate::units::millicelcius;
            use uom::si::electric_potential::microvolt;

            let Some(a) = raw.auxa.data() else {
                return Err(());
            };
            let Some(b) = raw.auxb.data() else {
                return Err(());
            };
            let Some(c) = raw.auxc.data() else {
                return Err(());
            };
            Ok(Self {
                vref1p25: Voltage::new::<microvolt>(a.vref1p25().as_microvolts() as f32),
                vreg: Voltage::new::<microvolt>(a.vreg().as_microvolts() as f32),
                vdd: Voltage::new::<microvolt>(b.vdd().as_microvolts() as f32),
                vdig: Voltage::new::<microvolt>(b.vdig().as_microvolts() as f32),
                epad: Voltage::new::<microvolt>(b.epad().as_microvolts() as f32),
                vdiv: Voltage::new::<microvolt>(c.vdiv().as_microvolts() as f32),
                die_temperature: Temperature::new::<millicelcius>(a.tmp1().as_millicelsius() as f32),
                secondary_temperature: Temperature::new::<millicelcius>(c.tmp2().as_millicelsius() as f32),
                osccnt: c.osccnt(),
            })
        }
    }

    impl CacheData {
        /// Reads AUXA through AUXC into the cache.
        ///
        /// This doesn't trigger the conversion -- the caller runs `adx_autoconvert` first.
        pub(in crate::hv_plate) async fn update_aux(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.auxa.update(api).await?;
            self.auxb.update(api).await?;
            self.auxc.update(api).await?;

            Ok(())
        }

        /// Gets the cached AUX readings.
        pub fn get_aux(&self) -> Raw {
            Raw { auxa: self.auxa.data(), auxb: self.auxb.data(), auxc: self.auxc.data() }
        }
    }
}

/// The STATUS register and the overcurrent comparator results.
pub mod status {
    use super::*;

    /// Raw readings.
    pub struct Raw {
        pub status: RegisterCacheData<Status>,
        pub oc: RegisterCacheData<OverCurrentResults>,
    }
    impl Raw {
        /// Tries to make it nice.
        ///
        /// Returns `Err(())` if any register this view needs has never been read
        /// successfully, since there is nothing to decode in that case.
        pub fn try_nice(&self) -> Result<NiceData, ()> {
            NiceData::try_from(self)
        }
    }

    /// Chip status and the raw overcurrent codes.
    pub struct NiceData {
        /// The whole STATUS register: ADC init flags, GPO/GPIO readbacks, revision.
        pub status: Status,
        /// The overcurrent results, as raw codes.
        ///
        /// Scaling depends on the channel's `OCxGC` gain bit in CFGB: 5 mV per code at gain 1,
        /// 2.5 mV at gain 2. Use `Api::overcurrent_microvolts(code, channel)`, which reads the
        /// gain from the cached `ConfigB`.
        pub overcurrent: OverCurrentResults,
    }

    impl TryFrom<&Raw> for NiceData {
        type Error = ();
        fn try_from(raw: &Raw) -> Result<Self, Self::Error> {
            let Some(status) = raw.status.data() else {
                return Err(());
            };
            let Some(oc) = raw.oc.data() else {
                return Err(());
            };
            Ok(Self { status: *status, overcurrent: *oc })
        }
    }

    impl CacheData {
        /// Reads STATUS and the overcurrent results into the cache.
        pub(in crate::hv_plate) async fn update_status(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.status.update(api).await?;
            self.oc.update(api).await?;

            Ok(())
        }

        /// Gets the cached STATUS and overcurrent readings.
        pub fn get_status(&self) -> Raw {
            Raw { status: self.status.data(), oc: self.oc.data() }
        }
    }
}
