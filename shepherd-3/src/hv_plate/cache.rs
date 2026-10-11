//! Module for caching SPI reads to the HV plate's ADBMS2950B chip.
//!
//! A failed read leaves the cache untouched, so staleness shows up as `last_successful_read`
//! not advancing. Device health (command counter, PEC tallies) lives in
//! [`adbms2950::api::DeviceState`], reachable via `HvPlate::device()`.
//!
//! Uses `StartupCell` rather than `RefCell` because the cache can be read by any task at any time.

use adbms2950::chip::registers::{
    ReadableGroup,
    aux::{AuxA, AuxB, AuxC},
    flag::Flag,
    results::{AccumulatedCurrentAndBatteryVoltage, CurrentAndBatteryVoltage, OverCurrentResults},
    status::Status,
    voltage::{Voltages1A, Voltages1C},
};
use adbms2950::line::Error;
use crate::helpers::startup_cell::{StartupCell, UnwriteableStartupCell};

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
    /// Unable to convert raw data to nice data after updating.
    NiceError,
}

/// Actual register cache data.
#[derive(Copy, Clone)]
pub struct RegisterCacheData<R: ReadableGroup> {
    /// The read data.
    data: R,
    /// Last instant this register cache was successfully read over SPI and updated.
    last_successful_read: embassy_time::Instant,
}

impl<R: ReadableGroup> RegisterCacheData<R> {
    /// Last instant this register cache was successfully read over SPI and updated.
    pub const fn last_successful_read(&self) -> embassy_time::Instant {
        self.last_successful_read
    }

    /// The register reading.
    pub const fn data(&self) -> &R {
        &self.data
    }
}

type RegisterCache<R> = StartupCell<RegisterCacheData<R>>;
pub type RegisterCacheReadHandle<R> = UnwriteableStartupCell<RegisterCacheData<R>>;
impl<R: ReadableGroup> RegisterCache<R> {
    /// Returns a unwriteable read-only handle to this register cache.
    pub(in crate::hv_plate::cache) const fn read_handle(&self) -> &RegisterCacheReadHandle<R> {
        self.to_unwriteable()
    }

    /// Reads the register and updates the cache.
    ///
    /// On any driver error -- including a failed PEC -- the cache keeps whatever it had and
    /// `last_successful_read` does not advance, so consumers see the reading go stale rather
    /// than seeing corrupt data.
    pub(in crate::hv_plate::cache) async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
        let data = match api.read::<R>().await {
            Ok(data) => data,
            Err(err) => {
                defmt::error!("HvPlate: cache: In `RegisterCache::update()`: SPI read of `{=str}` failed. Error: {}", core::any::type_name::<R>(), err);
                return Err(UpdateError::ReadFailed(err));
            },
        };

        self.set(RegisterCacheData { data, last_successful_read: embassy_time::Instant::now() });

        Ok(())
    }
}

/// All of the HV plate's register caches.
pub struct CacheData {
    current_voltage: current_voltage::CacheData,
    accumulated: accumulated::CacheData,
    flag: flag::CacheData,
    voltages: voltages::CacheData,
    aux: aux::CacheData,
    status: status::CacheData,
}

impl CacheData {
    /// New uninitialized cache. Meant to be called once, to build the static.
    pub const fn new() -> Self {
        Self {
            current_voltage: current_voltage::CacheData::new(),
            accumulated: accumulated::CacheData::new(),
            flag: flag::CacheData::new(),
            voltages: voltages::CacheData::new(),
            aux: aux::CacheData::new(),
            status: status::CacheData::new(),
        }
    }
}

/// Pack current and BATT voltage, read coherently inside one SNAP window.
pub mod current_voltage {
    use super::{RegisterCache, StartupCell, CurrentAndBatteryVoltage, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::hv_plate::board;
    use crate::units::{Current, Voltage};

    /// The raw readings with the board's shunt resistance and divider applied.
    #[derive(Copy, Clone)]
    pub struct NiceData {
        /// Current through the shunt. Positive is into the pack.
        pub pack_current: Current,
        /// BATT-side voltage, after the 3.6 MOhm / 9.1 kOhm divider.
        pub batt_voltage: Voltage,
    }

    /// Stored Cache Data for the IVB1 register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<NiceData>,
    }
    impl CacheData {
        /// Creates a new `CacheData` with uninitialized values.
        pub(super) const fn new() -> Self {
            Self { raw: Raw::new(), nice: StartupCell::new() }
        }
        /// Gets the "raw" version of this data.
        pub const fn raw(&self) -> &Raw {
            &self.raw
        }
        /// Gets the "nice" version of this data.
        pub const fn nice(&self) -> &UnwriteableStartupCell<NiceData> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw IVB1 register reading.
    pub struct Raw {
        ivb1: RegisterCache<CurrentAndBatteryVoltage>,
    }
    impl Raw {
        pub fn ivb1(&self) -> &RegisterCacheReadHandle<CurrentAndBatteryVoltage> {
            self.ivb1.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { ivb1: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<NiceData, UpdateError> {
            self.ivb1.update(api).await?;

            match self.try_nice() {
                Some(data) => Ok(data),
                None => Err(UpdateError::NiceError),
            }
        }

        /// Converts the Raw into NiceData.
        ///
        /// If not all the Raw registers have been filled in yet, this will return `None`.
        fn try_nice(&self) -> Option<NiceData> {
            let ivb1 = *self.ivb1.try_get()?.data();

            Some(NiceData {
                pack_current: board::pack_current(ivb1.i1().as_microvolts()),
                batt_voltage: board::batt_voltage(ivb1.vb1().as_microvolts()),
            })
        }
    }

    impl CacheData {
        /// Reads IVB1 into the cache.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let nice = self.raw.update(api).await?;
            self.nice.set(nice);
            Ok(())
        }
    }

    impl super::CacheData {
        /// Updates pack current and BATT voltage.
        pub(in crate::hv_plate) async fn update_current_voltage(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.current_voltage.update(api).await
        }

        /// Gets the current cached pack current and BATT voltage.
        pub fn get_current_voltage(&self) -> &CacheData {
            &self.current_voltage
        }
    }
}

/// Accumulated current and BATT voltage, for coulomb counting.
pub mod accumulated {
    use super::{RegisterCache, StartupCell, AccumulatedCurrentAndBatteryVoltage, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;

    /// The accumulator registers, as raw sums.
    ///
    /// These are sums of `ACCN` conversions, not averages, and are never cleared here. Divide
    /// by `ACCN` (from CFGA's `acci`) for an average; diff against your own previous value to
    /// integrate.
    #[derive(Copy, Clone)]
    pub struct NiceData {
        /// Summed shunt voltage, in microvolts.
        pub current_sum_microvolts: i32,
        /// Summed BATT-tap voltage, in microvolts.
        pub batt_sum_microvolts: i32,
    }

    /// Stored Cache Data for the IVB1ACC register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<NiceData>,
    }
    impl CacheData {
        /// Creates a new `CacheData` with uninitialized values.
        pub(super) const fn new() -> Self {
            Self { raw: Raw::new(), nice: StartupCell::new() }
        }
        /// Gets the "raw" version of this data.
        pub const fn raw(&self) -> &Raw {
            &self.raw
        }
        /// Gets the "nice" version of this data.
        pub const fn nice(&self) -> &UnwriteableStartupCell<NiceData> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw IVB1ACC register reading.
    pub struct Raw {
        ivb1acc: RegisterCache<AccumulatedCurrentAndBatteryVoltage>,
    }
    impl Raw {
        pub fn ivb1acc(&self) -> &RegisterCacheReadHandle<AccumulatedCurrentAndBatteryVoltage> {
            self.ivb1acc.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { ivb1acc: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<NiceData, UpdateError> {
            self.ivb1acc.update(api).await?;

            match self.try_nice() {
                Some(data) => Ok(data),
                None => Err(UpdateError::NiceError),
            }
        }

        /// Converts the Raw into NiceData.
        ///
        /// If not all the Raw registers have been filled in yet, this will return `None`.
        fn try_nice(&self) -> Option<NiceData> {
            let ivb1acc = *self.ivb1acc.try_get()?.data();

            Some(NiceData {
                current_sum_microvolts: ivb1acc.i1acc().as_microvolts(),
                batt_sum_microvolts: ivb1acc.vb1acc().as_microvolts(),
            })
        }
    }

    impl CacheData {
        /// Reads IVB1ACC into the cache.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        #[cfg_attr(feature = "hil", allow(dead_code))]
        async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let nice = self.raw.update(api).await?;
            self.nice.set(nice);
            Ok(())
        }
    }

    impl super::CacheData {
        /// Updates the accumulators.
        #[cfg(not(feature = "hil"))]
        pub(in crate::hv_plate) async fn update_accumulated(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.accumulated.update(api).await
        }

        /// Gets the cached accumulator readings.
        pub fn get_accumulated(&self) -> &CacheData {
            &self.accumulated
        }
    }
}

/// The FLAG register: fault latches plus the conversion counters.
pub mod flag {
    use super::{RegisterCache, StartupCell, Flag, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;

    /// Decoded FLAG contents.
    #[derive(Copy, Clone)]
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

    /// Stored Cache Data for the FLAG register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<NiceData>,
    }
    impl CacheData {
        /// Creates a new `CacheData` with uninitialized values.
        pub(super) const fn new() -> Self {
            Self { raw: Raw::new(), nice: StartupCell::new() }
        }
        /// Gets the "raw" version of this data.
        pub const fn raw(&self) -> &Raw {
            &self.raw
        }
        /// Gets the "nice" version of this data.
        pub const fn nice(&self) -> &UnwriteableStartupCell<NiceData> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw FLAG register reading.
    pub struct Raw {
        flag: RegisterCache<Flag>,
    }
    impl Raw {
        pub fn flag(&self) -> &RegisterCacheReadHandle<Flag> {
            self.flag.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { flag: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<NiceData, UpdateError> {
            self.flag.update(api).await?;

            match self.try_nice() {
                Some(data) => Ok(data),
                None => Err(UpdateError::NiceError),
            }
        }

        /// Converts the Raw into NiceData.
        ///
        /// If not all the Raw registers have been filled in yet, this will return `None`.
        fn try_nice(&self) -> Option<NiceData> {
            let flag = *self.flag.try_get()?.data();

            Some(NiceData {
                flags: flag,
                i1cnt: flag.i1cnt(),
                i1cntpha: flag.i1cntpha(),
                i2cnt: flag.i2cnt(),
            })
        }
    }

    impl CacheData {
        /// Reads FLAG into the cache.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let nice = self.raw.update(api).await?;
            self.nice.set(nice);
            Ok(())
        }
    }

    impl super::CacheData {
        /// Updates FLAG.
        pub(in crate::hv_plate) async fn update_flag(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.flag.update(api).await
        }

        /// Gets the cached FLAG reading.
        pub fn get_flag(&self) -> &CacheData {
            &self.flag
        }
    }
}

/// The V1ADC voltage channels this board actually uses: TS voltage and the shunt thermistor.
pub mod voltages {
    use super::{RegisterCache, StartupCell, Voltages1A, Voltages1C, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::hv_plate::board;
    use crate::units::{Temperature, Voltage};

    /// TS voltage and shunt temperature, with the board networks applied.
    #[derive(Copy, Clone)]
    pub struct NiceData {
        /// Tractive-system voltage, from channel V2 after the 3.6 MOhm / 4.53 kOhm divider.
        pub ts_voltage: Voltage,
        /// Shunt temperature, from the thermistor on channel V7.
        pub shunt_temperature: Temperature,
    }

    /// Stored Cache Data for the V1A and V1C register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<NiceData>,
    }
    impl CacheData {
        /// Creates a new `CacheData` with uninitialized values.
        pub(super) const fn new() -> Self {
            Self { raw: Raw::new(), nice: StartupCell::new() }
        }
        /// Gets the "raw" version of this data.
        pub const fn raw(&self) -> &Raw {
            &self.raw
        }
        /// Gets the "nice" version of this data.
        pub const fn nice(&self) -> &UnwriteableStartupCell<NiceData> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw V1A/V1C register readings.
    pub struct Raw {
        v1a: RegisterCache<Voltages1A>,
        v1c: RegisterCache<Voltages1C>,
    }
    impl Raw {
        pub fn v1a(&self) -> &RegisterCacheReadHandle<Voltages1A> {
            self.v1a.read_handle()
        }
        pub fn v1c(&self) -> &RegisterCacheReadHandle<Voltages1C> {
            self.v1c.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { v1a: RegisterCache::new(), v1c: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<NiceData, UpdateError> {
            self.v1a.update(api).await?;
            self.v1c.update(api).await?;

            match self.try_nice() {
                Some(data) => Ok(data),
                None => Err(UpdateError::NiceError),
            }
        }

        /// Converts the Raw into NiceData.
        ///
        /// If not all the Raw registers have been filled in yet, this will return `None`.
        fn try_nice(&self) -> Option<NiceData> {
            let v1a = *self.v1a.try_get()?.data();
            let v1c = *self.v1c.try_get()?.data();

            Some(NiceData {
                ts_voltage: board::ts_voltage(v1a.v2a().as_microvolts()),
                shunt_temperature: board::shunt_temperature(v1c.v7a().as_microvolts()),
            })
        }
    }

    impl CacheData {
        /// Reads V1A and V1C into the cache.
        ///
        /// This doesn't trigger the conversion -- the caller runs `adv_autoconvert` first.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let nice = self.raw.update(api).await?;
            self.nice.set(nice);
            Ok(())
        }
    }

    impl super::CacheData {
        /// Updates the voltage channels.
        pub(in crate::hv_plate) async fn update_voltages(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.voltages.update(api).await
        }

        /// Gets the cached voltage-channel readings.
        pub fn get_voltages(&self) -> &CacheData {
            &self.voltages
        }
    }
}

/// The AUX ADC rails and on-chip temperature sensors.
pub mod aux {
    use super::{RegisterCache, StartupCell, AuxA, AuxB, AuxC, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::{Temperature, Voltage, millicelcius};
    use uom::si::electric_potential::microvolt;

    /// The chip's internal rails and temperatures.
    #[derive(Copy, Clone)]
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

    /// Stored Cache Data for the AUXA through AUXC register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<NiceData>,
    }
    impl CacheData {
        /// Creates a new `CacheData` with uninitialized values.
        pub(super) const fn new() -> Self {
            Self { raw: Raw::new(), nice: StartupCell::new() }
        }
        /// Gets the "raw" version of this data.
        pub const fn raw(&self) -> &Raw {
            &self.raw
        }
        /// Gets the "nice" version of this data.
        pub const fn nice(&self) -> &UnwriteableStartupCell<NiceData> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw AUXA through AUXC register readings.
    pub struct Raw {
        auxa: RegisterCache<AuxA>,
        auxb: RegisterCache<AuxB>,
        auxc: RegisterCache<AuxC>,
    }
    impl Raw {
        pub fn auxa(&self) -> &RegisterCacheReadHandle<AuxA> {
            self.auxa.read_handle()
        }
        pub fn auxb(&self) -> &RegisterCacheReadHandle<AuxB> {
            self.auxb.read_handle()
        }
        pub fn auxc(&self) -> &RegisterCacheReadHandle<AuxC> {
            self.auxc.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                auxa: RegisterCache::new(),
                auxb: RegisterCache::new(),
                auxc: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<NiceData, UpdateError> {
            self.auxa.update(api).await?;
            self.auxb.update(api).await?;
            self.auxc.update(api).await?;

            match self.try_nice() {
                Some(data) => Ok(data),
                None => Err(UpdateError::NiceError),
            }
        }

        /// Converts the Raw into NiceData.
        ///
        /// If not all the Raw registers have been filled in yet, this will return `None`.
        fn try_nice(&self) -> Option<NiceData> {
            let a = *self.auxa.try_get()?.data();
            let b = *self.auxb.try_get()?.data();
            let c = *self.auxc.try_get()?.data();

            Some(NiceData {
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
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let nice = self.raw.update(api).await?;
            self.nice.set(nice);
            Ok(())
        }
    }

    impl super::CacheData {
        /// Updates AUX.
        pub(in crate::hv_plate) async fn update_aux(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.aux.update(api).await
        }

        /// Gets the cached AUX readings.
        pub fn get_aux(&self) -> &CacheData {
            &self.aux
        }
    }
}

/// The STATUS register and the overcurrent comparator results.
pub mod status {
    use super::{RegisterCache, StartupCell, Status, OverCurrentResults, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;

    /// Chip status and the raw overcurrent codes.
    #[derive(Copy, Clone)]
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

    /// Stored Cache Data for the STATUS and overcurrent results register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<NiceData>,
    }
    impl CacheData {
        /// Creates a new `CacheData` with uninitialized values.
        pub(super) const fn new() -> Self {
            Self { raw: Raw::new(), nice: StartupCell::new() }
        }
        /// Gets the "raw" version of this data.
        pub const fn raw(&self) -> &Raw {
            &self.raw
        }
        /// Gets the "nice" version of this data.
        pub const fn nice(&self) -> &UnwriteableStartupCell<NiceData> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw STATUS and overcurrent results register readings.
    pub struct Raw {
        status: RegisterCache<Status>,
        oc: RegisterCache<OverCurrentResults>,
    }
    impl Raw {
        pub fn status(&self) -> &RegisterCacheReadHandle<Status> {
            self.status.read_handle()
        }
        pub fn oc(&self) -> &RegisterCacheReadHandle<OverCurrentResults> {
            self.oc.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { status: RegisterCache::new(), oc: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<NiceData, UpdateError> {
            self.status.update(api).await?;
            self.oc.update(api).await?;

            match self.try_nice() {
                Some(data) => Ok(data),
                None => Err(UpdateError::NiceError),
            }
        }

        /// Converts the Raw into NiceData.
        ///
        /// If not all the Raw registers have been filled in yet, this will return `None`.
        fn try_nice(&self) -> Option<NiceData> {
            let status = *self.status.try_get()?.data();
            let oc = *self.oc.try_get()?.data();

            Some(NiceData { status, overcurrent: oc })
        }
    }

    impl CacheData {
        /// Reads STATUS and the overcurrent results into the cache.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let nice = self.raw.update(api).await?;
            self.nice.set(nice);
            Ok(())
        }
    }

    impl super::CacheData {
        /// Updates STATUS and the overcurrent results.
        pub(in crate::hv_plate) async fn update_status(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.status.update(api).await
        }

        /// Gets the cached STATUS and overcurrent readings.
        pub fn get_status(&self) -> &CacheData {
            &self.status
        }
    }
}
