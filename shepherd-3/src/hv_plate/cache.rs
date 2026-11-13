//! Caches SPI reads to the HV plate's ADBMS2950B chip.
//!
//! One cell per group holding the registers, the converted form built from those same registers,
//! and when they were read. A [`Reading`] is replaced whole, so the two can never disagree, and a
//! failed read leaves the cell untouched -- staleness shows as `read_at` not advancing.
//!
//! Device health (command counter, PEC tallies) lives in [`adbms2950::api::DeviceState`].
//! `StartupCell` rather than `RefCell` because any task may read at any time.

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
    /// The SNAP command failed.
    SnapError(LineError),
    /// The UNSNAP command failed.
    UnsnapError(LineError),
    /// Triggering or polling a conversion failed.
    ConversionError(LineError),
}

/// A group's converted values, its registers, and how to read them. The only part that differs
/// between groups; the cell, the timestamp and the update are generic over it.
pub(crate) trait Decode: Copy {
    /// The registers this group reads in one pass.
    type Registers: Copy;

    /// One SPI transaction per register, in the order the chip wants them.
    async fn read_registers(api: &mut alias::Api) -> Result<Self::Registers, UpdateError>;

    /// Applies the board's networks and the driver's scaling. Groups with nothing to convert
    /// alias `Self` to their own `Registers` and return a copy.
    fn decode(registers: &Self::Registers) -> Self;

    /// Taken *before* the first SPI read, so it bounds every register in the group.
    fn read_at() -> embassy_time::Instant {
        embassy_time::Instant::now()
    }
}

/// Reads one register group, logging and wrapping a driver failure.
async fn read<R: ReadableGroup>(api: &mut alias::Api) -> Result<R, UpdateError> {
    match api.read::<R>().await {
        Ok(data) => Ok(data),
        Err(err) => {
            defmt::error!("HvPlate: cache: SPI read of `{=str}` failed. Error: {}", core::any::type_name::<R>(), err);
            Err(UpdateError::ReadFailed(err))
        },
    }
}

/// A group's registers, their converted form, and when they were read.
#[derive(Copy, Clone)]
pub(crate) struct Reading<N: Decode> {
    registers: N::Registers,
    nice: N,
    read_at: embassy_time::Instant,
}

impl<N: Decode> Reading<N> {
    /// Exactly as the chip reported them.
    pub(crate) const fn registers(&self) -> &N::Registers {
        &self.registers
    }

    /// Made when the registers were read.
    pub(crate) const fn nice(&self) -> &N {
        &self.nice
    }

    /// Nothing in [`Self::registers`] predates this.
    pub(crate) const fn read_at(&self) -> embassy_time::Instant {
        self.read_at
    }
}

/// One cached group. `inner` is private so [`Self::update`] is the only writer -- aliasing this
/// to the cell directly would hand `set` to everything holding a group.
pub(crate) struct Group<N: Decode> {
    inner: StartupCell<Reading<N>>,
}

impl<N: Decode> Group<N> {
    /// A group that has never been read.
    const fn new() -> Self {
        Self { inner: StartupCell::new() }
    }

    /// The cached reading, once there is one. The `Option`/`await` is the validity boundary.
    pub(crate) const fn reading(&self) -> &UnwriteableStartupCell<Reading<N>> {
        self.inner.to_unwriteable()
    }

    /// Reads the registers and publishes them with their converted form. On any failure nothing
    /// is published, so a partially-read group never reaches a consumer.
    pub(in crate::hv_plate) async fn update(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
        let read_at = N::read_at();
        let registers = N::read_registers(api).await?;

        self.inner.set(Reading { nice: N::decode(&registers), registers, read_at });

        Ok(())
    }
}

/// All of the HV plate's register caches.
pub(crate) struct CacheData {
    current_voltage: Group<current_voltage::Nice>,
    accumulated: Group<accumulated::Nice>,
    flag: Group<Flag>,
    voltages: Group<voltages::Nice>,
    aux: Group<aux::Nice>,
    status: Group<status::Nice>,
}

impl CacheData {
    /// Uninitialized. Called once, to build the static.
    pub(crate) const fn new() -> Self {
        Self {
            current_voltage: Group::new(),
            accumulated: Group::new(),
            flag: Group::new(),
            voltages: Group::new(),
            aux: Group::new(),
            status: Group::new(),
        }
    }

    pub(crate) const fn current_voltage(&self) -> &Group<current_voltage::Nice> {
        &self.current_voltage
    }
    pub(crate) const fn accumulated(&self) -> &Group<accumulated::Nice> {
        &self.accumulated
    }
    pub(crate) const fn flag(&self) -> &Group<Flag> {
        &self.flag
    }
    pub(crate) const fn voltages(&self) -> &Group<voltages::Nice> {
        &self.voltages
    }
    pub(crate) const fn aux(&self) -> &Group<aux::Nice> {
        &self.aux
    }
    pub(crate) const fn status(&self) -> &Group<status::Nice> {
        &self.status
    }
}

/// Pack current and BATT voltage, read coherently inside one SNAP window.
pub mod current_voltage {
    use super::{CurrentAndBatteryVoltage, UpdateError, alias, read};
    use crate::hv_plate::board;
    use crate::units::{Current, Voltage};

    pub type Registers = CurrentAndBatteryVoltage;

    /// With the board's shunt resistance and divider applied.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// Positive is discharge, out of the pack.
        pub pack_current: Current,
        /// After the 3.6 MOhm / 9.1 kOhm divider.
        pub batt_voltage: Voltage,
    }

    impl super::Decode for Nice {
        type Registers = Registers;

        async fn read_registers(api: &mut alias::Api) -> Result<Registers, UpdateError> {
            read::<Registers>(api).await
        }

        fn decode(ivb1: &Registers) -> Self {
            Self {
                pack_current: board::pack_current(ivb1.i1().as_microvolts()),
                batt_voltage: board::batt_voltage(ivb1.vb1().as_microvolts()),
            }
        }
    }
}

/// Accumulated current and BATT voltage, for coulomb counting.
pub mod accumulated {
    use super::{AccumulatedCurrentAndBatteryVoltage, UpdateError, alias, read};
    use crate::hv_plate::board;

    pub type Registers = AccumulatedCurrentAndBatteryVoltage;
    pub type Reading = super::Reading<Nice>;

    /// Sums of `ACCN` conversions, not averages, never cleared here. Divide by `ACCN` (CFGA's
    /// `acci`) for an average; diff against your own previous value to integrate.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// Microvolts, discharge positive.
        pub current_sum_microvolts: i32,
        /// Microvolts.
        pub batt_sum_microvolts: i32,
    }

    impl super::Decode for Nice {
        type Registers = Registers;

        async fn read_registers(api: &mut alias::Api) -> Result<Registers, UpdateError> {
            read::<Registers>(api).await
        }

        fn decode(ivb1acc: &Registers) -> Self {
            Self {
                current_sum_microvolts: board::summed_shunt_microvolts(ivb1acc.i1acc().as_microvolts()),
                batt_sum_microvolts: ivb1acc.vb1acc().as_microvolts(),
            }
        }

        /// On a rig the accumulator's timebase is the model's, not the MCU's.
        #[cfg(feature = "hil")]
        fn read_at() -> embassy_time::Instant {
            crate::hv_plate::hil::sim_time::now()
        }
    }
}

/// FLAG: fault latches and the conversion counters. Nothing needs converting, so the register
/// *is* the converted form and no promoted copy of an accessor can rot beside it.
pub mod flag {
    use super::Flag;

    pub type Reading = super::Reading<Flag>;
}

impl Decode for Flag {
    type Registers = Self;

    async fn read_registers(api: &mut alias::Api) -> Result<Self, UpdateError> {
        read::<Self>(api).await
    }

    fn decode(flag: &Self) -> Self {
        *flag
    }
}

/// TS voltage and shunt temperature, off the V1 ADC channels.
pub mod voltages {
    use super::{UpdateError, Voltages1A, Voltages1C, alias, read};
    use crate::hv_plate::board;
    use crate::units::{Temperature, Voltage};

    #[derive(Copy, Clone)]
    pub struct Registers {
        pub v1a: Voltages1A,
        pub v1c: Voltages1C,
    }

    /// With the board networks applied.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// Channel V2, after the 3.6 MOhm / 4.53 kOhm divider.
        pub ts_voltage: Voltage,
        /// From the thermistor on channel V7.
        pub shunt_temperature: Temperature,
    }

    impl super::Decode for Nice {
        type Registers = Registers;

        /// The caller runs `adv_autoconvert` first; this does not trigger the conversion.
        async fn read_registers(api: &mut alias::Api) -> Result<Registers, UpdateError> {
            Ok(Registers {
                v1a: read::<Voltages1A>(api).await?,
                v1c: read::<Voltages1C>(api).await?,
            })
        }

        fn decode(r: &Registers) -> Self {
            Self {
                ts_voltage: board::ts_voltage(r.v1a.v2a().as_microvolts()),
                shunt_temperature: board::shunt_temperature(r.v1c.v7a().as_microvolts()),
            }
        }
    }
}

/// The chip's own rails and on-die temperatures.
pub mod aux {
    use super::{AuxA, AuxB, AuxC, UpdateError, alias, read};
    use crate::units::{Temperature, Voltage, millicelcius};
    use uom::si::electric_potential::microvolt;

    #[derive(Copy, Clone)]
    pub struct Registers {
        pub auxa: AuxA,
        pub auxb: AuxB,
        pub auxc: AuxC,
    }

    fn volts(microvolts: i32) -> Voltage {
        Voltage::new::<microvolt>(microvolts as f32)
    }

    fn celsius(millicelsius: i32) -> Temperature {
        Temperature::new::<millicelcius>(millicelsius as f32)
    }

    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The TS divider and shunt thermistor both depend on this.
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
        /// `TMP1`. Not the shunt temperature -- see [`super::voltages`].
        pub die_temperature: Temperature,
        /// `TMP2`.
        pub secondary_temperature: Temperature,
        /// Outside 0x34..=0x47 the chip asserts `OSCFLT`.
        pub osccnt: u8,
    }

    impl super::Decode for Nice {
        type Registers = Registers;

        async fn read_registers(api: &mut alias::Api) -> Result<Registers, UpdateError> {
            Ok(Registers {
                auxa: read::<AuxA>(api).await?,
                auxb: read::<AuxB>(api).await?,
                auxc: read::<AuxC>(api).await?,
            })
        }

        fn decode(r: &Registers) -> Self {
            Self {
                vref1p25: volts(r.auxa.vref1p25().as_microvolts()),
                vreg: volts(r.auxa.vreg().as_microvolts()),
                vdd: volts(r.auxb.vdd().as_microvolts()),
                vdig: volts(r.auxb.vdig().as_microvolts()),
                epad: volts(r.auxb.epad().as_microvolts()),
                vdiv: volts(r.auxc.vdiv().as_microvolts()),
                die_temperature: celsius(r.auxa.tmp1().as_millicelsius()),
                secondary_temperature: celsius(r.auxc.tmp2().as_millicelsius()),
                osccnt: r.auxc.osccnt(),
            }
        }
    }
}

/// STATUS and the overcurrent codes. Neither can be converted: scaling an OC code needs CFGB's
/// `OCxGC` gain bit, which only `Api` caches. Grouping them gets them one shared `read_at`.
pub mod status {
    use super::{OverCurrentResults, Status, UpdateError, alias, read};

    pub type Nice = Registers;

    #[derive(Copy, Clone)]
    pub struct Registers {
        /// ADC init flags, GPO/GPIO readbacks, revision.
        pub status: Status,
        /// Raw codes; see `Api::overcurrent_microvolts`.
        pub overcurrent: OverCurrentResults,
    }

    impl super::Decode for Registers {
        type Registers = Self;

        async fn read_registers(api: &mut alias::Api) -> Result<Self, UpdateError> {
            Ok(Self {
                status: read::<Status>(api).await?,
                overcurrent: read::<OverCurrentResults>(api).await?,
            })
        }

        fn decode(registers: &Self) -> Self {
            *registers
        }
    }
}
