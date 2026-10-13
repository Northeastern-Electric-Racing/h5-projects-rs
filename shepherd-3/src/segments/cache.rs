//! Module for caching SPI reads to the ADBMS6830B chips.
//!
//! (this module uses `StartupCell` because the cache can be accessed at any time by different tasks. it doesn't use RefCell because that can panic. maybe in the future it would be good to look into RefCell but the Cell copies are realistically never going to be an actual issue)

use adbms6830b::{
    chip::registers::{
        ReadableGroup,
        pwm::{PwmA, PwmB},
        results::{RedundantAuxillaryA, RedundantAuxillaryB, RedundantAuxillaryC, RedundantAuxillaryD},
        results::{CellVoltagesA, CellVoltagesB, CellVoltagesC, CellVoltagesD, CellVoltagesE},
        results::{AverageCellVoltagesA, AverageCellVoltagesB, AverageCellVoltagesC, AverageCellVoltagesD, AverageCellVoltagesE},
        results::{FilteredCellVoltagesA, FilteredCellVoltagesB, FilteredCellVoltagesC, FilteredCellVoltagesD, FilteredCellVoltagesE},
        results::{SVoltagesA, SVoltagesB, SVoltagesC, SVoltagesD, SVoltagesE},
        status::{StatusC, StatusD, StatusA, StatusB},
        clear::{ClearFlags, types::ClearAction, ClearOvervoltageUndervoltage},
        results::{AuxillaryA, AuxillaryB, AuxillaryC, AuxillaryD},
    },
    turnkey::api::LineId,
};
use adbms6830b::line::Error;
use crate::{helpers::startup_cell::UnwriteableStartupCell, segments::core::alias::SpiError};
use adbms6830b::line::PecStatus;
use core::cell::{Cell};
use super::chips::IndexByChip;
use super::core::alias;
use super::chips::ADBMS6830B_NUM_CHIPS;
use crate::helpers::startup_cell::StartupCell;

/// Cache to hold read data.
pub(super) static CACHE: CacheData = CacheData::new();

/// Errors that may occur when trying to update a value in the cache.
#[derive(Clone, Copy, Debug)]
#[derive(defmt::Format)]
pub enum UpdateError {
    /// Error occurred while trying to run the UNSNAP command.
    UnsnapError(Error<SpiError>),
    /// Error occurred while trying to run the SNAP command.
    SnapError(Error<SpiError>),
    /// Error occurred while trying to clear flags after reading them in an update call.
    ClearFlagsError(Error<SpiError>),
    /// Error occurred while polling a conversion completion (possibly via a ...autoconvert() function).
    PollError(Error<SpiError>),
    /// Line A failed during update. Inner contains the SPI error.
    LineAFailed(Error<SpiError>),
    /// Line B failed during update. Inner contains the SPI error.
    LineBFailed(Error<SpiError>),
    /// Both lines failed during update. Inner contains both SPI errors.
    BothLinesFailed { linea_err: Error<SpiError>, lineb_err: Error<SpiError> },
    /// Impossible error that should not be possible to happen. Using this instead of unreachable!()
    /// or an unwrap/expect so an impossible error that somehow happens doesn't panic the whole bms
    ImpossibleError,
}

/// Register reading for a single chip.
#[derive(Copy, Clone, Debug)]
pub struct Reading<R: ReadableGroup> {
    data: R,
    pec: PecStatus,
}
impl<R: ReadableGroup> Reading<R> {
    /// Actual register reading.
    pub const fn data(&self) -> R {
        self.data
    }
    /// The PEC status of the reading.
    pub const fn pec(&self) -> PecStatus {
        self.pec
    }
}

/// Actual register cache data.
#[derive(Copy, Clone)]
pub struct RegisterCacheData<R: ReadableGroup> {
    /// Contains the read data for each chip.
    data: IndexByChip<Reading<R>>,
    /// Last instant this register cache was successfully read over SPI and updated.
    last_sucessful_read: embassy_time::Instant,
}
// ^^ u_TODO ideas for maybe cool extra stuff we could add to RegisterCacheData:
// - a `read_duration: Option<embassy_time::Duration>` that stores how long the most recent read took. maybe also a `max_read_duration`, `min_read_duration`, and `avg_read_duration` field? would actually be helpful for optimizing our timing a bit. or just cool to look at
// - idk

impl<R: ReadableGroup> RegisterCacheData<R> {
    /// Last instant this register cache was successfully read over SPI and updated.
    pub const fn last_sucessful_read(&self) -> embassy_time::Instant {
        self.last_sucessful_read
    }

    /// Register read data for each chip.
    pub const fn data(&self) -> &IndexByChip<Reading<R>> {
        &self.data
    }
}

type RegisterCache<R> = StartupCell<RegisterCacheData<R>>;
pub type RegisterCacheReadHandle<R> = UnwriteableStartupCell<RegisterCacheData<R>>;
impl<R: ReadableGroup> RegisterCache<R> {
    /// Returns a unwriteable read-only handle to this register cache.
    pub(in crate::segments::cache) const fn read_handle(&self) -> &RegisterCacheReadHandle<R> {
        self.to_unwriteable()
    }

    /// Reads the register and updates the cache.
    pub(in crate::segments::cache) async fn update(&self, api: &mut alias::Api) -> Result<RegisterCacheData<R>, UpdateError> {
        let data: [Reading<R>; ADBMS6830B_NUM_CHIPS] = {
            let responses = api.read::<R>().await;

            match (responses.line_error(LineId::A), responses.line_error(LineId::B)) {
                // Both lines failed.
                (Some(linea_err), Some(lineb_err)) => {
                    defmt::error!("Segments: cache: In RegisterCache::update(): SPI Read of `{=str}` on both Line A and Line B failed. Errors: linea_err={}, lineb_err={}", core::any::type_name::<R>(), linea_err, lineb_err);
                    return Err(UpdateError::BothLinesFailed { linea_err: *linea_err, lineb_err: *lineb_err });
                },

                // Line A failed, but not Line B.
                (Some(linea_err), None) => {
                    defmt::error!("Segments: cache: In RegisterCache::update(): SPI Read of `{=str}` on Line A failed. Error: {}", core::any::type_name::<R>(), linea_err);
                    return Err(UpdateError::LineAFailed(*linea_err));
                },

                // Line B failed, but not Line A.
                (None, Some(lineb_err)) => {
                    defmt::error!("Segments: cache: In RegisterCache::update(): SPI Read of `{=str}` on Line B failed. Error: {}", core::any::type_name::<R>(), lineb_err);
                    return Err(UpdateError::LineBFailed(*lineb_err));
                },

                // Neither line failed so we're good
                (None, None) => (),
            }

            let readings: [Reading<R>; ADBMS6830B_NUM_CHIPS] = {
                let Some(readings) = responses.iter().map(|response| response.map(|response| Reading { data: response.data(), pec: response.pec() })).collect::<Option<heapless::Vec<Reading<R>, { ADBMS6830B_NUM_CHIPS }>>>().and_then(|readings| readings.into_array::<{ ADBMS6830B_NUM_CHIPS }>().ok()) else {
                    // u_Note: there is probably a way to restructure this so that ImpossibleError doesn't need to exist at all, but it might require going into the driver which is kinda annoying. so even though this existing is kinda gross it is probably fine for now
                    defmt::error!("Segments: cache: In RegisterCache::update(): a chip reading was `None` even though we already verified that no line errors occured. This should not be possible.");
                    return Err(UpdateError::ImpossibleError);
                };

                readings
            };

            readings
        };

        let data = RegisterCacheData {
            data: IndexByChip::new(data),
            last_sucessful_read: embassy_time::Instant::now(),
        };
        self.set(data);

        Ok(data)
    }
}

pub mod fault_counts {
    use crate::segments::chips::cells::{IndexByCell, CellId};
    use super::{CacheData, IndexByChip};

    /// Undervoltage/overvoltage flags from StatusD.
    pub mod undervotlage_overvoltage {
        use super::*;

        /// Undervoltage/overvoltage flags from StatusD.
        #[derive(Copy, Clone, Debug, defmt::Format)]
        pub struct UndervoltageOvervoltageFlags {
            pub c1uv: u32,
            pub c1ov: u32,

            pub c2uv: u32,
            pub c2ov: u32,

            pub c3uv: u32,
            pub c3ov: u32,

            pub c4uv: u32,
            pub c4ov: u32,

            pub c5uv: u32,
            pub c5ov: u32,

            pub c6uv: u32,
            pub c6ov: u32,

            pub c7uv: u32,
            pub c7ov: u32,

            pub c8uv: u32,
            pub c8ov: u32,

            pub c9uv: u32,
            pub c9ov: u32,

            pub c10uv: u32,
            pub c10ov: u32,

            pub c11uv: u32,
            pub c11ov: u32,

            pub c12uv: u32,
            pub c12ov: u32,

            pub c13uv: u32,
            pub c13ov: u32,

            pub c14uv: u32,
            pub c14ov: u32,

            pub c15uv: u32,
            pub c15ov: u32,

            pub c16uv: u32,
            pub c16ov: u32,
        }

        /// Undervoltage/overvoltage flag counts for a single cell.
        pub struct CellFlagCounts {
            /// How many times this cell's undervoltage flag has been read in as set.
            pub uv: u32,
            /// How many times this cell's overvoltage flag has been read in as set.
            pub ov: u32,
        }

        impl UndervoltageOvervoltageFlags {
            /// Lets you index the overvoltage/undervoltage flags by cell. This throws away c14uv/ov through c16uv/ov since we only have 13 cells.
            pub fn idx_by_cell(&self) -> IndexByCell<CellFlagCounts> {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => CellFlagCounts { uv: self.c1uv, ov: self.c1ov },
                    CellId::Cell2 => CellFlagCounts { uv: self.c2uv, ov: self.c2ov },
                    CellId::Cell3 => CellFlagCounts { uv: self.c3uv, ov: self.c3ov },
                    CellId::Cell4 => CellFlagCounts { uv: self.c4uv, ov: self.c4ov },
                    CellId::Cell5 => CellFlagCounts { uv: self.c5uv, ov: self.c5ov },
                    CellId::Cell6 => CellFlagCounts { uv: self.c6uv, ov: self.c6ov },
                    CellId::Cell7 => CellFlagCounts { uv: self.c7uv, ov: self.c7ov },
                    CellId::Cell8 => CellFlagCounts { uv: self.c8uv, ov: self.c8ov },
                    CellId::Cell9 => CellFlagCounts { uv: self.c9uv, ov: self.c9ov },
                    CellId::Cell10 => CellFlagCounts { uv: self.c10uv, ov: self.c10ov },
                    CellId::Cell11 => CellFlagCounts { uv: self.c11uv, ov: self.c11ov },
                    CellId::Cell12 => CellFlagCounts { uv: self.c12uv, ov: self.c12ov },
                    CellId::Cell13 => CellFlagCounts { uv: self.c13uv, ov: self.c13ov },
                })
            }
        }
    }

    /// Comparison fault flags from StatusC.
    pub mod comparison_faults {
        use super::*;

        /// Comparison fault flags from StatusC.
        #[derive(Copy, Clone, Debug, defmt::Format)]
        pub struct ComparisonFaultFlags {
            pub cs1flt: u32,
            pub cs2flt: u32,
            pub cs3flt: u32,
            pub cs4flt: u32,
            pub cs5flt: u32,
            pub cs6flt: u32,
            pub cs7flt: u32,
            pub cs8flt: u32,
            pub cs9flt: u32,
            pub cs10flt: u32,
            pub cs11flt: u32,
            pub cs12flt: u32,
            pub cs13flt: u32,
            pub cs14flt: u32,
            pub cs15flt: u32,
            pub cs16flt: u32,
        }
        impl ComparisonFaultFlags {
            /// Lets you index the comparison fault flags by cell. This throws away cs14flt through cs16flt since we only have 13 cells.
            pub fn idx_by_cell(&self) -> IndexByCell<u32> {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => self.cs1flt,
                    CellId::Cell2 => self.cs2flt,
                    CellId::Cell3 => self.cs3flt,
                    CellId::Cell4 => self.cs4flt,
                    CellId::Cell5 => self.cs5flt,
                    CellId::Cell6 => self.cs6flt,
                    CellId::Cell7 => self.cs7flt,
                    CellId::Cell8 => self.cs8flt,
                    CellId::Cell9 => self.cs9flt,
                    CellId::Cell10 => self.cs10flt,
                    CellId::Cell11 => self.cs11flt,
                    CellId::Cell12 => self.cs12flt,
                    CellId::Cell13 => self.cs13flt,
                })
            }
        }
    }

    /// Persistent counts of ADBMS6830B fault flags for CacheData.
    ///
    /// Basically, any time a fault flag gets read in here during an update (e.g., the fault flags on StatusC), that fault flag will be
    /// incremented here. The fault flags are W1C and are cleared each update, so this may help callers track the history of faults that
    /// have shown up, or detect that a fault occured during a cache read that they may have missed.
    ///
    /// Also, this isn't meant to be a super sophisticated error-detection thing. It is supposed to be pretty dumb and just a basic relay of fault values. If
    /// these are increased, it might not necessarily mean that something super serious is going on, it's just possibly useful data for debugging and diagnostic stuff.
    #[derive(Copy, Clone, Debug, defmt::Format)]
    pub struct FaultCounts {
        /// Source: StatusC.
        pub csxflt: comparison_faults::ComparisonFaultFlags,
        /// Source: StatusC.
        pub smed: u32,
        /// Source: StatusC.
        pub sed: u32,
        /// Source: StatusC.
        pub cmed: u32,
        /// Source: StatusC.
        pub ced: u32,
        /// Source: StatusC.
        pub vd_uv: u32,
        /// Source: StatusC.
        pub vd_ov: u32,
        /// Source: StatusC.
        pub va_uv: u32,
        /// Source: StatusC.
        pub va_ov: u32,
        /// Source: StatusC.
        pub oscchk: u32,
        /// Source: StatusC.
        pub tmodchk: u32,
        /// Source: StatusC.
        pub thsd: u32,
        /// Source: StatusC.
        pub sleep: u32,
        /// Source: StatusC.
        pub spiflt: u32,
        /// Source: StatusC.
        pub vde: u32,
        /// Source: StatusC.
        pub vdel: u32,

        /// Source: StatusD.
        pub cxovuv: undervotlage_overvoltage::UndervoltageOvervoltageFlags,
    }
    impl FaultCounts {
        /// Default FaultCounts where everything is zeroed.
        pub const fn new() -> Self {
            Self {
                csxflt: comparison_faults::ComparisonFaultFlags {
                    cs1flt: 0,
                    cs2flt: 0,
                    cs3flt: 0,
                    cs4flt: 0,
                    cs5flt: 0,
                    cs6flt: 0,
                    cs7flt: 0,
                    cs8flt: 0,
                    cs9flt: 0,
                    cs10flt: 0,
                    cs11flt: 0,
                    cs12flt: 0,
                    cs13flt: 0,
                    cs14flt: 0,
                    cs15flt: 0,
                    cs16flt: 0,
                },
                smed: 0,
                sed: 0,
                cmed: 0,
                ced: 0,
                vd_uv: 0,
                vd_ov: 0,
                va_uv: 0,
                va_ov: 0,
                oscchk: 0,
                tmodchk: 0,
                thsd: 0,
                sleep: 0,
                spiflt: 0,
                vde: 0,
                vdel: 0,
                cxovuv: undervotlage_overvoltage::UndervoltageOvervoltageFlags {
                    c1uv: 0,
                    c1ov: 0,
                    c2uv: 0,
                    c2ov: 0,
                    c3uv: 0,
                    c3ov: 0,
                    c4uv: 0,
                    c4ov: 0,
                    c5uv: 0,
                    c5ov: 0,
                    c6uv: 0,
                    c6ov: 0,
                    c7uv: 0,
                    c7ov: 0,
                    c8uv: 0,
                    c8ov: 0,
                    c9uv: 0,
                    c9ov: 0,
                    c10uv: 0,
                    c10ov: 0,
                    c11uv: 0,
                    c11ov: 0,
                    c12uv: 0,
                    c12ov: 0,
                    c13uv: 0,
                    c13ov: 0,
                    c14uv: 0,
                    c14ov: 0,
                    c15uv: 0,
                    c15ov: 0,
                    c16uv: 0,
                    c16ov: 0,
                },
            }
        }
    }

    impl CacheData {
        /// Gets persistent fault counts.
        pub fn get_fault_counts(&self) -> IndexByChip<FaultCounts> {
            self.fault_counts.lock(|inner| inner.get())
        }
    }
}

pub struct CacheData {
    fault_counts: embassy_sync::blocking_mutex::ThreadModeMutex<Cell<IndexByChip<fault_counts::FaultCounts>>>,

    redundant_aux: redundant_aux::CacheData,
    cell_voltages: cell_voltages::CacheData,
    average_cell_voltages: average_cell_voltages::CacheData,
    filtered_cell_voltages: filtered_cell_voltages::CacheData,
    s_voltages: s_voltages::kinds::IndexBySVoltageKind<s_voltages::CacheData>,
    status_c: status_c::CacheData,
    status_d: status_d::CacheData,
    aux: aux::CacheData,
    status_a: status_a::CacheData,
    status_b: status_b::CacheData,
    pwm: pwm::CacheData,
}
impl CacheData {
    pub(super) const fn new() -> Self {
        Self {
            fault_counts: embassy_sync::blocking_mutex::ThreadModeMutex::new(Cell::new(IndexByChip::new([fault_counts::FaultCounts::new(); ADBMS6830B_NUM_CHIPS]))),

            redundant_aux: redundant_aux::CacheData::new(),
            cell_voltages: cell_voltages::CacheData::new(),
            average_cell_voltages: average_cell_voltages::CacheData::new(),
            filtered_cell_voltages: filtered_cell_voltages::CacheData::new(),
            s_voltages: s_voltages::kinds::IndexBySVoltageKind::new([const { s_voltages::CacheData::new() }; s_voltages::kinds::NUM_S_VOLTAGE_KINDS]),
            status_c: status_c::CacheData::new(),
            status_d: status_d::CacheData::new(),
            aux: aux::CacheData::new(),
            status_a: status_a::CacheData::new(),
            status_b: status_b::CacheData::new(),
            pwm: pwm::CacheData::new(),
        }
    }
}

/// Register groups RedundantAuxillaryA through D.
pub mod redundant_aux {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, RedundantAuxillaryA, RedundantAuxillaryB, RedundantAuxillaryC, RedundantAuxillaryD, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::Voltage;
    use uom::si::{electric_potential::microvolt};
    use crate::segments::chips::gpios::{GpioId, IndexByGpio};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<IndexByGpio<Voltage>>;

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for Redundant Aux register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw Redundant Aux register readings.
    pub struct Raw {
        raxa: RegisterCache<RedundantAuxillaryA>,
        raxb: RegisterCache<RedundantAuxillaryB>,
        raxc: RegisterCache<RedundantAuxillaryC>,
        raxd: RegisterCache<RedundantAuxillaryD>,
    }
    impl Raw {
        pub fn raxa(&self) -> &RegisterCacheReadHandle<RedundantAuxillaryA> {
            self.raxa.read_handle()
        }
        pub fn raxb(&self) -> &RegisterCacheReadHandle<RedundantAuxillaryB> {
            self.raxb.read_handle()
        }
        pub fn raxc(&self) -> &RegisterCacheReadHandle<RedundantAuxillaryC> {
            self.raxc.read_handle()
        }
        pub fn raxd(&self) -> &RegisterCacheReadHandle<RedundantAuxillaryD> {
            self.raxd.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                raxa: RegisterCache::new(),
                raxb: RegisterCache::new(),
                raxc: RegisterCache::new(),
                raxd: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let a = self.raxa.update(api).await?;
            let b = self.raxb.update(api).await?;
            let c = self.raxc.update(api).await?;
            let d = self.raxd.update(api).await?;

            Ok(Self::nice(a, b, c, d))
        }

        /// Converts register readings into NiceData.
        fn nice(a: RegisterCacheData<RedundantAuxillaryA>, b: RegisterCacheData<RedundantAuxillaryB>, c: RegisterCacheData<RedundantAuxillaryC>, d: RegisterCacheData<RedundantAuxillaryD>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                IndexByGpio::from_fn(|gpio| match gpio {
                    GpioId::Gpio1 => Voltage::new::<microvolt>(a.data().get(chip).data().r_g1v().as_microvolts() as f32),
                    GpioId::Gpio2 => Voltage::new::<microvolt>(a.data().get(chip).data().r_g2v().as_microvolts() as f32),
                    GpioId::Gpio3 => Voltage::new::<microvolt>(a.data().get(chip).data().r_g3v().as_microvolts() as f32),

                    GpioId::Gpio4 => Voltage::new::<microvolt>(b.data().get(chip).data().r_g4v().as_microvolts() as f32),
                    GpioId::Gpio5 => Voltage::new::<microvolt>(b.data().get(chip).data().r_g5v().as_microvolts() as f32),
                    GpioId::Gpio6 => Voltage::new::<microvolt>(b.data().get(chip).data().r_g6v().as_microvolts() as f32),

                    GpioId::Gpio7 => Voltage::new::<microvolt>(c.data().get(chip).data().r_g7v().as_microvolts() as f32),
                    GpioId::Gpio8 => Voltage::new::<microvolt>(c.data().get(chip).data().r_g8v().as_microvolts() as f32),
                    GpioId::Gpio9 => Voltage::new::<microvolt>(c.data().get(chip).data().r_g9v().as_microvolts() as f32),

                    GpioId::Gpio10 => Voltage::new::<microvolt>(d.data().get(chip).data().r_g10v().as_microvolts() as f32),
                })
            });

            Nice {
                data,
                oldest_read: a.last_sucessful_read().min(b.last_sucessful_read()).min(c.last_sucessful_read()).min(d.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates caches RedundantAuxillaryA through D with new data.
        ///
        /// This doesn't run the `autoconvert` function!! The caller should do that. Otherwise this update will basically do nothing
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
        /// Updates Redundant Aux.
        pub(in crate::segments) async fn update_redundant_aux(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.redundant_aux.update(api).await
        }

        /// Gets the current cached Redundant Aux data.
        pub fn get_redundant_aux(&self) -> &CacheData {
            &self.redundant_aux
        }
    }
}

/// Register groups CellVoltages A through E (no F because we only use 13 cells).
pub mod cell_voltages {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, CellVoltagesA, CellVoltagesB, CellVoltagesC, CellVoltagesD, CellVoltagesE, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::Voltage;
    use uom::si::{electric_potential::microvolt};
    use crate::segments::chips::cells::{IndexByCell, CellId};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<IndexByCell<Voltage>>;

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for CellVoltages register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw CellVoltages register readings.
    pub struct Raw {
        cva: RegisterCache<CellVoltagesA>,
        cvb: RegisterCache<CellVoltagesB>,
        cvc: RegisterCache<CellVoltagesC>,
        cvd: RegisterCache<CellVoltagesD>,
        cve: RegisterCache<CellVoltagesE>,
    }
    impl Raw {
        pub fn cva(&self) -> &RegisterCacheReadHandle<CellVoltagesA> {
            self.cva.read_handle()
        }
        pub fn cvb(&self) -> &RegisterCacheReadHandle<CellVoltagesB> {
            self.cvb.read_handle()
        }
        pub fn cvc(&self) -> &RegisterCacheReadHandle<CellVoltagesC> {
            self.cvc.read_handle()
        }
        pub fn cvd(&self) -> &RegisterCacheReadHandle<CellVoltagesD> {
            self.cvd.read_handle()
        }
        pub fn cve(&self) -> &RegisterCacheReadHandle<CellVoltagesE> {
            self.cve.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                cva: RegisterCache::new(),
                cvb: RegisterCache::new(),
                cvc: RegisterCache::new(),
                cvd: RegisterCache::new(),
                cve: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let a = self.cva.update(api).await?;
            let b = self.cvb.update(api).await?;
            let c = self.cvc.update(api).await?;
            let d = self.cvd.update(api).await?;
            let e = self.cve.update(api).await?;

            Ok(Self::nice(a, b, c, d, e))
        }

        /// Converts register readings into NiceData.
        fn nice(a: RegisterCacheData<CellVoltagesA>, b: RegisterCacheData<CellVoltagesB>, c: RegisterCacheData<CellVoltagesC>, d: RegisterCacheData<CellVoltagesD>, e: RegisterCacheData<CellVoltagesE>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => Voltage::new::<microvolt>(a.data().get(chip).data().c1v().as_microvolts() as f32),
                    CellId::Cell2 => Voltage::new::<microvolt>(a.data().get(chip).data().c2v().as_microvolts() as f32),
                    CellId::Cell3 => Voltage::new::<microvolt>(a.data().get(chip).data().c3v().as_microvolts() as f32),

                    CellId::Cell4 => Voltage::new::<microvolt>(b.data().get(chip).data().c4v().as_microvolts() as f32),
                    CellId::Cell5 => Voltage::new::<microvolt>(b.data().get(chip).data().c5v().as_microvolts() as f32),
                    CellId::Cell6 => Voltage::new::<microvolt>(b.data().get(chip).data().c6v().as_microvolts() as f32),

                    CellId::Cell7 => Voltage::new::<microvolt>(c.data().get(chip).data().c7v().as_microvolts() as f32),
                    CellId::Cell8 => Voltage::new::<microvolt>(c.data().get(chip).data().c8v().as_microvolts() as f32),
                    CellId::Cell9 => Voltage::new::<microvolt>(c.data().get(chip).data().c9v().as_microvolts() as f32),

                    CellId::Cell10 => Voltage::new::<microvolt>(d.data().get(chip).data().c10v().as_microvolts() as f32),
                    CellId::Cell11 => Voltage::new::<microvolt>(d.data().get(chip).data().c11v().as_microvolts() as f32),
                    CellId::Cell12 => Voltage::new::<microvolt>(d.data().get(chip).data().c12v().as_microvolts() as f32),

                    CellId::Cell13 => Voltage::new::<microvolt>(e.data().get(chip).data().c13v().as_microvolts() as f32),
                })
            });

            Nice {
                data,
                oldest_read: a.last_sucessful_read().min(b.last_sucessful_read()).min(c.last_sucessful_read()).min(d.last_sucessful_read()).min(e.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates caches CellVoltages A through E with new data.
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
        /// Updates Cell Voltages.
        pub(in crate::segments) async fn update_cell_voltages(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.cell_voltages.update(api).await
        }

        /// Gets the current cached Cell Voltages data.
        pub fn get_cell_voltages(&self) -> &CacheData {
            &self.cell_voltages
        }
    }
}

/// Register groups AverageCellVoltages A through E (no F because we only use 13 cells).
pub mod average_cell_voltages {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, AverageCellVoltagesA, AverageCellVoltagesB, AverageCellVoltagesC, AverageCellVoltagesD, AverageCellVoltagesE, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::Voltage;
    use uom::si::{electric_potential::microvolt};
    use crate::segments::chips::cells::{IndexByCell, CellId};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<IndexByCell<Voltage>>;

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for AverageCellVoltages register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw AverageCellVoltages register readings.
    pub struct Raw {
        aca: RegisterCache<AverageCellVoltagesA>,
        acb: RegisterCache<AverageCellVoltagesB>,
        acc: RegisterCache<AverageCellVoltagesC>,
        acd: RegisterCache<AverageCellVoltagesD>,
        ace: RegisterCache<AverageCellVoltagesE>,
    }
    impl Raw {
        pub fn aca(&self) -> &RegisterCacheReadHandle<AverageCellVoltagesA> {
            self.aca.read_handle()
        }
        pub fn acb(&self) -> &RegisterCacheReadHandle<AverageCellVoltagesB> {
            self.acb.read_handle()
        }
        pub fn acc(&self) -> &RegisterCacheReadHandle<AverageCellVoltagesC> {
            self.acc.read_handle()
        }
        pub fn acd(&self) -> &RegisterCacheReadHandle<AverageCellVoltagesD> {
            self.acd.read_handle()
        }
        pub fn ace(&self) -> &RegisterCacheReadHandle<AverageCellVoltagesE> {
            self.ace.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                aca: RegisterCache::new(),
                acb: RegisterCache::new(),
                acc: RegisterCache::new(),
                acd: RegisterCache::new(),
                ace: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let a = self.aca.update(api).await?;
            let b = self.acb.update(api).await?;
            let c = self.acc.update(api).await?;
            let d = self.acd.update(api).await?;
            let e = self.ace.update(api).await?;

            Ok(Self::nice(a, b, c, d, e))
        }

        /// Converts register readings into NiceData.
        fn nice(a: RegisterCacheData<AverageCellVoltagesA>, b: RegisterCacheData<AverageCellVoltagesB>, c: RegisterCacheData<AverageCellVoltagesC>, d: RegisterCacheData<AverageCellVoltagesD>, e: RegisterCacheData<AverageCellVoltagesE>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => Voltage::new::<microvolt>(a.data().get(chip).data().ac1v().as_microvolts() as f32),
                    CellId::Cell2 => Voltage::new::<microvolt>(a.data().get(chip).data().ac2v().as_microvolts() as f32),
                    CellId::Cell3 => Voltage::new::<microvolt>(a.data().get(chip).data().ac3v().as_microvolts() as f32),

                    CellId::Cell4 => Voltage::new::<microvolt>(b.data().get(chip).data().ac4v().as_microvolts() as f32),
                    CellId::Cell5 => Voltage::new::<microvolt>(b.data().get(chip).data().ac5v().as_microvolts() as f32),
                    CellId::Cell6 => Voltage::new::<microvolt>(b.data().get(chip).data().ac6v().as_microvolts() as f32),

                    CellId::Cell7 => Voltage::new::<microvolt>(c.data().get(chip).data().ac7v().as_microvolts() as f32),
                    CellId::Cell8 => Voltage::new::<microvolt>(c.data().get(chip).data().ac8v().as_microvolts() as f32),
                    CellId::Cell9 => Voltage::new::<microvolt>(c.data().get(chip).data().ac9v().as_microvolts() as f32),

                    CellId::Cell10 => Voltage::new::<microvolt>(d.data().get(chip).data().ac10v().as_microvolts() as f32),
                    CellId::Cell11 => Voltage::new::<microvolt>(d.data().get(chip).data().ac11v().as_microvolts() as f32),
                    CellId::Cell12 => Voltage::new::<microvolt>(d.data().get(chip).data().ac12v().as_microvolts() as f32),

                    CellId::Cell13 => Voltage::new::<microvolt>(e.data().get(chip).data().ac13v().as_microvolts() as f32),
                })
            });

            Nice {
                data,
                oldest_read: a.last_sucessful_read().min(b.last_sucessful_read()).min(c.last_sucessful_read()).min(d.last_sucessful_read()).min(e.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates caches AverageCellVoltages A through E with new data.
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
        /// Updates Average Cell Voltages.
        pub(in crate::segments) async fn update_average_cell_voltages(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.average_cell_voltages.update(api).await
        }

        /// Gets the current cached Average Cell Voltages data.
        pub fn get_average_cell_voltages(&self) -> &CacheData {
            &self.average_cell_voltages
        }
    }
}

/// Register groups FilteredCellVoltages A through E (no F because we only use 13 cells).
pub mod filtered_cell_voltages {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, FilteredCellVoltagesA, FilteredCellVoltagesB, FilteredCellVoltagesC, FilteredCellVoltagesD, FilteredCellVoltagesE, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::Voltage;
    use uom::si::{electric_potential::microvolt};
    use crate::segments::chips::cells::{IndexByCell, CellId};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<IndexByCell<Voltage>>;

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for FilteredCellVoltages register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw FilteredCellVoltages register readings.
    pub struct Raw {
        fca: RegisterCache<FilteredCellVoltagesA>,
        fcb: RegisterCache<FilteredCellVoltagesB>,
        fcc: RegisterCache<FilteredCellVoltagesC>,
        fcd: RegisterCache<FilteredCellVoltagesD>,
        fce: RegisterCache<FilteredCellVoltagesE>,
    }
    impl Raw {
        pub fn fca(&self) -> &RegisterCacheReadHandle<FilteredCellVoltagesA> {
            self.fca.read_handle()
        }
        pub fn fcb(&self) -> &RegisterCacheReadHandle<FilteredCellVoltagesB> {
            self.fcb.read_handle()
        }
        pub fn fcc(&self) -> &RegisterCacheReadHandle<FilteredCellVoltagesC> {
            self.fcc.read_handle()
        }
        pub fn fcd(&self) -> &RegisterCacheReadHandle<FilteredCellVoltagesD> {
            self.fcd.read_handle()
        }
        pub fn fce(&self) -> &RegisterCacheReadHandle<FilteredCellVoltagesE> {
            self.fce.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                fca: RegisterCache::new(),
                fcb: RegisterCache::new(),
                fcc: RegisterCache::new(),
                fcd: RegisterCache::new(),
                fce: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let a = self.fca.update(api).await?;
            let b = self.fcb.update(api).await?;
            let c = self.fcc.update(api).await?;
            let d = self.fcd.update(api).await?;
            let e = self.fce.update(api).await?;

            Ok(Self::nice(a, b, c, d, e))
        }

        /// Converts register readings into NiceData.
        fn nice(a: RegisterCacheData<FilteredCellVoltagesA>, b: RegisterCacheData<FilteredCellVoltagesB>, c: RegisterCacheData<FilteredCellVoltagesC>, d: RegisterCacheData<FilteredCellVoltagesD>, e: RegisterCacheData<FilteredCellVoltagesE>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => Voltage::new::<microvolt>(a.data().get(chip).data().fc1v().as_microvolts() as f32),
                    CellId::Cell2 => Voltage::new::<microvolt>(a.data().get(chip).data().fc2v().as_microvolts() as f32),
                    CellId::Cell3 => Voltage::new::<microvolt>(a.data().get(chip).data().fc3v().as_microvolts() as f32),

                    CellId::Cell4 => Voltage::new::<microvolt>(b.data().get(chip).data().fc4v().as_microvolts() as f32),
                    CellId::Cell5 => Voltage::new::<microvolt>(b.data().get(chip).data().fc5v().as_microvolts() as f32),
                    CellId::Cell6 => Voltage::new::<microvolt>(b.data().get(chip).data().fc6v().as_microvolts() as f32),

                    CellId::Cell7 => Voltage::new::<microvolt>(c.data().get(chip).data().fc7v().as_microvolts() as f32),
                    CellId::Cell8 => Voltage::new::<microvolt>(c.data().get(chip).data().fc8v().as_microvolts() as f32),
                    CellId::Cell9 => Voltage::new::<microvolt>(c.data().get(chip).data().fc9v().as_microvolts() as f32),

                    CellId::Cell10 => Voltage::new::<microvolt>(d.data().get(chip).data().fc10v().as_microvolts() as f32),
                    CellId::Cell11 => Voltage::new::<microvolt>(d.data().get(chip).data().fc11v().as_microvolts() as f32),
                    CellId::Cell12 => Voltage::new::<microvolt>(d.data().get(chip).data().fc12v().as_microvolts() as f32),

                    CellId::Cell13 => Voltage::new::<microvolt>(e.data().get(chip).data().fc13v().as_microvolts() as f32),
                })
            });

            Nice {
                data,
                oldest_read: a.last_sucessful_read().min(b.last_sucessful_read()).min(c.last_sucessful_read()).min(d.last_sucessful_read()).min(e.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates caches FilteredCellVoltages A through E with new data.
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
        /// Updates Filtered Cell Voltages.
        pub(in crate::segments) async fn update_filtered_cell_voltages(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.filtered_cell_voltages.update(api).await
        }

        /// Gets the current cached Filtered Cell Voltages data.
        pub fn get_filtered_cell_voltages(&self) -> &CacheData {
            &self.filtered_cell_voltages
        }
    }
}

/// Register groups SVoltages A through E (no F because we only use 13 cells).
pub mod s_voltages {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, SVoltagesA, SVoltagesB, SVoltagesC, SVoltagesD, SVoltagesE, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::Voltage;
    use uom::si::{electric_potential::microvolt};
    use crate::segments::chips::cells::{IndexByCell, CellId};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<IndexByCell<Voltage>>;

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for SVoltages register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw SVoltages register readings.
    pub struct Raw {
        sca: RegisterCache<SVoltagesA>,
        scb: RegisterCache<SVoltagesB>,
        scc: RegisterCache<SVoltagesC>,
        scd: RegisterCache<SVoltagesD>,
        sce: RegisterCache<SVoltagesE>,
    }
    impl Raw {
        pub fn sca(&self) -> &RegisterCacheReadHandle<SVoltagesA> {
            self.sca.read_handle()
        }
        pub fn scb(&self) -> &RegisterCacheReadHandle<SVoltagesB> {
            self.scb.read_handle()
        }
        pub fn scc(&self) -> &RegisterCacheReadHandle<SVoltagesC> {
            self.scc.read_handle()
        }
        pub fn scd(&self) -> &RegisterCacheReadHandle<SVoltagesD> {
            self.scd.read_handle()
        }
        pub fn sce(&self) -> &RegisterCacheReadHandle<SVoltagesE> {
            self.sce.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                sca: RegisterCache::new(),
                scb: RegisterCache::new(),
                scc: RegisterCache::new(),
                scd: RegisterCache::new(),
                sce: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let a = self.sca.update(api).await?;
            let b = self.scb.update(api).await?;
            let c = self.scc.update(api).await?;
            let d = self.scd.update(api).await?;
            let e = self.sce.update(api).await?;

            Ok(Self::nice(a, b, c, d, e))
        }

        /// Converts register readings into NiceData.
        fn nice(a: RegisterCacheData<SVoltagesA>, b: RegisterCacheData<SVoltagesB>, c: RegisterCacheData<SVoltagesC>, d: RegisterCacheData<SVoltagesD>, e: RegisterCacheData<SVoltagesE>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => Voltage::new::<microvolt>(a.data().get(chip).data().s1v().as_microvolts() as f32),
                    CellId::Cell2 => Voltage::new::<microvolt>(a.data().get(chip).data().s2v().as_microvolts() as f32),
                    CellId::Cell3 => Voltage::new::<microvolt>(a.data().get(chip).data().s3v().as_microvolts() as f32),

                    CellId::Cell4 => Voltage::new::<microvolt>(b.data().get(chip).data().s4v().as_microvolts() as f32),
                    CellId::Cell5 => Voltage::new::<microvolt>(b.data().get(chip).data().s5v().as_microvolts() as f32),
                    CellId::Cell6 => Voltage::new::<microvolt>(b.data().get(chip).data().s6v().as_microvolts() as f32),

                    CellId::Cell7 => Voltage::new::<microvolt>(c.data().get(chip).data().s7v().as_microvolts() as f32),
                    CellId::Cell8 => Voltage::new::<microvolt>(c.data().get(chip).data().s8v().as_microvolts() as f32),
                    CellId::Cell9 => Voltage::new::<microvolt>(c.data().get(chip).data().s9v().as_microvolts() as f32),

                    CellId::Cell10 => Voltage::new::<microvolt>(d.data().get(chip).data().s10v().as_microvolts() as f32),
                    CellId::Cell11 => Voltage::new::<microvolt>(d.data().get(chip).data().s11v().as_microvolts() as f32),
                    CellId::Cell12 => Voltage::new::<microvolt>(d.data().get(chip).data().s12v().as_microvolts() as f32),

                    CellId::Cell13 => Voltage::new::<microvolt>(e.data().get(chip).data().s13v().as_microvolts() as f32),
                })
            });

            Nice {
                data,
                oldest_read: a.last_sucessful_read().min(b.last_sucessful_read()).min(c.last_sucessful_read()).min(d.last_sucessful_read()).min(e.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates caches SVoltages A through E with new data.
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
        /// Updates S Voltages.
        pub(in crate::segments) async fn update_s_voltages(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.s_voltages[kinds::SVoltageKind::Normal].update(api).await
        }

        /// Gets the current cached S Voltages data.
        pub fn get_s_voltages(&self) -> &CacheData {
            &self.s_voltages[kinds::SVoltageKind::Normal]
        }

        /// Updates caches SVoltages A through E with new data, and PUTS IT IN THE `OpenWireEven` storage.
        ///
        /// Before calling this, the caller should mmke sure to trigger a conversion with `OpenWire::EvenOnOddOff`. Then,
        /// after calling this, the caller should put the SVoltages back in continuous mode.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        pub(in crate::segments) async fn update_s_voltages_ow_even_on(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.s_voltages[kinds::SVoltageKind::OpenWireEvenOn].update(api).await
        }

        /// Gets the current cached S Voltages data for `OpenWire::EvenOnOddOff`.
        pub fn get_s_voltages_ow_even_on(&self) -> &CacheData {
            &self.s_voltages[kinds::SVoltageKind::OpenWireEvenOn]
        }

        /// Updates caches SVoltages A through E with new data, and PUTS IT IN THE `OpenWireOdd` storage.
        ///
        /// Before calling this, the caller should mmke sure to trigger a conversion with `OpenWire::EvenOffOddOn`. Then,
        /// after calling this, the caller should put the SVoltages back in continuous mode.
        ///
        /// ### Returns
        /// Will return `Ok(())`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(())`, the cached data was updated correctly and can be read now.
        pub(in crate::segments) async fn update_s_voltages_ow_odd_on(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.s_voltages[kinds::SVoltageKind::OpenWireOddOn].update(api).await
        }

        /// Gets the current cached S Voltages data for `OpenWire::EvenOffOddOn`.
        pub fn get_s_voltages_ow_odd_on(&self) -> &CacheData {
            &self.s_voltages[kinds::SVoltageKind::OpenWireOddOn]
        }
    }

    pub mod kinds {
        use strum::{EnumCount, VariantArray};

        /// The meaning of the voltages in the S voltage registers change depend
        /// on what OpenWire setting you set when triggering the conversions. This cache
        /// stores three different versions of the S voltages (for three different OpenWire settings).
        /// This enum is so we can index
        #[repr(usize)]
        #[derive(strum::FromRepr, strum::EnumCount, strum::VariantArray, strum::EnumIter)]
        #[derive(Clone, Copy, Debug)]
        pub enum SVoltageKind {
            /// Corresponds to OpenWire::OffForAll
            Normal,
            /// Corresponds to OpenWire::EvenOnOddOff
            OpenWireEvenOn,
            /// Corresponds to OpenWire::OddOnEvenOff
            OpenWireOddOn,
        }

        /// Number of S Voltage kinds.
        pub const NUM_S_VOLTAGE_KINDS: usize = SVoltageKind::COUNT;

        /// Wrapper around an array of type `T`, that lets you index using `SVoltageKind` instead of `usize`.
        ///
        /// We really should have a macro for these types
        #[derive(Copy, Clone, Debug)]
        pub struct IndexBySVoltageKind<T> {
            data: [T; SVoltageKind::COUNT],
        }
        pub type Kinds = core::iter::Copied<core::slice::Iter<'static, SVoltageKind>>;
        pub type Iter<'borrow, T> = core::iter::Zip<Kinds, core::slice::Iter<'borrow, T>>;
        pub type IterMut<'borrow, T> = core::iter::Zip<Kinds, core::slice::IterMut<'borrow, T>>;
        pub type IntoIter<T> = core::iter::Zip<Kinds, core::array::IntoIter<T, { SVoltageKind::COUNT }>>;

        impl<T> IndexBySVoltageKind<T> {
            /// Creates a new `IndexBySVoltageKind` directly from an array.
            pub const fn new(data: [T; SVoltageKind::COUNT]) -> Self {
                Self { data }
            }

            /// Retrives the data for `kind`.
            pub const fn get(&self, kind: SVoltageKind) -> &T {
                let i: usize = kind as usize;
                &self.data[i]
            }

            /// Retrives the data for `kind`.
            ///
            /// This is literally just an alias for `.get()`. It may be more readable in large method chains.
            pub const fn kind(&self, kind: SVoltageKind) -> &T {
                self.get(kind)
            }

            /// Retrieves a mutable reference to the data for `kind`.
            pub const fn get_mut(&mut self, kind: SVoltageKind) -> &mut T {
                let i: usize = kind as usize;
                &mut self.data[i]
            }

            pub fn from_fn(mut f: impl FnMut(SVoltageKind) -> T) -> Self {
                Self { data: core::array::from_fn(|i| f(SVoltageKind::VARIANTS[i])) }
            }

            pub fn iter(&self) -> Iter<'_, T> {
                SVoltageKind::VARIANTS.iter().copied().zip(self.data.iter())
            }

            pub fn iter_mut(&mut self) -> IterMut<'_, T> {
                SVoltageKind::VARIANTS.iter().copied().zip(self.data.iter_mut())
            }

            /// Converts this back into its inner array.
            pub fn into_array(self) -> [T; SVoltageKind::COUNT] {
                self.data
            }
        }

        impl<T: Copy> IndexBySVoltageKind<T> {
            /// Creates a new `IndexBySVoltageKind` by initializing every element to `value`.
            pub const fn from_value(value: T) -> Self {
                Self { data: [value; SVoltageKind::COUNT] }
            }
        }

        impl<T> core::ops::Index<SVoltageKind> for IndexBySVoltageKind<T> {
            type Output = T;

            fn index(&self, index: SVoltageKind) -> &Self::Output {
                &self.get(index)
            }
        }

        impl<T> core::ops::IndexMut<SVoltageKind> for IndexBySVoltageKind<T> {
            fn index_mut(&mut self, index: SVoltageKind) -> &mut Self::Output {
                let i: usize = index as usize;
                &mut self.data[i]
            }
        }

        impl<T> IntoIterator for IndexBySVoltageKind<T> {
            type Item = (SVoltageKind, T);
            type IntoIter = IntoIter<T>;

            fn into_iter(self) -> Self::IntoIter {
                SVoltageKind::VARIANTS.iter().copied().zip(self.data)
            }
        }

        impl<'borrow, T> IntoIterator for &'borrow IndexBySVoltageKind<T> {
            type Item = (SVoltageKind, &'borrow T);
            type IntoIter = Iter<'borrow, T>;

            fn into_iter(self) -> Self::IntoIter {
                self.iter()
            }
        }

        impl<'borrow, T> IntoIterator for &'borrow mut IndexBySVoltageKind<T> {
            type Item = (SVoltageKind, &'borrow mut T);
            type IntoIter = IterMut<'borrow, T>;

            fn into_iter(self) -> Self::IntoIter {
                self.iter_mut()
            }
        }
    }
}

/// Status C register group.
pub mod status_c {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, StatusC, UpdateError, RegisterCacheReadHandle, Reading, ClearFlags, ClearAction};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::segments::chips::cells::{IndexByCell, CellId};
    use adbms6830b::chip::registers::status::types::c::{ComparisonFault, ConversionsCount, STrimMultipleError, STrimError, CTrimMultipleError, CTrimError, DigitalRailOvervoltage, DigitalRailUndervoltage, AnalogRailUndervoltage, AnalogRailOvervoltage, OscillatorCheck, TestModeDetection, ThermalShutdownStatus, SleepModeDetection, SpiFault, ComparisonActive, SupplyRailDelta, SupplyRailDeltaLatent};

    /// "Nice" data for this group. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<NiceDataChip>;

    /// "Nice data" for a single chip.
    ///
    /// This data isn't even that "nice", since it is mostly just the raw StatusC data but with nicer ComparisonFault formatting (can be indexed by cells) and nicer
    /// conversions count formatting. Everything else is basically the same though.
    #[derive(Copy, Clone)]
    pub struct NiceDataChip {
        pub cell_channel_comparison_faults: IndexByCell<ComparisonFault>,
        pub conversions_count: ConversionsCount,
        pub s_trim_multiple_error: STrimMultipleError,
        pub s_trim_error: STrimError,
        pub c_trim_multiple_error: CTrimMultipleError,
        pub c_trim_error: CTrimError,
        pub digital_rail_undervoltage: DigitalRailUndervoltage,
        pub digital_rail_overvoltage: DigitalRailOvervoltage,
        pub analog_rail_undervoltage: AnalogRailUndervoltage,
        pub analog_rail_overvoltage: AnalogRailOvervoltage,
        pub oscillator_check: OscillatorCheck,
        pub test_mode_detection: TestModeDetection,
        pub thermal_shutdown_status: ThermalShutdownStatus,
        pub sleep_mode_detection: SleepModeDetection,
        pub spi_fault: SpiFault,
        pub comparison_active: ComparisonActive,
        pub supply_rail_delta: SupplyRailDelta,
        pub supply_rail_delta_latent: SupplyRailDeltaLatent,
    }

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for StatusC register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw StatusC register reading.
    pub struct Raw {
        statc: RegisterCache<StatusC>,
    }
    impl Raw {
        pub fn statc(&self) -> &RegisterCacheReadHandle<StatusC> {
            self.statc.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { statc: RegisterCache::new() }
        }

        /// Updates the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<RegisterCacheData<StatusC>, UpdateError> {
            self.statc.update(api).await
        }

        /// Converts register readings into NiceData.
        fn nice(statc: RegisterCacheData<StatusC>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                let chip_statc = statc.data().get(chip).data();
                NiceDataChip {
                    cell_channel_comparison_faults: IndexByCell::from_fn(|cell| match cell {
                        CellId::Cell1 => chip_statc.cs1flt(),
                        CellId::Cell2 => chip_statc.cs2flt(),
                        CellId::Cell3 => chip_statc.cs3flt(),
                        CellId::Cell4 => chip_statc.cs4flt(),
                        CellId::Cell5 => chip_statc.cs5flt(),
                        CellId::Cell6 => chip_statc.cs6flt(),
                        CellId::Cell7 => chip_statc.cs7flt(),
                        CellId::Cell8 => chip_statc.cs8flt(),
                        CellId::Cell9 => chip_statc.cs9flt(),
                        CellId::Cell10 => chip_statc.cs10flt(),
                        CellId::Cell11 => chip_statc.cs11flt(),
                        CellId::Cell12 => chip_statc.cs12flt(),
                        CellId::Cell13 => chip_statc.cs13flt(),
                    }),
                    conversions_count: ConversionsCount::new(chip_statc.ct_lower(), chip_statc.ct_upper(), chip_statc.cts()),
                    s_trim_multiple_error: chip_statc.smed(),
                    s_trim_error: chip_statc.sed(),
                    c_trim_multiple_error: chip_statc.cmed(),
                    c_trim_error: chip_statc.ced(),
                    digital_rail_overvoltage: chip_statc.vd_ov(),
                    digital_rail_undervoltage: chip_statc.vd_uv(),
                    analog_rail_overvoltage: chip_statc.va_ov(),
                    analog_rail_undervoltage: chip_statc.va_uv(),
                    oscillator_check: chip_statc.oscchk(),
                    test_mode_detection: chip_statc.tmodchk(),
                    thermal_shutdown_status: chip_statc.thsd(),
                    sleep_mode_detection: chip_statc.sleep(),
                    spi_fault: chip_statc.spiflt(),
                    comparison_active: chip_statc.comp(),
                    supply_rail_delta: chip_statc.vde(),
                    supply_rail_delta_latent: chip_statc.vdel(),
                }
            });

            Nice { data, oldest_read: statc.last_sucessful_read() }
        }
    }

    impl CacheData {
        /// Updates StatusC cache.
        ///
        /// ### Returns
        /// Will return `Ok(readings)`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(readings)`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<RegisterCacheData<StatusC>, UpdateError> {
            let statc = self.raw.update(api).await?;
            self.nice.set(Raw::nice(statc));
            Ok(statc)
        }
    }

    impl super::CacheData {
        /// Helper that updates fault counters for StatusC counts, and then returns an array of the ClearFlags we should clear
        fn update_status_c_fault_counts(&self, readings: &IndexByChip<Reading<StatusC>>) -> IndexByChip<ClearFlags> {
            let mut clears: IndexByChip<ClearFlags> = IndexByChip::from_fn(|_| ClearFlags::new());

            self.fault_counts.lock(|cell| {
                let mut counts = cell.get();

                for (chip, reading) in readings.iter() {
                    // if the PEC failed then we shouldnt count any of those fault flags because they could just be junk. for the same reason, we dont want to W1C those flags either. if they are really set then they will appear when we have a read with a PEC that actually passes
                    if !reading.pec().is_success() {
                        continue;
                    }

                    let statc = reading.data();
                    let c = counts.get_mut(chip);
                    let clear = clears.get_mut(chip);

                    macro_rules! record {
                        ($flag:ident, $count:expr, $with:ident) => {
                            if statc.$flag().is_set() {
                                $count = $count.saturating_add(1);
                                *clear = clear.$with(ClearAction::Clear);
                            }
                        };
                    }

                    record!(cs1flt, c.csxflt.cs1flt, with_cl_cs1flt);
                    record!(cs2flt, c.csxflt.cs2flt, with_cl_cs2flt);
                    record!(cs3flt, c.csxflt.cs3flt, with_cl_cs3flt);
                    record!(cs4flt, c.csxflt.cs4flt, with_cl_cs4flt);
                    record!(cs5flt, c.csxflt.cs5flt, with_cl_cs5flt);
                    record!(cs6flt, c.csxflt.cs6flt, with_cl_cs6flt);
                    record!(cs7flt, c.csxflt.cs7flt, with_cl_cs7flt);
                    record!(cs8flt, c.csxflt.cs8flt, with_cl_cs8flt);
                    record!(cs9flt, c.csxflt.cs9flt, with_cl_cs9flt);
                    record!(cs10flt, c.csxflt.cs10flt, with_cl_cs10flt);
                    record!(cs11flt, c.csxflt.cs11flt, with_cl_cs11flt);
                    record!(cs12flt, c.csxflt.cs12flt, with_cl_cs12flt);
                    record!(cs13flt, c.csxflt.cs13flt, with_cl_cs13flt);
                    record!(cs14flt, c.csxflt.cs14flt, with_cl_cs14flt);
                    record!(cs15flt, c.csxflt.cs15flt, with_cl_cs15flt);
                    record!(cs16flt, c.csxflt.cs16flt, with_cl_cs16flt);

                    record!(smed, c.smed, with_cl_smed);
                    record!(sed, c.sed, with_cl_sed);
                    record!(cmed, c.cmed, with_cl_cmed);
                    record!(ced, c.ced, with_cl_ced);
                    record!(vd_uv, c.vd_uv, with_cl_vduv);
                    record!(vd_ov, c.vd_ov, with_cl_vdov);
                    record!(va_uv, c.va_uv, with_cl_vauv);
                    record!(va_ov, c.va_ov, with_cl_vaov);
                    record!(oscchk, c.oscchk, with_cl_oscchk);
                    record!(tmodchk, c.tmodchk, with_cl_tmode);
                    record!(thsd, c.thsd, with_cl_thsd);
                    record!(spiflt, c.spiflt, with_cl_spiflt);
                    record!(vde, c.vde, with_cl_vde);
                    record!(vdel, c.vdel, with_cl_vdel);

                    // SPECIFIC CASE FOR THE SLEEP BIT
                    // the SLEEP bit gets read and cleared by the `Service` as part of the sleep detection stuff.
                    // so we should add increment the flag's counter in the fault counter, but shouldnt clear it. this way, the `Service` will be able to read the SLEEP bit.
                    // this makes it so the Service can correctly detect sleep and flag the app so we can reset (which includes clearing the SLEEP bit).
                    //
                    // this is kinda jank. realistically the Service should probably be the one that owns all the flag clears/fault counting, or maybe the sleep detection stuff (and maybe isospi recovery) should just be moved into the application entirely.
                    // but for now this is the most straightforward solution
                    if statc.sleep().is_set() {
                        c.sleep = c.sleep.saturating_add(1);
                        // dont clear it!!!
                    }
                }

                cell.set(counts);
            });

            clears
        }

        /// Updates StatusC.
        pub(in crate::segments) async fn update_status_c(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let statc = self.status_c.update(api).await?;
            let clears = self.update_status_c_fault_counts(statc.data());

            if let Err(err) = api.write(&clears.into_array()).await {
                defmt::error!("Segments: Cache: in `update_status_c()`: ClearFlags write failed. Error: {}", err);
                return Err(UpdateError::ClearFlagsError(err));
            }

            Ok(())
        }

        /// Gets the current cached StatusC data.
        pub fn get_status_c(&self) -> &CacheData {
            &self.status_c
        }
    }
}

/// Status D register group.
pub mod status_d {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, StatusD, UpdateError, RegisterCacheReadHandle, Reading, ClearOvervoltageUndervoltage, ClearAction};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::segments::chips::cells::{IndexByCell, CellId};
    use adbms6830b::chip::registers::status::types::d::{CellUndervoltageFlag, CellOvervoltageFlag, OscillatorCheckCounter};

    /// "Nice" data for this group. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<NiceDataChip>;

    /// Undervoltage/overvoltage flag state for an individual cell.
    #[derive(Copy, Clone)]
    pub struct CellUndervoltageOvervoltageState {
        pub undervoltage: CellUndervoltageFlag,
        pub overvoltage: CellOvervoltageFlag,
    }

    /// "Nice data" for a single chip.
    #[derive(Copy, Clone)]
    pub struct NiceDataChip {
        pub cell_undervoltage_overvoltage_state: IndexByCell<CellUndervoltageOvervoltageState>,
        pub oscillator_check_counter: OscillatorCheckCounter,
    }

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for StatusD register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw StatusD register reading.
    pub struct Raw {
        statd: RegisterCache<StatusD>,
    }
    impl Raw {
        pub fn statd(&self) -> &RegisterCacheReadHandle<StatusD> {
            self.statd.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { statd: RegisterCache::new() }
        }

        /// Updates the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<RegisterCacheData<StatusD>, UpdateError> {
            self.statd.update(api).await
        }

        /// Converts register readings into NiceData.
        fn nice(statd: RegisterCacheData<StatusD>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                let chip_statd = statd.data().get(chip).data();
                NiceDataChip {
                    cell_undervoltage_overvoltage_state: IndexByCell::from_fn(|cell| match cell {
                        CellId::Cell1 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c1uv(), overvoltage: chip_statd.c1ov() },
                        CellId::Cell2 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c2uv(), overvoltage: chip_statd.c2ov() },
                        CellId::Cell3 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c3uv(), overvoltage: chip_statd.c3ov() },
                        CellId::Cell4 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c4uv(), overvoltage: chip_statd.c4ov() },
                        CellId::Cell5 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c5uv(), overvoltage: chip_statd.c5ov() },
                        CellId::Cell6 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c6uv(), overvoltage: chip_statd.c6ov() },
                        CellId::Cell7 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c7uv(), overvoltage: chip_statd.c7ov() },
                        CellId::Cell8 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c8uv(), overvoltage: chip_statd.c8ov() },
                        CellId::Cell9 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c9uv(), overvoltage: chip_statd.c9ov() },
                        CellId::Cell10 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c10uv(), overvoltage: chip_statd.c10ov() },
                        CellId::Cell11 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c11uv(), overvoltage: chip_statd.c11ov() },
                        CellId::Cell12 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c12uv(), overvoltage: chip_statd.c12ov() },
                        CellId::Cell13 => CellUndervoltageOvervoltageState { undervoltage: chip_statd.c13uv(), overvoltage: chip_statd.c13ov() },
                    }),
                    oscillator_check_counter: chip_statd.oc_cntr(),
                }
            });

            Nice { data, oldest_read: statd.last_sucessful_read() }
        }
    }

    impl CacheData {
        /// Updates StatusD cache.
        ///
        /// ### Returns
        /// Will return `Ok(readings)`, or `Err(UpdateError)` if an error occurred. If this returns `Ok(readings)`, the cached data was updated correctly and can be read now.
        async fn update(&self, api: &mut alias::Api) -> Result<RegisterCacheData<StatusD>, UpdateError> {
            let statd = self.raw.update(api).await?;
            self.nice.set(Raw::nice(statd));
            Ok(statd)
        }
    }

    impl super::CacheData {
        /// Helper that updates fault counters for StatusD counts, and then returns an array of the ClearOvervoltageUndervoltage we should clear
        fn update_status_d_fault_counts(&self, readings: &IndexByChip<Reading<StatusD>>) -> IndexByChip<ClearOvervoltageUndervoltage> {
            let mut clears: IndexByChip<ClearOvervoltageUndervoltage> = IndexByChip::from_fn(|_| ClearOvervoltageUndervoltage::new());

            self.fault_counts.lock(|cell| {
                let mut counts = cell.get();

                for (chip, reading) in readings.iter() {
                    // if the PEC failed then we shouldnt count any of those fault flags because they could just be junk. for the same reason, we dont want to W1C those flags either. if they are really set then they will appear when we have a read with a PEC that actually passes
                    if !reading.pec().is_success() {
                        continue;
                    }

                    let statd = reading.data();
                    let c = counts.get_mut(chip);
                    let clear = clears.get_mut(chip);

                    macro_rules! record {
                        ($flag:ident, $count:expr, $with:ident) => {
                            if statd.$flag().is_set() {
                                $count = $count.saturating_add(1);
                                *clear = clear.$with(ClearAction::Clear);
                            }
                        };
                    }

                    record!(c1uv, c.cxovuv.c1uv, with_cl_c1uv);
                    record!(c1ov, c.cxovuv.c1ov, with_cl_c1ov);
                    record!(c2uv, c.cxovuv.c2uv, with_cl_c2uv);
                    record!(c2ov, c.cxovuv.c2ov, with_cl_c2ov);
                    record!(c3uv, c.cxovuv.c3uv, with_cl_c3uv);
                    record!(c3ov, c.cxovuv.c3ov, with_cl_c3ov);
                    record!(c4uv, c.cxovuv.c4uv, with_cl_c4uv);
                    record!(c4ov, c.cxovuv.c4ov, with_cl_c4ov);
                    record!(c5uv, c.cxovuv.c5uv, with_cl_c5uv);
                    record!(c5ov, c.cxovuv.c5ov, with_cl_c5ov);
                    record!(c6uv, c.cxovuv.c6uv, with_cl_c6uv);
                    record!(c6ov, c.cxovuv.c6ov, with_cl_c6ov);
                    record!(c7uv, c.cxovuv.c7uv, with_cl_c7uv);
                    record!(c7ov, c.cxovuv.c7ov, with_cl_c7ov);
                    record!(c8uv, c.cxovuv.c8uv, with_cl_c8uv);
                    record!(c8ov, c.cxovuv.c8ov, with_cl_c8ov);
                    record!(c9uv, c.cxovuv.c9uv, with_cl_c9uv);
                    record!(c9ov, c.cxovuv.c9ov, with_cl_c9ov);
                    record!(c10uv, c.cxovuv.c10uv, with_cl_c10uv);
                    record!(c10ov, c.cxovuv.c10ov, with_cl_c10ov);
                    record!(c11uv, c.cxovuv.c11uv, with_cl_c11uv);
                    record!(c11ov, c.cxovuv.c11ov, with_cl_c11ov);
                    record!(c12uv, c.cxovuv.c12uv, with_cl_c12uv);
                    record!(c12ov, c.cxovuv.c12ov, with_cl_c12ov);
                    record!(c13uv, c.cxovuv.c13uv, with_cl_c13uv);
                    record!(c13ov, c.cxovuv.c13ov, with_cl_c13ov);
                    record!(c14uv, c.cxovuv.c14uv, with_cl_c14uv);
                    record!(c14ov, c.cxovuv.c14ov, with_cl_c14ov);
                    record!(c15uv, c.cxovuv.c15uv, with_cl_c15uv);
                    record!(c15ov, c.cxovuv.c15ov, with_cl_c15ov);
                    record!(c16uv, c.cxovuv.c16uv, with_cl_c16uv);
                    record!(c16ov, c.cxovuv.c16ov, with_cl_c16ov);
                }

                cell.set(counts);
            });

            clears
        }

        /// Updates StatusD.
        pub(in crate::segments) async fn update_status_d(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            let statd = self.status_d.update(api).await?;
            let clears = self.update_status_d_fault_counts(statd.data());

            if let Err(err) = api.write(&clears.into_array()).await {
                defmt::error!("Segments: Cache: in `update_status_d()`: ClearOvervoltageUndervoltage write failed. Error: {}", err);
                return Err(UpdateError::ClearFlagsError(err));
            }

            Ok(())
        }

        /// Gets the current cached StatusD data.
        pub fn get_status_d(&self) -> &CacheData {
            &self.status_d
        }
    }
}

/// Register groups AuxillaryA through D.
pub mod aux {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, AuxillaryA, AuxillaryB, AuxillaryC, AuxillaryD, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::Voltage;
    use uom::si::{electric_potential::microvolt};
    use crate::segments::chips::gpios::{IndexByGpio, GpioId};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<NiceDataChip>;

    /// "Nice data" for a single chip.
    #[derive(Copy, Clone)]
    pub struct NiceDataChip {
        /// GPIO voltages.
        pub gpio_voltages: IndexByGpio<Voltage>,
        /// VMV voltage result.
        pub vmv: Voltage,
        /// VPV voltage result.
        pub vpv: Voltage,
    }

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for Aux register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw Aux register readings.
    pub struct Raw {
        auxa: RegisterCache<AuxillaryA>,
        auxb: RegisterCache<AuxillaryB>,
        auxc: RegisterCache<AuxillaryC>,
        auxd: RegisterCache<AuxillaryD>,
    }
    impl Raw {
        pub fn auxa(&self) -> &RegisterCacheReadHandle<AuxillaryA> {
            self.auxa.read_handle()
        }
        pub fn auxb(&self) -> &RegisterCacheReadHandle<AuxillaryB> {
            self.auxb.read_handle()
        }
        pub fn auxc(&self) -> &RegisterCacheReadHandle<AuxillaryC> {
            self.auxc.read_handle()
        }
        pub fn auxd(&self) -> &RegisterCacheReadHandle<AuxillaryD> {
            self.auxd.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self {
                auxa: RegisterCache::new(),
                auxb: RegisterCache::new(),
                auxc: RegisterCache::new(),
                auxd: RegisterCache::new(),
            }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let a = self.auxa.update(api).await?;
            let b = self.auxb.update(api).await?;
            let c = self.auxc.update(api).await?;
            let d = self.auxd.update(api).await?;

            Ok(Self::nice(a, b, c, d))
        }

        /// Converts register readings into NiceData.
        fn nice(a: RegisterCacheData<AuxillaryA>, b: RegisterCacheData<AuxillaryB>, c: RegisterCacheData<AuxillaryC>, d: RegisterCacheData<AuxillaryD>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| NiceDataChip {
                gpio_voltages: IndexByGpio::from_fn(|gpio| match gpio {
                    GpioId::Gpio1 => Voltage::new::<microvolt>(a.data().get(chip).data().g1v().as_microvolts() as f32),
                    GpioId::Gpio2 => Voltage::new::<microvolt>(a.data().get(chip).data().g2v().as_microvolts() as f32),
                    GpioId::Gpio3 => Voltage::new::<microvolt>(a.data().get(chip).data().g3v().as_microvolts() as f32),

                    GpioId::Gpio4 => Voltage::new::<microvolt>(b.data().get(chip).data().g4v().as_microvolts() as f32),
                    GpioId::Gpio5 => Voltage::new::<microvolt>(b.data().get(chip).data().g5v().as_microvolts() as f32),
                    GpioId::Gpio6 => Voltage::new::<microvolt>(b.data().get(chip).data().g6v().as_microvolts() as f32),

                    GpioId::Gpio7 => Voltage::new::<microvolt>(c.data().get(chip).data().g7v().as_microvolts() as f32),
                    GpioId::Gpio8 => Voltage::new::<microvolt>(c.data().get(chip).data().g8v().as_microvolts() as f32),
                    GpioId::Gpio9 => Voltage::new::<microvolt>(c.data().get(chip).data().g9v().as_microvolts() as f32),

                    GpioId::Gpio10 => Voltage::new::<microvolt>(d.data().get(chip).data().g10v().as_microvolts() as f32),
                }),
                vmv: Voltage::new::<microvolt>(d.data().get(chip).data().vmv().as_microvolts() as f32),
                vpv: Voltage::new::<microvolt>(d.data().get(chip).data().vpv().as_microvolts() as f32),
            });

            Nice {
                data,
                oldest_read: a.last_sucessful_read().min(b.last_sucessful_read()).min(c.last_sucessful_read()).min(d.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates caches AuxillaryA through D with new data.
        ///
        /// This doesn't run the `autoconvert` function!! The caller should do that. Otherwise this update will basically do nothing
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
        /// Updates Aux.
        pub(in crate::segments) async fn update_aux(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.aux.update(api).await
        }

        /// Gets the current cached Aux data.
        pub fn get_aux(&self) -> &CacheData {
            &self.aux
        }
    }
}

/// StatusA register group.
pub mod status_a {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, StatusA, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::{Voltage, Temperature, microcelcius};
    use uom::si::{electric_potential::microvolt};

    /// "Nice" data for this group. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<NiceDataChip>;

    /// "Nice data" for a single chip.
    #[derive(Copy, Clone)]
    pub struct NiceDataChip {
        /// Second reference voltage.
        pub vref2: Voltage,
        /// Internal die temperature.
        pub itmp: Temperature,
    }

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for StatusA register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw StatusA register reading.
    pub struct Raw {
        stata: RegisterCache<StatusA>,
    }
    impl Raw {
        pub fn stata(&self) -> &RegisterCacheReadHandle<StatusA> {
            self.stata.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { stata: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let stata = self.stata.update(api).await?;

            Ok(Self::nice(stata))
        }

        /// Converts register readings into NiceData.
        fn nice(stata: RegisterCacheData<StatusA>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| NiceDataChip {
                vref2: Voltage::new::<microvolt>(stata.data().get(chip).data().vref2().as_microvolts() as f32),
                itmp: Temperature::new::<microcelcius>(stata.data().get(chip).data().itmp().as_microcelsius() as f32),
            });

            Nice { data, oldest_read: stata.last_sucessful_read() }
        }
    }

    impl CacheData {
        /// Updates StatusA cache with new data.
        ///
        /// This doesn't run the `autoconvert` function!! The caller should do that. Otherwise this update will basically do nothing
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
        /// Updates StatusA.
        pub(in crate::segments) async fn update_status_a(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.status_a.update(api).await
        }

        /// Gets the current cached StatusA data.
        pub fn get_status_a(&self) -> &CacheData {
            &self.status_a
        }
    }
}

/// StatusB register group.
pub mod status_b {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, StatusB, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::units::{Voltage};
    use uom::si::{electric_potential::microvolt};

    /// "Nice" data for this group. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<NiceDataChip>;

    /// "Nice data" for a single chip.
    #[derive(Copy, Clone)]
    pub struct NiceDataChip {
        /// Digital power supply voltage.
        pub vd: Voltage,
        /// Analog power supply voltage.
        pub va: Voltage,
        /// VREF2 voltage across resistor.
        pub vres: Voltage,
    }

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for StatusB register group.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw StatusB register reading.
    pub struct Raw {
        statb: RegisterCache<StatusB>,
    }
    impl Raw {
        pub fn statb(&self) -> &RegisterCacheReadHandle<StatusB> {
            self.statb.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { statb: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let statb = self.statb.update(api).await?;

            Ok(Self::nice(statb))
        }

        /// Converts register readings into NiceData.
        fn nice(statb: RegisterCacheData<StatusB>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| NiceDataChip {
                vd: Voltage::new::<microvolt>(statb.data().get(chip).data().vd().as_microvolts() as f32),
                va: Voltage::new::<microvolt>(statb.data().get(chip).data().va().as_microvolts() as f32),
                vres: Voltage::new::<microvolt>(statb.data().get(chip).data().vres().as_microvolts() as f32),
            });

            Nice { data, oldest_read: statb.last_sucessful_read() }
        }
    }

    impl CacheData {
        /// Updates StatusB cache with new data.
        ///
        /// This doesn't run the `autoconvert` function!! The caller should do that. Otherwise this update will basically do nothing
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
        /// Updates StatusB.
        pub(in crate::segments) async fn update_status_b(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.status_b.update(api).await
        }

        /// Gets the current cached StatusB data.
        pub fn get_status_b(&self) -> &CacheData {
            &self.status_b
        }
    }
}

/// Cache for PwmA and PwmB register groups.
pub mod pwm {
    use super::{RegisterCache, RegisterCacheData, IndexByChip, StartupCell, PwmA, PwmB, UpdateError, RegisterCacheReadHandle};
    use super::alias;
    use adbms6830b::chip::registers::pwm::types::PwmDutyCycleConfig;
    use crate::helpers::startup_cell::UnwriteableStartupCell;
    use crate::segments::chips::cells::{IndexByCell, CellId};

    /// "Nice" data for these groups. Doesn't contain any of the PEC metadata and such,
    /// so you should probably look at the Raw readings if interested in that.
    pub type NiceData = IndexByChip<IndexByCell<PwmDutyCycleConfig>>;

    /// Container for "nice" data.
    #[derive(Copy, Clone)]
    pub struct Nice {
        /// The actual data.
        data: NiceData,
        /// The timestamp of the oldest data that makes up this NiceData.
        ///
        /// For context, a NiceData may be sourced from slightly different SPI reads. These reads
        /// typically occur in short succession, but there is still a tiny
        /// bit of time between them. So this timestamp provides a bound on how
        /// stale data from this NiceData might be. In other words, no data in the NiceData is
        /// from before `oldest_read`.
        oldest_read: embassy_time::Instant,
    }
    impl Nice {
        /// "Nice" data.
        pub fn data(&self) -> &NiceData {
            &self.data
        }
        /// The timestamp of the oldest data that makes up this NiceData.
        pub fn oldest_read(&self) -> embassy_time::Instant {
            self.oldest_read
        }
    }

    /// Stored Cache Data for Pwm register groups.
    /// Contains both the raw register readings, and the "nice" data.
    pub struct CacheData {
        raw: Raw,
        nice: StartupCell<Nice>,
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
        pub const fn nice(&self) -> &UnwriteableStartupCell<Nice> {
            self.nice.to_unwriteable()
        }
    }

    /// Raw PwmA/B register readings.
    pub struct Raw {
        pwma: RegisterCache<PwmA>,
        pwmb: RegisterCache<PwmB>,
    }
    impl Raw {
        pub fn pwma(&self) -> &RegisterCacheReadHandle<PwmA> {
            self.pwma.read_handle()
        }
        pub fn pwmb(&self) -> &RegisterCacheReadHandle<PwmB> {
            self.pwmb.read_handle()
        }

        /// Creates new uninitialized `Raw`.
        const fn new() -> Self {
            Self { pwma: RegisterCache::new(), pwmb: RegisterCache::new() }
        }

        /// Updates the Raw. If all updates succeed, this returns a NiceData created from the Raw.
        async fn update(&self, api: &mut alias::Api) -> Result<Nice, UpdateError> {
            let pwma = self.pwma.update(api).await?;
            let pwmb = self.pwmb.update(api).await?;

            Ok(Self::nice(pwma, pwmb))
        }

        /// Converts register readings into NiceData.
        fn nice(pwma: RegisterCacheData<PwmA>, pwmb: RegisterCacheData<PwmB>) -> Nice {
            let data: NiceData = IndexByChip::from_fn(|chip| {
                IndexByCell::from_fn(|cell| match cell {
                    CellId::Cell1 => pwma.data().get(chip).data().pwm1(),
                    CellId::Cell2 => pwma.data().get(chip).data().pwm2(),
                    CellId::Cell3 => pwma.data().get(chip).data().pwm3(),
                    CellId::Cell4 => pwma.data().get(chip).data().pwm4(),
                    CellId::Cell5 => pwma.data().get(chip).data().pwm5(),
                    CellId::Cell6 => pwma.data().get(chip).data().pwm6(),
                    CellId::Cell7 => pwma.data().get(chip).data().pwm7(),
                    CellId::Cell8 => pwma.data().get(chip).data().pwm8(),
                    CellId::Cell9 => pwma.data().get(chip).data().pwm9(),
                    CellId::Cell10 => pwma.data().get(chip).data().pwm10(),
                    CellId::Cell11 => pwma.data().get(chip).data().pwm11(),
                    CellId::Cell12 => pwma.data().get(chip).data().pwm12(),
                    CellId::Cell13 => pwmb.data().get(chip).data().pwm13(),
                })
            });

            Nice {
                data,
                oldest_read: pwma.last_sucessful_read().min(pwmb.last_sucessful_read()),
            }
        }
    }

    impl CacheData {
        /// Updates PwmA/B cache with new data.
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
        /// Updates PWM.
        pub(in crate::segments) async fn update_pwm(&self, api: &mut alias::Api) -> Result<(), UpdateError> {
            self.pwm.update(api).await
        }

        /// Gets the current cached PwmA/B data.
        pub fn get_pwm(&self) -> &CacheData {
            &self.pwm
        }
    }
}
