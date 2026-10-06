//! Device construction, startup configuration, read jobs, and the HV plate task.

use adbms2950::api::SnappedError;
use adbms2950::chip::commands;
use embassy_time::{Duration, Instant, Ticker, Timer};
use static_cell::StaticCell;

use crate::helpers::Deadline;

use super::cache::{self, UpdateError};
use super::isospi_recovery;
use super::precharge;
use super::soc;
use crate::broadcast::Broadcast;
use crate::job_diagnostics::JobDiagnosticsContainer;
use adbms2950::line::Error;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

#[cfg(not(feature = "hil"))]
embassy_stm32::bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH4>;
    GPDMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH5>;
    GPDMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH6>;
    GPDMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH7>;
});

pub mod alias {
    #[cfg(not(feature = "hil"))]
    use embassy_stm32::{
        gpio::Output,
        mode::Async,
        spi::{Spi, mode::Master},
    };
    #[cfg(not(feature = "hil"))]
    use embassy_time::Delay;
    #[cfg(not(feature = "hil"))]
    use embedded_hal_bus::spi::ExclusiveDevice;

    /// Type alias representing a SPI controller that implements `SpiDevice` from `embedded_hal_async`.
    ///
    /// Must be built with `ExclusiveDevice::new`, not `new_no_delay`: the driver's wake-up pulse
    /// is a delay-only transaction and panics without delay support.
    #[cfg(feature = "hil")]
    pub type SpiDevice = super::super::hil::HilDevice;
    #[cfg(not(feature = "hil"))]
    pub type SpiDevice = ExclusiveDevice<Spi<'static, Async, Master>, Output<'static>, Delay>;

    /// The error type our `SpiDevice` produces.
    pub type SpiError = <SpiDevice as embedded_hal_async::spi::ErrorType>::Error;

    /// Type alias representing one isoSPI line to our ADBMS2950B.
    pub type Line = adbms2950::line::Line<SpiDevice>;

    /// Type alias for the stateful API over both lines.
    pub type Api = adbms2950::api::Api<SpiDevice>;
}

/// Holds the ADBMS2950B device api
struct HvPlate {
    /// isoSPI break detection and recovery.
    recovery_service: isospi_recovery::Service,
    api: &'static mut alias::Api,
}

impl HvPlate {
    fn new(r: crate::HvPlateResources) -> Self {
        #[cfg(not(feature = "hil"))]
        let (line_a, line_b) = {
            use embassy_time::Delay;
            use embedded_hal_bus::spi::ExclusiveDevice;

            let mut spi_config = embassy_stm32::spi::Config::default();
            spi_config.frequency = embassy_stm32::time::mhz(2);

            let linea_spi = embassy_stm32::spi::Spi::new(r.linea_spi, r.linea_sck, r.linea_mosi, r.linea_miso, r.linea_tx_dma, r.linea_rx_dma, Irqs, spi_config);
            let linea_cs = embassy_stm32::gpio::Output::new(r.linea_cs, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::High);
            let linea_spi: alias::SpiDevice = ExclusiveDevice::new(linea_spi, linea_cs, Delay).unwrap();
            let line_a: alias::Line = alias::Line::new(linea_spi);

            let lineb_spi = embassy_stm32::spi::Spi::new(r.lineb_spi, r.lineb_sck, r.lineb_mosi, r.lineb_miso, r.lineb_tx_dma, r.lineb_rx_dma, Irqs, spi_config);
            let lineb_cs = embassy_stm32::gpio::Output::new(r.lineb_cs, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::High);
            let lineb_spi: alias::SpiDevice = ExclusiveDevice::new(lineb_spi, lineb_cs, Delay).unwrap();
            let line_b: alias::Line = alias::Line::new(lineb_spi);
            (line_a, line_b)
        };
        #[cfg(feature = "hil")]
        let (line_a, line_b) = (alias::Line::new(super::hil::HilDevice::new(r)), alias::Line::new(super::hil::HilDevice::disabled()));

        static API: StaticCell<alias::Api> = StaticCell::new();
        let api: &'static mut alias::Api = API.init(alias::Api::new(line_a, line_b));

        Self { api, recovery_service: isospi_recovery::Service::new(Instant::now()) }
    }

    /// Keeps the chip configured and the isoSPI link on a working port.
    async fn run_service(&mut self) -> bool {
        let startup = async |api: &mut alias::Api| -> Result<(), Error<alias::SpiError>> {
            use adbms2950::chip::registers::config_a::{ConfigA, types::*};

            #[cfg(not(feature = "hil"))]
            {
                // Reset to a known state and wait out the regulator startup.
                api.reset().await?;

                // Measurements before the references are up are not trustworthy.
                api.wait_for_reference().await?;
            }

            // Set up ConfigA
            let config_a = const {
                ConfigA::new()
                    // BATT and TS dividers sit on the 1.25 V rail, the shunt thermistor on ground.
                    .with_vs1(VoltageReferenceWide::Vref1p25)
                    .with_vs2(VoltageReferenceWide::Vref1p25)
                    .with_vs7(VoltageReference::Sgnd)
                    .with_acci(soc::ACCUMULATOR_DEPTH)
                    // HV control relay, open drain and active low. Starts released.
                    .with_gpo4c(GpoOutputState::Driven)
                    .with_gpo4od(GpoDriveMode::OpenDrain)
            };
            api.set_configa(config_a).await?;

            // Fault latches power up asserted. Clear them or everything reads as a live fault.
            api.write(adbms2950::chip::registers::flag::Flag::new().with_thsd(true)).await?;

            // Start continuous conversion. ACCI only relatches on ADI1, so this must stay after
            // `set_configa`.
            api.command(commands::adc::adi1(commands::adc::Redundancy::Enabled, commands::adc::Acquisition::Continuous, commands::adc::Diagnostic::Normal, commands::adc::OpenWire::Off)).await?;

            // Wait for the IxADC to finish calibrating
            {
                use adbms2950::chip::registers::status::{Status, types::InitializationStatus};

                let deadline = Instant::now() + Duration::from_millis(adbms2950::line::conversion_times::IXADC_INIT_MAX_MS as u64);

                loop {
                    if let Ok(status) = api.read::<Status>().await
                        && status.i1cal() == InitializationStatus::Complete
                    {
                        break;
                    }

                    if Instant::now() >= deadline {
                        defmt::warn!("HvPlate: startup: I1CAL never asserted within tIxADC_INIT; continuing anyway.");
                        break;
                    }

                    Timer::after_millis(1).await;
                }
            }

            defmt::info!("HvPlate: startup complete.");

            Ok(())
        };

        self.recovery_service.run(self.api, startup).await
    }

    /// Drives the HV control relay on GPO4. Active low, open drain.
    async fn set_hv_relay(&mut self, energized: bool) -> Result<(), Error<alias::SpiError>> {
        use adbms2950::chip::registers::config_a::types::GpoOutputState;

        let state = if energized {
            GpoOutputState::PulledLow
        } else {
            GpoOutputState::Driven
        };
        self.api.modify_configa(|cfg| cfg.with_gpo4c(state)).await
    }
}

/// PUBLIC API! for HV Plate
pub mod api {
    use crate::units::{Current, Ratio, Voltage};
    use core::cell::Cell;
    use embassy_sync::blocking_mutex::ThreadModeMutex;

    static TS_VOLTAGE: ThreadModeMutex<Cell<Option<Voltage>>> = ThreadModeMutex::new(Cell::new(None));
    static PACK_CURRENT: ThreadModeMutex<Cell<Option<Current>>> = ThreadModeMutex::new(Cell::new(None));
    static SOC: ThreadModeMutex<Cell<Option<Ratio>>> = ThreadModeMutex::new(Cell::new(None));
    static SOC_DRIFT: ThreadModeMutex<Cell<f32>> = ThreadModeMutex::new(Cell::new(0.0));

    /// Pack current
    pub fn pack_current() -> Option<Current> {
        shunt_current().or_else(|| PACK_CURRENT.lock(|cell| cell.get()))
    }

    /// The 2950 shunt reading, negated into "positive is discharge".
    fn shunt_current() -> Option<Current> {
        let raw = super::cache::CACHE.get_current_voltage();
        raw.try_nice().map(|n| -n.pack_current).ok()
    }

    /// Records a pack current reading from CAN. **Positive is discharge**
    pub fn store_pack_current(current: Current) {
        PACK_CURRENT.lock(|cell| cell.set(Some(current)));
    }

    /// State of charge, 0..1. `None` until an open-circuit voltage has seeded the count.
    pub fn state_of_charge() -> Option<Ratio> {
        SOC.lock(|cell| cell.get())
    }

    /// Signed drift
    pub fn soc_drift() -> f32 {
        SOC_DRIFT.lock(|cell| cell.get())
    }

    /// Publishes what `SocTracker` currently knows. Called by the HV plate task.
    pub(super) fn store_soc(state_of_charge: Option<Ratio>, drift: f32) {
        SOC.lock(|cell| cell.set(state_of_charge));
        SOC_DRIFT.lock(|cell| cell.set(drift));
    }

    /// Tractive-system voltage
    pub fn ts_voltage() -> Option<Voltage> {
        TS_VOLTAGE.lock(|cell| cell.get())
    }

    /// Records a TS voltage reading. Called by the CAN RX processor.
    pub fn store_ts_voltage(voltage: Voltage) {
        TS_VOLTAGE.lock(|cell| cell.set(Some(voltage)));
    }
}

pub mod jobs {
    use crate::job_diagnostics;

    use super::*;

    /// How long to allow a conversion to complete before giving up.
    #[cfg(not(feature = "hil"))]
    const CONVERSION_TIMEOUT: Duration = Duration::from_millis(100);

    impl HvPlate {
        /// Reads pack current, the accumulators, and FLAG inside one SNAP window.
        ///
        /// One window because FLAG's `i1cnt` counts the conversions the accumulator sums; read
        /// at different instants the coulomb count drifts.
        pub async fn job_update_snap_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            self.api
                .snapped(async |api| {
                    cache::CACHE.update_flag(api).await?;
                    #[cfg(not(feature = "hil"))]
                    cache::CACHE.update_accumulated(api).await?;
                    cache::CACHE.update_current_voltage(api).await?;
                    Ok(())
                })
                .await
                .map_err(|err| match err {
                    SnappedError::Snap(err) => UpdateError::SnapError(err),
                    SnappedError::Body(err) => err,
                    SnappedError::Unsnap(err) => UpdateError::UnsnapError(err),
                })?;

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_snap_registers", run.finish());

            Ok(())
        }

        /// Converts the voltage channels and reads TS voltage (V2) and the shunt thermistor (V7).
        ///
        /// One round-robin sweep covers both channels, so it is one conversion and two reads.
        pub async fn job_update_voltage_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            // HIL accepts conversion commands but does not serve conversion polls.
            #[cfg(feature = "hil")]
            self.api.command(commands::adc::adv(commands::adc::OpenWireVoltage::Off, commands::adc::VoltageChannel::RoundRobinCh0ToCh8)).await.map_err(UpdateError::ConversionError)?;
            #[cfg(not(feature = "hil"))]
            self.api.adv_autoconvert(commands::adc::OpenWireVoltage::Off, commands::adc::VoltageChannel::RoundRobinCh0ToCh8, CONVERSION_TIMEOUT).await.map_err(UpdateError::ConversionError)?;
            cache::CACHE.update_voltages(self.api).await?;

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_voltage_registers", run.finish());

            Ok(())
        }

        /// Converts and reads the AUX ADC rails and on-chip temperatures.
        pub async fn job_update_aux_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            // HIL accepts conversion commands but does not serve conversion polls.
            #[cfg(feature = "hil")]
            self.api.command(commands::adc::adx()).await.map_err(UpdateError::ConversionError)?;
            #[cfg(not(feature = "hil"))]
            self.api.adx_autoconvert(CONVERSION_TIMEOUT).await.map_err(UpdateError::ConversionError)?;
            cache::CACHE.update_aux(self.api).await?;

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_aux_registers", run.finish());

            Ok(())
        }

        /// Reads STATUS and the overcurrent comparator results. No conversion needed.
        pub async fn job_update_status_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            cache::CACHE.update_status(self.api).await?;

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_status_registers", run.finish());

            Ok(())
        }
    }
}

pub mod task {
    use super::*;

    /// `Broadcast` static for the HV plate task, so other tasks can await fresh cache data.
    pub mod signal {
        use super::*;

        const HV_PLATE_FRESH_DATA_MAX_WAITERS: usize = 10;
        pub static HV_PLATE_FRESH_DATA_SIGNAL: Broadcast<ThreadModeRawMutex, HV_PLATE_FRESH_DATA_MAX_WAITERS> = Broadcast::new();
    }

    impl HvPlate {
        /// Emits device health to `defmt_monitor`.
        ///
        /// Only the things that live on the `Api` and so are not reachable from the cache: the
        /// command counter, PEC tallies, and which line is in use. The measured values are
        /// emitted by `crate::debug::hv_plate_debug`
        #[cfg(not(feature = "hil"))]
        fn log_diagnostics(&self) {
            let api = &self.api;
            let device = api.device();
            defmt_monitor::monitor!("HvPlate/Device/ActiveLine", desc = "Which isoSPI line is currently in use. The service switches this when a line's failure rate goes over threshold.", "{}", api.active_line());
            defmt_monitor::monitor!("HvPlate/Device/LineAErrorCount", desc = "Transactions that have failed on isoSPI line A since boot.", "{=u32}", api.line_error_count(adbms2950::api::LineId::A));
            defmt_monitor::monitor!("HvPlate/Device/LineBErrorCount", desc = "Transactions that have failed on isoSPI line B since boot.", "{=u32}", api.line_error_count(adbms2950::api::LineId::B));
            defmt_monitor::monitor!("HvPlate/Device/PecSuccessCount", desc = "Reads whose data PEC verified, since boot.", "{=u32}", device.pec_success_count());
            defmt_monitor::monitor!("HvPlate/Device/PecFailedCount", desc = "Reads whose data PEC did not verify, since boot.", "{=u32}", device.pec_failed_count());
            defmt_monitor::monitor!("HvPlate/Device/ExpectedCommandCounter", desc = "Command counter we expect the chip to report next.", "{=u8}", device.expected_command_counter());
            defmt_monitor::monitor!("HvPlate/Device/ReportedCommandCounter", desc = "Command counter the chip reported on the most recent read. 255 if it has never been read.", "{=u8}", device.reported_command_counter().unwrap_or(u8::MAX));
            defmt_monitor::monitor!("HvPlate/Device/SuspectedReset", desc = "True if the chip reported counter 0 unexpectedly, meaning it rebooted and lost its configuration.", "{=bool}", device.suspected_reset());
        }
    }

    #[embassy_executor::task]
    pub async fn hv_plate_task(r: crate::HvPlateResources) {
        /// Read period.
        ///
        /// With [`soc::ACCUMULATOR_DEPTH`] at `Samples32` (ACCN = 32) and `IXADC_CONVERSION_MS = 1`,
        /// IVB1ACC is overwritten every 32 ms *nominally*, but the internal oscillator runs up to
        /// 10% fast, so a window can close in 32/1.1 = 29.09 ms. Reads must stay inside that --
        /// the guidance figure is 28.8 ms, and 25 ms leaves headroom for jitter on top. ~22%
        /// margin for jitter.
        const TICK: Duration = Duration::from_millis(25);
        /// How often the diagnostic reads run.
        const DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(1);

        let mut hv_plate = HvPlate::new(r);

        // Per-run state
        let mut precharge = precharge::Precharge::new();
        let mut soc = soc::SocTracker::new(Instant::now());

        let mut ticker = Ticker::every(TICK);

        // Aux fires on the first pass so there are diagnostics before the first second is out;
        // status is offset half a period so the two tend not to share a tick.
        let mut aux_due = Deadline::expire_now();
        let mut status_due = Deadline::expire_in(DIAGNOSTIC_PERIOD / 2);

        loop {
            let tick_start = Instant::now();

            // Run service to configure chip and also detect and recover from isospi break
            let restarted = hv_plate.run_service().await;

            // ADI1 zeroed I1CNT and relatched ACCI
            if restarted {
                soc.restart(Instant::now());
            }

            '_normal: {
                let snap_ok = match hv_plate.job_update_snap_registers().await {
                    Ok(()) => true,
                    Err(err) => {
                        defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_snap_registers()` failed. Error: {}", err);
                        false
                    },
                };

                let voltage_ok = match hv_plate.job_update_voltage_registers().await {
                    Ok(()) => true,
                    Err(err) => {
                        defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_voltage_registers()` failed. Error: {}", err);
                        false
                    },
                };

                if snap_ok & voltage_ok {
                    signal::HV_PLATE_FRESH_DATA_SIGNAL.signal();
                }

                // Re-seeds on every new rest period
                // `SocTracker::seed` ignores a snapshot it has already applied
                if let Some(analyzer) = crate::pack::analyzer::analyzer()
                    && let Some(settled_at) = analyzer.data.last_settled_at
                {
                    soc.seed(&soc::OcvSeed { min_cell_ocv: analyzer.data.min_ocv.value(), settled_at });
                }

                // Gated on a clean snap so IVB1ACC and FLAG are from the same window.
                if snap_ok {
                    soc.accumulate();
                }

                api::store_soc(soc.state_of_charge().and_then(crate::units::Ratio::from_ratio), soc.soc_drift());
            };

            // Run precharge and set relay
            let action = precharge.tick(Instant::now());
            defmt::trace!("HvPlate: precharge: {}", action.state);
            if let Err(err) = hv_plate.set_hv_relay(action.relay_closed).await {
                defmt::error!("HvPlate: precharge: relay write failed, will retry next tick. Error: {}", err);
            }

            '_diagnostic: {
                if aux_due.past() {
                    aux_due = Deadline::expire_in(DIAGNOSTIC_PERIOD);

                    if let Err(err) = hv_plate.job_update_aux_registers().await {
                        defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_aux_registers()` failed. Error: {}", err);
                    }
                }

                if status_due.past() {
                    status_due = Deadline::expire_in(DIAGNOSTIC_PERIOD);

                    if let Err(err) = hv_plate.job_update_status_registers().await {
                        defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_status_registers()` failed. Error: {}", err);
                    }

                    #[cfg(not(feature = "hil"))]
                    hv_plate.log_diagnostics();

                    defmt_monitor::monitor!("HvPlate/Soc/MissedWindows", desc = "Accumulator windows overwritten before the task read them. Should stay at zero; non-zero means the 25 ms budget is blown.", "{=u32}", soc.missed_windows());
                    defmt_monitor::monitor!("HvPlate/Soc/Desyncs", desc = "Observations rejected because I1CNT could have lapped (a gap over ~2 s).", "{=u32}", soc.desyncs());

                    if let Some(sample) = soc.last_sample() {
                        defmt_monitor::monitor!("HvPlate/Soc/AverageCurrent", desc = "Mean shunt current over the most recent accumulator window, in amps. Cross-check against the instantaneous reading from IVB1.", "{=f32}", sample.average_current.get::<uom::si::electric_current::ampere>());
                        defmt_monitor::monitor!("HvPlate/Soc/StateOfCharge", desc = "State of charge 0..1, or -1 while unreferenced.", "{=f32}", soc.state_of_charge().unwrap_or(-1.0));
                        defmt_monitor::monitor!("HvPlate/Soc/SampleCharge", desc = "Charge moved in the most recent accumulator window, in microcoulombs.", "{=i64}", sample.charge_microcoulombs);
                        defmt_monitor::monitor!("HvPlate/Soc/WindowsElapsed", desc = "Accumulator windows covered by the most recent sample. 1 is clean.", "{=u16}", sample.windows_elapsed);
                        defmt_monitor::monitor!("HvPlate/Soc/TConvUs", desc = "Measured t_CONV, the accumulator window, in us. Nominally 32000; drift from that is the internal oscillator, specified at +/-10%.", "{=u64}", soc.t_conv().as_micros());
                        defmt_monitor::monitor!("HvPlate/Soc/CC", desc = "The datasheet's CC: net charge moved since boot, in microcoulombs. Positive is discharge. Never reset -- not by a chip restart, not by a reseed.", "{=i64}", soc.cc_microcoulombs());
                        defmt_monitor::monitor!("HvPlate/Soc/Drift", desc = "Gap the last accepted OCV found against the coulomb count, in state of charge. Positive means the OCV read higher.", "{=f32}", soc.soc_drift());
                    }

                    defmt_monitor::monitor!("HvPlate/Precharge/State", desc = "Precharge supervisor state: 0 Open, 1 Floating, 2 Closed.", "{}", precharge::state());
                }
            }

            defmt_monitor::monitor!("HvPlate/TaskDiagnostics/last_duration", desc = "Duration of the most recent hv_plate task cycle, in ms.", "{=u64}", Instant::now().saturating_duration_since(tick_start).as_millis());

            ticker.next().await;
        }
    }
}
