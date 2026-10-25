//! Device construction, startup configuration, read jobs, and the HV plate task.

use adbms2950::chip::commands;
use embassy_time::{Duration, Instant, Ticker, Timer};
use static_cell::StaticCell;

use super::cache::{self, UpdateError};
use super::isospi_recovery;
use super::precharge;
use super::soc;
use crate::broadcast::Broadcast;
use crate::job_diagnostics::JobDiagnosticsContainer;
use adbms2950::api::Error;
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
    use embedded_hal_bus::spi::ExclusiveDevice;
    use crate::helpers::precision_delay::PrecisionDelay;

    /// Type alias representing a SPI controller that implements `SpiDevice` from `embedded_hal_async`.
    ///
    /// Must be built with `ExclusiveDevice::new`, not `new_no_delay`: the driver's wake-up pulse
    /// is a delay-only transaction and panics without delay support.
    #[cfg(feature = "hil")]
    pub type SpiDevice = super::super::hil::HilDevice;
    #[cfg(not(feature = "hil"))]
    pub type SpiDevice = ExclusiveDevice<Spi<'static, Async, Master>, Output<'static>, PrecisionDelay>;

    /// The error type our `SpiDevice` produces.
    pub type SpiError = <SpiDevice as embedded_hal_async::spi::ErrorType>::Error;

    /// Type alias for the stateful API over both lines.
    pub type Api = adbms2950::api::Api<SpiDevice, PrecisionDelay>;
}

/// Holds the ADBMS2950B device api
struct HvPlate {
    /// isoSPI break detection and recovery.
    recovery_service: isospi_recovery::Service,
    api: &'static mut alias::Api,
}

impl HvPlate {
    fn new(r: crate::HvPlateResources, delay: crate::helpers::precision_delay::PrecisionDelay) -> Self {
        #[cfg(not(feature = "hil"))]
        let (spi_a, spi_b) = {
            use embedded_hal_bus::spi::ExclusiveDevice;

            let mut spi_config = embassy_stm32::spi::Config::default();
            spi_config.frequency = embassy_stm32::time::mhz(2);

            let linea_spi = embassy_stm32::spi::Spi::new(r.linea_spi, r.linea_sck, r.linea_mosi, r.linea_miso, r.linea_tx_dma, r.linea_rx_dma, Irqs, spi_config);
            let linea_cs = embassy_stm32::gpio::Output::new(r.linea_cs, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::High);
            let linea_spi: alias::SpiDevice = ExclusiveDevice::new(linea_spi, linea_cs, delay).unwrap();

            let lineb_spi = embassy_stm32::spi::Spi::new(r.lineb_spi, r.lineb_sck, r.lineb_mosi, r.lineb_miso, r.lineb_tx_dma, r.lineb_rx_dma, Irqs, spi_config);
            let lineb_cs = embassy_stm32::gpio::Output::new(r.lineb_cs, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::High);
            let lineb_spi: alias::SpiDevice = ExclusiveDevice::new(lineb_spi, lineb_cs, delay).unwrap();
            (linea_spi, lineb_spi)
        };
        #[cfg(feature = "hil")]
        let (spi_a, spi_b) = (super::hil::HilDevice::new(r), super::hil::HilDevice::disabled());

        static API: StaticCell<alias::Api> = StaticCell::new();
        let api: &'static mut alias::Api = API.init(alias::Api::new(spi_a, spi_b, delay, delay));

        Self { api, recovery_service: isospi_recovery::Service::new(Instant::now()) }
    }

    /// Keeps the chip configured and the isoSPI link on a working port.
    /// Returns true if restarted the chip
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
                    // Enables HV sense dividers, so TS and BATT can be read
                    .with_gpo2c(GpoOutputState::Driven)
                    .with_gpo2od(GpoDriveMode::PushPull)
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

                let deadline = Instant::now() + Duration::from_millis(adbms2950::api::conversion_times::IXADC_INIT_MAX_MS as u64);

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

        // u_TODO - make sure relay control is wired to gpo4 on 27A HV Plate
        let state = if energized { GpoOutputState::PulledLow } else { GpoOutputState::Driven };
        self.api.modify_configa(|cfg| cfg.with_gpo4c(state)).await
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

            self.api.command(commands::misc::snap()).await.map_err(UpdateError::SnapError)?;

            // Run commands in async block so fail here still tries to unsnap
            let reads = async {
                cache::CACHE.update_flag(self.api).await?;
                #[cfg(not(feature = "hil"))]
                cache::CACHE.update_accumulated(self.api).await?;
                cache::CACHE.update_current_voltage(self.api).await?;
                Ok::<(), UpdateError>(())
            }
            .await;

            let unsnap = self.api.command(commands::misc::unsnap()).await;
            reads?;
            unsnap.map_err(UpdateError::UnsnapError)?;

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
        use embassy_sync::watch::Watch;

        const HV_PLATE_FRESH_DATA_MAX_WAITERS: usize = 10;
        pub static HV_PLATE_FRESH_DATA_SIGNAL: Broadcast<ThreadModeRawMutex, HV_PLATE_FRESH_DATA_MAX_WAITERS> = Broadcast::new();

        /// Helper info about chip to do coulomb counting properly.
        #[derive(Copy, Clone)]
        pub struct AccCycle {
            pub restarted_at: Option<Instant>,
            pub fresh: bool,
        }
        const ACC_DATA_RECEIVERS: usize = 2;
        pub static ACC_DATA_SIGNAL: Watch<ThreadModeRawMutex, AccCycle, ACC_DATA_RECEIVERS> = Watch::new();
    }

    impl HvPlate {
        /// Emits device health to `defmt_monitor`.
        ///
        /// Only the things that live on the `Api` and so are not reachable from the cache: the
        /// command counter, PEC tallies, and which line is in use. The measured values are
        /// emitted by `crate::debug::hv_plate_debug`
        fn log_diagnostics(&self) {
            let api = &self.api;
            let device = api.device();
            defmt_monitor::monitor!("HvPlate/IsoSpi/ServiceState", desc = "isoSPI recovery state: Normal, BreakDetected, Verifying, ServiceSuccess, ServiceFailed.", "{}", self.recovery_service.state());
            defmt_monitor::monitor!("HvPlate/IsoSpi/ErrorsInWindow", desc = "PEC failures accumulated since the detector last re-baselined. A spike over 0 arms a 1500 ms window; over 16 when it closes declares a break.", "{=u32}", self.recovery_service.errors_in_window(api));
            defmt_monitor::monitor!("HvPlate/Device/ActiveLine", desc = "Which isoSPI line is currently in use. The service switches this when a line's failure rate goes over threshold.", "{}", api.active_line());
            defmt_monitor::monitor!("HvPlate/Device/LineAErrorCount", desc = "Transactions that have failed on isoSPI line A since boot.", "{=u32}", api.line_error_count(adbms2950::api::LineId::A));
            defmt_monitor::monitor!("HvPlate/Device/LineBErrorCount", desc = "Transactions that have failed on isoSPI line B since boot.", "{=u32}", api.line_error_count(adbms2950::api::LineId::B));
            defmt_monitor::monitor!("HvPlate/Device/PecSuccessCount", desc = "Reads whose data PEC verified, since boot.", "{=u32}", device.pec_success_count());
            defmt_monitor::monitor!("HvPlate/Device/PecFailedCount", desc = "Reads whose data PEC did not verify, since boot.", "{=u32}", device.pec_failed_count());
            #[cfg(not(feature = "hil"))]
            {
                defmt_monitor::monitor!("HvPlate/Device/ExpectedCommandCounter", desc = "Command counter we expect the chip to report next.", "{=u8}", device.expected_command_counter());
                defmt_monitor::monitor!("HvPlate/Device/ReportedCommandCounter", desc = "Command counter the chip reported on the most recent read. 255 if it has never been read.", "{=u8}", device.reported_command_counter().unwrap_or(u8::MAX));
            }
            defmt_monitor::monitor!("HvPlate/Device/SuspectedReset", desc = "True if the chip reported counter 0 unexpectedly, meaning it rebooted and lost its configuration.", "{=bool}", device.suspected_reset());
        }
    }

    #[embassy_executor::task]
    pub async fn hv_plate_task(r: crate::HvPlateResources, delay: crate::helpers::precision_delay::PrecisionDelay) {
        /// With ACCN = 32 and IXADC_CONVERSION_MS = 1, IVB1ACC is overwritten every 32 ms *nominally*,
        /// but the internal oscillator runs up to 10% fast, so a window can close in 32/1.1 = 29.09 ms.
        /// 25 ms leaves headroom for jitter on top.
        const PERIOD: Duration = Duration::from_millis(25);
        const DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(1);
        const DIAGNOSTIC_TICKS: u32 = (DIAGNOSTIC_PERIOD.as_ticks() / PERIOD.as_ticks()) as u32;

        let mut hv_plate = HvPlate::new(r, delay);

        let mut ticker = Ticker::every(PERIOD);
        let mut tick_count: u32 = 0;

        loop {
            let tick_start = Instant::now();

            // Run startup and break detection + recovery
            let restarted_at = hv_plate.run_service().await.then(Instant::now);

            let snap_ok = match hv_plate.job_update_snap_registers().await {
                Ok(()) => true,
                Err(err) => {
                    defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_snap_registers()` failed. Error: {}", err);
                    false
                },
            };
            // Hand the window to `soc_task`
            signal::ACC_DATA_SIGNAL.sender().send(signal::AccCycle { restarted_at, fresh: snap_ok });

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

            if let Err(err) = hv_plate.set_hv_relay(precharge::relay_should_be_closed()).await {
                defmt::error!("HvPlate: precharge: relay write failed, will retry next tick. Error: {}", err);
            }

            '_diagnostic: {
                if tick_count == 0 {
                    if let Err(err) = hv_plate.job_update_aux_registers().await {
                        defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_aux_registers()` failed. Error: {}", err);
                    }
                }

                if tick_count == DIAGNOSTIC_TICKS / 2 {
                    if let Err(err) = hv_plate.job_update_status_registers().await {
                        defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_status_registers()` failed. Error: {}", err);
                    }

                    hv_plate.log_diagnostics();
                }
            }

            defmt_monitor::monitor!("HvPlate/TaskDiagnostics/last_duration", desc = "Duration of the most recent hv_plate task cycle, in ms.", "{=u64}", Instant::now().saturating_duration_since(tick_start).as_millis());

            tick_count = (tick_count.wrapping_add(1)) % DIAGNOSTIC_TICKS;
            ticker.next().await;
        }
    }
}
