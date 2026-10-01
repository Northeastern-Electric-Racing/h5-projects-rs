//! Device construction, startup configuration, read jobs, and the HV plate task.

use adbms2950::chip::commands;
use adbms2950::api::SnappedError;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Ticker};

use super::cache::{self, UpdateError};
use crate::job_diagnostics::JobDiagnosticsContainer;
use crate::broadcast::Broadcast;
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
        spi::{mode::Master, Spi},
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

static HV_PLATE: OnceLock<HvPlate> = OnceLock::new();

/// Public api for hv_plate
pub mod api {
    use adbms2950::line::Error;
    use super::HV_PLATE;
    use super::alias;

    /// Drives the HV control relay on GPO4. Active low, open drain.
    pub async fn set_hv_relay(energized: bool) -> Result<(), Error<alias::SpiError>> {
        use adbms2950::chip::registers::config_a::types::GpoOutputState;

        let state = if energized { GpoOutputState::PulledLow } else { GpoOutputState::Driven };
        HV_PLATE.get().await.api.lock().await.modify_configa(|cfg| cfg.with_gpo4c(state)).await
    }
}

/// Owns the ADBMS2950 and everything that talks to it.
struct HvPlate {
    api: Mutex<ThreadModeRawMutex, alias::Api>,
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

        Self { api: Mutex::new(alias::Api::new(line_a, line_b)) }
    }
}

/// isoSPI break detection and recovery for the HV plate.
///
/// Detection is a two-stage accumulator rather than a rate. A small spike in PEC errors *arms*
/// a window; a break is declared only if enough errors accumulate before the window closes.
mod service {
    use super::{alias, HvPlate};
    use adbms2950::api::LineId;
    use adbms2950::chip::commands;
    use adbms2950::line::Error;
    use embassy_time::{Duration, Instant, Timer};

    /// PEC errors in a closed window above which line A is declared broken.
    const PEC_ERROR_THRESHOLD: u32 = 16;
    /// PEC errors that arm the accumulation window.
    const PEC_ACCUM_START_THRESH: u32 = 3;
    /// How long an armed window runs.
    const ACCUM_PERIOD: Duration = Duration::from_millis(1500);
    /// PEC errors at or below which a verification pass counts as clean.
    const VALIDATION_THRESHOLD: u32 = 3;
    /// Verification passes allowed before recovery is declared failed.
    const VERIFICATION_READS: u32 = 5;
    /// Quiet period after boot before PEC errors are believed
    const STARTUP_MASK_TIME: Duration = Duration::from_millis(1500);

    #[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
    enum State {
        Normal,
        BreakDetected,
        Verifying,
        RecoverySuccess,
        RecoveryFailed,
    }

    /// Work `evaluate` could not do itself, because it needs the wire.
    pub(super) enum Action {
        None,
        /// The line was switched. Re-run startup to reconfigure and resynchronise the chip.
        Restart,
    }

    pub(super) struct Service {
        state: State,
        /// Lifetime PEC-failure count at the last accumulator reset.
        pec_baseline: u32,
        /// Set when a window is armed; `None` when idle.
        accum_deadline: Option<Instant>,
        /// PEC errors are ignored until this instant.
        startup_mask_until: Instant,
        verification_attempts: u32,
        /// Latches on the first successful recovery. There is only one spare line.
        recovery_successful: bool,
        /// Keeps the failure path from logging every cycle forever.
        fault_latched: bool,
        /// Whether the chip has been configured since it was last known to have reset.
        started: bool,
    }

    impl Service {
        pub(super) fn new(now: Instant) -> Self {
            Self {
                state: State::Normal,
                pec_baseline: 0,
                accum_deadline: None,
                startup_mask_until: now + STARTUP_MASK_TIME,
                verification_attempts: 0,
                recovery_successful: false,
                fault_latched: false,
                started: false,
            }
        }

        /// Whether the chip needs configuring: either it never has been, or it reset and threw
        /// the configuration away.
        ///
        /// The command counter only returns to 0 via `RSTCC` or `SRST`, so a device reporting 0
        /// when we expected otherwise has rebooted. Note the counter is stale for the rest of
        /// the cycle startup ran in, since nothing has read it back yet; the read jobs run
        /// immediately after, so it settles the same cycle.
        pub(super) fn needs_startup(&self, api: &alias::Api) -> bool {
            if !self.started {
                return true;
            }

            if api.device().suspected_reset() {
                defmt::warn!("HvPlate: Service: chip reported command counter 0 while we expected {}. It reset and lost its configuration; reconfiguring.", api.device().expected_command_counter());
                return true;
            }

            false
        }

        /// Records that startup succeeded.
        pub(super) const fn mark_started(&mut self) {
            self.started = true;
        }

        /// Advances the recovery state machine. Call once per cycle.
        pub(super) fn evaluate(&mut self, api: &mut alias::Api, now: Instant) -> Action {
            match self.state {
                State::Normal => {
                    if self.recovery_successful {
                        self.reset_accumulator(api);
                    } else {
                        self.detect_break(api, now);
                    }
                    Action::None
                },

                State::BreakDetected => {
                    defmt::warn!("HvPlate: isoSPI: recovery started, switching comms to line B.");
                    api.set_active_line(LineId::B);
                    self.reset_accumulator(api);
                    self.state = State::Verifying;
                    Action::Restart
                },

                State::Verifying => {
                    if self.verification_attempts >= VERIFICATION_READS {
                        defmt::error!("HvPlate: isoSPI: verification failed after {} attempts.", self.verification_attempts);
                        self.state = State::RecoveryFailed;
                    } else {
                        let errors = self.errors_since_reset(api);
                        if errors <= VALIDATION_THRESHOLD {
                            defmt::info!("HvPlate: isoSPI: line B verification passed. Recovery succeeded.");
                            self.state = State::RecoverySuccess;
                            self.recovery_successful = true;
                        } else {
                            defmt::warn!("HvPlate: isoSPI: line B verification failed ({} PEC errors).", errors);
                        }
                        self.reset_accumulator(api);
                        self.verification_attempts = self.verification_attempts.saturating_add(1);
                    }
                    Action::None
                },

                State::RecoverySuccess => {
                    // u_TODO - C also clears the comms fault and asks SoC to re-initialise from
                    // the minimum OCV here. Neither is available yet: the fault table only
                    // clears on its own timeout, and there is no SoC module to ask.
                    defmt::info!("HvPlate: isoSPI: recovery complete.");
                    self.state = State::Normal;
                    Action::None
                },

                State::RecoveryFailed => {
                    if !self.fault_latched {
                        defmt::error!("HvPlate: isoSPI: recovery failed. Comms fault latched.");
                        self.recovery_successful = false;
                        self.fault_latched = true;
                    }
                    self.reset_accumulator(api);
                    Action::None
                },
            }
        }

        /// Arms a window on a spike, and declares a break if the window closes over threshold.
        fn detect_break(&mut self, api: &alias::Api, now: Instant) {
            // Startup noise is not a break.
            if now < self.startup_mask_until {
                self.reset_accumulator(api);
                return;
            }

            let errors = self.errors_since_reset(api);

            match self.accum_deadline {
                None => {
                    if errors > PEC_ACCUM_START_THRESH {
                        self.accum_deadline = Some(now + ACCUM_PERIOD);
                    } else {
                        // Not climbing fast enough to be a break.
                        self.reset_accumulator(api);
                    }
                },
                Some(deadline) if now >= deadline => {
                    if errors > PEC_ERROR_THRESHOLD {
                        defmt::error!("HvPlate: isoSPI: line A break detected ({} PEC errors in the window).", errors);
                        self.state = State::BreakDetected;
                        let _ = crate::faults::try_queue(crate::faults::FaultId::HvPlateCommsFault);
                    }
                    self.accum_deadline = None;
                    self.reset_accumulator(api);
                },
                // Window still filling.
                Some(_) => {},
            }
        }

        fn errors_since_reset(&self, api: &alias::Api) -> u32 {
            api.device().pec_failed_count().saturating_sub(self.pec_baseline)
        }

        fn reset_accumulator(&mut self, api: &alias::Api) {
            self.pec_baseline = api.device().pec_failed_count();
        }
    }

    impl HvPlate {
        /// Brings the chip from reset to converting.
        async fn startup(api: &mut alias::Api) -> Result<(), Error<alias::SpiError>> {
            use adbms2950::chip::registers::config_a::{ConfigA, types::*};

            // HIL shares configuration startup but skips physical reset/reference polling.
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
                    .with_acci(AccumulatorDepth::Samples8)
                    // HV control relay, open drain and active low. Starts released.
                    .with_gpo4c(GpoOutputState::Driven)
                    .with_gpo4od(GpoDriveMode::OpenDrain)
            };
            api.set_configa(config_a).await?;

            // Fault latches power up asserted. Clear them or everything reads as a live fault.
            api.write(adbms2950::chip::registers::flag::Flag::new().with_thsd(true)).await?;

            // Start continuous conversion
            api.command(commands::adc::adi1(commands::adc::Redundancy::Enabled, commands::adc::Acquisition::Continuous, commands::adc::Diagnostic::Normal, commands::adc::OpenWire::Off)).await?;

            // Wait for the first conversion to land
            Timer::after_millis(adbms2950::line::conversion_times::IXADC_STARTUP_MAX_MS as u64).await;

            defmt::info!("HvPlate: startup complete.");

            Ok(())
        }

        /// Keeps the chip configured and the isoSPI link on a working port.
        pub(super) async fn run_service(&self, service: &mut Service) {
            let mut api = self.api.lock().await;

            if service.needs_startup(&api) {
                match HvPlate::startup(&mut api).await {
                    Ok(()) => service.mark_started(),
                    Err(err) => {
                        defmt::error!("HvPlate: `run_service()`: startup failed, will retry next cycle. Error: {}", err);
                        return;
                    },
                }
            }

            if let Action::Restart = service.evaluate(&mut api, Instant::now())
                && let Err(err) = HvPlate::startup(&mut api).await
            {
                defmt::error!("HvPlate: `run_service()`: restart after line switch failed. Error: {}", err);
            }
        }
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
        pub async fn job_update_snap_registers(&self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            self.api
                .lock()
                .await
                .snapped(async |api| {
                    cache::CACHE.update_current_voltage(api).await?;
                    #[cfg(not(feature = "hil"))]
                    cache::CACHE.update_accumulated(api).await?;
                    cache::CACHE.update_flag(api).await?;
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
        pub async fn job_update_voltage_registers(&self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            let mut api = self.api.lock().await;

            // HIL accepts conversion commands but does not serve conversion polls.
            #[cfg(feature = "hil")]
            api.command(commands::adc::adv(commands::adc::OpenWireVoltage::Off, commands::adc::VoltageChannel::RoundRobinCh0ToCh8)).await.map_err(UpdateError::ConversionError)?;
            #[cfg(not(feature = "hil"))]
            api.adv_autoconvert(commands::adc::OpenWireVoltage::Off, commands::adc::VoltageChannel::RoundRobinCh0ToCh8, CONVERSION_TIMEOUT).await.map_err(UpdateError::ConversionError)?;
            cache::CACHE.update_voltages(&mut api).await?;
            drop(api);

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_voltage_registers", run.finish());

            Ok(())
        }

        /// Converts and reads the AUX ADC rails and on-chip temperatures.
        pub async fn job_update_aux_registers(&self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            let mut api = self.api.lock().await;

            // HIL accepts conversion commands but does not serve conversion polls.
            #[cfg(feature = "hil")]
            api.command(commands::adc::adx()).await.map_err(UpdateError::ConversionError)?;
            #[cfg(not(feature = "hil"))]
            api.adx_autoconvert(CONVERSION_TIMEOUT).await.map_err(UpdateError::ConversionError)?;
            cache::CACHE.update_aux(&mut api).await?;
            drop(api);

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_aux_registers", run.finish());

            Ok(())
        }

        /// Reads STATUS and the overcurrent comparator results. No conversion needed.
        pub async fn job_update_status_registers(&self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            cache::CACHE.update_status(&mut *self.api.lock().await).await?;

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
        async fn log_diagnostics(&self) {
            let api = self.api.lock().await;
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
        const TICK: Duration = Duration::from_millis(50);
        const DIAGNOSTIC_TICKS: u32 = 20; // 1s intervals

        let hv_plate = HV_PLATE.get_or_init(|| HvPlate::new(r));
        let mut service = service::Service::new(Instant::now());
        let mut ticker = Ticker::every(TICK);
        let mut tick: u32 = 0;

        loop {
            let start_time = Instant::now();
            tick = tick.wrapping_add(1);

            // Re-arms and runs startup if the chip reset, then supervises the isoSPI link.
            hv_plate.run_service(&mut service).await;

            // Do the SPI transactions to update the register caches.
            let mut all_successful: bool = true;

            if let Err(err) = hv_plate.job_update_snap_registers().await {
                defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_snap_registers()` failed. Error: {}", err);
                all_successful = false;
            }

            if let Err(err) = hv_plate.job_update_voltage_registers().await {
                defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_voltage_registers()` failed. Error: {}", err);
                all_successful = false;
            }

            // `all_successful` covers only the per-tick jobs
            if all_successful {
                signal::HV_PLATE_FRESH_DATA_SIGNAL.signal();
            }

            if tick % DIAGNOSTIC_TICKS == 0
                && let Err(err) = hv_plate.job_update_aux_registers().await
            {
                defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_aux_registers()` failed. Error: {}", err);
            }

            if tick % DIAGNOSTIC_TICKS == DIAGNOSTIC_TICKS / 2 {
                if let Err(err) = hv_plate.job_update_status_registers().await {
                    defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_status_registers()` failed. Error: {}", err);
                }

                #[cfg(not(feature = "hil"))]
                hv_plate.log_diagnostics().await;
            }

            defmt_monitor::monitor!("HvPlate/TaskDiagnostics/last_duration", desc = "Duration of the most recent hv_plate task cycle, in ms.", "{=u64}", Instant::now().saturating_duration_since(start_time).as_millis());
            ticker.next().await;
        }
    }
}
