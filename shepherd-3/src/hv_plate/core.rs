//! Device construction, startup configuration, read jobs, and the HV plate task.

use adbms2950::chip::commands;
use adbms2950::line::Error;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use super::cache::{self, UpdateError};
use crate::job_diagnostics::JobDiagnosticsContainer;
use crate::broadcast::Broadcast;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

embassy_stm32::bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH4>;
    GPDMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH5>;
    GPDMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH6>;
    GPDMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH7>;
});

pub mod alias {
    use embassy_stm32::{
        gpio::Output,
        mode::Async,
        spi::{mode::Master, Spi},
    };
    use embassy_time::Delay;
    use embedded_hal_bus::spi::ExclusiveDevice;

    /// Type alias representing a SPI controller that implements `SpiDevice` from `embedded_hal_async`.
    ///
    /// Must be built with `ExclusiveDevice::new`, not `new_no_delay`: the driver's wake-up pulse
    /// is a delay-only transaction and panics without delay support.
    pub type SpiDevice = ExclusiveDevice<Spi<'static, Async, Master>, Output<'static>, Delay>;

    /// The error type our `SpiDevice` produces.
    pub type SpiError = <SpiDevice as embedded_hal_async::spi::ErrorType>::Error;

    /// Type alias representing one isoSPI line to our ADBMS2950B.
    pub type Line = adbms2950::line::Line<SpiDevice>;

    /// Type alias for the stateful API over both lines.
    pub type Api = adbms2950::api::Api<SpiDevice>;
}

/// Guy in charge of the HV plate.
pub(super) struct HvPlate {
    api: &'static mut alias::Api,
    /// Whether the startup sequence has completed since the last reset.
    started: bool,
}

impl HvPlate {
    pub fn new(r: crate::HvPlateResources) -> Self {
        use embassy_time::Delay;
        use embedded_hal_bus::spi::ExclusiveDevice;

        // The C project clocks both SPI3 and SPI4 at 2 MBit/s (prescaler 32).
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

        static API: StaticCell<alias::Api> = StaticCell::new();
        let api: &'static mut alias::Api = API.init(alias::Api::new(line_a, line_b));

        Self { api, started: false }
    }

    /// Brings the chip from reset to converting, if it hasn't been already.
    ///
    /// Safe to call every cycle: returns immediately once startup has succeeded, and on failure
    /// leaves `started` false so the next cycle retries.
    pub async fn startup(&mut self) -> Result<(), Error<alias::SpiError>> {
        use adbms2950::chip::registers::config_a::{ConfigA, types::*};

        if self.started {
            return Ok(());
        }

        // Reset to a known state and wait out the regulator startup
        self.api.reset().await?;

        // Measurements before the references are up are not trustworthy
        self.api.wait_for_reference().await?;

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
        self.api.set_configa(config_a).await?;

        // Fault latches power up asserted. Clear them or everything reads as a live fault.
        self.api.write(adbms2950::chip::registers::flag::Flag::new().with_thsd(true)).await?;

        // Start continuous conversion
        self.api.command(commands::adc::adi1(commands::adc::Redundancy::Enabled, commands::adc::Acquisition::Continuous, commands::adc::Diagnostic::Normal, commands::adc::OpenWire::Off)).await?;

        // Wait for the first conversion to land
        Timer::after_millis(adbms2950::line::conversion_times::IXADC_STARTUP_MAX_MS as u64).await;

        defmt::info!("HvPlate: startup complete.");
        self.started = true;

        Ok(())
    }

    /// Drives the HV control relay on GPO4. Active low.
    #[allow(unused)]
    pub async fn set_hv_relay(&mut self, energized: bool) -> Result<(), Error<alias::SpiError>> {
        use adbms2950::chip::registers::config_a::types::GpoOutputState;

        let state = if energized { GpoOutputState::PulledLow } else { GpoOutputState::Driven };
        self.api.modify_configa(|cfg| cfg.with_gpo4c(state)).await
    }

    /// Device health: command counter, PEC tallies, and when we last heard from the chip.
    ///
    /// `DeviceState::suspected_reset()` means the chip rebooted and dropped its configuration.
    #[allow(unused)]
    pub const fn device(&self) -> &adbms2950::api::DeviceState {
        self.api.device()
    }
}

pub mod jobs {
    use crate::job_diagnostics;

    use super::*;

    /// How long to allow a conversion to complete before giving up.
    const CONVERSION_TIMEOUT: Duration = Duration::from_millis(100);

    impl HvPlate {
        /// Reads pack current, the accumulators, and FLAG inside one SNAP window.
        ///
        /// One window because FLAG's `i1cnt` counts the conversions the accumulator sums; read
        /// at different instants the coulomb count drifts.
        pub async fn job_update_snap_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            // Error paths leave the registers snapped; the next cycle's SNAP/UNSNAP clears it.
            self.api.command(commands::misc::snap()).await.map_err(UpdateError::SnapError)?;

            cache::CACHE.update_current_voltage(self.api).await?;
            cache::CACHE.update_accumulated(self.api).await?;
            cache::CACHE.update_flag(self.api).await?;

            self.api.command(commands::misc::unsnap()).await.map_err(UpdateError::UnsnapError)?;

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_snap_registers", run.finish());

            Ok(())
        }

        /// Converts the voltage channels and reads TS voltage (V2) and the shunt thermistor (V7).
        ///
        /// One round-robin sweep covers both channels, so it is one conversion and two reads.
        pub async fn job_update_voltage_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

            self.api.adv_autoconvert(commands::adc::OpenWireVoltage::Off, commands::adc::VoltageChannel::RoundRobinCh0ToCh8, CONVERSION_TIMEOUT).await.map_err(UpdateError::ConversionError)?;
            cache::CACHE.update_voltages(self.api).await?;

            job_diagnostics::log_job_diagnostics!("HvPlate", "job_update_voltage_registers", run.finish());

            Ok(())
        }

        /// Converts and reads the AUX ADC rails and on-chip temperatures.
        pub async fn job_update_aux_registers(&mut self) -> Result<(), UpdateError> {
            static DIAGNOSTICS: JobDiagnosticsContainer = JobDiagnosticsContainer::new();
            let run = DIAGNOSTICS.start();

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
        fn log_diagnostics(&self) {
            let device = self.api.device();
            defmt_monitor::monitor!("HvPlate/Device/ActiveLine", desc = "Which isoSPI line is currently in use.", "{}", self.api.active_line());
            defmt_monitor::monitor!("HvPlate/Device/LineAErrorCount", desc = "Transactions that have failed on isoSPI line A since boot.", "{=u32}", self.api.line_error_count(adbms2950::api::LineId::A));
            defmt_monitor::monitor!("HvPlate/Device/LineBErrorCount", desc = "Transactions that have failed on isoSPI line B since boot.", "{=u32}", self.api.line_error_count(adbms2950::api::LineId::B));
            defmt_monitor::monitor!("HvPlate/Device/PecSuccessCount", desc = "Reads whose data PEC verified, since boot.", "{=u32}", device.pec_success_count());
            defmt_monitor::monitor!("HvPlate/Device/PecFailedCount", desc = "Reads whose data PEC did not verify, since boot.", "{=u32}", device.pec_failed_count());
            defmt_monitor::monitor!("HvPlate/Device/ExpectedCommandCounter", desc = "Command counter we expect the chip to report next.", "{=u8}", device.expected_command_counter());
            defmt_monitor::monitor!("HvPlate/Device/ReportedCommandCounter", desc = "Command counter the chip reported on the most recent read. 255 if it has never been read.", "{=u8}", device.reported_command_counter().unwrap_or(u8::MAX));
            defmt_monitor::monitor!("HvPlate/Device/SuspectedReset", desc = "True if the chip reported counter 0 unexpectedly, meaning it rebooted and lost its configuration.", "{=bool}", device.suspected_reset());
        }
    }

    #[embassy_executor::task]
    pub async fn hv_plate_task(r: crate::HvPlateResources) {
        /// Frequency (in ms) at which the HV plate task should run.
        const HV_PLATE_TASK_FREQUENCY_MS: u64 = 100;

        let mut hv_plate = HvPlate::new(r);

        loop {
            let start_time = Instant::now();

            // No-op once startup has succeeded; retries every cycle until then.
            if let Err(err) = hv_plate.startup().await {
                defmt::error!("HvPlate: Inside `hv_plate_task()`: `startup()` failed, will retry. Error: {}", err);
            }

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

            if let Err(err) = hv_plate.job_update_aux_registers().await {
                defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_aux_registers()` failed. Error: {}", err);
                all_successful = false;
            }

            if let Err(err) = hv_plate.job_update_status_registers().await {
                defmt::error!("HvPlate: Inside `hv_plate_task()`: `job_update_status_registers()` failed. Error: {}", err);
                all_successful = false;
            }

            if all_successful {
                signal::HV_PLATE_FRESH_DATA_SIGNAL.signal();
            }

            hv_plate.log_diagnostics();
            defmt_monitor::monitor!("HvPlate/TaskDiagnostics/last_duration", desc = "Duration of the most recent hv_plate task cycle, in ms.", "{=u64}", Instant::now().saturating_duration_since(start_time).as_millis());

            Timer::after_millis(HV_PLATE_TASK_FREQUENCY_MS).await;
        }
    }
}
