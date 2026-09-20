use strum::{EnumCount, VariantArray, EnumIter, EnumIs};
use embassy_time::Duration;

#[derive(EnumIs)]
#[derive(Copy, Clone)]
pub enum FaultSeverity {
    Critical,
    NonCritical,
}

/// Const config metadata for a fault.
pub struct FaultConfig {
    timeout: Duration,
    severity: FaultSeverity,
}
impl FaultConfig {
    /// How long a fault should stay active before expiring.
    pub const fn timeout(&self) -> Duration { self.timeout }
    /// The severity of a fault.
    pub const fn severity(&self) -> FaultSeverity { self.severity }
}

#[derive(EnumCount, VariantArray, EnumIter)]
#[derive(Copy, Clone)]
pub enum FaultId {
    DischargeLimitEnforcementFault,
	ChargeLimitEnforcement,
    CellVoltageTooLow,
    CellVoltageTooHigh,
    CellChargeVoltageTooHigh,
    PackTooHot,
    DieTempMaximumFault,
    HvPlateCommsFault,
    SegmentCommsFault,
    CellOpenWireFault,
}
impl FaultId {
    /// Returns this FaultId's config settings.
    #[rustfmt::skip]
    pub const fn config(self) -> FaultConfig {
        // This function body is for defining the config settings for each fault.

        // using a match statement instead of a lookup table here because rust doesnt have designated initializers for arrays
        // but this should probably (?) compile into a lookup table anyway since there doesn't seem to be a reason not to
        match self {
            Self::DischargeLimitEnforcementFault => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::ChargeLimitEnforcement         => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::CellVoltageTooLow              => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::CellVoltageTooHigh             => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::CellChargeVoltageTooHigh       => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::PackTooHot                     => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::DieTempMaximumFault            => FaultConfig { timeout: Duration::from_millis(55_000), severity: FaultSeverity::Critical },
            Self::HvPlateCommsFault              => FaultConfig { timeout: Duration::from_millis(20_000), severity: FaultSeverity::Critical },
            Self::SegmentCommsFault              => FaultConfig { timeout: Duration::from_millis(20_000), severity: FaultSeverity::NonCritical },
            Self::CellOpenWireFault              => FaultConfig { timeout: Duration::from_millis(40_000), severity: FaultSeverity::Critical },
        }
    }
} 