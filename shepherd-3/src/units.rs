//! Type aliases for units from the `uom` crate used by this project.

pub use uom::si::thermodynamic_temperature::degree_celsius;
pub use uom::si::electric_potential::volt;

/// Voltage!
///
/// Technically this is Electric Potential but who even calls it that
pub type Voltage = uom::si::f32::ElectricPotential;

/// Temperature!
pub type Temperature = uom::si::f32::ThermodynamicTemperature;

/// Ohms and such
pub type Resistance = uom::si::f32::ElectricalResistance;

/// Current!
pub type Current = uom::si::f32::ElectricCurrent;

/// adbms6830b temperature scale (microcelsius resolution).
mod microcelcius_unit {
    uom::unit! {
        system: uom::si;
        quantity: uom::si::thermodynamic_temperature;

        @microcelcius: 1.0e-6, 273.15; "uC", "degree (microcelcius)", "degrees (microcelcius)";
    }
}
pub use microcelcius_unit::microcelcius;

/// adbms2950 temperature scale (millicelsius resolution).
mod millicelcius_unit {
    uom::unit! {
        system: uom::si;
        quantity: uom::si::thermodynamic_temperature;

        @millicelcius: 1.0e-3, 273.15; "mC", "degree (millicelcius)", "degrees (millicelcius)";
    }
}
pub use millicelcius_unit::millicelcius;
