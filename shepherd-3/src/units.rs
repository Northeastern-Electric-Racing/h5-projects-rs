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

// /// 
// pub type Percentage = uom::si::f32::Ratio;

/// Percentage!
mod percentage {
    use uom::si::f32::Ratio;
    use uom::si::ratio::percent;

    /// Percentage!
    /// 
    /// This is a simple wrapper type around `Ratio` from `uom`. It is useful when you are basically
    /// only going to use a Percentage and don't care about the `Ratio` base unit.
    pub struct Percentage { inner: Ratio }
    impl Percentage {
        /// Creates a new `Percentage` from an f32.
        pub fn new(value: f32) -> Self { Self { inner: Ratio::new::<percent>(value) } }
        /// Returns the Percentage as an f32.
        pub fn as_f32(&self) -> f32 { self.inner.value }
        /// Gets a ref to the inner of this object.
        pub fn inner(&self) -> &Ratio { &self.inner }
        /// Gets a mutable ref to the inner of this object.
        pub fn inner_mut(&mut self) -> &mut Ratio { &mut self.inner }
    }
}
pub use percentage::Percentage;

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
