//! Newtypes and type aliases for units from the `uom` crate used by this project.

pub use uom::si::thermodynamic_temperature::degree_celsius;
pub use uom::si::electric_potential::volt;

/// Voltage!
///
/// Technically this is Electric Potential but who even calls it that
pub type Voltage = uom::si::f32::ElectricPotential;

/// Temperature!
pub type Temperature = uom::si::f32::ThermodynamicTemperature;

/// Current!
pub type Current = uom::si::f32::ElectricCurrent;

/// Ohms and such
pub type Resistance = uom::si::f32::ElectricalResistance;

/// Resistance per unit length
pub type ResistancePerLength = <Resistance as core::ops::Div<Length>>::Output;

/// Meters
pub type Length = uom::si::f32::Length;

/// Module for `const fn` constructors for certain units.
/// 
/// (for context, `uom` doesn't support `const fn` constructors because their types rely on trait methods internally)
pub mod consts {
    use super::*;
    use core::marker::PhantomData;

    /// Scalers for SI prefixes.
    pub mod scalers {
        pub const MILLI: f32 = 1e-3;
        pub const KILO: f32 = 1e3;
        pub const MEGA: f32 = 1e6;
    }

    // RESISTANCE
    /// Creates a `Resistance` from a value in Ohms.
    pub const fn from_ohms(value: f32) -> Resistance {
        // uom's base si unit is ohms, so you are able to just pass the value straight in.
        Resistance { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `Resistance` from a value in mOhms.
    pub const fn from_milliohms(value: f32) -> Resistance { from_ohms(value * scalers::MILLI) }
    /// Creates a `Resistance` from a value in kOhms.
    pub const fn from_kiloohms(value: f32) -> Resistance { from_ohms(value * scalers::KILO) }
    /// Creates a `Resistance` from a value in MOhms.
    pub const fn from_megaohms(value: f32) -> Resistance { from_ohms(value * scalers::MEGA) }

    // RESISTANCE PER LENGTH
    /// Creates a `ResistancePerLength` from a value in Ohms per mm.
    pub const fn from_ohms_per_millimeter(value: f32) -> ResistancePerLength {
        // uom's base si unit is ohms/m, so to convert to ohms/mm, we gotta divide by milli, because 1 ohms/mm = 1000 ohms/m
        ResistancePerLength { dimension: PhantomData, units: PhantomData, value: value / scalers::MILLI }
    }

    // LENGTH
    /// Creates a `Length` from a value in Meterse. 
    pub const fn from_meters(value: f32) -> Length {
        // uom's base si unit is meters, so you are able to just pass the value straight in.
        Length { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `Length` from a value in mm.
    pub const fn from_millimeters(value: f32) -> Length { from_meters(value * scalers::MILLI) }
    /// Creates a `Length` from a value in km.
    pub const fn from_kilometers(value: f32) -> Length { from_meters(value * scalers::KILO) }
    /// Creates a `Length` from a value in Mm.
    pub const fn from_megameters(value: f32) -> Length { from_meters(value * scalers::MEGA) }

    // CURRENT
    /// Creates a new `Current` from a value in Amps.
    pub const fn from_amps(value: f32) -> Current {
        // uom's base si unit is amps, so you are able to just pass the value straight in.
        Current { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `Current` from a value in mA.
    pub const fn from_milliamps(value: f32) -> Current { from_amps(value * scalers::MILLI) }
    /// Creates a `Current` from a value in kA. uh oh
    pub const fn from_kiloamps(value: f32) -> Current { from_amps(value * scalers::KILO) }
    /// Creates a `Current` from a value in MA. dont
    pub const fn from_megaamps(value: f32) -> Current { from_amps(value * scalers::MEGA) }
    
}

/// Percentage!
mod percentage {
    use uom::si::f32::Ratio;
    use uom::si::ratio::percent;

    /// Percentage!
    ///
    /// This is a simple wrapper type around `Ratio` from `uom`. It is useful when you are basically
    /// only going to use a Percentage and don't care about the `Ratio` base unit.
    pub struct Percentage {
        inner: Ratio,
    }
    impl Percentage {
        /// Creates a new `Percentage` from an f32.
        pub fn new(value: f32) -> Self {
            Self { inner: Ratio::new::<percent>(value) }
        }
        /// Returns the Percentage as an f32.
        pub fn as_f32(&self) -> f32 {
            self.inner.value
        }
        /// Gets a ref to the inner of this object.
        pub fn inner(&self) -> &Ratio {
            &self.inner
        }
        /// Gets a mutable ref to the inner of this object.
        pub fn inner_mut(&mut self) -> &mut Ratio {
            &mut self.inner
        }
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
