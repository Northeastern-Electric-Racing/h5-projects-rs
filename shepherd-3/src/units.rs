//! Newtypes and type aliases for units from the `uom` crate used by this project.

pub use uom::si::thermodynamic_temperature::degree_celsius;
pub use uom::si::electric_potential::volt;
pub use uom::si::electrical_resistance::ohm;
pub use uom::si::ratio::ratio;
pub use uom::si::ratio::percent;

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

/// A ratio. Basically just a percent. The lowest value is at `0.0`, and the highest value is at `1.0`.
pub type Ratio = uom::si::f32::Ratio;

/// Module for `const fn` constructors for certain units.
/// 
/// (for context, `uom` doesn't support `const fn` constructors because their types rely on trait methods internally)
pub mod consts {
    use super::*;
    use core::marker::PhantomData;

    /// Constant for zero volts (0V).
    pub const ZERO_VOLTS: Voltage = from_volts(0_f32);

    /// Scalers for SI prefixes.
    pub mod scalers {
        pub const MILLI: f32 = 1e-3;
        pub const KILO: f32 = 1e3;
        pub const MEGA: f32 = 1e6;
    }

    // Voltage
    /// Creates a `Voltage` from a value in Volts.
    pub const fn from_volts(value: f32) -> Voltage {
        // uom's base si unit is volts, so you are able to just pass the value straight in.
        Voltage { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `Voltage` from a value in mV.
    pub const fn from_millivolts(value: f32) -> Voltage { from_volts(value * scalers::MILLI) }
    /// Creates a `Voltage` from a value in kV.
    pub const fn from_kilovolts(value: f32) -> Voltage { from_volts(value * scalers::KILO) }
    /// Creates a `Voltage` from a value in MV.
    pub const fn from_megavolts(value: f32) -> Voltage { from_volts(value * scalers::MEGA) }

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

    // TEMPERATURE
    /// Creates a `Temperature` from a value in Kelvin.
    pub const fn from_kelvin(value: f32) -> Temperature {
        // uom's base si unit is kelvin, so you are able to just pass the value straight in.
        Temperature { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `Temperature` from a value in °C.
    pub const fn from_celsius(value: f32) -> Temperature {
        const KELVIN_OFFSET: f32 = 273.15;
        from_kelvin(value + KELVIN_OFFSET) 
    }
    /// Creates a `Temperature` from a value in m°C.
    pub const fn from_millicelsius(value: f32) -> Temperature { from_celsius(value * scalers::MILLI) }

    // RATIO
    /// Creates a `Ratio` from `value`. The `value` must range from `0.0` to `1.0`. If
    /// it is outside of that range, this will return `None`. This is meant to be used
    /// to initialize consts, so you can call this from a `const` context and then unwrap
    /// the result as a nice compile-time check that you've passed in a valid `value`.
    pub const fn from_ratio(value: f32) -> Option<Ratio> {
        if (value > 1.0_f32) || (value < 0.0_f32) { return None; }
        Some(Ratio { dimension: PhantomData, units: PhantomData, value })
    }

    
}

/// adbms6830b temperature scale (microcelsius resolution).
mod microcelcius_unit {
    uom::unit! {
        system: uom::si;
        quantity: uom::si::thermodynamic_temperature;

        @microcelcius: 1.0e-6, 273.15e6; "uC", "degree (microcelcius)", "degrees (microcelcius)";
    }
}
pub use microcelcius_unit::microcelcius;

/// adbms2950 temperature scale (millicelsius resolution).
mod millicelcius_unit {
    uom::unit! {
        system: uom::si;
        quantity: uom::si::thermodynamic_temperature;

        @millicelcius: 1.0e-3, 273.15e3; "mC", "degree (millicelcius)", "degrees (millicelcius)";
    }
}
pub use millicelcius_unit::millicelcius;
