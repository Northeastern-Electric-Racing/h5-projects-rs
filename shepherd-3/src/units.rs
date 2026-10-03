//! Newtypes and type aliases for units from the `uom` crate used by this project.
//! 
//! This is a fairly big wrapper around `uom` for two reasons:
//! 1. the raw `uom` types are quite ugly on rust-analyzer, even when aliased
//! 2. `uom` uses traits internally so there are no `const fn` constructors
//! 
//! This module serves to fix those issues.

pub use uom::si::thermodynamic_temperature::degree_celsius;
pub use uom::si::electric_potential::volt;
pub use uom::si::electrical_resistance::ohm;
pub use uom::si::ratio::ratio;
pub use uom::si::ratio::percent;

pub mod voltage {
    use super::consts;
    use uom::Conversion;
    use uom::si::electric_potential::Unit;

    // Module forces rust-analyzer to use the clean type name instead of ugly full type
    mod alias {
        /// Voltage!
        ///
        /// Technically this is Electric Potential but who even calls it that
        pub type UomVoltage = uom::si::f32::ElectricPotential;
    }
    pub use alias::*;

    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct Voltage { volts: f32 }
    impl Voltage {
        /// Create a new quantity from the given value and measurement unit.
        pub fn new<N>(value: f32) -> Self where N: Unit + Conversion<f32, T = f32> { Self::from_inner(UomVoltage::new::<N>(value)) }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub fn get<N>(&self) -> f32 where N: Unit + Conversion<f32, T = f32> { self.as_inner().get::<N>() }
        /// Creates a `Voltage` from a value in Volts.
        pub const fn from_volts(value: f32) -> Self { Self::from_inner(consts::from_volts(value)) }
        /// Creates a `Voltage` from a value in mV.
        pub const fn from_millivolts(value: f32) -> Self { Self::from_inner(consts::from_millivolts(value)) }
        /// Creates a `Voltage` from a value in kV.
        pub const fn from_kilovolts(value: f32) -> Self { Self::from_inner(consts::from_kilovolts(value)) }
        /// Creates a `Voltage` at 0V.
        pub const fn zero() -> Self { Self::from_inner(consts::ZERO_VOLTS) }
        /// Consumes this `Voltage` and turns it into its inner `UomVoltage`.
        pub const fn as_inner(self) -> UomVoltage { consts::from_volts(self.volts) }
        /// Creates a new `Voltage` from a `UomVoltage`.
        pub const fn from_inner(inner: UomVoltage) -> Self { Self { volts: inner.value } }
    }
}
pub use voltage::*;

pub mod temperature {
    use super::consts;
    use uom::Conversion;
    use uom::si::thermodynamic_temperature::Unit;

    /// Temperature!
    pub type UomTemperature = uom::si::f32::ThermodynamicTemperature;

    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct Temperature { kelvin: f32 }
    impl Temperature {
        /// Create a new quantity from the given value and measurement unit.
        pub fn new<N>(value: f32) -> Self where N: Unit + Conversion<f32, T = f32> { Self::from_inner(UomTemperature::new::<N>(value)) }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub fn get<N>(&self) -> f32 where N: Unit + Conversion<f32, T = f32> { self.as_inner().get::<N>() }
        /// Creates a `Temperature` from a value in Celsius.
        pub const fn from_celsius(value: f32) -> Self { Self::from_inner(consts::from_celsius(value)) }
        /// Creates a `Temperature` from a value in Kelvin.
        pub const fn from_kelvin(value: f32) -> Self { Self::from_inner(consts::from_kelvin(value)) }
        /// Creates a `Temperature` from a value in millicelsius.
        pub const fn from_millicelsius(value: f32) -> Self { Self::from_inner(consts::from_millicelsius(value)) }
        /// Consumes this `Temperature` and turns it into its inner `UomTemperature`.
        pub const fn as_inner(self) -> UomTemperature { consts::from_kelvin(self.kelvin) }
        /// Creates a new `Temperature` from a `UomTemperature`.
        pub const fn from_inner(inner: UomTemperature) -> Self { Self { kelvin: inner.value } }
    }
}
pub use temperature::*;

pub mod current {
    use super::consts;
    use uom::Conversion;
    use uom::si::electric_current::Unit;

    /// Current!
    pub type UomCurrent = uom::si::f32::ElectricCurrent;

    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct Current { amps: f32 }
    impl Current {
        /// Create a new quantity from the given value and measurement unit.
        pub fn new<N>(value: f32) -> Self where N: Unit + Conversion<f32, T = f32> { Self::from_inner(UomCurrent::new::<N>(value)) }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub fn get<N>(&self) -> f32 where N: Unit + Conversion<f32, T = f32> { self.as_inner().get::<N>() }
        /// Creates a `Current` from a value in Amps.
        pub const fn from_amps(value: f32) -> Self { Self::from_inner(consts::from_amps(value)) }
        /// Creates a `Current` from a value in mA.
        pub const fn from_milliamps(value: f32) -> Self { Self::from_inner(consts::from_milliamps(value)) }
        /// Creates a `Current` from a value in kA. probably will not need to do that ever
        pub const fn from_kiloamps(value: f32) -> Self { Self::from_inner(consts::from_kiloamps(value)) }
        /// Creates a `Current` from a value in MA. definitely will not need to do that ever
        pub const fn from_megaamps(value: f32) -> Self { Self::from_inner(consts::from_megaamps(value)) }
        /// Consumes this `Current` and turns it into its inner `UomCurrent`.
        pub const fn as_inner(self) -> UomCurrent { consts::from_amps(self.amps) }
        /// Creates a new `Current` from a `UomCurrent`.
        pub const fn from_inner(inner: UomCurrent) -> Self { Self { amps: inner.value } }
    }
}
pub use current::*;

pub mod resistance {
    use super::consts;
    use uom::Conversion;
    use uom::si::electrical_resistance::Unit;

    /// Ohms and such
    pub type UomResistance = uom::si::f32::ElectricalResistance;

    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct Resistance { ohms: f32 }
    impl Resistance {
        /// Create a new quantity from the given value and measurement unit.
        pub fn new<N>(value: f32) -> Self where N: Unit + Conversion<f32, T = f32> { Self::from_inner(UomResistance::new::<N>(value)) }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub fn get<N>(&self) -> f32 where N: Unit + Conversion<f32, T = f32> { self.as_inner().get::<N>() }
        /// Creates a `Resistance` from a value in Ohms.
        pub const fn from_ohms(value: f32) -> Self { Self::from_inner(consts::from_ohms(value)) }
        /// Creates a `Resistance` from a value in mOhms.
        pub const fn from_milliohms(value: f32) -> Self { Self::from_inner(consts::from_milliohms(value)) }
        /// Creates a `Resistance` from a value in kOhms.
        pub const fn from_kiloohms(value: f32) -> Self { Self::from_inner(consts::from_kiloohms(value)) }
        /// Creates a `Resistance` from a value in MOhms.
        pub const fn from_megaohms(value: f32) -> Self { Self::from_inner(consts::from_megaohms(value)) }
        /// Consumes this `Resistance` and turns it into its inner `UomResistance`.
        pub const fn as_inner(self) -> UomResistance { consts::from_ohms(self.ohms) }
        /// Creates a new `Resistance` from a `UomResistance`.
        pub const fn from_inner(inner: UomResistance) -> Self { Self { ohms: inner.value } }
    }
}
pub use resistance::*;

pub mod resistance_per_length {
    use super::{UomResistance, UomLength};
    use super::consts;
    use core::marker::PhantomData;

    mod alias {
        use uom::typenum::{P1, N2, N3, Z0};
        pub type UomResistancePerLength = uom::si::Quantity<uom::si::ISQ<P1, P1, N3, N2, Z0, Z0, Z0>, uom::si::SI<f32>, f32>;
    }
    pub use alias::*;

    // Checks that `UomResistancePerLength` is the same as what uom produces for `UomResistance / UomLength`.
    const _: fn(<UomResistance as core::ops::Div<UomLength>>::Output) -> UomResistancePerLength = |x| x;

    /// Our custom unit for this (since `uom` doesnt have a resistance per lentgh unit or quantity)
    pub trait Unit {
        /// How many ohms per meter one of this unit is.
        const OHMS_PER_METER: f32;
    }

    #[allow(non_camel_case_types)]
    pub struct ohm_per_meter;
    impl Unit for ohm_per_meter { const OHMS_PER_METER: f32 = 1.0; }

    #[allow(non_camel_case_types)]
    pub struct ohm_per_millimeter;
    impl Unit for ohm_per_millimeter { const OHMS_PER_METER: f32 = 1e3; }


    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct ResistancePerLength { ohms_per_meter: f32 }
    impl ResistancePerLength {
        /// Create a new quantity from the given value and measurement unit.
        pub const fn new<N: Unit>(value: f32) -> Self { Self { ohms_per_meter: value * N::OHMS_PER_METER } }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub const fn get<N: Unit>(&self) -> f32 { self.ohms_per_meter / N::OHMS_PER_METER }
        /// Creates a `ResistancePerLength` from a value in Ohms per millimeter.
        pub const fn from_ohms_per_millimeter(value: f32) -> Self { Self::from_inner(consts::from_ohms_per_millimeter(value)) }
        /// Consumes this `ResistancePerLength` and turns it into its inner `UomResistancePerLength`.
        pub const fn as_inner(self) -> UomResistancePerLength { UomResistancePerLength { dimension: PhantomData, units: PhantomData, value: self.ohms_per_meter } }
        /// Creates a new `ResistancePerLength` from a `UomResistancePerLength`.
        pub const fn from_inner(inner: UomResistancePerLength) -> Self { Self { ohms_per_meter: inner.value } }
    }
}
pub use resistance_per_length::*;

pub mod length {
    use super::consts;
    use uom::Conversion;
    use uom::si::length::Unit;

    /// Meters etc
    pub type UomLength = uom::si::f32::Length;

    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct Length { meters: f32 }
    impl Length {
        /// Create a new quantity from the given value and measurement unit.
        pub fn new<N>(value: f32) -> Self where N: Unit + Conversion<f32, T = f32> { Self::from_inner(UomLength::new::<N>(value)) }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub fn get<N>(&self) -> f32 where N: Unit + Conversion<f32, T = f32> { self.as_inner().get::<N>() }
        /// Creates a `Length` from a value in Meters.
        pub const fn from_meters(value: f32) -> Self { Self::from_inner(consts::from_meters(value)) }
        /// Creates a `Length` from a value in mm.
        pub const fn from_millimeters(value: f32) -> Self { Self::from_inner(consts::from_millimeters(value)) }
        /// Creates a `Length` from a value in km.
        pub const fn from_kilometers(value: f32) -> Self { Self::from_inner(consts::from_kilometers(value)) }
        /// Creates a `Length` from a value in Mm.
        pub const fn from_megameters(value: f32) -> Self { Self::from_inner(consts::from_megameters(value)) }
        /// Consumes this `Length` and turns it into its inner `UomLength`.
        pub const fn as_inner(self) -> UomLength { consts::from_meters(self.meters) }
        /// Creates a new `Length` from a `UomLength`.
        pub const fn from_inner(inner: UomLength) -> Self { Self { meters: inner.value } }
    }
}
pub use length::*;

pub mod ratios {
    use super::consts;
    use uom::Conversion;
    use uom::si::ratio::Unit;
    use core::marker::PhantomData;

    /// A ratio. Basically just a percent. The lowest value is at `0.0`, and the highest value is at `1.0`.
    pub type UomRatio = uom::si::f32::Ratio;

    #[derive(Clone, Copy, PartialEq, PartialOrd)]
    pub struct Ratio { ratio: f32 }
    impl Ratio {
        /// Create a new quantity from the given value and measurement unit.
        pub fn new<N>(value: f32) -> Self where N: Unit + Conversion<f32, T = f32> { Self::from_inner(UomRatio::new::<N>(value)) }
        /// Retrieve the value of the quantity in the given measurement unit.
        pub fn get<N>(&self) -> f32 where N: Unit + Conversion<f32, T = f32> { self.as_inner().get::<N>() }
        /// Creates a `Ratio` from `value`. The `value` must range from `0.0` to `1.0`. If
        /// it is outside of that range, this will return `None`. This is meant to be used
        /// to initialize consts, so you can call this from a `const` context and then unwrap
        /// the result as a nice compile-time check that you've passed in a valid `value`.
        pub const fn from_ratio(value: f32) -> Option<Self> { Some(Self::from_inner(
            match consts::from_ratio(value) {
                Some(s) => s,
                None => { return None; }
            }))
        }
        /// Consumes this `Ratio` and turns it into its inner `UomRatio`.
        pub const fn as_inner(self) -> UomRatio { UomRatio { dimension: PhantomData, units: PhantomData, value: self.ratio } }
        /// Creates a new `Ratio` from a `UomRatio`.
        pub const fn from_inner(inner: UomRatio) -> Self { Self { ratio: inner.value } }
    }
}
pub use ratios::*;

/// Operator overloading for the wrapper types.
pub mod ops {
    use super::*;
    use core::ops::{Add, Sub, Mul, Div, AddAssign, SubAssign, MulAssign, DivAssign};

    /// Implemented by every wrapper type.
    pub trait Wrapper: Copy {
        /// The `uom` type this wraps.
        type Inner;
        /// Turns this wrapper into its inner `uom` type.
        fn as_inner(self) -> Self::Inner;
        /// Creates this wrapper from its inner `uom` type.
        fn from_inner(inner: Self::Inner) -> Self;
    }

    /// Implemented by every `uom` type that has a wrapper. This is just used to map the `uom` type back to its wrapper.
    pub trait HasWrapper {
        /// The wrapper for this `uom` type.
        type Wrapper: Wrapper<Inner = Self>;
    }

    /// Forwards binary operations (like +).
    macro_rules! forward_binop {
        ($wrapper:ident, $inner:ty, $Trait:ident, $method:ident) => {
            impl<R: Wrapper> $Trait<R> for $wrapper
            where
                $inner: $Trait<R::Inner>,
                <$inner as $Trait<R::Inner>>::Output: HasWrapper,
            {
                type Output = <<$inner as $Trait<R::Inner>>::Output as HasWrapper>::Wrapper;
                fn $method(self, rhs: R) -> Self::Output { Wrapper::from_inner(self.as_inner().$method(rhs.as_inner())) }
            }
        };
    }

    /// Forwards compound assignment operations (like +=)
    macro_rules! forward_assign_op {
        ($wrapper:ident, $inner:ty, $Trait:ident, $method:ident) => {
            impl<R: Wrapper> $Trait<R> for $wrapper where $inner: $Trait<R::Inner> {
                fn $method(&mut self, rhs: R) {
                    let mut inner = self.as_inner();
                    inner.$method(rhs.as_inner());
                    *self = Self::from_inner(inner);
                }
            }
        };
    }

    /// Implements everything for the wrappers.
    macro_rules! wrapper_ops {
        ($wrapper:ident, $inner:ty) => {
            impl Wrapper for $wrapper {
                type Inner = $inner;
                fn as_inner(self) -> $inner { $wrapper::as_inner(self) }
                fn from_inner(inner: $inner) -> Self { $wrapper::from_inner(inner) }
            }
            impl HasWrapper for $inner { type Wrapper = $wrapper; }

            forward_binop!($wrapper, $inner, Add, add);
            forward_binop!($wrapper, $inner, Sub, sub);
            forward_binop!($wrapper, $inner, Mul, mul);
            forward_binop!($wrapper, $inner, Div, div);
            forward_assign_op!($wrapper, $inner, AddAssign, add_assign);
            forward_assign_op!($wrapper, $inner, SubAssign, sub_assign);

            // Scaling by a plain number.
            impl Mul<f32> for $wrapper { type Output = Self; fn mul(self, rhs: f32) -> Self { Self::from_inner(self.as_inner() * rhs) } }
            impl Div<f32> for $wrapper { type Output = Self; fn div(self, rhs: f32) -> Self { Self::from_inner(self.as_inner() / rhs) } }
            impl Mul<$wrapper> for f32 { type Output = $wrapper; fn mul(self, rhs: $wrapper) -> $wrapper { $wrapper::from_inner(self * rhs.as_inner()) } }
            impl MulAssign<f32> for $wrapper { fn mul_assign(&mut self, rhs: f32) { *self = *self * rhs; } }
            impl DivAssign<f32> for $wrapper { fn div_assign(&mut self, rhs: f32) { *self = *self / rhs; } }
        };
    }

    wrapper_ops!(Voltage, UomVoltage);
    wrapper_ops!(Temperature, UomTemperature);
    wrapper_ops!(Current, UomCurrent);
    wrapper_ops!(Resistance, UomResistance);
    wrapper_ops!(ResistancePerLength, UomResistancePerLength);
    wrapper_ops!(Length, UomLength);
    wrapper_ops!(Ratio, UomRatio);
}

/// Module for `const fn` constructors for certain units.
/// 
/// (for context, `uom` doesn't support `const fn` constructors because their types rely on trait methods internally)
pub mod consts {
    use super::*;
    use core::marker::PhantomData;

    /// Constant for zero volts (0V).
    pub const ZERO_VOLTS: UomVoltage = from_volts(0_f32);

    /// Scalers for SI prefixes.
    pub mod scalers {
        pub const MILLI: f32 = 1e-3;
        pub const KILO: f32 = 1e3;
        pub const MEGA: f32 = 1e6;
    }

    // VOLTAGE
    /// Creates a `UomVoltage` from a value in Volts.
    pub const fn from_volts(value: f32) -> UomVoltage {
        // uom's base si unit is volts, so you are able to just pass the value straight in.
        UomVoltage { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `UomVoltage` from a value in mV.
    pub const fn from_millivolts(value: f32) -> UomVoltage { from_volts(value * scalers::MILLI) }
    /// Creates a `UomVoltage` from a value in kV.
    pub const fn from_kilovolts(value: f32) -> UomVoltage { from_volts(value * scalers::KILO) }
    /// Creates a `UomVoltage` from a value in MV.
    pub const fn from_megavolts(value: f32) -> UomVoltage { from_volts(value * scalers::MEGA) }

    // RESISTANCE
    /// Creates a `UomResistance` from a value in Ohms.
    pub const fn from_ohms(value: f32) -> UomResistance {
        // uom's base si unit is ohms, so you are able to just pass the value straight in.
        UomResistance { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `UomResistance` from a value in mOhms.
    pub const fn from_milliohms(value: f32) -> UomResistance { from_ohms(value * scalers::MILLI) }
    /// Creates a `UomResistance` from a value in kOhms.
    pub const fn from_kiloohms(value: f32) -> UomResistance { from_ohms(value * scalers::KILO) }
    /// Creates a `UomResistance` from a value in MOhms.
    pub const fn from_megaohms(value: f32) -> UomResistance { from_ohms(value * scalers::MEGA) }

    // RESISTANCE PER LENGTH
    /// Creates a `UomResistancePerLength` from a value in Ohms per mm.
    pub const fn from_ohms_per_millimeter(value: f32) -> UomResistancePerLength {
        // uom's base si unit is ohms/m, so to convert to ohms/mm, we gotta divide by milli, because 1 ohms/mm = 1000 ohms/m
        UomResistancePerLength { dimension: PhantomData, units: PhantomData, value: value / scalers::MILLI }
    }

    // LENGTH
    /// Creates a `UomLength` from a value in Meterse. 
    pub const fn from_meters(value: f32) -> UomLength {
        // uom's base si unit is meters, so you are able to just pass the value straight in.
        UomLength { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `UomLength` from a value in mm.
    pub const fn from_millimeters(value: f32) -> UomLength { from_meters(value * scalers::MILLI) }
    /// Creates a `UomLength` from a value in km.
    pub const fn from_kilometers(value: f32) -> UomLength { from_meters(value * scalers::KILO) }
    /// Creates a `UomLength` from a value in Mm.
    pub const fn from_megameters(value: f32) -> UomLength { from_meters(value * scalers::MEGA) }

    // CURRENT
    /// Creates a new `UomCurrent` from a value in Amps.
    pub const fn from_amps(value: f32) -> UomCurrent {
        // uom's base si unit is amps, so you are able to just pass the value straight in.
        UomCurrent { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `UomCurrent` from a value in mA.
    pub const fn from_milliamps(value: f32) -> UomCurrent { from_amps(value * scalers::MILLI) }
    /// Creates a `UomCurrent` from a value in kA. uh oh
    pub const fn from_kiloamps(value: f32) -> UomCurrent { from_amps(value * scalers::KILO) }
    /// Creates a `UomCurrent` from a value in MA. dont
    pub const fn from_megaamps(value: f32) -> UomCurrent { from_amps(value * scalers::MEGA) }

    // TEMPERATURE
    /// Creates a `UomTemperature` from a value in Kelvin.
    pub const fn from_kelvin(value: f32) -> UomTemperature {
        // uom's base si unit is kelvin, so you are able to just pass the value straight in.
        UomTemperature { dimension: PhantomData, units: PhantomData, value }
    }
    /// Creates a `UomTemperature` from a value in °C.
    pub const fn from_celsius(value: f32) -> UomTemperature {
        const KELVIN_OFFSET: f32 = 273.15;
        from_kelvin(value + KELVIN_OFFSET) 
    }
    /// Creates a `UomTemperature` from a value in m°C.
    pub const fn from_millicelsius(value: f32) -> UomTemperature { from_celsius(value * scalers::MILLI) }

    // RATIO
    /// Creates a `UomRatio` from `value`. The `value` must range from `0.0` to `1.0`. If
    /// it is outside of that range, this will return `None`. This is meant to be used
    /// to initialize consts, so you can call this from a `const` context and then unwrap
    /// the result as a nice compile-time check that you've passed in a valid `value`.
    pub const fn from_ratio(value: f32) -> Option<UomRatio> {
        if (value > 1.0_f32) || (value < 0.0_f32) { return None; }
        Some(UomRatio { dimension: PhantomData, units: PhantomData, value })
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
