//! HV plate board constants and the physics that turns chip readings into real quantities.
//!
//! The `adbms2950` driver deliberately stops at the pin: it hands back microvolts at the shunt
//! and microvolts at each voltage input, because shunt resistance and divider networks are
//! properties of *this board*, not of the chip. Everything that bridges that gap lives here,
//! the same way `segments::chips::gpios` holds the cell-thermistor curve.
//!
//! Ported from `TSECU-Shepherd/Core/Src/hv_plate.c`.

use crate::units::{Current, Resistance, Temperature, Voltage};

/// Which V1ADC channel carries the tractive-system voltage divider.
///
/// Read as `Voltages1A::v2a`. Note this is the **V1ADC** (+100 uV per code); the V2ADC's `v2b`
/// measures the same pin through the redundant path at -85 uV and is not interchangeable.
pub const TS_VOLTAGE_CHANNEL: &str = "V2 (V1ADC)";

/// Which V1ADC channel carries the shunt thermistor.
///
/// Read as `Voltages1C::v7a`.
pub const SHUNT_THERMISTOR_CHANNEL: &str = "V7 (V1ADC)";

/// Shunt resistance, in ohms. 0.05 milliohms.
///
/// At the I1ADC's 1 uV per code this makes one code worth 20 mA.
const SHUNT_RESISTANCE_OHMS: f32 = 0.05_f32 / 1000.0_f32;

/// Top resistor of the BATT voltage divider, in ohms.
const BATT_DIVIDER_R1_OHMS: f32 = 3_600_000.0_f32;
/// Bottom resistor of the BATT voltage divider, in ohms.
const BATT_DIVIDER_R2_OHMS: f32 = 9_100.0_f32;

/// Top resistor of the TS voltage divider, in ohms.
const TS_DIVIDER_R1_OHMS: f32 = 3_600_000.0_f32;
/// Bottom resistor of the TS voltage divider, in ohms.
const TS_DIVIDER_R2_OHMS: f32 = 4_530.0_f32;

/// The chip's 1.25 V reference, in volts.
///
/// This does double duty: it is the excitation supply for the shunt thermistor divider, and it
/// is the negative reference the TS channel is measured against (`VS2 = VREF1P25` in CFGA),
/// which is why it gets added back in [`ts_voltage`].
const VREF1P25_VOLTS: f32 = 1.25_f32;

/// Series resistor feeding the shunt thermistor from VREF1P25, in ohms.
const THERMISTOR_DIVIDER_OHMS: f32 = 10_000.0_f32;
/// Shunt thermistor nominal resistance at [`THERMISTOR_T0_KELVIN`], in ohms.
const THERMISTOR_R0_OHMS: f32 = 10_000.0_f32;
/// Shunt thermistor beta coefficient.
const THERMISTOR_BETA: f32 = 3380.0_f32;
/// Reference temperature for the thermistor beta equation, in kelvin.
const THERMISTOR_T0_KELVIN: f32 = 298.0_f32;
/// Offset between kelvin and celsius.
const KELVIN_OFFSET: f32 = 273.15_f32;

/// Pack current, from the voltage the I1ADC measured across the shunt.
///
/// Takes the driver's raw microvolts rather than a `Voltage`, since that is what
/// `Current1::as_microvolts()` returns.
///
/// Note the C's `get_current_conversion` divides the raw register *code* by the shunt resistance
/// without ever applying the 1 uV LSB, which would read 1e6 times high. This applies it.
pub fn pack_current(shunt_microvolts: i32) -> Current {
    use uom::si::electric_current::ampere;

    let shunt_volts = shunt_microvolts as f32 * 1.0e-6_f32;
    Current::new::<ampere>(shunt_volts / SHUNT_RESISTANCE_OHMS)
}

/// BATT voltage, from the VB1ADC reading at the divider tap.
pub fn batt_voltage(tap_microvolts: i32) -> Voltage {
    use uom::si::electric_potential::volt;

    let tap_volts = tap_microvolts as f32 * 1.0e-6_f32;
    let volts = (BATT_DIVIDER_R1_OHMS + BATT_DIVIDER_R2_OHMS) * tap_volts / BATT_DIVIDER_R2_OHMS;
    Voltage::new::<volt>(volts)
}

/// Tractive-system voltage, from the V1ADC reading at the divider tap.
///
/// The divider's bottom sits on VREF1P25 rather than ground, so the reference is added back
/// after undoing the division.
pub fn ts_voltage(tap_microvolts: i32) -> Voltage {
    use uom::si::electric_potential::volt;

    let tap_volts = tap_microvolts as f32 * 1.0e-6_f32;
    let volts = (TS_DIVIDER_R1_OHMS + TS_DIVIDER_R2_OHMS) * tap_volts / TS_DIVIDER_R2_OHMS + VREF1P25_VOLTS;
    Voltage::new::<volt>(volts)
}

/// Resistance of the shunt thermistor, from the V1ADC reading at the divider tap.
///
/// The network is VREF1P25 through [`THERMISTOR_DIVIDER_OHMS`] into the thermistor to ground,
/// with the channel measuring the midpoint against SGND (`VS7 = SGND` in CFGA).
pub fn shunt_thermistor_resistance(tap_microvolts: i32) -> Resistance {
    use uom::si::electrical_resistance::ohm;

    let tap_volts = tap_microvolts as f32 * 1.0e-6_f32;
    Resistance::new::<ohm>((THERMISTOR_DIVIDER_OHMS * tap_volts) / (VREF1P25_VOLTS - tap_volts))
}

/// Shunt temperature, from the V1ADC reading at the thermistor divider tap.
///
/// Beta equation: `T = (T0 * B) / (T0 * ln(R / R0) + B)`, in kelvin, converted to celsius.
pub fn shunt_temperature(tap_microvolts: i32) -> Temperature {
    use uom::si::electrical_resistance::ohm;
    use uom::si::thermodynamic_temperature::degree_celsius;

    let ohms = shunt_thermistor_resistance(tap_microvolts).get::<ohm>();
    let kelvin = (THERMISTOR_T0_KELVIN * THERMISTOR_BETA) / (THERMISTOR_T0_KELVIN * libm::logf(ohms / THERMISTOR_R0_OHMS) + THERMISTOR_BETA);
    Temperature::new::<degree_celsius>(kelvin - KELVIN_OFFSET)
}
