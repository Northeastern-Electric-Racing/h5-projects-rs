use crate::units::{Current, Resistance, Temperature, Voltage};

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
/// Excites the shunt thermistor divider, and is the negative reference for the TS channel
/// (`VS2 = VREF1P25` in CFGA).
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

/// Converts a driver reading (microvolts) to volts, which is what the networks below use.
const fn volts(microvolts: i32) -> f32 {
    microvolts as f32 * 1.0e-6_f32
}

/// Pack current, from the shunt voltage the I1ADC measured (`Current1::as_microvolts()`).
pub fn pack_current(shunt_microvolts: i32) -> Current {
    use uom::si::electric_current::ampere;

    let shunt_volts = volts(shunt_microvolts);
    Current::new::<ampere>(shunt_volts / SHUNT_RESISTANCE_OHMS)
}

/// BATT voltage, from the VB1ADC reading at the divider tap.
pub fn batt_voltage(tap_microvolts: i32) -> Voltage {
    use uom::si::electric_potential::volt;

    let tap_volts = volts(tap_microvolts);
    let batt_volts = (BATT_DIVIDER_R1_OHMS + BATT_DIVIDER_R2_OHMS) * tap_volts / BATT_DIVIDER_R2_OHMS;
    Voltage::new::<volt>(batt_volts)
}

/// Tractive-system voltage, from `Voltages1A::v2a` -- channel V2 on the V1ADC.
///
/// The divider's bottom sits on VREF1P25, not ground, so the reference is added back.
///
/// Not `Voltages2A::v2b`: that is the same pin on the V2ADC at -85 uV per code.
pub fn ts_voltage(tap_microvolts: i32) -> Voltage {
    use uom::si::electric_potential::volt;

    let tap_volts = volts(tap_microvolts);
    let ts_volts = (TS_DIVIDER_R1_OHMS + TS_DIVIDER_R2_OHMS) * tap_volts / TS_DIVIDER_R2_OHMS + VREF1P25_VOLTS;
    Voltage::new::<volt>(ts_volts)
}

/// Resistance of the shunt thermistor, from the V1ADC reading at the divider tap.
///
/// The network is VREF1P25 through [`THERMISTOR_DIVIDER_OHMS`] into the thermistor to ground,
/// with the channel measuring the midpoint against SGND (`VS7 = SGND` in CFGA).
fn shunt_thermistor_resistance(tap_microvolts: i32) -> Resistance {
    use uom::si::electrical_resistance::ohm;

    let tap_volts = volts(tap_microvolts);
    Resistance::new::<ohm>((THERMISTOR_DIVIDER_OHMS * tap_volts) / (VREF1P25_VOLTS - tap_volts))
}

/// Shunt temperature, from `Voltages1C::v7a` -- channel V7 on the V1ADC.
///
/// Beta equation: `T = (T0 * B) / (T0 * ln(R / R0) + B)`, in kelvin, converted to celsius.
pub fn shunt_temperature(tap_microvolts: i32) -> Temperature {
    use uom::si::electrical_resistance::ohm;
    use uom::si::thermodynamic_temperature::degree_celsius;

    let ohms = shunt_thermistor_resistance(tap_microvolts).get::<ohm>();
    let kelvin = (THERMISTOR_T0_KELVIN * THERMISTOR_BETA) / (THERMISTOR_T0_KELVIN * libm::logf(ohms / THERMISTOR_R0_OHMS) + THERMISTOR_BETA);
    Temperature::new::<degree_celsius>(kelvin - KELVIN_OFFSET)
}
