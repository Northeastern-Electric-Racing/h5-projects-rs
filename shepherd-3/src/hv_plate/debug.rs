//! The plate's defmt dump: a console block plus the `HvPlate/*` monitors.

use super::cache;
use super::core::task::signal;

/// Task that sends out debug HV plate data.
#[embassy_executor::task]
pub async fn hv_plate_debug() {
    use crate::units::{degree_celsius, volt};
    use uom::si::electric_current::ampere;

    // Subscribe to the HV plate fresh data signal so we are notified when new data comes in.
    let mut subscription = signal::HV_PLATE_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

    let current_voltage_ready = cache::CACHE.current_voltage().reading().ready().await;
    let voltages_ready = cache::CACHE.voltages().reading().ready().await;
    let accumulated_ready = cache::CACHE.accumulated().reading().ready().await;
    let flag_ready = cache::CACHE.flag().reading().ready().await;
    let aux_ready = cache::CACHE.aux().reading().ready().await;
    let status_ready = cache::CACHE.status().reading().ready().await;

    loop {
        // Run one loop of this task every time new HV plate data arrives.
        subscription.wait().await;

        // The accumulator, FLAG and STATUS readings are only consumed by the monitor block, so
        // they are annotated for the `DEFMT_MONITOR=off` build.
        let current_voltage = current_voltage_ready.get().nice();
        let voltages = voltages_ready.get().nice();
        #[cfg_attr(not(defmt_monitor), allow(unused_variables))]
        let accumulated = accumulated_ready.get().nice();
        #[cfg_attr(not(defmt_monitor), allow(unused_variables))]
        let flag = flag_ready.get().nice();
        let aux = aux_ready.get().nice();
        #[cfg_attr(not(defmt_monitor), allow(unused_variables))]
        let status = status_ready.get().nice();

        '_println: {
            defmt::println!("HV Plate Data:");
            defmt::println!("TS Voltage: {=f32} V", voltages.ts_voltage.get::<volt>());
            defmt::println!("BATT Voltage: {=f32} V", current_voltage.batt_voltage.get::<volt>());
            defmt::println!("Shunt Temp: {=f32} C", voltages.shunt_temperature.get::<degree_celsius>());
            defmt::println!("Pack Current: {=f32} A", current_voltage.pack_current.get::<ampere>());
            defmt::println!("VREG: {=f32} V", aux.vreg.get::<volt>());
            defmt::println!("VREF1P25: {=f32} V", aux.vref1p25.get::<volt>());
            defmt::println!("EPAD: {=f32} V", aux.epad.get::<volt>());
            defmt::println!("VDIG: {=f32} V", aux.vdig.get::<volt>());
            defmt::println!("VDD: {=f32} V", aux.vdd.get::<volt>());
            defmt::println!("VDIV: {=f32} V", aux.vdiv.get::<volt>());
            defmt::println!("Primary Internal Temperature: {=f32} C", aux.die_temperature.get::<degree_celsius>());
            defmt::println!("Secondary Internal Temperature: {=f32} C", aux.secondary_temperature.get::<degree_celsius>());
        }

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            // The headline measurements.
            defmt_monitor::monitor!("HvPlate/PackCurrent", desc = "Pack current through the shunt, in amps. Positive is into the pack.", "{=f32}", current_voltage.pack_current.get::<ampere>());
            defmt_monitor::monitor!("HvPlate/BattVoltage", desc = "BATT-side voltage, in volts.", "{=f32}", current_voltage.batt_voltage.get::<volt>());
            defmt_monitor::monitor!("HvPlate/TsVoltage", desc = "Tractive system voltage, in volts.", "{=f32}", voltages.ts_voltage.get::<volt>());
            defmt_monitor::monitor!("HvPlate/ShuntTemperature", desc = "Shunt thermistor temperature, in celsius.", "{=f32}", voltages.shunt_temperature.get::<degree_celsius>());

            defmt_monitor::monitor!("HvPlate/Accumulated/CurrentSumMicrovolts", desc = "Summed shunt voltage across ACCN conversions, in microvolts, discharge positive. A running total, never reset here.", "{=i32}", accumulated.current_sum_microvolts);
            defmt_monitor::monitor!("HvPlate/Accumulated/BattSumMicrovolts", desc = "Summed BATT-tap voltage across ACCN conversions, in microvolts.", "{=i32}", accumulated.batt_sum_microvolts);
            defmt_monitor::monitor!("HvPlate/Accumulated/I1Cnt", desc = "I1ADC conversion counter. Says how many conversions the accumulator sums cover.", "{=u16}", flag.i1cnt());
            defmt_monitor::monitor!("HvPlate/Accumulated/I2Cnt", desc = "I2ADC conversion counter.", "{=u8}", flag.i2cnt());

            // The chip's own rails and sensors.
            defmt_monitor::monitor!("HvPlate/Aux/Vref1p25", desc = "The 1.25 V reference, in volts. The TS divider and shunt thermistor both depend on it.", "{=f32}", aux.vref1p25.get::<volt>());
            defmt_monitor::monitor!("HvPlate/Aux/Vreg", desc = "Regulator output, in volts.", "{=f32}", aux.vreg.get::<volt>());
            defmt_monitor::monitor!("HvPlate/Aux/Vdd", desc = "VDD supply, in volts.", "{=f32}", aux.vdd.get::<volt>());
            defmt_monitor::monitor!("HvPlate/Aux/Vdig", desc = "Digital rail, in volts.", "{=f32}", aux.vdig.get::<volt>());
            defmt_monitor::monitor!("HvPlate/Aux/Epad", desc = "Exposed-pad voltage, in volts.", "{=f32}", aux.epad.get::<volt>());
            defmt_monitor::monitor!("HvPlate/Aux/Vdiv", desc = "Divided reference, in volts.", "{=f32}", aux.vdiv.get::<volt>());
            defmt_monitor::monitor!("HvPlate/Aux/DieTemperature", desc = "ADBMS2950B die temperature, in celsius. Not the shunt temperature.", "{=f32}", aux.die_temperature.get::<degree_celsius>());
            defmt_monitor::monitor!("HvPlate/Aux/SecondaryTemperature", desc = "The chip's second on-chip temperature sensor (TMP2), in celsius.", "{=f32}", aux.secondary_temperature.get::<degree_celsius>());
            defmt_monitor::monitor!("HvPlate/Aux/OscCount", desc = "Oscillator counter. Outside 0x34..=0x47 the chip asserts OSCFLT.", "{=u8}", aux.osccnt);

            // Chip status. The OCxR codes stay raw: scaling them needs the OCxGC gain bits from
            // CFGB, which only the Api caches (see Api::overcurrent_microvolts).
            defmt_monitor::monitor!("HvPlate/Status/I1CalComplete", desc = "Whether the I1ADC has finished initializing.", "{}", status.status.i1cal());
            defmt_monitor::monitor!("HvPlate/Status/I2CalComplete", desc = "Whether the I2ADC has finished initializing.", "{}", status.status.i2cal());
            defmt_monitor::monitor!("HvPlate/Status/RevId", desc = "Device revision identifier, raw four-bit code.", "{=u8}", status.status.revid());
            defmt_monitor::monitor!("HvPlate/OverCurrent/Oc1Code", desc = "OC1ADC raw result code. Scale with the OC1GC gain from CFGB.", "{=i8}", status.overcurrent.oc1r().raw());
            defmt_monitor::monitor!("HvPlate/OverCurrent/Oc2Code", desc = "OC2ADC raw result code. Scale with the OC2GC gain from CFGB.", "{=i8}", status.overcurrent.oc2r().raw());
            defmt_monitor::monitor!("HvPlate/OverCurrent/Oc3Code", desc = "OC3ADC raw result code. Scale with the OC3GC gain from CFGB.", "{=i8}", status.overcurrent.oc3r().raw());
        }
    }
}
