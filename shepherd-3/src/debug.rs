//! Debug data for segments.

use crate::segments;

/// Task that sends out debug segments data.
///
/// This is probably (?) just going to be a temporary thing until we get more organized (i.e., we may remove this task or rework it as the overall structure of shepherd-3 starts coming together).
/// For now, this is just meant to test out the segments subsystem and give an example of reading data from it.
#[embassy_executor::task]
pub async fn segments_debug() {
    use segments::{SEGMENTS_FRESH_DATA_SIGNAL, ChipId, ChipKind, CellId};
    use crate::units::{degree_celsius, volt};
    use crate::can;

    // Subscribe to Segments fresh data signal subscription so we are notified when new segments data comes in.
    let mut subscription = SEGMENTS_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

    loop {
        // Run one loop of this task every time new Segments data arrives.
        subscription.wait().await;

        // Get "raw" cache readings.
        let redundant_aux_raw = segments::cache().get_redundant_aux();
        let filtered_cell_voltages_raw = segments::cache().get_filtered_cell_voltages();
        let cell_voltages_raw = segments::cache().get_cell_voltages();
        let pwm_raw = segments::cache().get_pwm();
        let status_c_raw = segments::cache().get_status_c();
        let s_voltages_raw = segments::cache().get_s_voltages();
        let _fault_counts = segments::cache().get_fault_counts();

        // u_TODO - should probably inspect the PEC status and other metadata before transforming into NiceData, but i don't think TSECU-Shepherd does that so for now this is probably fine

        // Convert "raw" readings to NiceData. When `try_nice()` fails, it means that the cache hasn't been updated yet (since starting up), so we skip for now and go back to top of the loop.
        let Ok(redundant_aux) = redundant_aux_raw.try_nice() else {
            continue;
        };
        let Ok(filtered_cell_voltages) = filtered_cell_voltages_raw.try_nice() else {
            continue;
        };
        let Ok(_cell_voltages) = cell_voltages_raw.try_nice() else {
            continue;
        };
        let Ok(pwm) = pwm_raw.try_nice() else {
            continue;
        };
        let Ok(status_c) = status_c_raw.try_nice() else {
            continue;
        };
        let Ok(_s_voltages) = s_voltages_raw.try_nice() else {
            continue;
        };

        // Iterate through every chip and send data over CAN.
        #[cfg(true)]
        '_can: {
            for chip in ChipId::iter() {
                let temps = redundant_aux.chip(chip).to_temps().cell_temperatures;
                let volts = filtered_cell_voltages.chip(chip); // u_TODO - if charging, we use cell_voltages, if not charging, we use filtered_cell_voltages. This is what TSECU-Shepherd did. but there's no charging state rn so for now who cares
                let pwm = pwm.chip(chip);
                let cvs = status_c.chip(chip).cell_channel_comparison_faults;

                match chip.kind() {
                    ChipKind::Alpha => {
                        for (cell_a, cell_b) in CellId::iter_pairs() {
                            match can::try_send(
                                can::types::AlphaCellDataDebug {
                                    therm: temps.cell(cell_a).get::<degree_celsius>(),
                                    chip_id: chip.segment().as_u8(),

                                    // Cell A data.
                                    voltage_a: volts.cell(cell_a).get::<volt>(),
                                    cell_a: cell_a.as_u8(),
                                    discharging_a: pwm.cell(cell_a).is_balancing(),
                                    cvs_a: cvs.cell(cell_a).is_set(),
                                    ow_a: false,

                                    // Cell B data. When cell_b is `None` (due to the enum having an odd number of variants), just pass in random obviously-wrong numbers
                                    voltage_b: cell_b.map(|cell_b| volts.cell(cell_b).get::<volt>()).unwrap_or(14_f32),
                                    cell_b: cell_b.map(|cell_b| cell_b.as_u8()).unwrap_or(14),
                                    discharging_b: cell_b.map(|cell_b| pwm.cell(cell_b).is_balancing()).unwrap_or(false),
                                    cvs_b: cell_b.map(|cell_b| pwm.cell(cell_b).is_balancing()).unwrap_or(false),
                                    ow_b: false,
                                }
                                .as_frame(),
                            ) {
                                Ok(_) => (),
                                Err(_) => (),
                            }
                        }
                    },

                    ChipKind::Beta => {
                        for (cell_a, cell_b) in CellId::iter_pairs() {
                            match can::try_send(
                                can::types::BetaCellDataDebug {
                                    therm: temps.cell(cell_a).get::<degree_celsius>(),
                                    chip_id: chip.segment().as_u8(),

                                    // Cell A data.
                                    voltage_a: volts.cell(cell_a).get::<volt>(),
                                    cell_a: cell_a.as_u8(),
                                    discharging_a: pwm.cell(cell_a).is_balancing(),
                                    cvs_a: cvs.cell(cell_a).is_set(),
                                    ow_a: false,

                                    // Cell B data. When cell_b is `None` (due to the enum having an odd number of variants), just pass in random obviously-wrong numbers
                                    voltage_b: cell_b.map(|cell_b| volts.cell(cell_b).get::<volt>()).unwrap_or(14_f32),
                                    cell_b: cell_b.map(|cell_b| cell_b.as_u8()).unwrap_or(14),
                                    discharging_b: cell_b.map(|cell_b| pwm.cell(cell_b).is_balancing()).unwrap_or(false),
                                    cvs_b: cell_b.map(|cell_b| pwm.cell(cell_b).is_balancing()).unwrap_or(false),
                                    ow_b: false,
                                }
                                .as_frame(),
                            ) {
                                Ok(_) => (),
                                Err(_) => (),
                            }
                        }
                    },
                }
            }
        }

        #[cfg(defmt_monitor)]
        '_defmt_monitor: {
            for chip in ChipId::iter() {
                // Chip-level logs.
                defmt_monitor::monitor!(["SegmentDebug/Chips/Chip{=u8}/Segment", chip.as_u8()], desc = "What segment this chip is on (0 through 4).", "{=u8}", chip.segment().as_u8());
                defmt_monitor::monitor!(["SegmentDebug/Chips/Chip{=u8}/Kind", chip.as_u8()], desc = "If this chip is Alpha or Beta.", "{}", chip.kind());

                let volts = filtered_cell_voltages.chip(chip);
                let temps = redundant_aux.chip(chip).to_temps().cell_temperatures;
                let comparison_fault_counts = _fault_counts.chip(chip).csxflt.idx_by_cell();

                for cell in CellId::iter() {
                    // Cell-level logs.
                    defmt_monitor::monitor!(["SegmentDebug/Chips/Chip{=u8}/Cell{=u8}/Voltage", chip.as_u8(), cell.as_u8()], desc = "Cell voltage, in volts.", "{=f32}", volts.cell(cell).get::<volt>());
                    defmt_monitor::monitor!(["SegmentDebug/Chips/Chip{=u8}/Cell{=u8}/Temperature", chip.as_u8(), cell.as_u8()], desc = "Cell temperautre, in celsius.", "{=f32}", temps.cell(cell).get::<degree_celsius>());
                    defmt_monitor::monitor!(["SegmentDebug/Chips/Chip{=u8}/Cell{=u8}/ComparisonFaultCounts", chip.as_u8(), cell.as_u8()], desc = "Total number of comparison faults that have been read back for this cell so far since boot.", "{=u32}", comparison_fault_counts.cell(cell));
                }
            }
        }
    }
}

/// Task that sends out debug HV plate data.
/// Lowk sends way too much stuff but we can use it for verification of life for now.
#[cfg(not(feature = "hil"))]
#[embassy_executor::task]
pub async fn hv_plate_debug() {
    use crate::hv_plate::{self, HV_PLATE_FRESH_DATA_SIGNAL};
    use crate::units::{degree_celsius, volt};
    use uom::si::electric_current::ampere;

    // Subscribe to the HV plate fresh data signal so we are notified when new data comes in.
    let mut subscription = HV_PLATE_FRESH_DATA_SIGNAL.subscribe().expect("There are too many waiters on this signal. We should probably increase the waiters capacity.");

    loop {
        // Run one loop of this task every time new HV plate data arrives.
        subscription.wait().await;

        // Get "raw" cache readings.
        let current_voltage_raw = hv_plate::cache().get_current_voltage();
        let voltages_raw = hv_plate::cache().get_voltages();
        let accumulated_raw = hv_plate::cache().get_accumulated();
        let flag_raw = hv_plate::cache().get_flag();
        let aux_raw = hv_plate::cache().get_aux();
        let status_raw = hv_plate::cache().get_status();

        // Convert "raw" readings to NiceData. When `try_nice()` fails, it means that the cache hasn't been updated yet (since starting up), so we skip for now and go back to top of the loop.
        //
        // The accumulator, FLAG and STATUS readings are only consumed by the monitor block, so
        // they are annotated for the `DEFMT_MONITOR=off` build. They are not dead even then:
        // each `else { continue }` is what holds the cycle back until that register group has
        // actually been read.
        let Ok(current_voltage) = current_voltage_raw.try_nice() else {
            continue;
        };
        let Ok(voltages) = voltages_raw.try_nice() else {
            continue;
        };
        #[cfg_attr(not(defmt_monitor), allow(unused_variables))]
        let Ok(accumulated) = accumulated_raw.try_nice() else {
            continue;
        };
        #[cfg_attr(not(defmt_monitor), allow(unused_variables))]
        let Ok(flag) = flag_raw.try_nice() else {
            continue;
        };
        let Ok(aux) = aux_raw.try_nice() else {
            continue;
        };
        #[cfg_attr(not(defmt_monitor), allow(unused_variables))]
        let Ok(status) = status_raw.try_nice() else {
            continue;
        };

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

            // Coulomb counting. The sums are running totals this firmware never clears, so a
            // consumer integrates by diffing against its own previous value; the conversion
            // count says how many samples each sum covers.
            defmt_monitor::monitor!("HvPlate/Accumulated/CurrentSumMicrovolts", desc = "Summed shunt voltage across ACCN conversions, in microvolts. A running total, never reset here.", "{=i32}", accumulated.current_sum_microvolts);
            defmt_monitor::monitor!("HvPlate/Accumulated/BattSumMicrovolts", desc = "Summed BATT-tap voltage across ACCN conversions, in microvolts.", "{=i32}", accumulated.batt_sum_microvolts);
            defmt_monitor::monitor!("HvPlate/Accumulated/I1Cnt", desc = "I1ADC conversion counter. Says how many conversions the accumulator sums cover.", "{=u16}", flag.i1cnt);
            defmt_monitor::monitor!("HvPlate/Accumulated/I2Cnt", desc = "I2ADC conversion counter.", "{=u8}", flag.i2cnt);

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
