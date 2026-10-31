use defmt::{info, warn};
use embassy_stm32::{Peri, gpio::Output, peripherals::PG15};
use embassy_time::{Duration, Instant, Timer};
use heapless::{Vec, index_set::FnvIndexSet};
use strum::{EnumCount, FromRepr};
#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy, FromRepr, EnumCount, defmt::Format)]
pub enum Fault {
    CAN_OUTGOING_FAULT = 0,
    CAN_INCOMING_FAULT,
    BMS_CAN_MONITOR_FAULT,
    LIGHTNING_CAN_MONITOR_FAULT,
    PRECHARGE_FLOATING_FAULT,
    MOTOR_TEMP_SENSOR_FAULT,

    ONBOARD_TEMP_FAULT,
    IMU_ACCEL_FAULT,
    IMU_GYRO_FAULT,
    BSPD_PREFAULT,
    ONBOARD_BRAKE_OPEN_CIRCUIT_FAULT,
    ONBOARD_ACCEL_OPEN_CIRCUIT_FAULT,
    ONBOARD_BRAKE_SHORT_CIRCUIT_FAULT,
    ONBOARD_ACCEL_SHORT_CIRCUIT_FAULT,
    ONBOARD_PEDAL_DIFFERENCE_FAULT,
    RTDS_FAULT,
    LV_LOW_VOLTAGE_FAULT,
    // DO NOT ADD MORE THAN 32 FAULTS. THIS MUST FIT NEATLY IN A U32
}
const _: () = assert!(Fault::COUNT <= 32);

impl Fault {
    pub fn is_critical(&self) -> bool {
        self.as_bitmap_component() & Fault::CRITICAL_MASK != 0
    }
    fn as_bitmap_component(&self) -> u32 {
        1 << (*self as u32)
    }
    fn get_timeout(&self) -> Duration {
        match self {
            // This is a stub, but it can be updated as needed
            _ => Duration::from_millis(5000),
        }
    }

    // fn get_duration(&self) -> Duration {}
    const CRITICAL_MASK: u32 = (1 << Fault::CAN_OUTGOING_FAULT as u32)
        | (1 << Fault::CAN_INCOMING_FAULT as u32)
        | (1 << Fault::BMS_CAN_MONITOR_FAULT as u32)
        | (1 << Fault::LIGHTNING_CAN_MONITOR_FAULT as u32)
        | (1 << Fault::PRECHARGE_FLOATING_FAULT as u32)
        | (1 << Fault::MOTOR_TEMP_SENSOR_FAULT as u32);
}

pub struct AllFaults {
    // A bitmap of faults s.t `1<<(fault as usize)` is the bit in question
    faults: u32,
    // expirations[fault] gives the deadline for the pertinent fault
    expirations: [Instant; Fault::COUNT],
}

impl AllFaults {
    pub fn new() -> Self {
        AllFaults {
            faults: 0x0,
            expirations: [Instant::MAX; Fault::COUNT],
        }
    }
    pub fn bitmap(&self) -> u32 {
        self.faults
    }
    pub fn has_a_fault(&self) -> bool {
        self.faults > 0
    }
    pub fn critical_fault_present(&self) -> bool {
        self.faults & Fault::CRITICAL_MASK != 0
    }
    pub fn num_faults(self) -> u8 {
        self.faults.count_ones() as u8
    }
    pub fn trigger_fault(&mut self, fault: Fault) {
        info!("Fault triggered: {:?}", fault);
        if fault.is_critical() {
            warn!("Critical fault!");
        }
        self.faults |= fault.as_bitmap_component();
        self.expirations[fault as usize] = Instant::now() + fault.get_timeout();
    }
    pub fn reset_fault(&mut self, fault: Fault) {
        self.faults &= !fault.as_bitmap_component();
        self.expirations[fault as usize] = Instant::MAX;
        info!("Reset fault: {:?}", fault);
    }
}

pub async fn fault_task(mut faults: AllFaults) {
    loop {
        if let Some((i, _)) = faults
            .expirations
            .iter()
            .enumerate()
            .find(|(_, e)| **e <= Instant::now())
        {
            if let Some(dubious_fault) = Fault::from_repr(i) {
                faults.reset_fault(dubious_fault);
            }
        }
        Timer::after_millis(10).await;
    }
}

pub struct MCUFaultSetter {
    pin: Output<'static>,
}
impl MCUFaultSetter {
    pub fn new(pin: Output<'static>) -> Self {
        MCUFaultSetter { pin: pin }
    }
    // Ripped from cerb C impl
    pub fn set_mcu_fault(&mut self, faulted: bool) {
        // The MCU Fault pin is kind of "swapped".
        // Setting the pin to HIGH indicates that there is no fault.
        // Setting the pin to LOW indicates that there is a fault.
        // The pin has a default state of HIGH (i.e. no fault).
        if faulted {
            // If there is a fault, set the fault pin to LOW.
            info!("Turned on MCU fault.");
            self.pin.set_low();
        } else {
            // If there is not a fault, set the pin to HIGH.
            info!("Turned off MCU fault.");
            self.pin.set_high();
        }
    }
}
