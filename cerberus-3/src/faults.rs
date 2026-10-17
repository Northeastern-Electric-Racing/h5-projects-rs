use heapless::{Vec, index_set::FnvIndexSet};

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum Fault {
    CAN_OUTGOING_FAULT = 1,
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
}

impl Fault {
    pub fn is_critical(&self) -> bool {
        match self {
            Fault::CAN_OUTGOING_FAULT
            | Fault::CAN_INCOMING_FAULT
            | Fault::BMS_CAN_MONITOR_FAULT
            | Fault::LIGHTNING_CAN_MONITOR_FAULT
            | Fault::PRECHARGE_FLOATING_FAULT
            | Fault::MOTOR_TEMP_SENSOR_FAULT => true,
            _ => false,
        }
    }
    fn as_bitmap_component(&self) -> u32 {
        let two: u32 = 2; // This is the worst line of code I have ever written
        two.pow(self.clone() as u32) // Assuming a correct ordering of enums, this converts an enum to a bit
        // in a guaranteed unique position
    }
}

pub struct AllFaults {
    faults: FnvIndexSet<Fault, 32>,
}

impl AllFaults {
    pub fn bitmap(&self) -> u32 {
        self.faults
            .iter()
            .map(|fault| fault.as_bitmap_component())
            .collect::<Vec<u32, 32>>()
            .iter()
            .sum()
    }
    pub fn critical_fault_present(&self) -> bool {
        self.faults
            .iter()
            .map(|fault| fault.is_critical())
            .collect::<Vec<bool, 32>>()
            .contains(&true)
    }
    pub fn num_faults(self) -> u8 {
        self.faults.len() as u8
    }
}
