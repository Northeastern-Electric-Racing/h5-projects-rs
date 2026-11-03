use defmt::{info, warn};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::{can::Frame, gpio::Output};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use strum::{EnumCount, FromRepr};

use cangen::ToCanFrame;

use crate::{
    can,
    statemachine::{self, set_ready_state},
};

pub static INBOUND_FAULTS: Channel<ThreadModeRawMutex, Fault, 32> = Channel::new();

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
        Duration::from_millis(500)
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

#[allow(dead_code)]
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
    pub fn get_fault(&self, fault: Fault) -> bool {
        self.faults & fault.as_bitmap_component() != 0
    }
    pub fn has_a_fault(&self) -> bool {
        self.faults != 0
    }
    pub fn critical_fault_present(&self) -> bool {
        self.faults & Fault::CRITICAL_MASK != 0
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
    pub fn as_can_frame(&self) -> Frame {
        // WARN: MOTOR_TEMP_SENSOR_FAULT has no field in the Faults CAN message.
        cangen::Faults::new()
            .with_CAN_OUTGOING_FAULT(self.get_fault(Fault::CAN_OUTGOING_FAULT))
            .with_CAN_INCOMING_FAULT(self.get_fault(Fault::CAN_INCOMING_FAULT))
            .with_BMS_CAN_MONITOR_FAULT(self.get_fault(Fault::BMS_CAN_MONITOR_FAULT))
            .with_LIGHTNING_CAN_MONITOR_FAULT(self.get_fault(Fault::LIGHTNING_CAN_MONITOR_FAULT))
            // .with_MOTOR_TEMP_SENSOR_FAULT(self.get_fault(Fault::MOTOR_TEMP_SENSOR_FAULT)) This
            // one doesn't exist
            .with_ONBOARD_TEMP_FAULT(self.get_fault(Fault::ONBOARD_TEMP_FAULT))
            .with_IMU_ACCEL_FAULT(self.get_fault(Fault::IMU_ACCEL_FAULT))
            .with_IMU_GYRO_FAULT(self.get_fault(Fault::IMU_GYRO_FAULT))
            .with_BSPD_PREFAULT(self.get_fault(Fault::BSPD_PREFAULT))
            .with_ONBOARD_BRAKE_OPEN_CIRCUIT_FAULT(
                self.get_fault(Fault::ONBOARD_BRAKE_OPEN_CIRCUIT_FAULT),
            )
            .with_ONBOARD_ACCEL_OPEN_CIRCUIT_FAULT(
                self.get_fault(Fault::ONBOARD_ACCEL_OPEN_CIRCUIT_FAULT),
            )
            .with_ONBOARD_BRAKE_SHORT_CIRCUIT_FAULT(
                self.get_fault(Fault::ONBOARD_BRAKE_SHORT_CIRCUIT_FAULT),
            )
            .with_ONBOARD_ACCEL_SHORT_CIRCUIT_FAULT(
                self.get_fault(Fault::ONBOARD_ACCEL_SHORT_CIRCUIT_FAULT),
            )
            .with_ONBOARD_PEDAL_DIFFERENCE_FAULT(
                self.get_fault(Fault::ONBOARD_PEDAL_DIFFERENCE_FAULT),
            )
            .with_RTDS_FAULT(self.get_fault(Fault::RTDS_FAULT))
            .with_LV_LOW_VOLTAGE_FAULT(self.get_fault(Fault::LV_LOW_VOLTAGE_FAULT))
            .with_PRECHARGE_FLOATING_FAULT(self.get_fault(Fault::PRECHARGE_FLOATING_FAULT))
            .to_can_frame()
    }
}

/// To trigger a fault, simply append it to the channel and the task takes care of the rest
#[embassy_executor::task]
pub async fn fault_task() {
    let mut faults = AllFaults::new();
    const TIMEOUT_DURATION: Duration = Duration::from_millis(5000);

    loop {
        let (position, deadline) = faults
            .expirations
            .iter()
            .copied()
            .enumerate()
            .min_by_key(|&(_, deadline)| deadline)
            .unwrap();

        match select3(
            async move {
                Timer::at(deadline).await;
                (deadline, position)
            },
            Timer::after(TIMEOUT_DURATION),
            INBOUND_FAULTS.receive(),
        )
        .await
        {
            Either3::First(expired_fault) => {
                if let Some(expired_fault) = Fault::from_repr(expired_fault.1) {
                    faults.reset_fault(expired_fault);
                    if !faults.critical_fault_present() {
                        set_ready_state().await;
                    }
                }
            }
            Either3::Second(_) => {
                info!("Timed out");
            }
            Either3::Third(newfault) => {
                faults.trigger_fault(newfault);
                if faults.critical_fault_present()
                    && statemachine::get_func_state().await != statemachine::FuncState::Faulted
                {
                    statemachine::set_fault_state().await;
                }
            }
        };
        can::OUTGOING.send(faults.as_can_frame()).await;
    }
}

/// The MCU Fault pin is kind of "swapped".
/// Setting the pin to HIGH indicates that there is no fault.
/// Setting the pin to LOW indicates that there is a fault.
/// The pin has a default state of HIGH (i.e. no fault).
pub fn set_mcu_fault(pin: &mut Output<'static>, faulted: bool) {
    if faulted {
        // If there is a fault, set the fault pin to LOW.
        info!("Turned on MCU fault.");
        pin.set_low();
    } else {
        // If there is not a fault, set the pin to HIGH.
        info!("Turned off MCU fault.");
        pin.set_high();
    }
}
