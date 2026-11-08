use defmt::{debug, info, warn};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::{can::Frame, gpio::Output};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use strum::{EnumCount, FromRepr};

use cangen::ToCanFrame;

use crate::can;
use crate::statemachine::{self, set_ready_state};

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

const FAULT_CHANNEL_SIZE: usize = 32;

pub static INBOUND_FAULTS: Channel<ThreadModeRawMutex, Fault, FAULT_CHANNEL_SIZE> = Channel::new();

impl Fault {
    pub fn is_critical(&self) -> bool {
        self.as_bitmap_component() & Fault::CRITICAL_MASK != 0
    }
    fn as_bitmap_component(&self) -> u32 {
        1 << (*self as u32)
    }
    fn get_timeout(&self) -> Duration {
        // This should be made into a match if different faults are to get different timeouts
        Duration::from_millis(500)
    }

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
        debug!("Fault triggered: {:?}", fault);
        if fault.is_critical() {
            warn!("Critical fault triggered! {:?}", &fault);
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
        cangen::Faults::new()
            .with_CAN_OUTGOING_FAULT(self.get_fault(Fault::CAN_OUTGOING_FAULT))
            .with_CAN_INCOMING_FAULT(self.get_fault(Fault::CAN_INCOMING_FAULT))
            .with_BMS_CAN_MONITOR_FAULT(self.get_fault(Fault::BMS_CAN_MONITOR_FAULT))
            .with_LIGHTNING_CAN_MONITOR_FAULT(self.get_fault(Fault::LIGHTNING_CAN_MONITOR_FAULT))
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

    pub fn reset_expired_faults(&mut self) {
        self.expirations
            .clone()
            .iter_mut()
            .enumerate()
            .for_each(|(id, deadline)| {
                if deadline.elapsed() >= Duration::from_ticks(0)
                    && let Some(some_fault) = Fault::from_repr(id)
                {
                    self.reset_fault(some_fault);
                }
            })
    }
}

pub async fn raise_fault(fault: Fault) {
    INBOUND_FAULTS.send(fault).await;
}

/// To trigger a fault, simply append it to the channel and the task takes care of the rest
#[embassy_executor::task]
pub async fn fault_task() {
    let mut faults = AllFaults::new();
    const TIMEOUT_DURATION: Duration = Duration::from_millis(500);

    loop {
        let (_, deadline) = faults
            .expirations
            .iter()
            .copied()
            .enumerate()
            .min_by_key(|&(_, deadline)| deadline)
            .unwrap_or((0, Instant::MAX));

        match select3(
            async move { Timer::at(deadline).await },
            INBOUND_FAULTS.receive(),
            Timer::after(TIMEOUT_DURATION),
        )
        .await
        {
            Either3::First(_) => {
                faults.reset_expired_faults();
                if !faults.critical_fault_present() {
                    set_ready_state().await;
                }
            }
            Either3::Second(newfault) => {
                faults.trigger_fault(newfault);
                if faults.critical_fault_present()
                    && statemachine::get_func_state().await != statemachine::FuncState::Faulted
                {
                    statemachine::set_fault_state().await;
                }
            }
            Either3::Third(_) => {}
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
