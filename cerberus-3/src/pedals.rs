use defmt::bitflags;
use embassy_time::{Duration, Instant, Timer};
use variant_count::VariantCount;

const PEDALS_PERIOD: Duration = Duration::from_millis(10);
const BRAKE_FAULT_DEBOUNCE: Duration = Duration::from_millis(300);
const PEDAL_FAULT_DEBOUNCE: Duration = Duration::from_millis(95);
const BRAKE_SENSOR_IRREGULAR_HIGH: f32 = 4.5; // Volts
const BRAKE_SENSOR_IRREGULAR_LOW: f32 = 0.5; // Volts
const BRAKE_THRESHOLD_TOLERANCE: f32 = 0.25; // Volts
const APPS_THRESHOLD_TOLERANCE: f32 = 0.20; // Volts
const PEDAL_DIFF_THRESH: f32 = 0.20;
const PEDAL_HARD_BRAKE_THRESH: f32 = 0.20;
pub const PEDAL_BRAKE_THRESH: f32 = 0.15;
const MAX_VOLTS_UNSCALED: f32 = 5.0; // Volts
const MIN_APPS1_VOLTS: f32 = 2.1; // Volts
const MIN_APPS2_VOLTS: f32 = 1.1; // Volts
pub const PEDAL_BRAKE_TURNOFF_THRESH: f32 = 0.004;

#[derive(VariantCount, Clone, Copy)]
pub enum DriveLocks {
    BrakeOc,
    BrakeSc,
    AccelOc,
    AccelSc,
    AccelDiff,
    BspdPref,
    BmsNotPrechargedYet,
}

bitflags!(
    pub struct DriveFaultFlags: u8 {
        const BRAKE_OC = 1 << 1;
        const BRAKE_SC = 1 << 2;
        const ACCEL_OC = 1 << 3;
        const ACCEL_SC = 1 << 4;
        const ACCEL_DIFF = 1 << 5;
        const BSPD_PREF = 1 << 6;
        const BMS_NOT_PRECHARGED_YET = 1 << 7;
    }
);

impl DriveLocks {
    fn get_flag(&self) -> DriveFaultFlags {
        match self {
            DriveLocks::BrakeOc => DriveFaultFlags::BRAKE_OC,
            DriveLocks::BrakeSc => DriveFaultFlags::BRAKE_SC,
            DriveLocks::AccelOc => DriveFaultFlags::ACCEL_OC,
            DriveLocks::AccelSc => DriveFaultFlags::ACCEL_SC,
            DriveLocks::AccelDiff => DriveFaultFlags::ACCEL_DIFF,
            DriveLocks::BspdPref => DriveFaultFlags::BSPD_PREF,
            DriveLocks::BmsNotPrechargedYet => DriveFaultFlags::BMS_NOT_PRECHARGED_YET,
        }
    }
}

struct DriveLocksState {
    drive_locks: DriveFaultFlags,
    brake_pressed: bool,
    accel_pressed: bool,
    debouncers: [Option<Instant>; DriveLocks::VARIANT_COUNT],
    motor_disabled: bool,
}

impl DriveLocksState {
    const INIT: Self = Self {
        drive_locks: DriveFaultFlags::empty(),
        brake_pressed: false,
        accel_pressed: false,
        debouncers: [None; DriveLocks::VARIANT_COUNT],
        motor_disabled: false,
    };

    fn set_lock(&mut self, fault: DriveLocks) {
        self.drive_locks.insert(fault.get_flag());
    }

    fn queue_fault(&self, fault: DriveLocks) {
        match fault {
            DriveLocks::BrakeOc => {}
            DriveLocks::BrakeSc => {}
            DriveLocks::AccelOc => {}
            DriveLocks::AccelSc => {}
            DriveLocks::AccelDiff => {}
            DriveLocks::BspdPref => {}
            DriveLocks::BmsNotPrechargedYet => {}
        }
        todo!("Call appropriate fault");
    }

    fn unset_lock(&mut self, fault: DriveLocks) {
        self.drive_locks.remove(fault.get_flag());
    }

    fn any_locked(&self) -> bool {
        !self.drive_locks.is_empty()
    }

    fn get_lock_state(&self, fault: DriveLocks) -> bool {
        self.drive_locks.contains(fault.get_flag())
    }

    /// Sets the debounce start, queues the fault and sets the debounce if expired
    fn debounce(&mut self, fault: DriveLocks, condition: bool, time: &Duration) {
        let i = fault as usize;
        if condition {
            if self.debouncers[i].is_none() {
                self.debouncers[i] = Some(Instant::now());
            }
            if self.debouncers[i].unwrap().elapsed().ge(time) {
                self.set_lock(fault);
                self.queue_fault(fault)
            }
        } else {
            self.debouncers[i] = None;
            self.unset_lock(fault);
        }
    }

    fn calculate_brake_fault(&mut self, brake1_voltage: f32, brake2_voltage: f32) {
        let open_circuit_fault = (brake1_voltage
            > BRAKE_SENSOR_IRREGULAR_HIGH + BRAKE_THRESHOLD_TOLERANCE)
            || (brake2_voltage > BRAKE_SENSOR_IRREGULAR_HIGH + BRAKE_THRESHOLD_TOLERANCE);
        self.debounce(
            DriveLocks::BrakeOc,
            open_circuit_fault,
            &BRAKE_FAULT_DEBOUNCE,
        );

        let short_circuit_fault = (brake1_voltage
            < BRAKE_SENSOR_IRREGULAR_LOW - BRAKE_THRESHOLD_TOLERANCE)
            || (brake2_voltage < BRAKE_SENSOR_IRREGULAR_LOW - BRAKE_THRESHOLD_TOLERANCE);
        self.debounce(
            DriveLocks::BrakeSc,
            short_circuit_fault,
            &BRAKE_FAULT_DEBOUNCE,
        );
    }

    fn calculate_accel_fault(
        &mut self,
        accel1_voltage: f32,
        accel2_voltage: f32,
        accel1_percentage: f32,
        accel2_percentage: f32,
    ) {
        let open_circuit_fault = (accel1_voltage > MAX_VOLTS_UNSCALED + APPS_THRESHOLD_TOLERANCE)
            || (accel2_voltage > MAX_VOLTS_UNSCALED + APPS_THRESHOLD_TOLERANCE);
        self.debounce(
            DriveLocks::AccelOc,
            open_circuit_fault,
            &PEDAL_FAULT_DEBOUNCE,
        );

        let short_circuit_fault = (accel1_voltage < MIN_APPS1_VOLTS - APPS_THRESHOLD_TOLERANCE)
            || (accel2_voltage < MIN_APPS2_VOLTS - APPS_THRESHOLD_TOLERANCE);
        self.debounce(
            DriveLocks::AccelSc,
            short_circuit_fault,
            &PEDAL_FAULT_DEBOUNCE,
        );

        let pedal_diff_fault = (accel1_percentage - accel2_percentage).abs() > PEDAL_DIFF_THRESH;
        self.debounce(
            DriveLocks::AccelDiff,
            pedal_diff_fault,
            &PEDAL_FAULT_DEBOUNCE,
        );
    }

    fn calculate_bspd_fault(
        &mut self,
        accel_percentage: f32,
        brake_percentage: f32,
        dc_current: f32,
    ) {
        if brake_percentage > PEDAL_HARD_BRAKE_THRESH
            && (accel_percentage > 0.25 || dc_current > 0.1)
        {
            self.motor_disabled = true;
            // TODO: queue BSPD_prefault
            self.queue_fault(DriveLocks::BspdPref);
            self.set_lock(DriveLocks::BspdPref);
        }

        if self.motor_disabled
            && accel_percentage < 0.05
            && brake_percentage < PEDAL_HARD_BRAKE_THRESH
        {
            self.motor_disabled = false;
            self.unset_lock(DriveLocks::BspdPref);
        }
    }
}

#[embassy_executor::task]
pub async fn pedal_task() {
    let mut drive_lock: DriveLocksState = DriveLocksState::INIT;

    loop {
        drive_lock.calculate_accel_fault(0.0, 0.0, 0.0, 0.0);
        drive_lock.calculate_brake_fault(0.0, 0.0);

        if
        /* todo: percentage_brake > */
        0.0 > PEDAL_BRAKE_THRESH {
            drive_lock.brake_pressed = true;
        } else if
        /* todo: percentage_brake */
        0.0 < PEDAL_BRAKE_THRESH - PEDAL_BRAKE_TURNOFF_THRESH {
            drive_lock.brake_pressed = false;
        }

        drive_lock.accel_pressed = /* percentage_accel */ 0.0 > 0.05;

        drive_lock.calculate_bspd_fault(0.0, 0.0, 0.0);

        if
        /* todo: BMS precharge not set */
        false {
            drive_lock.set_lock(DriveLocks::BmsNotPrechargedYet);
        } else {
            drive_lock.unset_lock(DriveLocks::BmsNotPrechargedYet);
        }

        if drive_lock.any_locked() {
            // TODO: dti set torque 0
            continue;
        }

        // todo: call tc_process

        Timer::after(PEDALS_PERIOD).await;
    }
}
