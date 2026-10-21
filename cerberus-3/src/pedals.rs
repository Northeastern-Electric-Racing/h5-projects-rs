//! pedals
//! handles all pedals related functions <br>
//! ie, drive locks, torque and regen calls

use cangen::ToCanFrame;
use core::marker::PhantomData;
use defmt::bitflags;
use embassy_stm32::adc::{Adc, AdcChannel, Clock, Config, Prescaler, Resolution, SampleTime};
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Timer};
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::f32::{ElectricCurrent, ElectricPotential, Pressure, Ratio};
use uom::si::pressure::psi;
use uom::si::ratio::ratio;
use variant_count::VariantCount;


/// Build a uom quantity in a const context. Plain values are in SI base units.
macro_rules! q {
    (Ratio, $v:expr, percent) => {
        Ratio {
            dimension: PhantomData,
            units: PhantomData,
            value: $v / 100.0,
        }
    };
    ($t:ident, $v:expr) => {
        $t {
            dimension: PhantomData,
            units: PhantomData,
            value: $v,
        }
    };
}
bind_interrupts!(struct Irqs {
    ADC2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
    GPDMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH4>;
});

const PEDALS_TASK_PERIOD: Duration = Duration::from_millis(100);
const BRAKE_FAULT_DEBOUNCE: Duration = Duration::from_millis(300);
const PEDAL_FAULT_DEBOUNCE: Duration = Duration::from_millis(95);

const BRAKE_SENSOR_IRREGULAR_HIGH: ElectricPotential = q!(ElectricPotential, 4.5);
const BRAKE_SENSOR_IRREGULAR_LOW: ElectricPotential = q!(ElectricPotential, 0.5);
const BRAKE_THRESHOLD_TOLERANCE: ElectricPotential = q!(ElectricPotential, 0.25);
const APPS_THRESHOLD_TOLERANCE: ElectricPotential = q!(ElectricPotential, 0.2);
const PEDAL_DIFF_THRESH: Ratio = q!(Ratio, 20.0, percent);
const PEDAL_HARD_BRAKE_THRESH: Ratio = q!(Ratio, 20.0, percent);
pub const PEDAL_BRAKE_THRESH: Ratio = q!(Ratio, 15.0, percent);
const MAX_VOLTS_UNSCALED: ElectricPotential = q!(ElectricPotential, 5.0);
const MIN_APPS1_VOLTS: ElectricPotential = q!(ElectricPotential, 2.1);
const MAX_APPS1_VOLTS: ElectricPotential = q!(ElectricPotential, 3.4);
const MIN_APPS2_VOLTS: ElectricPotential = q!(ElectricPotential, 1.1);
const MAX_APPS2_VOLTS: ElectricPotential = q!(ElectricPotential, 2.2);
pub const PEDAL_BRAKE_TURNOFF_THRESH: Ratio = q!(Ratio, 0.4, percent);

static LOCK_MUTEX: Mutex<ThreadModeRawMutex, DriveLocksState> = Mutex::new(DriveLocksState::INIT);

#[derive(VariantCount, Clone, Copy, Debug)]
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

#[derive(Clone, Copy)]
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

    fn calculate_brake_fault(
        &mut self,
        brake1_voltage: ElectricPotential,
        brake2_voltage: ElectricPotential,
    ) {
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
        accel1_voltage: ElectricPotential,
        accel2_voltage: ElectricPotential,
        accel1_percentage: Ratio,
        accel2_percentage: Ratio,
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
        accel_percentage: Ratio,
        brake_percentage: Ratio,
        dc_current: ElectricCurrent,
    ) {
        if brake_percentage > PEDAL_HARD_BRAKE_THRESH
            && (accel_percentage > Ratio::new::<ratio>(0.25)
                || dc_current > ElectricCurrent::new::<ampere>(0.1))
        {
            self.motor_disabled = true;
            // TODO: queue BSPD_prefault
            self.queue_fault(DriveLocks::BspdPref);
            self.set_lock(DriveLocks::BspdPref);
        }

        if self.motor_disabled
            && accel_percentage < Ratio::new::<ratio>(0.05)
            && brake_percentage < PEDAL_HARD_BRAKE_THRESH
        {
            self.motor_disabled = false;
            self.unset_lock(DriveLocks::BspdPref);
        }
    }
}

pub async fn get_brake_state() -> bool {
    LOCK_MUTEX.lock().await.brake_pressed
}

pub async fn get_accel_state() -> bool {
    LOCK_MUTEX.lock().await.accel_pressed
}

#[embassy_executor::task]
pub async fn pedals_task() {
    let mut drive_lock: DriveLocksState = DriveLocksState::INIT;

    let mut pedal_data: PedalData = PedalData {
        accel1_v: ElectricPotential::new::<volt>(0.0),
        accel2_v: ElectricPotential::new::<volt>(0.0),
        brake1_v: ElectricPotential::new::<volt>(0.0),
        brake2_v: ElectricPotential::new::<volt>(0.0),
        accel1_r: Ratio::new::<ratio>(0.0),
        accel2_r: Ratio::new::<ratio>(0.0),
        accel_r: Ratio::new::<ratio>(0.0),
        brake_r: Ratio::new::<ratio>(0.0),
        brake1_p: Default::default(),
        brake2_p: Default::default(),
    };

    loop {
        let pedal_data: PedalData = get_raw_pedal_data().await.into();

        drive_lock.calculate_accel_fault(
            pedal_data.accel1_v,
            pedal_data.accel2_v,
            pedal_data.accel1_r,
            pedal_data.accel2_r,
        );
        drive_lock.calculate_brake_fault(pedal_data.brake1_v, pedal_data.brake2_v);

        if pedal_data.brake_r > PEDAL_BRAKE_THRESH {
            drive_lock.brake_pressed = true;
        } else if pedal_data.brake_r < PEDAL_BRAKE_THRESH - PEDAL_BRAKE_TURNOFF_THRESH {
            drive_lock.brake_pressed = false;
        }

        drive_lock.accel_pressed = pedal_data.accel_r > Ratio::new::<ratio>(0.05);

        drive_lock.calculate_bspd_fault(
            pedal_data.accel_r,
            pedal_data.brake_r,
            ElectricCurrent::new::<ampere>(0.0),
        );

        if
        /* todo: BMS precharge not set */
        false {
            drive_lock.set_lock(DriveLocks::BmsNotPrechargedYet);
        } else {
            drive_lock.unset_lock(DriveLocks::BmsNotPrechargedYet);
        }

        if drive_lock.any_locked() {
            // TODO: dti set torque 0
        } else {
            // todo: call tc_process

            crate::can::OUTGOING
                .send(pedal_data.to_volt_frame().to_can_frame())
                .await;
            crate::can::OUTGOING
                .send(pedal_data.to_per_frame().to_can_frame())
                .await;
        }
        *LOCK_MUTEX.lock().await = drive_lock;
        Timer::after(PEDALS_TASK_PERIOD).await;
    }
}

const SAMPLE_TIME: SampleTime = SampleTime::Cycles475;
const PEDAL_ADC_SETTLE: Duration = Duration::from_millis(100);

#[derive(VariantCount)]
pub enum Adc2Channels {
    Adc2Channel12, // APPS_1_ADC
    Adc2Channel10, // APPS_2_ADC
    Adc2Channel2,  // BSE_1_ADC
    Adc2Channel6,  // BSE_2_ADC
}

/// Peripherals ADC2.
///
/// Channel pins follow the CubeMX rank order; see `Core/Inc/main.h:90-117` and
/// the raw GPIOF writes in `Core/Src/stm32h5xx_hal_msp.c:155-158`.
pub struct AdcPedalsResources {
    pub adc: Peri<'static, peripherals::ADC2>,
    pub dma: Peri<'static, peripherals::GPDMA1_CH4>,
    pub apps1: Peri<'static, peripherals::PC2>,
    pub apps2: Peri<'static, peripherals::PC0>,
    pub bse1: Peri<'static, peripherals::PF13>,
    pub bse2: Peri<'static, peripherals::PF14>,
}

#[embassy_executor::task]
pub async fn adc_pedals_task(r: AdcPedalsResources) {
    let mut config = Config::default();
    config.resolution = Some(Resolution::Bits12);
    config.clock = Clock::Async(Prescaler::Div64);

    let sequence = [
        (r.apps1.degrade_adc(), SAMPLE_TIME),
        (r.apps2.degrade_adc(), SAMPLE_TIME),
        (r.bse1.degrade_adc(), SAMPLE_TIME),
        (r.bse2.degrade_adc(), SAMPLE_TIME),
    ];

    let mut adc2 = Adc::new(r.adc, Irqs, config);

    let mut sequence = adc2.configure_sequence(r.dma, sequence.into_iter(), Irqs);

    let mut buffer = [0u16; Adc2Channels::VARIANT_COUNT];

    let _ = PEDAL_ADC_MUTEX.init(Mutex::new(PedalBuffer::new()));

    loop {
        sequence.read(&mut buffer).await;

        let mut mux = PEDAL_ADC_MUTEX.get().await.lock().await;

        mux.update(&buffer);

        Timer::after(PEDAL_ADC_SETTLE).await;
    }
}

static PEDAL_ADC_MUTEX: OnceLock<Mutex<ThreadModeRawMutex, PedalBuffer>> = OnceLock::new();

/// Maps the raw adc value to equivalent 3.3 volt voltage.
fn voltage_from_raw(raw_adc: u16) -> ElectricPotential {
    ElectricPotential::new::<volt>((raw_adc as f32) * 3.3 / 4096.0 * 5000.0 / 3000.0)
}

fn fraction_from_voltage(
    value: ElectricPotential,
    min: ElectricPotential,
    max: ElectricPotential,
) -> Ratio {
    if value > max {
        return Ratio::new::<ratio>(1.0);
    }
    if value < min {
        return Ratio::new::<ratio>(0.0);
    }
    (value - min) / (max - min)
}

fn pressure_from_voltage(value: ElectricPotential) -> Pressure {
    Pressure::new::<psi>(1250.0f32 * value.get::<volt>() - 625f32)
}

/// Takes a snapshot of the latest readings.
async fn get_raw_pedal_data() -> RawPedalData {
    PEDAL_ADC_MUTEX.get().await.lock().await.get_pedal_data()
}

#[derive(Clone, Copy)]
struct PedalBuffer {
    adc_buffer: [u16; Adc2Channels::VARIANT_COUNT],
}

struct RawPedalData {
    accel1: u16,
    accel2: u16,
    brake1: u16,
    brake2: u16,
}


struct PedalData {
    accel1_v: ElectricPotential,
    accel2_v: ElectricPotential,
    brake1_v: ElectricPotential,
    brake2_v: ElectricPotential,
    accel1_r: Ratio,
    accel2_r: Ratio,
    accel_r: Ratio,
    brake_r: Ratio,
    brake1_p: Pressure,
    brake2_p: Pressure,
}

impl PedalData {
    fn to_volt_frame(&self) -> cangen::PedalSensorVoltages {
        cangen::PedalSensorVoltages::new()
            .with_accel1_volts(self.accel1_v.get::<volt>())
            .with_accel2_volts(self.accel2_v.get::<volt>())
            .with_brake1_volts(self.brake1_v.get::<volt>())
            .with_brake2_volts(self.brake2_v.get::<volt>())
    }

    fn to_per_frame(&self) -> cangen::PedalPercentPressedValues {
        cangen::PedalPercentPressedValues::new()
            .with_accel_norm(self.accel_r.get::<ratio>())
            .with_brake_norm(self.brake_r.get::<ratio>())
            .with_brake_psi_brake1(self.brake1_p.get::<psi>())
            .with_brake_psi_brake2(self.brake2_p.get::<psi>())
    }
}


impl From<RawPedalData> for PedalData {
    fn from(raw: RawPedalData) -> Self {
        let accel1_v = voltage_from_raw(raw.accel1);
        let accel2_v = voltage_from_raw(raw.accel2);
        let brake1_v = voltage_from_raw(raw.brake1);
        let brake2_v = voltage_from_raw(raw.brake2);

        let accel1_r = fraction_from_voltage(accel1_v, MIN_APPS1_VOLTS, MAX_APPS1_VOLTS);
        let accel2_r = fraction_from_voltage(accel2_v, MIN_APPS2_VOLTS, MAX_APPS2_VOLTS);

        let zero = ElectricPotential::new::<volt>(0.0);
        let brake1_r = fraction_from_voltage(brake1_v, zero, MAX_VOLTS_UNSCALED);
        let brake2_r = fraction_from_voltage(brake2_v, zero, MAX_VOLTS_UNSCALED);

        Self {
            accel1_v,
            accel2_v,
            brake1_v,
            brake2_v,
            accel1_r,
            accel2_r,
            accel_r: (accel1_r + accel2_r) / 2.0f32,
            brake_r: (brake1_r + brake2_r) / 2.0f32,
            brake1_p: pressure_from_voltage(brake1_v),
            brake2_p: pressure_from_voltage(brake2_v),
        }
    }
}

impl PedalBuffer {
    pub fn new() -> Self {
        Self {
            adc_buffer: [0; Adc2Channels::VARIANT_COUNT],
        }
    }

    pub fn update(&mut self, buffer: &[u16; Adc2Channels::VARIANT_COUNT]) {
        self.adc_buffer = *buffer;
    }

    pub fn get_pedal_data(self) -> RawPedalData {
        RawPedalData {
            accel1: self.adc_buffer[Adc2Channels::Adc2Channel12 as usize],
            accel2: self.adc_buffer[Adc2Channels::Adc2Channel10 as usize],
            brake1: self.adc_buffer[Adc2Channels::Adc2Channel2 as usize],
            brake2: self.adc_buffer[Adc2Channels::Adc2Channel6 as usize],
        }
    }
}
