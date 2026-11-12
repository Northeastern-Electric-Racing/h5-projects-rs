//! CAN stuff.

use can_handler::{ExtFilter, NerCan, StdFilter};
use cangen::CanMessage;
use embassy_stm32::can::filter::{ExtendedFilterSlot, StandardFilterSlot};
use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Frame};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

/// Capacity of each CAN channel, in `Frame`s.
pub const CHANNEL_SIZE: usize = 256;

/// Frames received off the bus, waiting to be parsed by `can_inbox_task`.
pub static INCOMING: Channel<ThreadModeRawMutex, Frame, CHANNEL_SIZE> = Channel::new();
/// Frames queued by the program, waiting to go out on the bus.
pub static OUTGOING: Channel<ThreadModeRawMutex, Frame, CHANNEL_SIZE> = Channel::new();

/// Add a frame to the outgoing CAN channel.
pub async fn send(frame: Frame) {
    can_handler::send(OUTGOING.dyn_sender(), frame).await
}

/// Tries to add a frame to the outgoing CAN channel.
pub fn try_send(frame: Frame) -> Result<(), ()> {
    can_handler::try_send(OUTGOING.dyn_sender(), frame)
}

/// Initializes the FDCAN peripheral and returns the split CanTx and CanRx structs.
pub fn init(r: crate::CanResources) -> (CanTx<'static>, CanRx<'static>) {
    let configurator = CanConfigurator::new(r.can, r.can_rx, r.can_tx, interrupts::Irqs);

    const SLOT_0: StdFilter = StdFilter {
        slot: StandardFilterSlot::_0,
        id1: cangen::ErpmDutyInputVoltageStatus::ID,
        id2: Some(cangen::CurrentsStatus::ID),
    };
    const SLOT_1: StdFilter = StdFilter {
        slot: StandardFilterSlot::_1,
        id1: cangen::ShepherdBmsPwmPercent::ID,
        id2: None,
    };
    const SLOT_0E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_0,
        id1: cangen::ChargerBoxStatus::ID,
        id2: None,
    };

    let (tx, rx, _props) = NerCan::init(configurator).with_standard_filters(&[SLOT_0, SLOT_1]).with_extended_filters(&[SLOT_0E]).start();
    interrupts::enable_extra();

    (tx, rx)
}

/// Reads incoming CAN frames from the software queue and dispatches them.
#[embassy_executor::task]
pub async fn can_inbox_task() -> ! {
    use crate::units::{Current, Voltage};
    use crate::{charging, pack};
    use cangen::Msg;
    use uom::si::electric_current::ampere;
    use uom::si::electric_potential::volt;

    loop {
        let frame = INCOMING.receive().await;

        match Msg::decode(&frame) {
            Some(Msg::ChargerBoxStatus(m)) => {
                charging::alert_charger_received();
                pack::analyzer::store_can_pack_current(Current::new::<ampere>(m.current()));
                pack::analyzer::store_can_ts_voltage(Voltage::new::<volt>(m.voltage()));
            },
            Some(Msg::CurrentsStatus(_)) if !charging::is_charger_connected() => {
                if let Some(current) = decode_dti_current(frame.data()) {
                    pack::analyzer::store_can_pack_current(current);
                }
            },
            Some(Msg::ErpmDutyInputVoltageStatus(_)) if !charging::is_charger_connected() => {
                if let Some(voltage) = decode_dti_voltage(frame.data()) {
                    pack::analyzer::store_can_ts_voltage(voltage);
                }
            },
            Some(Msg::ShepherdBmsPwmPercent(pwm)) => charging::update_pwm_duty_cycle(&pwm),
            // DTI frames while the charger is connected.
            Some(_) => {},
            None => defmt::debug!("Failed to decode CAN frame"),
        }
    }
}

// u_TODO: the DTI points in Odyssey-Definitions `dti.json` have no names, so cangen generates no getters for them.
// Name them upstream, then replace these with `m.dc_current()` / `m.input_voltage()`-style getters.

/// DC current from a DTI `Currents_Status` frame: big-endian `i16` at bytes 2..4, 0.1 A per LSB.
fn decode_dti_current(data: &[u8]) -> Option<crate::units::Current> {
    use uom::si::electric_current::ampere;

    let raw = i16::from_be_bytes([*data.get(2)?, *data.get(3)?]);
    Some(crate::units::Current::new::<ampere>(f32::from(raw) / 10.0))
}

/// TS voltage from a DTI `ERPM_Duty_Input_Voltage_Status` frame: big-endian `i16` at bytes 6..8, 1 V per LSB.
fn decode_dti_voltage(data: &[u8]) -> Option<crate::units::Voltage> {
    use uom::si::electric_potential::volt;

    let raw = i16::from_be_bytes([*data.get(6)?, *data.get(7)?]);
    Some(crate::units::Voltage::new::<volt>(f32::from(raw)))
}

/// CAN message types. This isn't really needed at all, the builder pattern is just somewhat messy for large CAN structs like this.
/// u_TODO - eventually try adding stuff to `cangen` that generates these structs automatically so you don't need to use builder pattern
pub mod types {
    use super::Frame;
    use cangen::ToCanFrame;

    ///use cangen::{AlphaCellDataDebug, BetaCellDataDebug};

    pub struct AlphaCellDataDebug {
        pub therm: f32,
        pub voltage_a: f32,
        pub voltage_b: f32,
        pub chip_id: u8,
        pub cell_a: u8,
        pub cell_b: u8,
        pub discharging_a: bool,
        pub discharging_b: bool,
        pub cvs_a: bool,
        pub cvs_b: bool,
        pub ow_a: bool,
        pub ow_b: bool,
    }
    impl AlphaCellDataDebug {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::AlphaCellDataDebug::new().with_therm(self.therm).with_voltage_a(self.voltage_a).with_voltage_b(self.voltage_b).with_chip_id(self.chip_id).with_cell_a(self.cell_a).with_cell_b(self.cell_b).with_discharging_a(self.discharging_a).with_discharging_b(self.discharging_b).with_cvs_a(self.cvs_a).with_cvs_b(self.cvs_b);

            frame.to_can_frame()
        }
    }

    pub struct BetaCellDataDebug {
        pub therm: f32,
        pub voltage_a: f32,
        pub voltage_b: f32,
        pub chip_id: u8,
        pub cell_a: u8,
        pub cell_b: u8,
        pub discharging_a: bool,
        pub discharging_b: bool,
        pub cvs_a: bool,
        pub cvs_b: bool,
        pub ow_a: bool,
        pub ow_b: bool,
    }
    impl BetaCellDataDebug {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::BetaCellDataDebug::new().with_therm(self.therm).with_voltage_a(self.voltage_a).with_voltage_b(self.voltage_b).with_chip_id(self.chip_id).with_cell_a(self.cell_a).with_cell_b(self.cell_b).with_discharging_a(self.discharging_a).with_discharging_b(self.discharging_b).with_cvs_a(self.cvs_a).with_cvs_b(self.cvs_b);

            frame.to_can_frame()
        }
    }

    pub struct CellVoltage {
        pub high_val: f32,
        pub high_chip: u8,
        pub high_cell: u8,
        pub low_val: f32,
        pub low_chip: u8,
        pub low_cell: u8,
        pub avg_val: f32,
    }
    impl CellVoltage {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::CellVoltage::new().with_high_val(self.high_val).with_high_chip(self.high_chip).with_high_cell(self.high_cell).with_low_val(self.low_val).with_low_chip(self.low_chip).with_low_cell(self.low_cell).with_avg_val(self.avg_val);

            frame.to_can_frame()
        }
    }

    pub struct SegmentAverageVoltages {
        pub seg1: f32,
        pub seg2: f32,
        pub seg3: f32,
        pub seg4: f32,
        pub seg5: f32,
    }
    impl SegmentAverageVoltages {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::SegmentAverageVoltages::new().with_seg1(self.seg1).with_seg2(self.seg2).with_seg3(self.seg3).with_seg4(self.seg4).with_seg5(self.seg5);

            frame.to_can_frame()
        }
    }

    pub struct SegmentTotalVoltages {
        pub seg1: f32,
        pub seg2: f32,
        pub seg3: f32,
        pub seg4: f32,
        pub seg5: f32,
    }
    impl SegmentTotalVoltages {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::SegmentTotalVoltages::new().with_seg1(self.seg1).with_seg2(self.seg2).with_seg3(self.seg3).with_seg4(self.seg4).with_seg5(self.seg5);

            frame.to_can_frame()
        }
    }

    pub struct CellTemperatures {
        pub high_val: f32,
        pub high_chip: u8,
        pub high_cell: u8,
        pub low_val: f32,
        pub low_chip: u8,
        pub low_cell: u8,
        pub avg_val: f32,
    }
    impl CellTemperatures {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::CellTemperatures::new().with_high_val(self.high_val).with_high_chip(self.high_chip).with_high_cell(self.high_cell).with_low_val(self.low_val).with_low_chip(self.low_chip).with_low_cell(self.low_cell).with_avg_val(self.avg_val);

            frame.to_can_frame()
        }
    }

    pub struct SegmentTemperatures {
        pub seg1: f32,
        pub seg2: f32,
        pub seg3: f32,
        pub seg4: f32,
        pub seg5: f32,
    }
    impl SegmentTemperatures {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::SegmentTemperatures::new().with_seg1(self.seg1).with_seg2(self.seg2).with_seg3(self.seg3).with_seg4(self.seg4).with_seg5(self.seg5);

            frame.to_can_frame()
        }
    }

    pub struct PackSocStatus {
        pub pack_soc: f32,
        pub pack_soc_drift: f32,
    }
    impl PackSocStatus {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::PackSocStatus::new().with_Pack_SoC(self.pack_soc).with_Pack_SoC_Drift(self.pack_soc_drift);

            frame.to_can_frame()
        }
    }

    pub struct BmsChargeMessageSend {
        pub charge_volts: f32,
        pub charge_current: f32,

        /// 0xFF = stop charging, 0 = yes charge
        pub charging_state: u8,
    }
    impl BmsChargeMessageSend {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::BmsChargeMessageSend::new().with_charge_volts(self.charge_volts).with_charge_current(self.charge_current).with_enable_charging(self.charging_state);
            frame.to_can_frame()
        }
    }

    pub struct ShutdownAsReadByBms {
        pub shutdown_state: bool,
        pub shutdown_ts_minus_sense: bool,
        pub shutdown_ts_plus_sense: bool,
        pub shutdown_acc_sense: bool,
        pub shutdown_tsip_sense: bool,
    }
    impl ShutdownAsReadByBms {
        pub fn as_frame(&self) -> Frame {
            let frame = cangen::ShutdownAsReadByBms::new().with_shutdown_state(self.shutdown_state).with_shutdown_ts_minus_sense(self.shutdown_ts_minus_sense).with_shutdown_ts_plus_sense(self.shutdown_ts_plus_sense).with_shutdown_acc_sense(self.shutdown_acc_sense).with_shutdown_tsip_sense(self.shutdown_tsip_sense);
            frame.to_can_frame()
        }
    }
}

/// Interrupt config and diagnostics.
/// u_TODO: can-handler's `start()` doesn't enable these interrupts, so shepherd keeps them locally. Decide with the team whether to upstream or drop them.
mod interrupts {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Number of times the FDCAN2 IT0 interrupt has fired.
    pub static IT0_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Number of times the FDCAN2 IT1 interrupt has fired.
    pub static IT1_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

    /// Enables the extra FDCAN interrupts that `It0Counter` clears. Call after `NerCan::start()`.
    pub(super) fn enable_extra() {
        let regs = embassy_stm32::pac::FDCAN2;

        // Enable the transmission-cancellation interrupt.
        regs.txbcie().write(|w| w.0 = 0xffff_ffff);
        regs.ie().modify(|w| w.set_tcfe(true));

        // Enable timestamp wraparound interrupt.
        regs.ie().modify(|w| {
            w.set_tswe(true);
            w.set_peae(true);
            w.set_pede(true);
        });
    }

    /// Counts FDCAN2 IT0 entries. Also clears the TCF flag. This runs alongside embassy's internal ISR (it doesn't replace it or anything)
    struct It0Counter;
    impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::FDCAN2_IT0> for It0Counter {
        unsafe fn on_interrupt() {
            // Clear IR.TCF (transmission cancellation finished) interrupt flag.
            // We need to do this because we enable this interrupt manually and embassy doesn't clear this in its internal interrupt handler.
            let regs = embassy_stm32::pac::FDCAN2;
            // if we don't clear all interrupts we enable it is over
            let ir = regs.ir().read();
            if ir.tcf() {
                regs.ir().write(|w| w.set_tcf(true));
            }
            if ir.tsw() {
                regs.ir().write(|w| w.set_tsw(true));
            }
            if ir.pea() {
                regs.ir().write(|w| w.set_pea(true));
            }
            if ir.ped() {
                regs.ir().write(|w| w.set_ped(true));
            }

            IT0_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Counts FDCAN2 IT1 entries. This runs alongside embassy's internal ISR (it doesn't replace it or anything)
    struct It1Counter;
    impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::FDCAN2_IT1> for It1Counter {
        unsafe fn on_interrupt() {
            IT1_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
        }
    }

    embassy_stm32::bind_interrupts!(pub struct Irqs {
        // NOTE: `It0Counter` is listed first on purpose so it runs before the embassy ISR handler.
        FDCAN2_IT0 => It0Counter, embassy_stm32::can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN2>;
        FDCAN2_IT1 => embassy_stm32::can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN2>, It1Counter;
    });

    /// Publishes the IRQ counters. can-handler's `can_props` covers every other `CanDebug/` register reading.
    #[cfg(defmt_monitor)]
    #[embassy_executor::task]
    pub async fn can_irq_monitor() -> ! {
        loop {
            defmt_monitor::monitor!("CanDebug/it0_irq_count", desc = "Times the FDCAN2 IT0 ISR has fired.", "{=u32}", IT0_IRQ_COUNT.load(Ordering::Relaxed));
            defmt_monitor::monitor!("CanDebug/it1_irq_count", desc = "Times the FDCAN2 IT1 ISR has fired.", "{=u32}", IT1_IRQ_COUNT.load(Ordering::Relaxed));

            embassy_time::Timer::after_millis(500).await;
        }
    }
}
#[cfg(defmt_monitor)]
pub use interrupts::can_irq_monitor;
