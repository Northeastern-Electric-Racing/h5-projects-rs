//! CAN stuff.

use embassy_stm32::can::Frame;

/// MAIN PUBLIC API THE REST OF THE PROGRAM USES TO INTERACT WITH CAN.
mod api {
    use embassy_stm32::can::Frame;
    use super::{channels, handler, interrupts};

    /// Add a frame to the outgoing CAN channel.
    pub async fn send(frame: Frame) {
        match channels::OUTGOING.try_send(frame) {
            Ok(_) => {},
            Err(_) => {
                defmt::warn!("Tried to add a frame to the OUTGOING Channel, but the Channel was full. This is not a failure, because we will .await until the Channel is able to accept the frame. However, consider increasing the capacity of the Channel if this is occurring often.");
                channels::OUTGOING.send(frame).await
            },
        }
    }

    /// Tries to add a frame to the outgoing CAN channel.
    pub fn try_send(frame: Frame) -> Result<(), ()> {
        match channels::OUTGOING.try_send(frame) {
            Ok(_) => Ok(()),
            Err(_) => {
                defmt::warn!("Tried to add a frame to the OUTGOING Channel, but the Channel was full. This is not a failure, because we will .await until the Channel is able to accept the frame. However, consider increasing the capacity of the Channel if this is occurring often.");
                Err(())
            },
        }
    }

    /// Get a frame from the incoming CAN channel.
    /// (this doesn't need to check for an error because of `receive()` is empty there is no problem, it just means there is no pending messages)
    /// u_TODO - this probably shouldn't be public at all, since all recieving should be done inside the can handler itself. but for now we will keep this here since the state of this file is probably temporary (assuming much of this stuff will be moved into the can-handler in the `firmware-rs` repo)
    pub async fn recieve() -> Frame {
        channels::INCOMING.receive().await
    }

    /// Initializes CAN and starts up the CAN handler.
    #[embassy_executor::task]
    pub async fn can_task(spawner: embassy_executor::Spawner, r: crate::CanResources) {
        use embassy_stm32::can::{CanConfigurator};
        use embassy_stm32::can::filter::{StandardFilterSlot};

        /// CAN ID for DTI status message. We use this to make sure we are always triggering our CAN interrupt (which this will do because
        /// this message always gets recieved). We need to do this because it protects against our TX waker from going to sleep forever,
        /// which happens right now for some reason. Ideally this will be a temporary fix.
        const DTI_ERPM_STATUS_MESSAGE: u16 = 0x416;

        let configurator = CanConfigurator::new(r.can, r.can_rx, r.can_tx, interrupts::Irqs);
        let can = handler::NerCan::init(configurator).add_standard_filter(StandardFilterSlot::_0, DTI_ERPM_STATUS_MESSAGE, None);

        let (tx, rx, props) = can.start();

        spawner.spawn(handler::can_tx(tx).expect("Failed to spawn can_handler::can_tx()."));
        spawner.spawn(handler::can_rx(rx).expect("Failed to spawn can_handler::can_rx()."));
        spawner.spawn(handler::can_rx_processer().expect("Failed to spawn can_handler::can_rx_processer()."));

        #[cfg(defmt_monitor)]
        spawner.spawn(handler::can_props(props).expect("Failed to spawn can_handler::can_props()."));
    }
}
pub use api::*;

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
}

/// Interrupt config and diagnostics.
mod interrupts {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Number of times the FDCAN2 IT0 interrupt has fired.
    pub static IT0_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Number of times the FDCAN2 IT1 interrupt has fired.
    pub static IT1_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

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
}

mod channels {
    use super::Frame;
    use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
    use embassy_sync::channel::Channel;

    /// Capacity of our incoming CAN channel, in `Frame`s.
    pub const INCOMING_CHANNEL_SIZE: usize = 256;
    /// Capacity of our outgoing CAN channel, in `Frame`s.
    pub const OUTGOING_CHANNEL_SIZE: usize = 256;

    /// Channel for frames that we recieve.
    pub(super) static INCOMING: Channel<ThreadModeRawMutex, Frame, INCOMING_CHANNEL_SIZE> = Channel::new();
    /// Channel for frames we queue to send.
    pub(super) static OUTGOING: Channel<ThreadModeRawMutex, Frame, OUTGOING_CHANNEL_SIZE> = Channel::new();
}

mod handler {
    use defmt::{warn};
    use embassy_stm32::can::filter::FilterType::{DedicatedDual, DedicatedSingle};
    use embassy_stm32::can::filter::{Action, ExtendedFilter, ExtendedFilterSlot, StandardFilter, StandardFilterSlot};
    use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Properties};
    use embassy_time::Timer;
    use embedded_can::{ExtendedId, StandardId};

    use heapless::Vec;

    pub struct NerCan {
        pub can_configurator: CanConfigurator<'static>,
        used_std_slots: Vec<StandardFilterSlot, 28>,
        used_ext_slots: Vec<ExtendedFilterSlot, 28>,
    }

    impl NerCan {
        pub fn init(mut can_configurator: CanConfigurator<'static>) -> Self {
            use embassy_stm32::can::config::*;

            let can_config = FdCanConfig::default().set_automatic_bus_off_recovery(true).set_automatic_retransmit(false).set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly).set_transmit_pause(true).set_global_filter(GlobalFilter::reject_all());
            can_configurator.set_config(can_config);
            can_configurator.set_bitrate(500_000);

            Self {
                can_configurator,
                used_std_slots: Vec::new(),
                used_ext_slots: Vec::new(),
            }
        }

        /// Starts up CAN in normal mode and returns the split objects.
        pub fn start(self) -> (CanTx<'static>, CanRx<'static>, Properties) {
            let split = self.can_configurator.into_normal_mode().split();

            // Enable the transmission-cancellation interrupt.
            let regs = embassy_stm32::pac::FDCAN2;
            regs.txbcie().write(|w| w.0 = 0xffff_ffff);
            regs.ie().modify(|w| w.set_tcfe(true));

            // Enable timestamp wraparound interrupt.
            regs.ie().modify(|w| {
                w.set_tswe(true);
                w.set_peae(true);
                w.set_pede(true);
            });

            split
        }

        /// Sets adds a new CAN Standard Filter at the given slot
        /// NOTE: will panic if the given slot is already in use
        #[allow(dead_code)]
        pub fn add_standard_filter(mut self, std_filter_slot: StandardFilterSlot, std_id1: u16, std_id2: Option<u16>) -> Self {
            if self.used_std_slots.contains(&std_filter_slot) {
                panic!("The selected CAN Standard Filter Slot is already in use.");
            }

            let mut std = StandardFilter::default();
            match std_id2 {
                Some(id2) => {
                    std.filter = DedicatedDual(StandardId::new(std_id1).unwrap(), StandardId::new(id2).unwrap());
                },
                None => {
                    std.filter = DedicatedSingle(StandardId::new(std_id1).unwrap());
                },
            }
            std.action = Action::StoreInFifo0;
            self.can_configurator.properties().set_standard_filter(std_filter_slot, std);
            let _ = self.used_std_slots.push(std_filter_slot);

            self
        }

        /// Sets adds a new CAN Extended Filter at the given slot
        /// NOTE: will panic if the given slot is already in use
        #[allow(dead_code)]
        pub fn add_extended_filter(mut self, ext_filter_slot: ExtendedFilterSlot, ext_id1: u32, ext_id2: Option<u32>) -> Self {
            if self.used_ext_slots.contains(&ext_filter_slot) {
                panic!("The selected CAN Extended Filter Slot is already in use.");
            }

            let mut ext = ExtendedFilter::default();
            match ext_id2 {
                Some(id2) => {
                    ext.filter = DedicatedDual(ExtendedId::new(ext_id1).unwrap(), ExtendedId::new(id2).unwrap());
                },
                None => {
                    ext.filter = DedicatedSingle(ExtendedId::new(ext_id1).unwrap());
                },
            }
            ext.action = Action::StoreInFifo0;
            self.can_configurator.properties().set_extended_filter(ext_filter_slot, ext);
            let _ = self.used_ext_slots.push(ext_filter_slot);

            self
        }
    }

    /// Drains the outgoing channel onto the bus.
    #[embassy_executor::task]
    pub async fn can_tx(mut tx: CanTx<'static>) -> ! {
        use embassy_time::Timer;
        use embassy_futures::select::{select, Either};

        let mut send_count: u32 = 0;
        let mut dropped_due_to_outgoing_full_count: u32 = 0;
        let mut dropped_due_to_stalled_tx_count: u32 = 0;

        loop {
            let frame = super::channels::OUTGOING.receive().await;

            match select(tx.write(&frame), Timer::after_millis(50)).await {
                // Case: frame was dropped
                Either::First(Some(dropped)) => {
                    // If we dropped a frame, try to send it back to OUTGOING so it can get sent again.
                    // We can't do a normal `send().await` since this task is the one that drains OUTGOING, so doing
                    // that could probably cause a deadlock somehow.
                    match super::channels::OUTGOING.try_send(dropped) {
                        Ok(_) => (),
                        Err(_) => {
                            dropped_due_to_outgoing_full_count += 1;
                            warn!("Had to drop an outgoing CAN frame because OUTGOING was full! Not good.");
                        },
                    }
                },

                // Case: frame was sent successfully
                Either::First(None) => {
                    send_count += 1;
                },

                // Case: The Timer::after await returned before tx.write(), so CAN TX has stalled and we drop the frame.
                // the "stall" shouldn't be a permanant thing, we just need to make sure this task can't sleep forever.
                Either::Second(_) => {
                    dropped_due_to_stalled_tx_count += 1;
                    warn!("Had to drop an outgoing CAN frame because CAN TX stalled! Probably not good.");
                },
            }

            defmt_monitor::monitor!("CanDebug/send_count", desc = "Send count", "{}", send_count);
            defmt_monitor::monitor!("CanDebug/dropped_due_to_outgoing_full_count", desc = "Frames dropped due to the OUTGOING channel being full.", "{}", dropped_due_to_outgoing_full_count);
            defmt_monitor::monitor!("CanDebug/dropped_due_to_stalled_tx_count", desc = "Frames dropped due to TX stalling.", "{}", dropped_due_to_stalled_tx_count);
        }
    }

    /// Passes frames received off the bus to the incoming channel.
    #[embassy_executor::task]
    pub async fn can_rx(mut rx: CanRx<'static>) -> ! {
        let mut rx_count: u32 = 0;
        let mut rx_err_count: u32 = 0;

        loop {
            match rx.read().await {
                Ok(can_recv) => {
                    super::channels::INCOMING.send(can_recv.frame).await;
                    rx_count += 1;
                },
                Err(err) => {
                    warn!("Bus error! {}", err);
                    rx_err_count += 1;
                },
            }

            defmt_monitor::monitor!("CanDebug/rx_count", desc = "RX count", "{}", rx_count);
            defmt_monitor::monitor!("CanDebug/rx_err_count", desc = "RX err count", "{}", rx_err_count);
        }
    }

    /// Reads incoming CAN frames from the software queue and dispatches them.
    #[embassy_executor::task]
    pub async fn can_rx_processer() -> ! {
        use embedded_can::Id;
        use crate::{hv_plate, state_machine};

        const CHARGER_BOX: u32 = 0x18FF_50E5;
        const DTI_DC_CURRENT: u16 = 0x436;
        const DTI_INPUT_VOLTAGE: u16 = 0x416;

        loop {
            let frame = super::channels::INCOMING.receive().await;

            match frame.id() {
                Id::Extended(id) if id.as_raw() == CHARGER_BOX => {
                    state_machine::charger_frame_received();
                    if let Some(current) = decode_current(frame.data()) {
                        hv_plate::store_pack_current(current);
                    }
                    if let Some(voltage) = decode_charger_voltage(frame.data()) {
                        hv_plate::store_ts_voltage(voltage);
                    }
                },
                Id::Standard(id) if id.as_raw() == DTI_DC_CURRENT && !crate::state_machine::charger_connected() => {
                    if let Some(current) = decode_current(frame.data()) {
                        hv_plate::store_pack_current(current);
                    }
                },
                Id::Standard(id) if id.as_raw() == DTI_INPUT_VOLTAGE && !crate::state_machine::charger_connected() => {
                    if let Some(voltage) = decode_dti_voltage(frame.data()) {
                        hv_plate::store_ts_voltage(voltage);
                    }
                },
                _ => {},
            }
        }
    }

    // u_TODO: change to use FromCanFrame later

    /// Decodes pack current from a DTI or charger frame.
    fn decode_current(data: &[u8]) -> Option<crate::units::Current> {
        use uom::si::electric_current::ampere;

        let raw = i16::from_be_bytes([*data.get(2)?, *data.get(3)?]);
        Some(crate::units::Current::new::<ampere>(f32::from(raw) / 10.0))
    }

    /// TS voltage from a DTI input-voltage frame
    fn decode_dti_voltage(data: &[u8]) -> Option<crate::units::Voltage> {
        use uom::si::electric_potential::volt;

        let raw = i16::from_be_bytes([*data.get(6)?, *data.get(7)?]);
        Some(crate::units::Voltage::new::<volt>(f32::from(raw)))
    }

    /// TS voltage from a charger frame: big-endian `i16` at bytes 0..2, 0.1 V per LSB.
    fn decode_charger_voltage(data: &[u8]) -> Option<crate::units::Voltage> {
        use uom::si::electric_potential::volt;

        let raw = i16::from_be_bytes([*data.get(0)?, *data.get(1)?]);
        Some(crate::units::Voltage::new::<volt>(f32::from(raw) / 10.0))
    }

    /// Publishes CAN health diagnostics exposed by embassy-stm32.
    #[embassy_executor::task]
    pub async fn can_props(_props: Properties) -> ! {
        use core::sync::atomic::Ordering;
        use embassy_stm32::can::enums::BusErrorMode;
        use embassy_stm32::pac;

        /// Number of hardware TX mailboxes on STM32H563.
        const TX_MAILBOX_COUNT: usize = 3;
        /// How often this task should run, in ms.
        const PROPS_SAMPLE_PERIOD_MS: u64 = 500;

        let regs = pac::FDCAN2;

        loop {
            // Readings from CAN registers.
            let psr = regs.psr().read();
            let ecr = regs.ecr().read();
            let cccr = regs.cccr().read();
            let ir = regs.ir().read();
            let ie = regs.ie().read();
            let ils = regs.ils().read();
            let ile = regs.ile().read();
            let txfqs = regs.txfqs().read();
            let txbrp = regs.txbrp().read();
            let txbto = regs.txbto().read();
            let txbcf = regs.txbcf().read();

            // Find error mode. This is what embassy does internally (at least as of writing this).
            let bus_error_mode = match (psr.bo(), psr.ep()) {
                (false, false) => BusErrorMode::ErrorActive,
                (false, true) => BusErrorMode::ErrorPassive,
                (true, _) => BusErrorMode::BusOff,
            };

            // One bit per hardware TX mailbox.
            let mut pending_mask = 0_u8;
            let mut occurred_mask = 0_u8;
            let mut cancelled_mask = 0_u8;
            let mut pending_count = 0_u8;
            for i in 0..TX_MAILBOX_COUNT {
                if txbrp.trp(i) {
                    pending_mask |= 1_u8 << i;
                    pending_count += 1_u8;
                }
                if txbto.to(i) {
                    occurred_mask |= 1_u8 << i;
                }
                if txbcf.cf(i) {
                    cancelled_mask |= 1_u8 << i;
                }
            }

            // Error counters and protocol status.
            defmt_monitor::monitor!("CanDebug/tx_error_count", desc = "FDCAN TEC (ECR.TEC). Climbs by 8 per failed transmission. >255 means bus-off.", "{=u8}", ecr.tec());
            defmt_monitor::monitor!("CanDebug/rx_error_count", desc = "FDCAN REC (ECR.REC).", "{=u8}", ecr.rec());
            defmt_monitor::monitor!("CanDebug/bus_error_mode", desc = "FDCAN bus error state, from PSR.BO/PSR.EP.", "{}", bus_error_mode);
            defmt_monitor::monitor!("CanDebug/error_warning", desc = "PSR.EW. An error counter has passed 96.", "{=bool}", psr.ew());
            defmt_monitor::monitor!("CanDebug/node_activity", desc = "PSR.ACT. SYNC=still synchronizing to the bus, IDLE=neither sending nor receiving, RX/TX=actively on the bus.", "{}", psr.act());

            // TX mailbox occupancy stuff.
            defmt_monitor::monitor!("CanDebug/tx_pending_mask", desc = "TXBRP, one bit per mailbox. Set bit means a transmission is requested and not yet finished.", "{=u8}", pending_mask);
            defmt_monitor::monitor!("CanDebug/tx_pending_count", desc = "Number of TX mailboxes with a pending request, 0 to 3.", "{=u8}", pending_count);
            defmt_monitor::monitor!("CanDebug/tx_occurred_mask", desc = "TXBTO, one bit per mailbox. Set bit means a frame was successfully transmitted. Stays 0 if nothing has ever reached the bus.", "{=u8}", occurred_mask);
            defmt_monitor::monitor!("CanDebug/tx_cancelled_mask", desc = "TXBCF, one bit per mailbox. Set bit means a transmission was cancelled.", "{=u8}", cancelled_mask);
            defmt_monitor::monitor!("CanDebug/tx_fifo_full", desc = "TXFQS.TFQF. No free mailbox.", "{=bool}", txfqs.tfqf());
            defmt_monitor::monitor!("CanDebug/tx_fifo_free_level", desc = "TXFQS.TFFL. Number of consecutive free mailboxes. Reads 0 in queue mode (TXBC.TFQM=1).", "{=u8}", txfqs.tffl());
            defmt_monitor::monitor!("CanDebug/tx_put_index", desc = "TXFQS.TFQPI. The mailbox the next write goes into.", "{=u8}", txfqs.tfqpi());

            // Interrupt stuff.
            defmt_monitor::monitor!("CanDebug/it0_irq_count", desc = "Times the FDCAN2 IT0 ISR has fired.", "{=u32}", crate::can::interrupts::IT0_IRQ_COUNT.load(Ordering::Relaxed));
            defmt_monitor::monitor!("CanDebug/it1_irq_count", desc = "Times the FDCAN2 IT1 ISR has fired.", "{=u32}", crate::can::interrupts::IT1_IRQ_COUNT.load(Ordering::Relaxed));
            defmt_monitor::monitor!("CanDebug/ir_tc_latched", desc = "IR.TC still set at sample time. Embassy's ISR clears this on entry, so persistently true means the ISR is not running.", "{=bool}", ir.tc());
            defmt_monitor::monitor!("CanDebug/ir", desc = "Raw FDCAN IR, all latched interrupt flags.", "{=u32}", ir.0);
            defmt_monitor::monitor!("CanDebug/ie", desc = "Raw FDCAN IE, enabled interrupt sources.", "{=u32}", ie.0);
            defmt_monitor::monitor!("CanDebug/ils", desc = "Raw FDCAN ILS, interrupt line select.", "{=u32}", ils.0);
            defmt_monitor::monitor!("CanDebug/ile", desc = "Raw FDCAN ILE, interrupt line enable.", "{=u32}", ile.0);

            // Operating mode stuff.
            defmt_monitor::monitor!("CanDebug/cccr_init", desc = "CCCR.INIT. True means the peripheral is held out of bus traffic, which hardware does on bus-off.", "{=bool}", cccr.init());
            defmt_monitor::monitor!("CanDebug/cccr_dar", desc = "CCCR.DAR. True means automatic retransmission is disabled, so a failed frame is discarded after one attempt.", "{=bool}", cccr.dar());
            defmt_monitor::monitor!("CanDebug/cccr_mon", desc = "CCCR.MON. Bus monitoring mode. True means we never drive the bus dominant.", "{=bool}", cccr.mon());
            defmt_monitor::monitor!("CanDebug/cccr_test", desc = "CCCR.TEST. True in loopback modes.", "{=bool}", cccr.test());

            Timer::after_millis(PROPS_SAMPLE_PERIOD_MS).await;
        }
    }
}
