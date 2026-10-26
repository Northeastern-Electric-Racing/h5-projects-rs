//! CAN bring-up for Cerberus.

use crate::{buttons, shutdown};
use cangen::{CanMessage, Msg};
use defmt::debug;
use embassy_stm32::can::filter::{ExtendedFilterSlot, StandardFilterSlot};
use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame, IT0InterruptHandler, IT1InterruptHandler,
};
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use ner_can::{ExtFilter, NerCan, StdFilter};

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => IT1InterruptHandler<peripherals::FDCAN2>;
});

/// Capacity of each CAN channel, in `Frame`s.
pub const CHANNEL_SIZE: usize = 256;

/// Frames received off the bus, waiting to be parsed by the rest of the program.
pub static INCOMING: Channel<ThreadModeRawMutex, Frame, CHANNEL_SIZE> = Channel::new();
/// Frames queued by the program, waiting to go out on the bus.
pub static OUTGOING: Channel<ThreadModeRawMutex, Frame, CHANNEL_SIZE> = Channel::new();

/// Peripherals the CAN bus is wired to.
pub struct CanPins {
    pub can: Peri<'static, peripherals::FDCAN2>,
    pub rx: Peri<'static, peripherals::PB5>,
    pub tx: Peri<'static, peripherals::PB6>,
}

/// initializes FDCAN peripheral and returns the split CanTx and CanRx structs
pub fn init(pins: CanPins) -> (CanTx<'static>, CanRx<'static>) {
    let configurator = CanConfigurator::new(pins.can, pins.rx, pins.tx, Irqs);

    const SLOT_1: StdFilter = StdFilter {
        slot: StandardFilterSlot::_1,
        id1: cangen::CellTemperatures::ID,
        id2: None,
    };

    const SLOT_2: StdFilter = StdFilter {
        slot: StandardFilterSlot::_2,
        id1: cangen::FrontWheelRpm::ID,
        id2: Some(cangen::TempsFaultCodeStatus::ID),
    };

    const SLOT_3: StdFilter = StdFilter {
        slot: StandardFilterSlot::_3,
        id1: cangen::ErpmDutyInputVoltageStatus::ID,
        id2: Some(cangen::CurrentsStatus::ID),
    };

    const SLOT_4: StdFilter = StdFilter {
        slot: StandardFilterSlot::_4,
        id1: cangen::PrechargeStatus::ID,
        id2: Some(cangen::WheelButtons::ID),
    };

    const SLOT_5: StdFilter = StdFilter {
        slot: StandardFilterSlot::_5,
        id1: cangen::ShutdownAsReadByBms::ID,
        id2: None,
    };

    // --- CAN bootloader support ---
    const SLOT_6: StdFilter = StdFilter {
        slot: StandardFilterSlot::_6,
        id1: cangen::VcuBootRequest::ID,
        id2: None,
    };
    // --- End CAN bootloader support ---

    const SLOT_1E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_1,
        id1: cangen::RtdsCommandMessage::ID,
        id2: Some(cangen::LightningPulseMessage::ID),
    };

    const SLOT_2E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_2,
        id1: cangen::DashboardEfuseState::ID,
        id2: Some(cangen::BrakeEfuseState::ID),
    };

    const SLOT_3E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_3,
        id1: cangen::ShutdownEfuseState::ID,
        id2: Some(cangen::LvEfuseState::ID),
    };

    const SLOT_4E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_4,
        id1: cangen::RadfanEfuseState::ID,
        id2: Some(cangen::FanbattEfuseState::ID),
    };

    const SLOT_5E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_5,
        id1: cangen::PumponeEfuseState::ID,
        id2: Some(cangen::PumptwoEfuseState::ID),
    };

    const SLOT_6E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_6,
        id1: cangen::BattboxEfuseState::ID,
        id2: Some(cangen::McEfuseState::ID),
    };

    const SLOT_7E: ExtFilter = ExtFilter {
        slot: ExtendedFilterSlot::_7,
        id1: cangen::SpareEfuseState::ID,
        id2: None,
    };

    let can = NerCan::init(configurator)
        .with_standard_filters(&[SLOT_1, SLOT_2, SLOT_3, SLOT_4, SLOT_5, SLOT_6])
        .with_extended_filters(&[
            SLOT_1E, SLOT_2E, SLOT_3E, SLOT_4E, SLOT_5E, SLOT_6E, SLOT_7E,
        ]);
    let (tx, rx, _props) = can.start();

    (tx, rx)
}

#[embassy_executor::task]
pub async fn can_inbox_task(mut bootloader: boot_control::Bootloader<'static>) -> ! {
    loop {
        let frame = INCOMING.receive().await;
        match Msg::decode(&frame) {
            // --- CAN bootloader support ---
            Some(Msg::VcuBootRequest(_)) => bootloader
                .handle_frame(&frame)
                .unwrap_or_else(|_| defmt::error!("Failed to persist bootloader request")),
            // --- End CAN bootloader support ---
            Some(Msg::CellTemperatures(_)) => {}
            Some(Msg::LightningPulseMessage(_)) => {}
            Some(Msg::FrontWheelRpm(_)) => {}
            Some(Msg::TempsFaultCodeStatus(_)) => {}
            Some(Msg::ErpmDutyInputVoltageStatus(_)) => {}
            Some(Msg::CurrentsStatus(_)) => {}

            Some(Msg::DashboardEfuseState(_)) => {}
            Some(Msg::BrakeEfuseState(_)) => {}
            Some(Msg::ShutdownEfuseState(_)) => {}
            Some(Msg::LvEfuseState(_)) => {}
            Some(Msg::RadfanEfuseState(_)) => {}
            Some(Msg::PumponeEfuseState(_)) => {}
            Some(Msg::PumptwoEfuseState(_)) => {}
            Some(Msg::BattboxEfuseState(_)) => {}
            Some(Msg::McEfuseState(_)) => {}
            Some(Msg::SpareEfuseState(_)) => {}

            Some(Msg::PrechargeStatus(_)) => {}
            Some(Msg::RtdsCommandMessage(_)) => {}

            Some(Msg::WheelButtons(m)) => buttons::register_press(m.button_id()).await,
            Some(Msg::ShutdownAsReadByBms(m)) => {
                shutdown::update_bms_shutdown(m.shutdown_state().into()).await
            }
            Some(_) => debug!("Unknown message"),
            _ => {
                debug!("failed to parse id")
            }
        }
    }
}
