//! CAN bring-up for Cerberus.

use embassy_stm32::can::filter::StandardFilterSlot;
use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame, IT0InterruptHandler, IT1InterruptHandler,
};
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use ner_can::{NerCan, StdFilter};

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

    let (tx, rx, _props) = NerCan::init(configurator)
        .with_standard_filters(&[StdFilter {
            slot: StandardFilterSlot::_0,
            id1: crate::BOOT_REQUEST_ID,
            id2: None,
        }])
        .start();

    (tx, rx)
}

// Queue received frames for the application processor.
#[embassy_executor::task]
pub async fn receive(mut rx: CanRx<'static>) {
    loop {
        match rx.read().await {
            Ok(envelope) => {
                INCOMING.send(envelope.frame).await;
            }
            Err(error) => defmt::warn!("CAN receive error: {}", error),
        }
    }
}

// Handle application commands after receiving them from the queue.
#[embassy_executor::task]
pub(super) async fn can_rx_processer(mut bootloader: boot_control::Bootloader<'static>) {
    loop {
        let frame = INCOMING.receive().await;
        if bootloader.handle_frame(&frame).is_err() {
            defmt::error!("Failed to persist bootloader request");
        }
    }
}
