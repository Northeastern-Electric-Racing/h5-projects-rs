//! CAN bring-up for Cerberus.

use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame, IT0InterruptHandler, IT1InterruptHandler,
};
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use ner_can::NerCan;

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

    let (tx, rx, _props) = NerCan::init(configurator).start();

    (tx, rx)
}
