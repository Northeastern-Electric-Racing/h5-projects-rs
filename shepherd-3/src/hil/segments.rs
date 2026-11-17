//! The segments' line: UART9, standing in for isoSPI line A. The rig does not wire up line B.

use embassy_stm32::usart::{self, Uart};

use super::HilDevice;

embassy_stm32::bind_interrupts!(struct Irqs {
    UART9 => usart::InterruptHandler<embassy_stm32::peripherals::UART9>;
    GPDMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH4>;
    GPDMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH5>;
});

/// The segments' stand-in for all ten ADBMS6830Bs.
pub fn device(r: crate::SegmentIsoSpiLineAResources) -> HilDevice {
    let mut config = usart::Config::default();
    config.baudrate = super::BAUDRATE;

    let uart = Uart::new(r.uart, r.tx, r.rx, r.tx_dma, r.rx_dma, Irqs, config).expect("Invalid segments UART9 configuration");

    HilDevice::frames_only(uart)
}
