//! The HV plate's line: UART4, and the one line that carries the model's clock.

use embassy_stm32::usart::{self, Uart};

use super::HilDevice;

embassy_stm32::bind_interrupts!(struct Irqs {
    UART4 => usart::InterruptHandler<embassy_stm32::peripherals::UART4>;
    GPDMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH6>;
    GPDMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH7>;
});

/// The plate's stand-in for its ADBMS2950B.
pub fn device(r: crate::HvPlateResources) -> HilDevice {
    let mut config = usart::Config::default();
    config.baudrate = super::BAUDRATE;

    let uart = Uart::new(r.uart, r.tx, r.rx, r.tx_dma, r.rx_dma, Irqs, config).expect("Invalid HV plate UART4 configuration");

    HilDevice::serving_sim_time(uart)
}
