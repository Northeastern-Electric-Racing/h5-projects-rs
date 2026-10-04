use embassy_stm32::{bind_interrupts, dma, peripherals};

bind_interrupts!(pub struct Irqs {
    // Mux scan (multiplexor_handler)
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});
