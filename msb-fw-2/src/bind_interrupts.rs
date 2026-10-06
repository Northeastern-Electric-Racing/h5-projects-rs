use embassy_stm32::{bind_interrupts, dma, i2c, peripherals};

bind_interrupts!(pub struct Irqs {
    // Mux scan (multiplexor_handler)
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
    // Magnetometer (SPI2)
    GPDMA1_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA1_CH2>;
    GPDMA1_CHANNEL3 => dma::InterruptHandler<peripherals::GPDMA1_CH3>;
    // IMU (SPI1)
    GPDMA1_CHANNEL4 => dma::InterruptHandler<peripherals::GPDMA1_CH4>;
    GPDMA1_CHANNEL5 => dma::InterruptHandler<peripherals::GPDMA1_CH5>;
    // HDC2021 (I2C1)
    GPDMA1_CHANNEL6 => dma::InterruptHandler<peripherals::GPDMA1_CH6>;
    GPDMA1_CHANNEL7 => dma::InterruptHandler<peripherals::GPDMA1_CH7>;
    // I2C1 interrupts
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});
