//! runs the AS3935 lightning sensor using the as3935-ner driver (in firmware-rs/drivers)

use as3935_ner::{Afe, As3935};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

/// SPI2 plus the AS3935’s chip select pin
// type alias for spi device
pub type As3935Spi = ExclusiveDevice<Spi<'static, Async, Master>, Output<'static>, Delay>;

#[embassy_executor::task]
pub async fn lightning_sensor_task(spi: As3935Spi, _irq: ExtiInput<'static, Async>) {
    let mut sensor = As3935::new(spi);

    loop {
        let result = async {
            sensor.preset_default().await?;
            sensor.calibrate_rco().await?;
            Timer::after_millis(10).await;
            sensor.set_afe(Afe::Indoor).await
        }
        .await;

        match result {
            Ok(()) => break,
            Err(e) => {
                defmt::error!(
                    "As3935 setup failed: {} (retrying)",
                    defmt::Debug2Format(&e)
                );
                Timer::after_millis(500).await;
            }
        }
    }

    defmt::info!("As3935 connected!");
}
