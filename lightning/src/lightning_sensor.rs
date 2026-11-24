//! runs the AS3935 lightning sensor using the as3935-ner driver (in firmware-rs/drivers)

use as3935_ner::{Afe, As3935, Interrupt};
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
pub async fn lightning_sensor_task(spi: As3935Spi, mut irq: ExtiInput<'static, Async>) {
    let mut sensor = As3935::new(spi);

    // setup loop
    loop {
        let result = async {
            sensor.preset_default().await?;
            sensor.calibrate_rco().await?;
            Timer::after_millis(10).await;
            sensor.set_afe(Afe::Indoor).await?;
            sensor.afe().await
        }
        .await;

        match result {
            Ok(Afe::Indoor) => break,
            Ok(other) => {
                defmt::error!("As3935 AFE readback mismatch: got {} (retrying)", other);
                Timer::after_millis(500).await;
            }
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

    // main loop
    loop {
        irq.wait_for_high().await;
        Timer::after_millis(2).await;

        match sensor.interrupt().await {
            Ok(Interrupt::Lightning) => {
                let distance = sensor.distance().await;
                let energy = sensor.energy().await;
                match (distance, energy) {
                    (Ok(d), Ok(e)) => {
                        defmt::info!("AS3935: lightning, distance {} energy {}", d, e)
                    }
                    _ => defmt::warn!("AS3935: lightning distance or energy failed!"),
                }
            }
            Ok(Interrupt::Disturber) => defmt::info!("AS3935: disturber detected."),
            Ok(Interrupt::NoiseTooHigh) => defmt::warn!("AS3935: noise too high"),
            Ok(Interrupt::None) => {}
            Err(e) => {
                defmt::warn!("As3935 interrupt read failed: {} ", defmt::Debug2Format(&e));
                Timer::after_millis(100).await;
            }
        }
    }
}
