//! HIL support for the HV plate.

use embassy_stm32::{
    mode::Async,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_time::{with_timeout, Duration};
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

const READ_TIMEOUT: Duration = Duration::from_secs(1);
const READ_RETRIES: usize = 2;
// About two byte times at 115200 baud: once the line is quiet this long, RX is drained.
const DRAIN_IDLE: Duration = Duration::from_micros(200);
// Bounds the drain so a stream of noise or errors can't stall the transaction.
const DRAIN_MAX_BYTES: usize = 64;

embassy_stm32::bind_interrupts!(struct Irqs {
    UART4 => usart::InterruptHandler<embassy_stm32::peripherals::UART4>;
    GPDMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH6>;
    GPDMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::GPDMA1_CH7>;
});

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum HilError {
    Uart(usart::Error),
    Timeout,
    DisabledLine,
    UnsupportedTransaction,
}
impl embedded_hal_async::spi::Error for HilError {
    fn kind(&self) -> embedded_hal_async::spi::ErrorKind {
        embedded_hal_async::spi::ErrorKind::Other
    }
}

pub struct HilDevice(Option<(UartTx<'static, Async>, UartRx<'static, Async>)>);
impl HilDevice {
    pub fn new(r: crate::HvPlateResources) -> Self {
        let mut config = usart::Config::default();
        config.baudrate = 115_200;
        let uart = Uart::new(r.uart, r.tx, r.rx, r.tx_dma, r.rx_dma, Irqs, config).expect("Invalid HV plate UART4 configuration");
        Self(Some(uart.split()))
    }

    pub const fn disabled() -> Self {
        Self(None)
    }
}
impl ErrorType for HilDevice {
    type Error = HilError;
}
impl SpiDevice for HilDevice {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), HilError> {
        let (tx, rx) = self.0.as_mut().ok_or(HilError::DisabledLine)?;
        match operations {
            // Chip-select wakeup pulses have no UART equivalent.
            [] | [Operation::DelayNs(_)] => Ok(()),
            // Send command + PEC, then receive the fixed-size reply.
            [Operation::Write(command), Operation::Read(data)] if command.len() == 4 => {
                // Retry read timeouts; report only the final failure.
                for attempt in 0..=READ_RETRIES {
                    // Drop stale bytes (e.g. a late reply to a timed-out attempt) so the reply lands at data[0].
                    drain_rx(rx).await;

                    let result = with_timeout(READ_TIMEOUT, async {
                        tx.write(command).await?;
                        tx.flush().await?;
                        rx.read(data).await?;

                        Ok::<(), usart::Error>(())
                    })
                    .await;

                    match result {
                        Ok(result) => return result.map_err(HilError::Uart),
                        Err(_) if attempt < READ_RETRIES => continue,
                        Err(_) => return Err(HilError::Timeout),
                    }
                }

                unreachable!()
            },
            // Do not retry writes: they may already have taken effect.
            [Operation::Write(command)] => with_timeout(Duration::from_millis(100), async {
                tx.write(command).await?;
                tx.flush().await
            })
            .await
            .map_err(|_| HilError::Timeout)?
            .map_err(HilError::Uart),

            [Operation::Write(command), Operation::Write(data)] => with_timeout(Duration::from_millis(100), async {
                tx.write(command).await?;
                tx.write(data).await?;
                tx.flush().await
            })
            .await
            .map_err(|_| HilError::Timeout)?
            .map_err(HilError::Uart),

            _ => Err(HilError::UnsupportedTransaction),
        }
    }
}

/// Discards bytes waiting in the RX FIFO. `UartRx::read` puts already-received bytes at the front of
/// its buffer, so anything left over would shift the next reply.
async fn drain_rx(rx: &mut UartRx<'static, Async>) {
    let mut byte = [0u8; 1];
    let mut dropped = 0usize;
    for _ in 0..DRAIN_MAX_BYTES {
        match with_timeout(DRAIN_IDLE, rx.read(&mut byte)).await {
            Ok(Ok(())) => dropped += 1,
            // Errors clear their flags; keep draining.
            Ok(Err(_)) => {},
            // Line is quiet.
            Err(_) => break,
        }
    }
    if dropped > 0 {
        defmt::warn!("HIL: dropped {} stale RX bytes", dropped);
    }
}