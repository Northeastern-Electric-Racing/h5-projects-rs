//! HIL support for segments.

use embassy_stm32::{
    mode::Async,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_time::{with_timeout, Duration};
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

const READ_TIMEOUT: Duration = Duration::from_secs(1);
const READ_RETRIES: usize = 2;

embassy_stm32::bind_interrupts!(struct Irqs {
    UART9 => usart::InterruptHandler<embassy_stm32::peripherals::UART9>;

    GPDMA1_CHANNEL4 =>
        embassy_stm32::dma::InterruptHandler<
            embassy_stm32::peripherals::GPDMA1_CH4
        >;

    GPDMA1_CHANNEL5 =>
        embassy_stm32::dma::InterruptHandler<
            embassy_stm32::peripherals::GPDMA1_CH5
        >;
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

/// `None` disables the unused line B.
pub struct HilDevice(Option<(UartTx<'static, Async>, UartRx<'static, Async>)>);

impl HilDevice {
    pub fn new(r: crate::SegmentIsoSpiLineAResources) -> Self {
        let mut config = usart::Config::default();
        config.baudrate = 115_200;

        let uart = Uart::new(r.uart, r.tx, r.rx, r.tx_dma, r.rx_dma, Irqs, config).expect("Invalid HIL UART9 configuration");

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
