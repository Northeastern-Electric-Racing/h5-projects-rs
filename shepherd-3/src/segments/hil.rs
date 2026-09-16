//! HIL wire format: raw ADBMS commands/replies on UART9, 115200 8N1.
//! Build: `cargo build -p shepherd-3 --release --features hil`.
//! Connect emulator RX to PD15 (TX), emulator TX to PD14 (RX), and common ground.
//! HIL skips conversion polling and status B/C/D reads. These caches remain
//! unpopulated; service discovery, sleep detection and split recovery do not run.
//! Ordinary register replies still require valid chip data and PEC bytes.

use embassy_stm32::{
    mode::Async,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_time::{with_timeout, Duration};
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

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

/// `None` provides the unused line B required by the driver's constructor.
/// It owns no peripheral and rejects accidental transactions.
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

        // Support the exact transaction shapes used by adbms6830b::Line.
        match operations {
            // Chip-select wakeup pulses have no UART equivalent.
            [] | [Operation::DelayNs(_)] => Ok(()),

            [Operation::Write(command), Operation::Read(data)] if command.len() == 4 => {
                with_timeout(Duration::from_secs(60), async {
                    // Send exactly one ADBMS command first.
                    tx.write(command).await?;
                    tx.flush().await?;

                    // Wait for the complete reply for this command before
                    // allowing the driver to begin another transaction.
                    rx.read(data).await?;

                    Ok::<(), usart::Error>(())
                })
                .await
                .map_err(|_| HilError::Timeout)?
                .map_err(HilError::Uart)?;

                Ok(())
            },

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
