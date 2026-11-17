//! The UART that stands in for isoSPI on a HIL rig.
//!
//! There is no chip and no isoSPI on a rig: a Simulink model answers the same command frames over a
//! UART, one per line. The transport below is shared; what differs per line is the peripheral and
//! whether it carries the model's clock, and that lives in [`hv_plate`] and [`segments`].

use embassy_stm32::{
    mode::Async,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_time::{with_timeout, Duration};
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

pub mod hv_plate;
pub mod segments;
mod sim_time;

const BAUDRATE: u32 = 115_200;
const READ_TIMEOUT: Duration = Duration::from_secs(1);
const READ_RETRIES: usize = 2;
// About two byte times at 115200 baud: once the line is quiet this long, RX is drained.
const DRAIN_IDLE: Duration = Duration::from_micros(200);
// Bounds the drain so a stream of noise or errors can't stall the transaction.
const DRAIN_MAX_BYTES: usize = 64;

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

pub struct HilDevice {
    /// `None` disables a line the rig does not wire up.
    uart: Option<(UartTx<'static, Async>, UartRx<'static, Async>)>,
    serves_sim_time: bool,
}
impl HilDevice {
    /// The plate's line, which also carries the model's clock: RDHILTIME rides on the back of its accumulator read.
    fn serving_sim_time(uart: Uart<'static, Async>) -> Self {
        Self { uart: Some(uart.split()), serves_sim_time: true }
    }

    /// A line that only moves frames.
    fn frames_only(uart: Uart<'static, Async>) -> Self {
        Self { uart: Some(uart.split()), serves_sim_time: false }
    }

    /// Keeps a line-B argument honest without reserving hardware for it; every transaction fails with [`HilError::DisabledLine`].
    pub const fn disabled() -> Self {
        Self { uart: None, serves_sim_time: false }
    }
}
impl ErrorType for HilDevice {
    type Error = HilError;
}

fn without_cs_timing<'a, 'b>(operations: &'a mut [Operation<'b, u8>]) -> &'a mut [Operation<'b, u8>] {
    let is_delay = |op: &Operation<'_, u8>| matches!(op, Operation::DelayNs(_));
    let start = operations.iter().position(|op| !is_delay(op)).unwrap_or(operations.len());
    let end = operations.iter().rposition(|op| !is_delay(op)).map_or(start, |index| index + 1);
    &mut operations[start..end]
}

impl SpiDevice for HilDevice {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), HilError> {
        let serves_sim_time = self.serves_sim_time;
        let (tx, rx) = self.uart.as_mut().ok_or(HilError::DisabledLine)?;
        match without_cs_timing(operations) {
            // Chip-select wakeup pulses have no UART equivalent, and trim to nothing.
            [] => Ok(()),
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
                        Ok(Err(err)) => return Err(HilError::Uart(err)),
                        Ok(Ok(())) => {
                            if serves_sim_time {
                                // SNAP does not freeze RDHILTIME, so this is the model's clock a fraction of a frame after the registers above were sent.
                                sim_time::refresh_after(command, tx, rx).await;
                            }
                            return Ok(());
                        },
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

/// Discards bytes waiting in the RX FIFO, which `UartRx::read` would otherwise put at the front of the next reply.
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
