//! HIL support for the HV plate.

use embassy_stm32::{
    mode::Async,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_time::{with_timeout, Duration};
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

const READ_TIMEOUT: Duration = Duration::from_secs(1);
const READ_RETRIES: usize = 2;

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

fn without_cs_timing<'a, 'b>(operations: &'a mut [Operation<'b, u8>]) -> &'a mut [Operation<'b, u8>] {
    let is_delay = |op: &Operation<'_, u8>| matches!(op, Operation::DelayNs(_));
    let start = operations.iter().position(|op| !is_delay(op)).unwrap_or(operations.len());
    let end = operations.iter().rposition(|op| !is_delay(op)).map_or(start, |index| index + 1);
    &mut operations[start..end]
}

impl SpiDevice for HilDevice {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), HilError> {
        let (tx, rx) = self.0.as_mut().ok_or(HilError::DisabledLine)?;
        match without_cs_timing(operations) {
            // Chip-select wakeup pulses have no UART equivalent, and trim to nothing.
            [] => Ok(()),
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
                        Ok(Err(err)) => return Err(HilError::Uart(err)),
                        Ok(Ok(())) => {
                            // Fetch the model's clock while it still describes this reading.
                            sim_time::refresh_after(command, tx, rx).await;
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

/// The simulink model's own clock, read over UART.
pub mod sim_time {
    use super::{Async, READ_TIMEOUT, UartRx, UartTx, with_timeout};
    use adbms2950::chip::{
        pec::{CommandPec, DataPecRx},
        registers::{ReadableGroup, results::AccumulatedCurrentAndBatteryVoltage},
    };
    use core::cell::Cell;
    use embassy_sync::blocking_mutex::ThreadModeMutex;

    const RDHILTIME: u16 = 0b111_1111_1111;
    /// Last simulation time the rig reported, in microseconds. Zero until the first answer.
    static MICROS: ThreadModeMutex<Cell<u64>> = ThreadModeMutex::new(Cell::new(0));

    /// The wire frame for [`RDHILTIME`]: opcode then command PEC, same as any other command.
    const fn query() -> [u8; 4] {
        let command = [(RDHILTIME >> 8_i32) as u8, RDHILTIME as u8];
        let pec = CommandPec::new(&command);

        [command[0], command[1], pec.pec0(), pec.pec1()]
    }

    /// The model's clock as of its last answer, on the model's own timebase.
    pub fn now() -> embassy_time::Instant {
        embassy_time::Instant::from_micros(MICROS.lock(|micros| micros.get()))
    }

    /// Reads the rig's clock, but only after the one read whose timestamp is load bearing.
    pub(super) async fn refresh_after(command: &[u8], tx: &mut UartTx<'static, Async>, rx: &mut UartRx<'static, Async>) {
        if command != AccumulatedCurrentAndBatteryVoltage::READ_COMMAND.to_bytes().as_slice() {
            return;
        }

        let mut reply = [0_u8; 8];
        let fetched = with_timeout(READ_TIMEOUT, async {
            tx.write(&query()).await?;
            tx.flush().await?;
            rx.read(&mut reply).await
        })
        .await;

        if !matches!(fetched, Ok(Ok(()))) {
            defmt::warn!("HvPlate: hil: the rig did not answer RDHILTIME; the next SOC window is measured against a stale simulation time.");
            return;
        }

        if !DataPecRx::from_bytes([reply[6], reply[7]]).verify(&reply[..6]) {
            defmt::warn!("HvPlate: hil: RDHILTIME failed its PEC; the next SOC window is measured against a stale simulation time.");
            return;
        }

        // Six bytes of microseconds, little endian. 48 bits is about nine years of simulated time.
        let mut micros: u64 = 0;
        for (index, byte) in reply[..6].iter().enumerate() {
            micros |= u64::from(*byte) << (index * 8);
        }

        MICROS.lock(|stamp| stamp.set(micros));
    }
}
