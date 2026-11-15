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
                            // The model answers RDHILTIME from its live clock -- SNAP does not
                            // freeze it -- so this is the simulation time a fraction of a frame
                            // after the registers above were sent. `cache::Group::update` samples
                            // it once this read returns, which is what pins it to this reading.
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

/// The simulink model's own clock, read over UART.
///
/// This is the plant clock on a rig: see [`crate::time`] for what that is and is not for.
pub mod sim_time {
    use super::{Async, READ_TIMEOUT, UartRx, UartTx, with_timeout};
    use adbms2950::chip::{
        pec::{CommandPec, DataPecRx},
        registers::{ReadableGroup, results::AccumulatedCurrentAndBatteryVoltage},
    };
    use core::cell::Cell;
    use crate::time::PlantTime;
    use embassy_sync::blocking_mutex::ThreadModeMutex;

    const RDHILTIME: u16 = 0b111_1111_1111;

    /// Last simulation time the rig reported, in microseconds.
    ///
    /// `None` until the rig has answered. Never zero-as-unknown: a consumer that cannot tell those
    /// apart measures its first dwell against a clock that was never read.
    static MICROS: ThreadModeMutex<Cell<Option<u64>>> = ThreadModeMutex::new(Cell::new(None));

    /// The wire frame for [`RDHILTIME`]: opcode then command PEC, same as any other command.
    const fn query() -> [u8; 4] {
        let command = [(RDHILTIME >> 8_i32) as u8, RDHILTIME as u8];
        let pec = CommandPec::new(&command);

        [command[0], command[1], pec.pec0(), pec.pec1()]
    }

    /// The model's clock as of its last answer, on the model's own timebase.
    ///
    /// `None` until the rig has answered. Refreshed once per HV plate pass, so it is quantised to
    /// that period and can read the same value twice -- callers must not assume it advances
    /// between two calls.
    pub fn now() -> Option<PlantTime> {
        MICROS.lock(|micros| micros.get()).map(PlantTime::from_micros)
    }

    /// Reads the rig's clock, but only after the one read whose timestamp is load bearing.
    ///
    /// `job_update_snap_registers` reads this group on every pass, so the gate is already "once per
    /// HV plate tick" -- ungating it would buy nothing and cost a frame per register group.
    ///
    /// The reply is the model's clock at the moment it handles this frame, which is a couple of
    /// frames after the SNAP that froze the registers being stamped. That offset is near enough
    /// constant and the accumulator only ever takes differences of two stamps, so it cancels.
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
            defmt::warn!("HvPlate: hil: the rig did not answer RDHILTIME; everything measured in plant time is holding against a stale simulation clock.");
            return;
        }

        if !DataPecRx::from_bytes([reply[6], reply[7]]).verify(&reply[..6]) {
            defmt::warn!("HvPlate: hil: RDHILTIME failed its PEC; everything measured in plant time is holding against a stale simulation clock.");
            return;
        }

        // Six bytes of microseconds, little endian. 48 bits is about nine years of simulated time.
        let mut micros: u64 = 0;
        for (index, byte) in reply[..6].iter().enumerate() {
            micros |= u64::from(*byte) << (index * 8);
        }

        // The model restarted under a firmware that did not. Every dwell and deadline measured in
        // plant time is now counting against a world that no longer exists: they will not fire
        // until the clock climbs back past where they were set. Reported, not papered over --
        // restart the firmware with the model.
        if let Some(previous) = MICROS.lock(|stamp| stamp.get())
            && micros < previous
        {
            defmt::error!("HvPlate: hil: the rig's clock went backwards ({=u64} us -> {=u64} us). Every deadline and dwell measured in plant time is now stale; restart the firmware along with the model.", previous, micros);
        }

        MICROS.lock(|stamp| stamp.set(Some(micros)));
    }
}
