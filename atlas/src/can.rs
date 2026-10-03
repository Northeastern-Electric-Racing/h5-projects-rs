//! Transport bootloader command and data frames over classical CAN.

use embassy_stm32::bind_interrupts;
use embassy_stm32::can::config::{FdCanConfig, GlobalFilter, NonMatchingFilter};
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter, StandardFilterSlot};
use embassy_stm32::can::frame::Frame;
use embassy_stm32::can::{Can, CanConfigurator, IT0InterruptHandler, IT1InterruptHandler};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_can::{Id, StandardId};

use crate::commands::*;
use crate::config::{BAUD_DELAY_MS, TX_TIMEOUT_MS};

// Peripheral, pins, registers, and interrupt bindings follow the selected ECU.
include!(concat!(env!("OUT_DIR"), "/ecu_can_hardware.rs"));

/// Own the CAN controller through receive, transmit, and bitrate changes.
pub struct CanHandler<'d> {
    // Temporarily taken while synchronously changing controller configuration.
    can: Option<Can<'d>>,
}

impl<'d> CanHandler<'d> {
    /// Start the selected controller with only bootloader receive IDs enabled.
    pub fn new(
        fdcan: embassy_stm32::Peri<'d, CanPeripheral>,
        rx: embassy_stm32::Peri<'d, CanRxPin>,
        tx: embassy_stm32::Peri<'d, CanTxPin>,
    ) -> Self {
        let mut config = CanConfigurator::new(fdcan, rx, tx, Irqs);

        config.set_config(FdCanConfig::default().set_global_filter(GlobalFilter {
            handle_standard_frames: NonMatchingFilter::Reject,
            handle_extended_frames: NonMatchingFilter::Reject,
            reject_remote_standard_frames: true,
            reject_remote_extended_frames: true,
        }));
        config.set_bitrate(DEFAULT_BIT_RATE); // Reset restores the configured initial rate.
        Self::install_filters(&mut config);

        Self {
            can: Some(config.into_normal_mode()),
        }
    }

    // Configuration mode clears filter RAM, so install filters on every start.
    fn install_filters(config: &mut CanConfigurator<'d>) {
        config.properties().set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter {
                filter: FilterType::DedicatedDual(
                    StandardId::new(REQUEST_ID).unwrap(),
                    StandardId::new(DATA_ID).unwrap(),
                ),
                action: Action::StoreInFifo0, // Commands and data share the receive loop.
            },
        );
    }

    /// Wait for a bootloader frame, ignoring bus errors and unrelated traffic.
    pub async fn recv(&mut self) -> Option<Frame> {
        loop {
            // A bus error or unrelated frame is not a receive timeout.
            let Ok(envelope) = self.can.as_mut().unwrap().read().await else {
                continue;
            };
            let frame = envelope.frame;
            let Some(id) = standard_id(&frame) else {
                continue;
            };

            if id == REQUEST_ID || id == DATA_ID {
                return Some(frame);
            }
        }
    }

    /// Return None when no matching frame arrives before the deadline.
    pub async fn recv_timeout(&mut self, timeout_ms: u64) -> Option<Frame> {
        // One deadline covers all discarded frames and recoverable bus errors.
        with_timeout(Duration::from_millis(timeout_ms), self.recv())
            .await
            .ok()
            .flatten()
    }

    /// Queue a frame; success does not mean it has finished transmitting.
    pub async fn send(&mut self, id: u16, data: &[u8]) -> bool {
        let Ok(frame) = Frame::new_standard(id, data) else {
            return false;
        };

        // Embassy returns a displaced queued frame when the mailbox was replaced.
        self.can.as_mut().unwrap().write(&frame).await.is_none()
    }

    /// Queue a command ACK containing the command opcode.
    pub async fn ack(&mut self, command: u8) -> bool {
        self.send(RESPONSE_ID, &[ACK, command]).await
    }

    /// Include the packet sequence so the host can distinguish data ACKs.
    pub async fn data_ack(&mut self, sequence: u8) -> bool {
        self.send(RESPONSE_ID, &[ACK, WRITE_MEMORY, sequence]).await
    }

    /// Queue a rejection for the given command.
    pub async fn nack(&mut self, command: u8) -> bool {
        self.send(RESPONSE_ID, &[NACK, command]).await
    }

    /// Wait for pending transmissions to drain, with a bounded timeout.
    pub async fn flush(&mut self) -> bool {
        // A queued ACK must leave the controller before reset or reconfiguration.
        with_timeout(Duration::from_millis(TX_TIMEOUT_MS), async {
            while CAN_REGS.txbrp().read().0 != 0 {
                // Yield so interrupts and other executor work can keep progressing.
                Timer::after_millis(1).await;
            }
        })
        .await
        .is_ok()
    }

    /// Apply a bitrate code after flushing the ACK queued at the old rate.
    pub async fn set_baud(&mut self, value: u8) -> bool {
        let Some(bitrate) = baud_rate(value) else {
            return false;
        };

        if !self.flush().await {
            // Keep the current bitrate when pending transmissions cannot drain.
            return false;
        }

        // Give the host time to receive the ACK before the bus rate changes.
        Timer::after_millis(BAUD_DELAY_MS).await;

        // No await while ownership is taken; restore cleared filters before start.
        let mut config = self.can.take().unwrap().into_config_mode();
        config.set_bitrate(bitrate);
        Self::install_filters(&mut config);
        self.can = Some(config.into_normal_mode());

        true
    }

    /// Stop CAN and clear its interrupt sources before application handoff.
    pub fn shutdown(mut self) {
        let _config = self.can.take().unwrap().into_config_mode();
        let regs = CAN_REGS;
        regs.ie().write(|w| w.0 = 0);
        regs.ile().write(|w| w.0 = 0);
        // Interrupt flags are cleared by writing ones.
        regs.ir().write(|w| w.0 = u32::MAX);
    }
}

/// Extract an 11-bit ID; extended frames are outside the bootloader protocol.
pub fn standard_id(frame: &Frame) -> Option<u16> {
    match frame.id() {
        Id::Standard(id) => Some(id.as_raw()),

        Id::Extended(_) => None,
    }
}
