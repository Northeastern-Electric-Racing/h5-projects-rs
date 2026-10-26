//! Transport bootloader command and data frames over classical CAN.

use embassy_stm32::bind_interrupts;
use embassy_stm32::can::filter::{ExtendedFilterSlot, StandardFilterSlot};
use embassy_stm32::can::frame::Frame;
use embassy_stm32::can::{Can, CanConfigurator, IT0InterruptHandler, IT1InterruptHandler};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_can::Id;
use ner_can::{ExtFilter, NerCan, StdFilter};

use crate::commands::*;
use crate::config::TX_TIMEOUT_MS;

use embassy_stm32::pac::FDCAN2 as CAN_REGS;
use embassy_stm32::peripherals::FDCAN2 as CanPeripheral;
#[cfg(any(ecu = "vcu", ecu = "msb_front", ecu = "msb_back"))]
use embassy_stm32::peripherals::{PB5 as CanRxPin, PB6 as CanTxPin};
#[cfg(ecu = "lightning")]
use embassy_stm32::peripherals::{PB12 as CanRxPin, PB13 as CanTxPin};
#[cfg(ecu = "bms")]
use embassy_stm32::peripherals::{PB13 as CanTxPin, PD9 as CanRxPin};

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => IT0InterruptHandler<CanPeripheral>;
    FDCAN2_IT1 => IT1InterruptHandler<CanPeripheral>;
});

/// Own the CAN controller through receive, transmit, and application handoff.
pub struct CanHandler<'d> {
    can: Can<'d>,
}

impl CanHandler<'static> {
    /// Start the selected controller with only bootloader receive IDs enabled.
    pub fn new(
        fdcan: embassy_stm32::Peri<'static, CanPeripheral>,
        rx: embassy_stm32::Peri<'static, CanRxPin>,
        tx: embassy_stm32::Peri<'static, CanTxPin>,
    ) -> Self {
        let mut ner_can = NerCan::init(CanConfigurator::new(fdcan, rx, tx, Irqs));
        for (id, standard_slot, extended_slot) in [
            (REQUEST_ID, StandardFilterSlot::_0, ExtendedFilterSlot::_0),
            (DATA_ID, StandardFilterSlot::_1, ExtendedFilterSlot::_1),
        ] {
            ner_can = match id {
                Id::Standard(id) => ner_can.with_standard_filters(&[StdFilter {
                    slot: standard_slot,
                    id1: id.as_raw(),
                    id2: None,
                }]),
                Id::Extended(id) => ner_can.with_extended_filters(&[ExtFilter {
                    slot: extended_slot,
                    id1: id.as_raw(),
                    id2: None,
                }]),
            };
        }
        let config = &mut ner_can.can_configurator;
        // Keep ACKs pending for retry until transmission succeeds or flush times out.
        config.set_config(config.config().set_automatic_retransmit(true));
        config.set_bitrate(CAN_LINE_BITRATE);

        // Keep the controller together so shutdown can return it to configuration mode.
        Self {
            can: ner_can.can_configurator.into_normal_mode(),
        }
    }
}

impl<'d> CanHandler<'d> {
    /// Wait for a bootloader frame, ignoring bus errors and unrelated traffic.
    pub async fn recv(&mut self) -> Frame {
        loop {
            // A bus error or unrelated frame is not a receive timeout.
            let Ok(envelope) = self.can.read().await else {
                continue;
            };
            let frame = envelope.frame;
            let Some(id) = data_id(&frame) else {
                continue;
            };

            if id == REQUEST_ID || id == DATA_ID {
                return frame;
            }
        }
    }

    /// Return None when no matching frame arrives before the deadline.
    pub async fn recv_timeout(&mut self, timeout_ms: u64) -> Option<Frame> {
        // One deadline covers all discarded frames and recoverable bus errors.
        with_timeout(Duration::from_millis(timeout_ms), self.recv())
            .await
            .ok()
    }

    /// Queue a frame; success does not mean it has finished transmitting.
    pub async fn send(&mut self, id: Id, data: &[u8]) -> bool {
        let Ok(frame) = Frame::new_data(id, data) else {
            return false;
        };

        // Embassy returns a displaced queued frame when the mailbox was replaced.
        self.can.write(&frame).await.is_none()
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
        // A queued ACK must leave the controller before reset or application handoff.
        with_timeout(Duration::from_millis(TX_TIMEOUT_MS), async {
            while CAN_REGS.txbrp().read().0 != 0 {
                // Yield so interrupts and other executor work can keep progressing.
                Timer::after_millis(1).await;
            }
        })
        .await
        .is_ok()
    }

    /// Stop CAN and clear its interrupt sources before application handoff.
    pub fn shutdown(self) {
        let _config = self.can.into_config_mode();
        let regs = CAN_REGS;
        regs.ie().write(|w| w.0 = 0);
        regs.ile().write(|w| w.0 = 0);
        // Interrupt flags are cleared by writing ones.
        regs.ir().write(|w| w.0 = u32::MAX);
    }
}

/// Return the full identifier only for data frames.
pub fn data_id(frame: &Frame) -> Option<Id> {
    (!embedded_can::Frame::is_remote_frame(frame)).then_some(*frame.id())
}
