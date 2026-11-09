//! Transport bootloader command and data frames over classical CAN.

use can_bootloader_core::{BootloaderCanIds, Frame, Transport};
use can_handler::{ExtFilter, NerCan, StdFilter};
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::filter::{ExtendedFilterSlot, StandardFilterSlot};
use embassy_stm32::can::{Can, CanConfigurator, IT0InterruptHandler, IT1InterruptHandler};
use embedded_can::Id;

use crate::ecu::CAN_LINE_BITRATE;

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
pub struct CanHandler {
    can: Can<'static>,
}

impl CanHandler {
    /// Start the selected controller with only bootloader receive IDs enabled.
    pub fn new(
        fdcan: embassy_stm32::Peri<'static, CanPeripheral>,
        rx: embassy_stm32::Peri<'static, CanRxPin>,
        tx: embassy_stm32::Peri<'static, CanTxPin>,
        ids: BootloaderCanIds,
    ) -> Self {
        let mut ner_can = NerCan::init(CanConfigurator::new(fdcan, rx, tx, Irqs));
        for (id, standard_slot, extended_slot) in [
            (ids.request, StandardFilterSlot::_0, ExtendedFilterSlot::_0),
            (ids.data, StandardFilterSlot::_1, ExtendedFilterSlot::_1),
        ] {
            ner_can = match id {
                Id::Standard(_) => ner_can.with_standard_filters(&[StdFilter {
                    slot: standard_slot,
                    id1: id,
                    id2: None,
                }]),
                Id::Extended(_) => ner_can.with_extended_filters(&[ExtFilter {
                    slot: extended_slot,
                    id1: id,
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

    /// Stop CAN and clear its interrupt sources before application handoff.
    pub fn shutdown(self) {
        let _config = self.can.into_config_mode();
        let registers = CAN_REGS;
        registers.ie().write(|w| w.0 = 0);
        registers.ile().write(|w| w.0 = 0);
        // Interrupt flags are cleared by writing ones.
        registers.ir().write(|w| w.0 = u32::MAX);
    }
}

impl Transport for CanHandler {
    async fn receive(&mut self) -> Frame {
        loop {
            // A bus error is not a receive timeout.
            let Ok(envelope) = self.can.read().await else {
                continue;
            };
            let frame = envelope.frame;
            if embedded_can::Frame::is_remote_frame(&frame) {
                continue;
            }
            if let Some(frame) = Frame::new(*frame.id(), frame.data()) {
                return frame;
            }
        }
    }

    async fn send(&mut self, frame: &Frame) -> bool {
        let Ok(frame) = embassy_stm32::can::Frame::new_data(frame.id(), frame.data()) else {
            return false;
        };

        // Embassy returns a displaced queued frame when the mailbox was replaced.
        self.can.write(&frame).await.is_none()
    }

    async fn flush(&mut self) {
        // Wait on each of the three TX buffers using the TX-complete interrupt.
        for idx in 0..3 {
            self.can.flush(idx).await;
        }
    }
}
