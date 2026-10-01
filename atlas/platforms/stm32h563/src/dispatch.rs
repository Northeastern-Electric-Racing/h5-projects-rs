//! Route command opcodes to handlers and propagate reset or startup decisions.

use embassy_boot_stm32::BlockingFirmwareUpdater;
use embassy_stm32::can::frame::Frame;
use embedded_storage::nor_flash::NorFlash;

use crate::can::{CanHandler, standard_id};
use crate::commands::*;
use crate::handlers::Handlers;
use crate::types::{Context, Control};

/// Dispatch one request frame; data packets are consumed inside WRITE_MEMORY.
pub async fn run<DFU, STATE>(
    frame: &Frame,
    can: &mut CanHandler<'_>,
    updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
    handlers: &mut Handlers,
    context: &mut Context,
) -> Control
where
    DFU: NorFlash,
    STATE: NorFlash,
{
    if standard_id(frame) != Some(REQUEST_ID) || frame.data().is_empty() {
        return Control::Continue;
    }

    match frame.data()[0] {
        GET_INFO => handlers.get_info(can).await,
        GET_STATUS => handlers.get_status(can, context).await,
        START_UPDATE => handlers.start_update(frame, can, context).await,
        WRITE_MEMORY => handlers.write_memory(frame, can, updater, context).await,
        COMPUTE_CRC => handlers.compute_crc(can, updater, context).await,
        ACTIVATE => return handlers.activate(can, updater, context).await,
        START_APP => return handlers.start_app(can).await,
        SET_BAUD_RATE => handlers.set_baud(frame, can).await,
        command => {
            let _ = can.nack(command).await;
        }
    }

    Control::Continue
}
