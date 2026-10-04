//! Implement host commands, packet acknowledgements, and staged-image activation.

use embassy_boot_stm32::BlockingFirmwareUpdater;
use embassy_stm32::can::frame::Frame;
use embassy_stm32::flash::WRITE_SIZE;
use embedded_storage::nor_flash::NorFlash;

use crate::can::{CanHandler, standard_id};
use crate::commands::*;
use crate::config::*;
use crate::flash;
use crate::types::{Context, Control, UpdateState};

// Request lengths include the opcode byte.
const WRITE_REQUEST_LEN: usize = 7;
const START_UPDATE_LEN: usize = 8;
const SET_BAUD_LEN: usize = 2;

// Remember the last committed block so a lost final ACK does not cause a rewrite.
#[derive(Clone, Copy)]
struct CompletedWrite {
    valid: bool,
    offset: u32,
    count: u32,
}

impl CompletedWrite {
    const fn new() -> Self {
        Self {
            valid: false,
            offset: 0,
            count: 0,
        }
    }
}

/// Keep retry bookkeeping across commands within one bootloader session.
pub struct Handlers {
    completed: CompletedWrite,
}

impl Handlers {
    /// Start with no completed block available for duplicate detection.
    pub const fn new() -> Self {
        Self {
            completed: CompletedWrite::new(),
        }
    }

    /// ACK the ping, then advertise the version and supported command opcodes.
    pub async fn get_info(&mut self, can: &mut CanHandler<'_>) {
        // Split the capability list across two classical CAN frames.
        let first = [
            GET_INFO,
            VERSION,
            COMMAND_COUNT,
            GET_INFO,
            GET_STATUS,
            START_APP,
            WRITE_MEMORY,
            ACTIVATE,
        ];
        let second = [GET_INFO, COMPUTE_CRC, START_UPDATE, SET_BAUD_RATE];

        if can.ack(GET_INFO).await {
            let _ = can.send(RESPONSE_ID, &first).await;
            let _ = can.send(RESPONSE_ID, &second).await;
        }
    }

    /// Return version and session state using the existing GET_STATUS wire format.
    pub async fn get_status(&mut self, can: &mut CanHandler<'_>, context: &Context) {
        let response = [GET_STATUS, VERSION, context.state.code()];
        if can.ack(GET_STATUS).await {
            let _ = can.send(RESPONSE_ID, &response).await;
        }
    }

    /// Accept image metadata before any DFU blocks are written.
    pub async fn start_update(
        &mut self,
        frame: &Frame,
        can: &mut CanHandler<'_>,
        context: &mut Context,
    ) {
        let data = frame.data();
        if data.len() != START_UPDATE_LEN {
            let _ = can.nack(START_UPDATE).await;
            return;
        }

        // Payload: opcode, 24-bit image length, then 32-bit CRC, all big-endian.
        let image_size = u24(&data[1..4]);
        let expected_crc = u32be(&data[4..8]);

        if image_size == 0 || image_size > app_size() {
            let _ = can.nack(START_UPDATE).await;
            return;
        }

        context.start(image_size, expected_crc); // A new transfer invalidates verification.
        self.completed = CompletedWrite::new();
        let _ = can.ack(START_UPDATE).await;
    }

    /// Receive sequenced packets, commit one DFU block, then ACK completion.
    pub async fn write_memory<DFU, STATE>(
        &mut self,
        frame: &Frame,
        can: &mut CanHandler<'_>,
        updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
        context: &mut Context,
    ) where
        DFU: NorFlash,
        STATE: NorFlash,
    {
        let data = frame.data();
        if data.len() != WRITE_REQUEST_LEN || context.state == UpdateState::Idle {
            let _ = can.nack(WRITE_MEMORY).await;
            return;
        }

        // Payload: opcode, 32-bit DFU offset, then 16-bit byte count.
        let offset = u32be(&data[1..5]);
        let count = u16be(&data[5..7]) as u32;

        // A repeated completed request needs an ACK, not another flash write.
        if self.completed.valid && self.completed.offset == offset && self.completed.count == count
        {
            let _ = can.ack(WRITE_MEMORY).await;
            return;
        }

        if !write_valid(context, offset, count) {
            let _ = can.nack(WRITE_MEMORY).await;
            return;
        }

        self.completed = CompletedWrite::new(); // Retain retry state for only one block.
        if !can.ack(WRITE_MEMORY).await {
            return;
        }

        let mut buffer = [0u8; MAX_WRITE]; // Stage the whole block before touching flash.
        let mut received = 0usize;
        let mut expected_sequence = 0u8;

        // Sequence numbers restart at zero for each block; each packet gets an ACK.
        while received < count as usize {
            let Some(frame) = can.recv_timeout(WRITE_TIMEOUT_MS).await else {
                let _ = can.nack(WRITE_MEMORY).await;
                return;
            };

            let id = standard_id(&frame);
            let payload = frame.data();

            if id == Some(DATA_ID) && payload.len() >= 2 {
                // DATA_ID carries one sequence byte and up to seven image bytes.
                let sequence = payload[0];
                let available = payload.len() - 1;
                let remaining = count as usize - received; // Reject packets past this block.

                if sequence == expected_sequence && available <= remaining {
                    buffer[received..received + available].copy_from_slice(&payload[1..]);
                    received += available; // Duplicate packets must not advance this count.
                    expected_sequence = expected_sequence.wrapping_add(1);

                    if !can.data_ack(sequence).await {
                        return;
                    }
                } else if received > 0 && sequence == expected_sequence.wrapping_sub(1) {
                    // Re-ACK a frame whose previous ACK was lost.
                    if !can.data_ack(sequence).await {
                        return;
                    }
                } else {
                    // Out-of-order or oversized packets invalidate this block transfer.
                    let _ = can.nack(WRITE_MEMORY).await;
                    return;
                }
            } else if received == 0
                && id == Some(REQUEST_ID)
                && payload.len() == WRITE_REQUEST_LEN
                && payload[0] == WRITE_MEMORY
            {
                let retry_offset = u32be(&payload[1..5]);
                let retry_count = u16be(&payload[5..7]) as u32;

                if retry_offset == offset && retry_count == count {
                    // Re-ACK WRITE_MEMORY if the first ACK was lost.
                    if !can.ack(WRITE_MEMORY).await {
                        return;
                    }
                } else {
                    let _ = can.nack(WRITE_MEMORY).await;
                    return;
                }
            }
        }

        // Packet ACKs confirm receipt; the final command ACK confirms persistence.
        if !flash::write(updater, offset, &buffer[..count as usize]) {
            let _ = can.nack(WRITE_MEMORY).await;
            return;
        }

        context.next_offset += count; // Advance only after the flash write succeeds.
        context.state = UpdateState::Updating;
        self.completed = CompletedWrite {
            valid: true,
            offset,
            count,
        };
        let _ = can.ack(WRITE_MEMORY).await;
    }

    /// Verify the complete staged image before allowing activation.
    pub async fn compute_crc<DFU, STATE>(
        &mut self,
        can: &mut CanHandler<'_>,
        updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
        context: &mut Context,
    ) where
        DFU: NorFlash,
        STATE: NorFlash,
    {
        // A CRC request cannot verify an unannounced or partially received image.
        if context.state == UpdateState::Idle || context.next_offset != context.image_size {
            let _ = can.nack(COMPUTE_CRC).await;
            return;
        }

        let Some(value) = flash::crc(updater, context.image_size) else {
            let _ = can.nack(COMPUTE_CRC).await;
            return;
        };

        if value != context.expected_crc || !flash::image_valid(updater) {
            context.state = UpdateState::Updating; // Revoke any earlier verification.
            let _ = can.nack(COMPUTE_CRC).await;
            return;
        }

        context.state = UpdateState::Verified; // ACTIVATE is permitted from this point.
        let bytes = value.to_be_bytes();
        let response = [COMPUTE_CRC, bytes[0], bytes[1], bytes[2], bytes[3]];

        if can.ack(COMPUTE_CRC).await {
            let _ = can.send(RESPONSE_ID, &response).await;
        }
    }

    /// Persist the swap request and reset only after transmitting the ACK.
    pub async fn activate<DFU, STATE>(
        &mut self,
        can: &mut CanHandler<'_>,
        updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
        context: &Context,
    ) -> Control
    where
        DFU: NorFlash,
        STATE: NorFlash,
    {
        if context.state != UpdateState::Verified {
            let _ = can.nack(ACTIVATE).await;
            return Control::Continue;
        }

        // BootLoader::prepare performs the swap on the next boot.
        if updater.mark_updated().is_err() {
            let _ = can.nack(ACTIVATE).await;
            return Control::Continue;
        }

        // Resetting while the ACK is queued would leave the host unsure of success.
        if can.ack(ACTIVATE).await && can.flush().await {
            Control::Reset
        } else {
            Control::Continue
        }
    }

    /// ACK and launch the current active image without activating staged firmware.
    pub async fn start_app(&mut self, can: &mut CanHandler<'_>) -> Control {
        if !crate::app::active_valid() {
            let _ = can.nack(START_APP).await;
            return Control::Continue;
        }

        if can.ack(START_APP).await && can.flush().await {
            Control::StartApp
        } else {
            Control::Continue
        }
    }

    /// ACK at the current rate before applying the requested CAN bitrate.
    pub async fn set_baud(&mut self, frame: &Frame, can: &mut CanHandler<'_>) {
        let data = frame.data();
        if data.len() != SET_BAUD_LEN || baud_rate(data[1]).is_none() {
            let _ = can.nack(SET_BAUD_RATE).await;
            return;
        }

        // The host must receive this response before either side changes bitrate.
        if can.ack(SET_BAUD_RATE).await {
            let _ = can.set_baud(data[1]).await;
        }
    }
}

// Accept sequential, bounded blocks with aligned starts and final-block padding.
fn write_valid(context: &Context, offset: u32, count: u32) -> bool {
    // Disallow holes and rewrites; the duplicate-block path handles lost ACKs.
    if count == 0 || count as usize > MAX_WRITE || offset != context.next_offset {
        return false;
    }

    if !offset.is_multiple_of(WRITE_SIZE as u32) || offset + count > context.image_size {
        return false;
    }

    // Only the final block may need padding to WRITE_SIZE.
    offset + count == context.image_size || count.is_multiple_of(WRITE_SIZE as u32)
}

// Callers validate request lengths before decoding these big-endian fields.
fn u32be(data: &[u8]) -> u32 {
    ((data[0] as u32) << 24) | ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32
}

fn u24(data: &[u8]) -> u32 {
    ((data[0] as u32) << 16) | ((data[1] as u32) << 8) | data[2] as u32
}

fn u16be(data: &[u8]) -> u16 {
    ((data[0] as u16) << 8) | data[1] as u16
}
