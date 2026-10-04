//! Write and validate staged firmware without modifying the active application.

use crc::{CRC_32_MPEG_2, Crc};
use embassy_boot_stm32::BlockingFirmwareUpdater;
use embassy_stm32::flash::WRITE_SIZE;
use embedded_storage::nor_flash::NorFlash;

use crate::app;
use crate::config::*;

/// Stage a validated block of at most MAX_WRITE bytes at a DFU-relative offset.
pub fn write<DFU, STATE>(
    updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
    offset: u32,
    data: &[u8],
) -> bool
where
    DFU: NorFlash,
    STATE: NorFlash,
{
    if offset >= app_size() || data.is_empty() || offset + data.len() as u32 > app_size() {
        return false;
    }

    // Pad the final flash word with erased bytes; the CRC excludes this padding.
    let padded_len = (data.len() + WRITE_SIZE - 1) & !(WRITE_SIZE - 1);
    let mut padded = [0xFFu8; MAX_WRITE + WRITE_SIZE]; // Match the erased flash value.
    padded[..data.len()].copy_from_slice(data);

    // Embassy erases DFU sectors as needed and enforces the persisted boot state.
    updater
        .write_firmware(offset as usize, &padded[..padded_len])
        .is_ok()
}

/// Compute CRC-32/MPEG-2 over exactly the declared image length.
pub fn crc<DFU, STATE>(
    updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
    image_size: u32,
) -> Option<u32>
where
    DFU: NorFlash,
    STATE: NorFlash,
{
    if image_size == 0 || image_size > app_size() {
        return None;
    }

    let crc = Crc::<u32>::new(&CRC_32_MPEG_2);
    let mut digest = crc.digest();
    let mut buffer = [0u8; 256];
    let mut offset = 0u32;

    while offset < image_size {
        let count = core::cmp::min(buffer.len() as u32, image_size - offset) as usize;
        // Align the flash read while hashing only actual image bytes.
        let read_len = (count + WRITE_SIZE - 1) & !(WRITE_SIZE - 1);

        if updater.read_dfu(offset, &mut buffer[..read_len]).is_err() {
            return None;
        }

        digest.update(&buffer[..count]); // Ignore alignment bytes beyond image_size.
        offset += count as u32;
    }

    Some(digest.finalize())
}

/// Require the staged vectors to target SRAM and the final active partition.
pub fn image_valid<DFU, STATE>(updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>) -> bool
where
    DFU: NorFlash,
    STATE: NorFlash,
{
    // Read one flash programming unit to obtain the first two vector entries.
    let mut vector = [0u8; 16];
    if updater.read_dfu(0, &mut vector).is_err() {
        return false;
    }

    let first = [
        vector[0], vector[1], vector[2], vector[3], vector[4], vector[5], vector[6], vector[7],
    ];

    app::vector_valid(&first)
}
