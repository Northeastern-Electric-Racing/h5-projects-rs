//! Firmware layout, protocol limits, and timing constants.

// External oscillator frequency used by the bootloader.
pub const HSE_HZ: u32 = 25_000_000;

// Version packs the major and minor numbers into the high and low nibbles.
pub const VERSION: u8 = 0x10;
// Keep this count consistent with the GET_INFO opcode list.
pub const COMMAND_COUNT: u8 = 8;

unsafe extern "C" {
    // Absolute linker symbols carry values in their addresses, not stored data.
    static __bootloader_active_address: u8;
    static __bootloader_active_size: u8;
    static __bootloader_sram_start: u8;
    static __bootloader_sram_end: u8;
}

pub(super) fn app_start() -> u32 {
    core::ptr::addr_of!(__bootloader_active_address) as u32
}

pub(super) fn app_size() -> u32 {
    core::ptr::addr_of!(__bootloader_active_size) as u32
}

pub(super) fn sram_start() -> u32 {
    core::ptr::addr_of!(__bootloader_sram_start) as u32
}

pub(super) fn sram_end() -> u32 {
    core::ptr::addr_of!(__bootloader_sram_end) as u32
}

// Maximum host transfer block, in bytes.
pub const MAX_WRITE: usize = 256;
// Maximum silence between packets of an in-progress WRITE_MEMORY command.
pub const WRITE_TIMEOUT_MS: u64 = 5000;
// Delay after ACK transmission before applying a new CAN bitrate.
pub const BAUD_DELAY_MS: u64 = 50;
// Bound hardware transmit draining when the bus is unavailable.
pub const TX_TIMEOUT_MS: u64 = 1000;
