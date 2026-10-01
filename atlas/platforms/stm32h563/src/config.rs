//! Firmware layout, protocol limits, and timing constants.

// Version packs the major and minor numbers into the high and low nibbles.
pub const VERSION: u8 = 0x10;
// Keep this count consistent with the GET_INFO opcode list.
pub const COMMAND_COUNT: u8 = 8;

// Generated alongside the linker layout to keep address checks synchronized.
include!(concat!(env!("OUT_DIR"), "/ecu_memory.rs"));

// H563 flash programming granularity and maximum host transfer block, in bytes.
pub const WRITE_SIZE: usize = 16;
pub const MAX_WRITE: usize = 256;
// Maximum silence between packets of an in-progress WRITE_MEMORY command.
pub const WRITE_TIMEOUT_MS: u64 = 5000;
// Delay after ACK transmission before applying a new CAN bitrate.
pub const BAUD_DELAY_MS: u64 = 50;
// Bound hardware transmit draining when the bus is unavailable.
pub const TX_TIMEOUT_MS: u64 = 1000;
