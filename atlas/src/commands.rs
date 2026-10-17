//! Command opcodes, response markers, and CAN IDs for the bootloader protocol.

// The first request byte selects a command; remaining bytes are its arguments.
pub const GET_INFO: u8 = 0xA1;
pub const GET_STATUS: u8 = 0xA2;
pub const START_APP: u8 = 0xA4;
pub const WRITE_MEMORY: u8 = 0xA5;
pub const ACTIVATE: u8 = 0xA7;
pub const COMPUTE_CRC: u8 = 0xA8;
pub const START_UPDATE: u8 = 0xA9;

// Responses prefix the command opcode with ACK or NACK.
pub const ACK: u8 = 0x5A;
pub const NACK: u8 = 0xB5;

// BOOTLOADER_ECU selects the protocol IDs at build time.
#[cfg(ecu = "bms")]
pub const REQUEST_ID: u16 = 0x010;
#[cfg(ecu = "vcu")]
pub const REQUEST_ID: u16 = 0x014;
#[cfg(ecu = "lightning")]
pub const REQUEST_ID: u16 = 0x018;
#[cfg(ecu = "msb_front")]
pub const REQUEST_ID: u16 = 0x01C;
#[cfg(ecu = "msb_back")]
pub const REQUEST_ID: u16 = 0x020;

// Each ECU reserves consecutive request, response, data, and app-entry IDs.
pub const RESPONSE_ID: u16 = REQUEST_ID + 0x01;
pub const DATA_ID: u16 = REQUEST_ID + 0x02;
pub const CAN_LINE_BITRATE: u32 = 500_000;
