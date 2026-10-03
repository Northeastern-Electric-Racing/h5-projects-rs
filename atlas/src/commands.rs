//! Command opcodes, response markers, and CAN IDs for the bootloader protocol.

// The first request byte selects a command; remaining bytes are its arguments.
pub const GET_INFO: u8 = 0xA1;
pub const GET_STATUS: u8 = 0xA2;
pub const START_APP: u8 = 0xA4;
pub const WRITE_MEMORY: u8 = 0xA5;
pub const ACTIVATE: u8 = 0xA7;
pub const COMPUTE_CRC: u8 = 0xA8;
pub const START_UPDATE: u8 = 0xA9;
pub const SET_BAUD_RATE: u8 = 0xAA;

// Responses prefix the command opcode with ACK or NACK.
pub const ACK: u8 = 0x5A;
pub const NACK: u8 = 0xB5;

// Selected ECU IDs and initial bitrate generated from ecus.json.
include!(concat!(env!("OUT_DIR"), "/ecu_can.rs"));

/// Translate the protocol's one-byte bitrate code into bits per second.
pub fn baud_rate(code: u8) -> Option<u32> {
    match code {
        0x00 => Some(125_000),
        0x01 => Some(250_000),
        0x02 => Some(500_000),
        0x03 => Some(1_000_000),
        _ => None,
    }
}
