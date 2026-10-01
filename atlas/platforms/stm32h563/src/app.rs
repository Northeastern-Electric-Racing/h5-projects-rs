//! Check application vectors before transferring execution to active flash.

use crate::config::*;

/// Read the active image after boot preparation has completed any swap.
pub fn active_valid() -> bool {
    // APP_START maps readable active flash, untouched by CAN DFU writes.
    let vector = unsafe {
        let sp = core::ptr::read_volatile(APP_START as *const u32);
        let reset = core::ptr::read_volatile((APP_START + 4) as *const u32);
        let mut vector = [0; 8];
        vector[..4].copy_from_slice(&sp.to_le_bytes());
        vector[4..].copy_from_slice(&reset.to_le_bytes());
        vector
    };
    vector_valid(&vector)
}

/// Check stack and Thumb entry addresses; this does not verify image integrity.
pub fn vector_valid(vector: &[u8; 8]) -> bool {
    let sp = u32::from_le_bytes([vector[0], vector[1], vector[2], vector[3]]);
    let reset = u32::from_le_bytes([vector[4], vector[5], vector[6], vector[7]]);
    // Bit zero selects Thumb execution and is not part of the code address.
    let reset_addr = reset & !1;

    // A descending stack may start one byte past the last SRAM address.
    (SRAM_START..=SRAM_END).contains(&sp)
        && (reset & 1) != 0
        && (APP_START..APP_START + APP_SIZE).contains(&reset_addr)
}
