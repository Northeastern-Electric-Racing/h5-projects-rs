#![no_std]

//! Application startup confirmation and CAN bootloader entry for STM32H563ZI.
//! CAN initialization, receive filters, and frame dispatch stay with the application.

use core::cell::RefCell;

use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState};
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_stm32::Peri;
use embassy_stm32::flash::{Blocking, Flash, WRITE_SIZE};
use embassy_stm32::peripherals::FLASH;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_can::{Frame, Id, StandardId};

pub use embassy_boot_stm32::FirmwareUpdaterError;

// Match BOOTLOADER_STATE in atlas/memory.x; offsets here are relative to all flash.
const STATE_OFFSET: u32 = 0x001F_A000;
const STATE_SIZE: u32 = 0x2000;

/// Own flash access for startup confirmation and persistent bootloader requests.
pub struct Bootloader<'d> {
    flash: Mutex<NoopRawMutex, RefCell<Flash<'d, Blocking>>>,
    request_id: StandardId,
    request_data: &'static [u8],
}

impl<'d> Bootloader<'d> {
    /// Configure an exact standard CAN data-frame match without initializing CAN.
    /// Panics for an invalid 11-bit ID or a payload outside 1..=8 bytes.
    pub fn new(flash: Peri<'d, FLASH>, request_id: u16, request_data: &'static [u8]) -> Self {
        let request_id = StandardId::new(request_id).expect("Boot request ID must be 11-bit");
        assert!(!request_data.is_empty() && request_data.len() <= 8);

        Self {
            flash: Mutex::new(RefCell::new(Flash::new_blocking(flash))),
            request_id,
            request_data,
        }
    }

    /// Confirm successful application initialization to prevent rollback on reset.
    /// Call only after the application's required startup checks have succeeded.
    pub fn confirm_boot(&mut self) -> Result<(), FirmwareUpdaterError> {
        self.write_state(false)
    }

    /// Check the ID, frame type, and complete payload before accepting a request.
    pub fn is_request(&self, frame: &impl Frame) -> bool {
        !frame.is_remote_frame()
            && frame.id() == Id::Standard(self.request_id)
            && frame.data() == self.request_data
    }

    /// Ignore unrelated frames; persist a matching request and reset into Atlas.
    /// Returns an error without resetting if the flash operation fails.
    /// Call from a task, after any application-specific shutdown needed before reset.
    pub fn handle_frame(&mut self, frame: &impl Frame) -> Result<(), FirmwareUpdaterError> {
        if self.is_request(frame) {
            self.write_state(true)?;
            cortex_m::peripheral::SCB::sys_reset();
        }
        Ok(())
    }

    // Construct the short-lived partition and aligned buffer without self-references.
    fn write_state(&mut self, detach: bool) -> Result<(), FirmwareUpdaterError> {
        let partition = BlockingPartition::new(&self.flash, STATE_OFFSET, STATE_SIZE);
        let mut aligned = AlignedBuffer([0; WRITE_SIZE]);
        let mut state = BlockingFirmwareState::new(partition, aligned.as_mut());
        if detach {
            state.mark_dfu()
        } else {
            state.mark_booted()
        }
    }
}
