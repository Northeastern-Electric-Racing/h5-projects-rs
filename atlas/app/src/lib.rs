#![no_std]

//! Confirm application startup and handle CAN requests to enter the bootloader.

use cortex_m::peripheral::SCB;
use embedded_can::{Frame, Id, StandardId};
use embedded_storage::nor_flash::NorFlash;

pub use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterError};

/// A standard CAN data frame that requests bootloader entry.
pub struct BootRequest {
    id: StandardId,
    payload: [u8; 8],
    len: usize,
}

impl BootRequest {
    /// Load a request generated from ecus.json; unknown names return None.
    /// Names match the JSON keys exactly, such as "bms" or "vcu".
    pub fn for_ecu(name: &str) -> Option<Self> {
        include!(concat!(env!("OUT_DIR"), "/ecu_requests.rs"))
    }

    /// Return the identifier to allow through the application's CAN filter.
    pub fn id(&self) -> StandardId {
        self.id
    }

    /// Use the selected ECU's application boot request ID and payload.
    /// Returns None unless the payload contains 1 through 8 bytes.
    pub fn new(id: StandardId, data: &[u8]) -> Option<Self> {
        if !(1..=8).contains(&data.len()) {
            return None;
        }
        let mut payload = [0; 8];
        payload[..data.len()].copy_from_slice(data);
        Some(Self {
            id,
            payload,
            len: data.len(),
        })
    }

    /// Match the frame type, identifier, and entire payload.
    pub fn matches(&self, frame: &impl Frame) -> bool {
        !frame.is_remote_frame()
            && frame.id() == Id::Standard(self.id)
            && frame.data() == &self.payload[..self.len]
    }
}

/// Own boot state access while the application retains its CAN controller.
pub struct Bootloader<'d, STATE> {
    state: BlockingFirmwareState<'d, STATE>,
    request: BootRequest,
}

impl<'d, STATE: NorFlash> Bootloader<'d, STATE> {
    /// Supply only the boot state partition, never the entire flash bank.
    pub fn new(state: BlockingFirmwareState<'d, STATE>, request: BootRequest) -> Self {
        Self { state, request }
    }

    /// Confirm startup after application checks pass to prevent rollback.
    pub fn confirm_boot(&mut self) -> Result<(), FirmwareUpdaterError> {
        self.state.mark_booted()
    }

    /// Ignore unrelated frames; persist a matching request and reset.
    /// Call from the receive task, not an ISR, because this writes flash.
    /// A flash error is returned without resetting the processor.
    pub fn handle_frame(&mut self, frame: &impl Frame) -> Result<(), FirmwareUpdaterError> {
        if !self.request.matches(frame) {
            return Ok(());
        }

        self.state.mark_dfu()?;
        // Reset only after the next boot mode has been persisted successfully.
        // The host confirms entry by contacting the bootloader after reset.
        SCB::sys_reset()
    }
}
