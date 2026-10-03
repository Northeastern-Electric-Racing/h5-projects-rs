//! Choose application startup or a CAN update session from the persisted boot state.

use embassy_boot_stm32::{BlockingFirmwareUpdater, State};
use embedded_storage::nor_flash::NorFlash;

use crate::app;
use crate::can::CanHandler;
use crate::dispatch;
use crate::handlers::Handlers;
use crate::types::{Context, Control};

/// Serve commands until the host requests application startup or reset.
pub async fn run<DFU, STATE>(
    can: &mut CanHandler<'_>,
    updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
    boot_state: &State,
) -> Control
where
    DFU: NorFlash,
    STATE: NorFlash,
{
    // Only an explicit application request keeps a valid image in bootloader mode.
    if *boot_state != State::DfuDetach && app::active_valid() {
        return Control::StartApp;
    }

    let mut handlers = Handlers::new();
    let mut context = Context::new();

    // Serve updates after a DFU request or recover an invalid active image.
    loop {
        let Some(frame) = can.recv().await else {
            continue;
        };

        let control = dispatch::run(&frame, can, updater, &mut handlers, &mut context).await;
        if !matches!(control, Control::Continue) {
            // The caller owns the final reset or interrupt cleanup and jump.
            return control;
        }
    }
}
