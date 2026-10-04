//! Application CAN setup and explicit boot-entry settings.

use atlas_app::{Bootloader, FirmwareUpdaterError};
use embassy_executor::Spawner;
use embassy_stm32::can::config::{FdCanConfig, GlobalFilter};
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{Can, CanConfigurator, IT0InterruptHandler, IT1InterruptHandler};
use embassy_stm32::peripherals::{FDCAN2, FLASH, PB13, PD9};
use embassy_stm32::{Peri, bind_interrupts};
use embedded_can::StandardId;

// Match these settings to the host configuration for this ECU.
const REQUEST_ID: u16 = 0x013;
const REQUEST_DATA: &[u8] = &[0xB0, 0x07, 0x10, 0xAD];
const BIT_RATE: u32 = 500_000;

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => IT1InterruptHandler<FDCAN2>;
});

pub fn start(
    spawner: Spawner,
    flash: Peri<'static, FLASH>,
    fdcan: Peri<'static, FDCAN2>,
    rx: Peri<'static, PD9>,
    tx: Peri<'static, PB13>,
) -> Result<(), FirmwareUpdaterError> {
    let mut config = CanConfigurator::new(fdcan, rx, tx, Irqs);
    config.set_config(FdCanConfig::default().set_global_filter(GlobalFilter::reject_all()));
    config.set_bitrate(BIT_RATE);
    config.properties().set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter {
            filter: FilterType::DedicatedSingle(StandardId::new(REQUEST_ID).unwrap()),
            action: Action::StoreInFifo0,
        },
    );
    let can = config.into_normal_mode();

    let mut bootloader = Bootloader::new(flash, REQUEST_ID, REQUEST_DATA);
    bootloader.confirm_boot()?;
    spawner.spawn(receive(can, bootloader).expect("Failed to spawn bootloader receiver"));
    Ok(())
}

#[embassy_executor::task]
async fn receive(mut can: Can<'static>, mut bootloader: Bootloader<'static>) {
    loop {
        let Ok(envelope) = can.read().await else {
            continue;
        };
        if bootloader.handle_frame(&envelope.frame).is_err() {
            defmt::error!("Failed to persist bootloader request");
        }
    }
}
