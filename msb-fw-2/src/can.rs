use embassy_stm32::can::config::{FdCanConfig, GlobalFilter};
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{Can, CanConfigurator, IT0InterruptHandler, IT1InterruptHandler};
use embassy_stm32::peripherals::{FDCAN2, PB5, PB6};
use embassy_stm32::{Peri, bind_interrupts};
use embedded_can::StandardId;

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => IT1InterruptHandler<FDCAN2>;
});

pub(super) fn init(
    peripheral: Peri<'static, FDCAN2>,
    rx: Peri<'static, PB5>,
    tx: Peri<'static, PB6>,
) -> Can<'static> {
    let mut can = CanConfigurator::new(peripheral, rx, tx, Irqs);
    can.set_config(
        FdCanConfig::default()
            .set_automatic_bus_off_recovery(true)
            .set_global_filter(GlobalFilter::reject_all()),
    );
    can.set_bitrate(500_000);
    can.properties().set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter {
            filter: FilterType::DedicatedSingle(StandardId::new(crate::BOOT_REQUEST_ID).unwrap()),
            action: Action::StoreInFifo0,
        },
    );
    can.into_normal_mode()
}

#[embassy_executor::task]
pub(super) async fn receive(
    mut can: Can<'static>,
    mut bootloader: boot_control::Bootloader<'static>,
) {
    loop {
        match can.read().await {
            Ok(envelope) => {
                if bootloader.handle_frame(&envelope.frame).is_err() {
                    defmt::error!("Failed to persist bootloader request");
                }
            }
            Err(error) => defmt::warn!("CAN receive error: {}", error),
        }
    }
}
