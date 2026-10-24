use core::sync::atomic::{AtomicBool, Ordering};
use cangen::ToCanFrame;
use embassy_stm32::gpio::Input;
use crate::can;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BmsShutdownState {
    Shutdown,
    Closed
}

static BMS_SHUTDOWN: AtomicBool = AtomicBool::new(true);

pub fn update_bms_shutdown(state: BmsShutdownState) {
    BMS_SHUTDOWN.store(
        state == BmsShutdownState::Shutdown,
        Ordering::Relaxed,
    );
}


pub fn is_shutdown_closed() -> bool {
    #[cfg(feature = "TSMS_OVERIDE")]
    {
        return true;
    }

    #[cfg(not(feature = "TSMS_OVERIDE"))]
    {
        !BMS_SHUTDOWN.load(Ordering::Relaxed)
    }
}

pub struct ShutdownInputs {
    pub bms: Peri<'static, peripherals::PF6>,
    pub bots: Peri<'static, peripherals::PF7>,
    pub bspd: Peri<'static, peripherals::PF9>,
    pub hv_c: Peri<'static, peripherals::PF10>,
    pub hvd: Peri<'static, peripherals::PF15>,
    pub imd: Peri<'static, peripherals::PE0>,
    pub ckpt: Peri<'static, peripherals::PG2>,
    pub inertia_sw: Peri<'static, peripherals::PG3>,
    pub tstm: Peri<'static, peripherals::PG9>
}

#[embassy_executor::task]
pub async fn report_shutdown_task(pins: ShutdownInputs) {

    loop {
        let shutdown_pins = cangen::ShutdownPins::new()
            .with_bms_gpio(pins.bms.is_high())
            .with_bots_gpio(pins.bots.is_high())
            .with_bspd_gpio(pins.bspd.is_high())
            .with_hv_c(pins.hv_c.is_high())
            .with_hvd_gpio(pins.hvd.is_high())
            .with_imd_gpio(pins.imd.is_high())
            .with_ckpt_gpio(pins.ckpt.is_high())
            .with_inertia_sw_gpio(pins.inertia_sw.is_high())
            .with_tsms_gpio(pins.tsms.is_high());

        let bms_shutdown = BMS_SHUTDOWN.load(Ordering::Relaxed);
        let bms_status = cangen::BmsShutdownStatusAsReportedByVcu::new()
            .with_bms_shutdown_as_reported_by_vcu(bms_shutdown);

        can::OUTGOING.send(shutdown_pins.to_can_frame()).await;
        can::OUTGOING.send(bms_status.to_can_frame()).await;

        Timer::after_millis(100).await;
    }
}