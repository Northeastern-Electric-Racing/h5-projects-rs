use cangen::ToCanFrame;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::{Peri, peripherals};
use embassy_time::Timer;

use crate::can;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BmsShutdownState {
    /// corresponds to false over CAN (bad state)
    Open,
    /// corresponds to true over CAN (normal state)
    Closed,
}

impl From<bool> for BmsShutdownState {
    fn from(s: bool) -> Self {
        match s {
            true => BmsShutdownState::Closed,
            false => BmsShutdownState::Open,
        }
    }
}

/// default state is false (open) until BMS confirms that shutdown is false (closed)
static BMS_SHUTDOWN: AtomicBool = AtomicBool::new(false);

pub async fn update_bms_shutdown(state: BmsShutdownState) {
    BMS_SHUTDOWN.store(state == BmsShutdownState::Closed, Ordering::Relaxed);
}

pub fn get_shutdown_state() -> BmsShutdownState {
    if cfg!(feature = "tsms_overide") {
        BmsShutdownState::Closed
    } else {
        BMS_SHUTDOWN.load(Ordering::Relaxed).into()
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
    pub tsms: Peri<'static, peripherals::PG9>,
}

#[embassy_executor::task]
pub async fn report_shutdown_task(pins: ShutdownInputs) {
    let bms = Input::new(pins.bms, Pull::None);
    let bots = Input::new(pins.bots, Pull::None);
    let bspd = Input::new(pins.bspd, Pull::None);
    let hv_c = Input::new(pins.hv_c, Pull::None);
    let hvd = Input::new(pins.hvd, Pull::None);
    let imd = Input::new(pins.imd, Pull::None);
    let ckpt = Input::new(pins.ckpt, Pull::None);
    let inertia_sw = Input::new(pins.inertia_sw, Pull::None);
    let tsms = Input::new(pins.tsms, Pull::None);

    loop {
        let shutdown_pins = cangen::ShutdownPins::new()
            .with_bms_gpio(bms.is_high())
            .with_bots_gpio(bots.is_high())
            .with_bspd_gpio(bspd.is_high())
            .with_hv_c(hv_c.is_high())
            .with_hvd_gpio(hvd.is_high())
            .with_imd_gpio(imd.is_high())
            .with_ckpt_gpio(ckpt.is_high())
            .with_inertia_sw_gpio(inertia_sw.is_high())
            .with_tsms_gpio(tsms.is_high());

        let bms_shutdown = BMS_SHUTDOWN.load(Ordering::Relaxed) as u8;
        let bms_status = cangen::BmsShutdownStatusAsReportedByVcu::new()
            .with_bms_shutdown_as_reported_by_vcu(bms_shutdown);

        can::OUTGOING.send(shutdown_pins.to_can_frame()).await;
        can::OUTGOING.send(bms_status.to_can_frame()).await;

        Timer::after_millis(100).await;
    }
}
