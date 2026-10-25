use core::sync::atomic::{AtomicBool, Ordering};
use cangen::ToCanFrame;
use embassy_stm32::gpio::{Input,Pull};
use embassy_time::Timer;
use embassy_stm32::{Peri, peripherals};

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

pub fn get_shutdown_state() -> BmsShutdownState {
    if cfg!(feature = "TSMS_OVERIDE") {  
        return BmsShutdownState::Closed; 
    } else {  
        if BMS_SHUTDOWN.load(Ordering::Relaxed){
            return BmsShutdownState::Shutdown
        }
        else{
            return BmsShutdownState::Closed
        }
    } 
}

pub fn is_shutdown_closed() -> bool {
    if cfg!(feature = "TSMS_OVERIDE") {  
        return true; 
    } else {  
        return !BMS_SHUTDOWN.load(Ordering::Relaxed);
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
    pub tsms: Peri<'static, peripherals::PG9>
}

#[embassy_executor::task]
pub async fn report_shutdown_task(pins: ShutdownInputs) {
    let bms = Input::new(pins.bms, Pull::Up);
    let bots = Input::new(pins.bots, Pull::Up);
    let bspd = Input::new(pins.bspd, Pull::Up);
    let hv_c = Input::new(pins.hv_c, Pull::Up);
    let hvd = Input::new(pins.hvd, Pull::Up);
    let imd = Input::new(pins.imd, Pull::Up);
    let ckpt = Input::new(pins.ckpt, Pull::Up);
    let inertia_sw = Input::new(pins.inertia_sw, Pull::Up);
    let tsms = Input::new(pins.tsms, Pull::Up);

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