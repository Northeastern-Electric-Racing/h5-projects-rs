use cangen::{CanMessage, Msg};
use core::fmt::Debug;
use defmt::warn;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::{CanRx, CanTx, Frame};
use embassy_stm32::{Peri, bind_interrupts, can, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

const CAN_RECV_TIMEOUT: Duration = Duration::from_millis(500);
pub const GRACE_PERIOD_DURATION: Duration = Duration::from_secs(5); // Needs to be public such that state can read it

use crate::state::state_machine;
use embassy_time::{Duration, Instant, Timer};
use ner_can::{ExtFilter, NerCan, StdFilter};

bind_interrupts!(struct IrqsCan {
    FDCAN2_IT0 => can::IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<peripherals::FDCAN2>;
});

#[derive(Debug, PartialEq, defmt::Format)]
pub enum FaultframeState {
    BMSFault,
    BMSOk,
    IMDFault,
    IMDOk,
    ResetRequested,
}

/// Capacity of each CAN channel, in `Frame`s.
pub const CHANNEL_SIZE: usize = 256;

/// Frames received off the bus, waiting to be parsed by the rest of the program.
pub static INCOMING: Channel<ThreadModeRawMutex, Frame, CHANNEL_SIZE> = Channel::new();
/// Frames queued by the program, waiting to go out on the bus.
pub static OUTGOING: Channel<ThreadModeRawMutex, Frame, CHANNEL_SIZE> = Channel::new();

/// Peripherals the CAN bus is wired to.
pub struct CanPins {
    pub can: Peri<'static, peripherals::FDCAN2>,
    pub rx: Peri<'static, peripherals::PB5>,
    pub tx: Peri<'static, peripherals::PB6>,
}
pub fn init(pins: CanPins) -> (CanTx<'static>, CanRx<'static>) {
    let configurator = can::CanConfigurator::new(pins.can, pins.rx, pins.tx, IrqsCan);

    let can = NerCan::init(configurator)
        .with_standard_filters(&[
            StdFilter {
                slot: can::filter::StandardFilterSlot::_0,
                id1: cangen::ResetLatchingFault::ID,
                id2: None,
            },
            StdFilter {
                slot: can::filter::StandardFilterSlot::_1,
                id1: cangen::ImdGeneralInformation::ID,
                id2: None,
            },
        ])
        .with_extended_filters(&[ExtFilter {
            slot: can::filter::ExtendedFilterSlot::_2,
            id1: cangen::BmsCriticallyFaulted::ID,
            id2: None,
        }]);
    let (tx, rx, _props) = can.start();

    (tx, rx)
}

#[embassy_executor::task]
pub async fn can_inbox_task() -> ! {
    let boot_time: Instant = Instant::now();

    loop {
        let either: Either<Frame, ()> =
            select(INCOMING.receive(), Timer::after(CAN_RECV_TIMEOUT)).await;
        match either {
            Either::First(frame) => {
                let latest: Option<FaultframeState> = match Msg::decode(&frame) {
                    Some(Msg::ImdGeneralInformation(m)) => {
                        if m.device_error() || m.HV_pos_conn_fail() {
                            Some(FaultframeState::IMDFault)
                        } else {
                            Some(FaultframeState::IMDOk)
                        }
                    }
                    Some(Msg::BmsCriticallyFaulted(m)) => {
                        if m.critically_faulted() {
                            Some(FaultframeState::BMSFault)
                        } else {
                            Some(FaultframeState::BMSOk)
                        }
                    }
                    Some(Msg::ResetLatchingFault(m)) => {
                        if m.reset_latching() {
                            Some(FaultframeState::ResetRequested)
                        } else {
                            None
                        }
                    }
                    Some(_) => {
                        warn!("Unknown ID. Somthing is wrong with the filters. ");
                        None
                    }
                    None => {
                        warn!("Did not receive CAN Frame.");
                        None
                    }
                };

                if let Some(f) = latest
                    && (Instant::now() - boot_time) > GRACE_PERIOD_DURATION
                {
                    state_machine::FAULT_QUEUE.send(f).await;
                }
            }
            Either::Second(_) => {}
        }
    }
}
