//! RTDS (ready to drive sound)
//!
//! rtds_task owns the pin. everything else just calls `sound_rtds` / `cancel_rtds` which
//! throw a command on the queue. The task just sleeps until a new command or the deadline is reached.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::can;
use crate::shutdown::{self, BmsShutdownState};
use cangen::ToCanFrame;
use defmt::{debug, error};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};

/// how long the rtds sound plays
const RTDS_DURATION: Duration = Duration::from_millis(1500);
/// reverse beeps with a 1s period: 500ms on, 500ms off
const REVERSE_HALF_PERIOD: Duration = Duration::from_millis(500);
/// how often the RtdsStateMessage is sent on CAN
const CAN_SEND_PERIOD: Duration = Duration::from_millis(100);

const COMMAND_QUEUE_SIZE: usize = 4;

#[derive(Clone, Copy, defmt::Format)]
pub(crate) enum RtdsCommand {
    /// play the ready to drive sound once
    SoundRTDS,
    /// stop ready to drive sound
    CancelRTDS,
    /// beep continuously while in reverse
    StartReverse,
    /// stop reverse sound
    StopReverse,
}

impl TryFrom<u8> for RtdsCommand {
    type Error = u8; // the unrecognized byte

    /// map the `command` byte from RtdsCommandMessage
    fn try_from(byte: u8) -> Result<Self, Self::Error> {
        match byte {
            0 => Ok(RtdsCommand::SoundRTDS),
            1 => Ok(RtdsCommand::CancelRTDS),
            2 => Ok(RtdsCommand::StartReverse),
            3 => Ok(RtdsCommand::StopReverse),
            other => Err(other),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
enum RtdsState {
    Idle,
    Drive,
    Reverse,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum RtdsError {
    /// queue was full so the command got dropped
    QueueFull,
}

// TODO: faults aren't ported yet, swap this out once we have faults
fn send_rtds_fault() {
    error!("RTDS_FAULT (placeholder, faults not ported yet).");
}

static COMMANDS: Channel<ThreadModeRawMutex, RtdsCommand, COMMAND_QUEUE_SIZE> = Channel::new();

// copies of the task state so other tasks can check it without needing the pin
static PIN_ON: AtomicBool = AtomicBool::new(false);
static SOUNDING: AtomicBool = AtomicBool::new(false);
static REVERSING: AtomicBool = AtomicBool::new(false);

/// Puts a new command on the queue. Fault gets raised here regardless of if the caller ignores the error
pub(crate) fn send(command: RtdsCommand) -> Result<(), RtdsError> {
    COMMANDS.try_send(command).map_err(|_| {
        error!("RTDS command queue full, dropped {}.", command);
        send_rtds_fault();
        RtdsError::QueueFull
    })
}

pub fn sound_rtds() -> Result<(), RtdsError> {
    send(RtdsCommand::SoundRTDS)
}
pub fn start_reverse_rtds() -> Result<(), RtdsError> {
    send(RtdsCommand::StartReverse)
}
pub fn stop_reverse_rtds() -> Result<(), RtdsError> {
    send(RtdsCommand::StopReverse)
}

/// is the pin high
pub fn is_pin_on() -> bool {
    PIN_ON.load(Ordering::Relaxed)
}

/// is the main rtds sound playing
pub fn is_sounding() -> bool {
    SOUNDING.load(Ordering::Relaxed)
}

/// is the reverse sound active (beeping)
pub fn is_reversing() -> bool {
    REVERSING.load(Ordering::Relaxed)
}

struct Rtds {
    pin: Output<'static>,
    state: RtdsState,
    /// when the current state next needs attention. `Instant::MAX` if never (idle)
    deadline: Instant,
}

impl Rtds {
    fn set_pin(&mut self) {
        // shutdown open = can't drive, so don't let rtds go off
        if shutdown::get_shutdown_state() == BmsShutdownState::Open {
            debug!("RTDS shut down due to shutdown_open.");
            return;
        }

        // only enable on --release builds
        if cfg!(not(debug_assertions)) {
            self.pin.set_high();
        } else {
            debug!("beeeeeeeeeeeeeeeeeeeeeeeeeeeeep");
        }
        PIN_ON.store(true, Ordering::Relaxed);
        debug!("Turned on RTDS pin.");
    }

    fn clear_pin(&mut self) {
        self.pin.set_low();
        PIN_ON.store(false, Ordering::Relaxed);
        debug!("Turned off RTDS pin.");
    }

    fn toggle_pin(&mut self) {
        if is_pin_on() {
            self.clear_pin();
        } else {
            self.set_pin();
        }
    }

    fn set_state(&mut self, state: RtdsState, deadline: Instant) {
        self.state = state;
        self.deadline = deadline;
        SOUNDING.store(state == RtdsState::Drive, Ordering::Relaxed);
        REVERSING.store(state == RtdsState::Reverse, Ordering::Relaxed);
    }

    fn go_idle(&mut self) {
        self.clear_pin();
        self.set_state(RtdsState::Idle, Instant::MAX);
    }

    fn handle_command(&mut self, command: RtdsCommand) {
        debug!("Handling RTDS command: {:?}", command);
        match command {
            RtdsCommand::SoundRTDS => {
                self.set_pin();
                self.set_state(RtdsState::Drive, Instant::now() + RTDS_DURATION);
            }
            RtdsCommand::CancelRTDS => {
                // only stop the rtds sound, leave reverse alone
                if self.state == RtdsState::Drive {
                    self.go_idle();
                }
            }
            RtdsCommand::StartReverse => {
                // already beeping, don't restart the beep pattern
                if self.state != RtdsState::Reverse {
                    self.clear_pin();
                    self.set_pin();
                    self.set_state(RtdsState::Reverse, Instant::now() + REVERSE_HALF_PERIOD);
                }
            }
            RtdsCommand::StopReverse => {
                // only stop reverse, leave an rtds sound alone
                if self.state == RtdsState::Reverse {
                    self.go_idle();
                }
            }
        }
    }

    fn handle_timeout(&mut self) {
        match self.state {
            // rtds sound is done
            RtdsState::Drive => {
                self.clear_pin();
                self.set_state(RtdsState::Idle, Instant::MAX);
            }
            // Toggle the beep and schedule the next toggle 500 ms from now.
            RtdsState::Reverse => {
                self.toggle_pin();
                self.deadline = Instant::now() + REVERSE_HALF_PERIOD;
            }
            // nothing to time
            RtdsState::Idle => self.deadline = Instant::MAX,
        }
    }
}

/// owns the rtds pin and handles the timing.
///
/// `shutdown_closed` gets checked every time we'd turn the pin on, so rtds never plays with shutdown open
#[embassy_executor::task]
pub async fn rtds_task(pin: Output<'static>) {
    let mut rtds = Rtds {
        pin,
        state: RtdsState::Idle,
        deadline: Instant::MAX,
    };
    rtds.clear_pin();

    let mut can_ticker = Ticker::every(CAN_SEND_PERIOD);

    loop {
        match select3(
            COMMANDS.receive(),       // new command, handle it
            Timer::at(rtds.deadline), // state deadline (drive done / reverse toggle)
            can_ticker.next(),        // periodically send CAN
        )
        .await
        {
            Either3::First(command) => rtds.handle_command(command),
            Either3::Second(()) => rtds.handle_timeout(),
            Either3::Third(()) => {
                let rtds_pins = cangen::RtdsStateMessage::new()
                    .with_pin_state(u8::from(is_pin_on()))
                    .with_reverse_state(u8::from(is_reversing()))
                    .with_sounding_state(u8::from(is_sounding()));
                can::OUTGOING.send(rtds_pins.to_can_frame()).await;
            }
        }
    }
}
