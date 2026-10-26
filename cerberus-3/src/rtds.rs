//! RTDS (ready to drive sound)
//!
//! rtds_task owns the pin. everything else just calls `sound_rtds` / `cancel_rtds` which
//! throw a command on the queue. The task just sleeps until a new command or the deadline is reached.

use core::sync::atomic::{AtomicBool, Ordering};

use cangen::ToCanFrame;
use defmt::{debug, error};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::{
    can,
    shutdown::{self, BmsShutdownState},
};

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
    Sound,
    /// beep continuously until cancelled
    Reverse,
    /// go back to idle (pin off)
    Cancel,
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

/// play the rtds
pub fn sound_rtds() -> Result<(), RtdsError> {
    send(RtdsCommand::Sound)
}

/// start the reverse beeping, plays until `cancel_rtds` is called
pub fn start_reverse_rtds() -> Result<(), RtdsError> {
    send(RtdsCommand::Reverse)
}

/// stop whatever is playing (rtds or reverse) and go idle,
/// call when leaving reverse or when shutdown is open
pub fn cancel_sounds() -> Result<(), RtdsError> {
    send(RtdsCommand::Cancel)
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
    shutdown_closed: bool,
    state: RtdsState,
    /// when the current state next needs attention. `Instant::MAX` if never (idle)
    deadline: Instant,
}

impl Rtds {
    fn set_pin(&mut self) {
        // shutdown open = can't drive, so don't let rtds go off
        if !(self.shutdown_closed) {
            debug!("RTDS shut down due to shutdown_closed.");
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

    fn handle_command(&mut self, command: RtdsCommand) {
        debug!("Handling RTDS command: {:?}", command);
        match command {
            RtdsCommand::Sound => {
                self.set_pin();
                self.set_state(RtdsState::Drive, Instant::now() + RTDS_DURATION);
            }
            RtdsCommand::Reverse => {
                // already beeping, don't restart the beep pattern
                if self.state != RtdsState::Reverse {
                    self.clear_pin();
                    self.set_pin();
                    self.set_state(RtdsState::Reverse, Instant::now() + REVERSE_HALF_PERIOD);
                }
            }
            RtdsCommand::Cancel => {
                self.clear_pin();
                self.set_state(RtdsState::Idle, Instant::MAX);
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
        shutdown_closed: shutdown::get_shutdown_state() == BmsShutdownState::Closed,
        state: RtdsState::Idle,
        deadline: Instant::MAX,
    };
    rtds.clear_pin();

    // Ticker (not Timer::after) so handling commands/timeouts doesn't push the CAN period back
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
