//! RTDS (ready to drive sound)
//!
//! rtds_task owns the pin. everything else just calls `sound_rtds` / `cancel_rtds` which
//! throw a command on the queue. The task just sleeps until a new command or the deadline is reached.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, error};
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};

/// how long the rtds sound plays
const RTDS_DURATION: Duration = Duration::from_millis(1500);
const COMMAND_QUEUE_SIZE: usize = 4;

#[derive(Clone, Copy, defmt::Format)]
pub(crate) enum RtdsCommand {
    Sound,
    Cancel,
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

// TODO: shutdown isn't ported yet, pass the real one into rtds_task once shutdown exists
pub fn is_shutdown_closed_placeholder() -> bool {
    true
}

static COMMANDS: Channel<ThreadModeRawMutex, RtdsCommand, COMMAND_QUEUE_SIZE> = Channel::new();

// copies of the task state so other tasks can check it without needing the pin
static PIN_ON: AtomicBool = AtomicBool::new(false);
static SOUNDING: AtomicBool = AtomicBool::new(false);

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

/// stop the rtds ahead of deadline, call when shutdown is open
pub fn cancel_rtds() -> Result<(), RtdsError> {
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

struct Rtds {
    pin: Output<'static>,
    shutdown_closed: fn() -> bool,
    /// when to stop the sound, Some while it's playing
    sound_deadline: Option<Instant>,
}

impl Rtds {
    fn set_pin(&mut self) {
        // shutdown open = can't drive, so don't let rtds go off
        if !(self.shutdown_closed)() {
            debug!("RTDS shut down due to shutdown_closed.");
            return;
        }

        self.pin.set_high();
        PIN_ON.store(true, Ordering::Relaxed);
        debug!("Turned on RTDS pin.");
    }

    fn clear_pin(&mut self) {
        self.pin.set_low();
        PIN_ON.store(false, Ordering::Relaxed);
        debug!("Turned off RTDS pin.");
    }

    fn set_sound_deadline(&mut self, deadline: Option<Instant>) {
        self.sound_deadline = deadline;
        SOUNDING.store(deadline.is_some(), Ordering::Relaxed);
    }

    fn handle_command(&mut self, command: RtdsCommand) {
        debug!("Handling RTDS command: {:?}", command);
        match command {
            RtdsCommand::Sound => {
                self.set_pin();
                self.set_sound_deadline(Some(Instant::now() + RTDS_DURATION));
            }
            RtdsCommand::Cancel => {
                self.clear_pin();
                self.set_sound_deadline(None);
            }
        }
    }

    /// sound's done
    fn handle_sound_deadline(&mut self) {
        self.clear_pin();
        self.set_sound_deadline(None);
    }
}

/// owns the rtds pin and handles the timing.
///
/// `shutdown_closed` gets checked every time we'd turn the pin on, so rtds never plays with shutdown open
#[embassy_executor::task]
pub async fn rtds_task(pin: Output<'static>, shutdown_closed: fn() -> bool) -> ! {
    let mut rtds = Rtds {
        pin,
        shutdown_closed,
        sound_deadline: None,
    };
    rtds.clear_pin();

    loop {
        match rtds.sound_deadline {
            Some(deadline) => {
                // if there is a deadline (sound is playing)
                // wait until either there is a new command or the timer expires
                let event: Either<RtdsCommand, ()> =
                    select(COMMANDS.receive(), Timer::at(deadline)).await;
                match event {
                    // then if it is a new command, handle it, otherwise end the sound playing.
                    Either::First(command) => rtds.handle_command(command),
                    Either::Second(()) => rtds.handle_sound_deadline(),
                }
            }
            None => rtds.handle_command(COMMANDS.receive().await), // wait for a new command, then handle it
        }
    }
}
