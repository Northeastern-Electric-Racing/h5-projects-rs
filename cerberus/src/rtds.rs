//! RTDS (ready to drive sound), ported from the old C `u_rtds.c`.
//!
//! rtds_task owns the pin. everything else just calls `sound_rtds` / `cancel_rtds` which
//! throw a command on the queue. the threadx one shot timer is gone, the task just sleeps
//! until the deadline instead.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, error};
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};

/// how long the rtds sound plays
const RTDS_DURATION: Duration = Duration::from_millis(1500); // was 1500 ticks in C, pretty sure the tick was 1kHz

const COMMAND_QUEUE_SIZE: usize = 4;

#[derive(Clone, Copy, defmt::Format)]
enum RtdsCommand {
    Sound,
    Cancel,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum RtdsError {
    /// queue was full so the command got dropped
    QueueFull,
}

// TODO: faults aren't ported yet. C did `queue_send(&faults, &(fault_t){RTDS_FAULT}, TX_NO_WAIT)`,
// swap this out once we have faults
fn send_rtds_fault() {
    error!("RTDS_FAULT (placeholder, faults not ported yet).");
}

// TODO: shutdown isn't ported yet. in C `is_shutdown_closed()` returned the bms_shutdown flag
// (or just true w/ TSMS_OVERRIDE). returns false for now so rtds never goes off.
// pass the real one into rtds_task once shutdown exists
pub fn is_shutdown_closed_placeholder() -> bool {
    false
}

static COMMANDS: Channel<ThreadModeRawMutex, RtdsCommand, COMMAND_QUEUE_SIZE> = Channel::new();

// copies of the task state so other tasks can check it without needing the pin
static PIN_ON: AtomicBool = AtomicBool::new(false);
static SOUNDING: AtomicBool = AtomicBool::new(false);

/// puts a command on the queue. fault gets raised in here so it still happens if the caller ignores the error
fn send(command: RtdsCommand) -> Result<(), RtdsError> {
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

/// stop the rtds early (it turns off by itself after RTDS_DURATION anyway)
pub fn cancel_rtds() -> Result<(), RtdsError> {
    send(RtdsCommand::Cancel)
}

/// is the pin high rn (mostly for debugging)
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
            Some(deadline) => match select(COMMANDS.receive(), Timer::at(deadline)).await {
                Either::First(command) => rtds.handle_command(command),
                Either::Second(()) => rtds.handle_sound_deadline(),
            },
            None => rtds.handle_command(COMMANDS.receive().await),
        }
    }
}
