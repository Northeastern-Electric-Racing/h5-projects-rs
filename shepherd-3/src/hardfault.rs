//! Cortex-M hardfault and panic handling.
//!
//! A panic or HardFault saves a crash record in RAM that survives the reset, then resets the MCU
//! (or hangs with the `halt-on-crash` feature, so a debugger can be attached). On the next boot the
//! record is logged and sent over CAN. The shutdown pin is left alone on purpose.
//!
//! This replaces `panic-probe` (a program can only have one `#[panic_handler]`).

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::peripheral::scb::Exception;
use cortex_m::peripheral::{DCB, SCB};
use defmt::{info, warn};
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Timer};

/// magic number constant to mark valid crash data. the hex value is meant to spell "balls"
/// this is set in CrashInfo by the crash handlers, and then when the next boot reads the memory, it uses this const to verify that there is real crash data there instead of random junk
const CRASH_MAGIC: u32 = 0xBA115;

/// `CrashInfo::kind` values.
pub mod kind {
    /// A real HardFault.
    pub const HARDFAULT: u32 = 1;
    /// A Rust panic.
    pub const PANIC: u32 = 2;
    /// A panic raised through defmt (`defmt::unwrap!` etc., which embassy uses internally).
    /// defmt doesn't give its panic handler the location, so `line` and `file_hash` are 0.
    pub const DEFMT_PANIC: u32 = 3;
}

/// Standard CAN ID of the crash report frame, next to Shepherd's version messages (0x69A, 0x69B).
/// TODO: add "Shepherd Crash Report" to Odyssey-Definitions bms.json so Calypso can decode it
const CRASH_FRAME_ID: u16 = 0x69C;
/// Sent more than once because CAN automatic retransmit is off.
const CRASH_FRAME_REPEATS: usize = 3;
const CRASH_FRAME_PERIOD: Duration = Duration::from_secs(1);
/// How long the board has to run after boot before the crash count is cleared.
const STABLE_RUNTIME: Duration = Duration::from_secs(10);

/// Stores CPU state and fault data as of the last crash.
/// `pc` through `bfar` are only filled in for a HardFault, `line` and `file_hash` only for a panic.
#[repr(C)]
#[derive(Clone, Copy, defmt::Format)]
pub struct CrashInfo {
    magic: u32,
    /// One of the `kind` values.
    kind: u32,
    /// Program Counter.
    pc: u32,
    /// Link Register.
    lr: u32,
    /// Configurable Fault Status Register.
    cfsr: u32,
    /// HardFault Status.
    hfsr: u32,
    /// MemManage Fault Address (valid only when CFSR marks it).
    mmfar: u32,
    /// BusFault Address.
    bfar: u32,
    /// Source line of the panic.
    line: u32,
    /// FNV-1a hash of the panic's source file path (see `fnv1a`).
    file_hash: u32,
    /// Crashes since the count was last cleared (used to catch boot loops).
    count: u32,
    /// Set once `report_last_reset()` has reported this record.
    reported: u32,
}

impl CrashInfo {
    /// An empty record of the given kind.
    const fn new(kind: u32) -> Self {
        Self {
            magic: CRASH_MAGIC,
            kind,
            pc: 0,
            lr: 0,
            cfsr: 0,
            hfsr: 0,
            mmfar: 0,
            bfar: 0,
            line: 0,
            file_hash: 0,
            count: 0,
            reported: 0,
        }
    }

    /// Packs the record into one 8-byte CAN frame, big endian like our other messages:
    /// [kind, count, line (u16), file_hash for a panic / pc for a HardFault (u32)]
    /// TODO: switch to the cangen struct once the message is in Odyssey-Definitions
    pub fn as_frame(&self) -> Frame {
        let kind = self.kind as u8;
        let count = self.count.min(u8::MAX as u32) as u8;
        let line = (self.line.min(u16::MAX as u32) as u16).to_be_bytes();
        let detail = if self.kind == kind::HARDFAULT { self.pc } else { self.file_hash };
        let detail = detail.to_be_bytes();
        let data = [kind, count, line[0], line[1], detail[0], detail[1], detail[2], detail[3]];

        Frame::new_standard(CRASH_FRAME_ID, &data).unwrap()
    }
}

/// SAFETY: `.uninit` is a NOLOAD section, and `CrashInfo` data is static (so putting it there is fine)
#[unsafe(link_section = ".uninit.CRASH")]
static mut CRASH: MaybeUninit<CrashInfo> = MaybeUninit::uninit();

/// Set once a crash handler starts. Stops a panic inside a panic handler from looping, and tells
/// `HardFault` when a panic handler triggered it on purpose.
static IN_CRASH: AtomicBool = AtomicBool::new(false);

/// Reports why we last reset, logs any crash record left in RAM, and returns it.
/// this is meant to be called at the top of main() after embassy_stm32::init
pub fn report_last_reset() -> Option<CrashInfo> {
    use embassy_stm32::pac;

    let rsr = pac::RCC.rsr().read();
    info!("Reset cause: pin={} bor={} sw={} iwdg={} wwdg={} lowpower={}", rsr.pinrstf(), rsr.borrstf(), rsr.sftrstf(), rsr.iwdgrstf(), rsr.wwdgrstf(), rsr.lpwrrstf());
    pac::RCC.rsr().modify(|w| w.set_rmvf(true));

    let mut crash = read_record()?;
    if crash.reported != 0 {
        return None;
    }
    warn!("Recovered crash record from previous boot: {}", crash);

    // marked instead of erased, so `count` keeps going if we crash again before it's cleared
    crash.reported = 1;
    write_record(crash);
    Some(crash)
}

/// With `halt-on-crash`, freezes the watchdog while a debugger has the core halted. Otherwise it keeps
/// counting, so a board left stuck at a breakpoint still resets. Set both ways because DBGMCU can survive a reset.
pub fn set_watchdog_debug_freeze() {
    embassy_stm32::pac::DBGMCU.apb1lfzr().modify(|w| w.set_iwdg_stop(cfg!(feature = "halt-on-crash")));
}

/// Sends the crash record from `report_last_reset()` (if there was one) over CAN, then clears the
/// crash count once the board has run for `STABLE_RUNTIME` without crashing again.
#[embassy_executor::task]
pub async fn crash_report_task(crash: Option<CrashInfo>) {
    let stable_at = Instant::now() + STABLE_RUNTIME;

    if let Some(crash) = crash {
        for _ in 0..CRASH_FRAME_REPEATS {
            crate::can::send(crash.as_frame()).await;
            Timer::after(CRASH_FRAME_PERIOD).await;
        }
    }

    Timer::at(stable_at).await;
    clear_crash_count();
}

/// Erases the crash record, so the next crash counts from 1 again.
pub fn clear_crash_count() {
    // SAFETY: see `write_record`
    unsafe { (&raw mut CRASH).write_volatile(MaybeUninit::zeroed()) };
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    if !IN_CRASH.swap(true, Ordering::Relaxed) {
        let (line, file_hash) = info.location().map_or((0, 0), |loc| (loc.line(), fnv1a(loc.file().as_bytes())));
        // saved before logging, since logging is the step most likely to go wrong
        save_crash(CrashInfo { line, file_hash, ..CrashInfo::new(kind::PANIC) });

        defmt::error!("panic: {}", defmt::Display2Format(info));
    }

    finish_panic()
}

/// Called by `defmt::panic!`, `defmt::unwrap!`, etc. after defmt has already logged the message and location.
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::interrupt::disable();

    if !IN_CRASH.swap(true, Ordering::Relaxed) {
        save_crash(CrashInfo::new(kind::DEFMT_PANIC));
    }

    finish_panic()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    // a panic handler triggered this on purpose and already saved its record
    if IN_CRASH.swap(true, Ordering::Relaxed) {
        end_crash();
    }

    // SAFETY: the SCB block is at a fixed address, is always mapped, and doesn't need initialization. every field is a VolatileCell. this is how you're supposed to access the block
    let scb = unsafe { &*SCB::PTR };

    let info = save_crash(CrashInfo {
        pc: frame.pc(),
        lr: frame.lr(),
        cfsr: scb.cfsr.read(),
        hfsr: scb.hfsr.read(),
        mmfar: scb.mmfar.read(),
        bfar: scb.bfar.read(),
        ..CrashInfo::new(kind::HARDFAULT)
    });

    defmt::error!("HardFault: {}", info);

    if DCB::is_debugger_attached() {
        cortex_m::asm::bkpt();
    }

    end_crash()
}

/// With a probe attached, triggers a HardFault on purpose (like panic-probe does) so probe-rs halts
/// and prints a backtrace. Otherwise ends the crash right away.
fn finish_panic() -> ! {
    if DCB::is_debugger_attached() {
        trigger_hardfault();
    }

    end_crash()
}

/// Disables UsageFault first, otherwise `udf` raises UsageFault instead of HardFault.
fn trigger_hardfault() -> ! {
    // SAFETY: we never return to normal code after this, so steal() is fine
    let mut scb = unsafe { cortex_m::Peripherals::steal() }.SCB;
    scb.disable(Exception::UsageFault);
    cortex_m::asm::udf()
}

/// Resets right away, or hangs with the `halt-on-crash` feature.
/// `cfg!` instead of `#[cfg]` so both paths get compiled whichever features are on.
fn end_crash() -> ! {
    if cfg!(feature = "halt-on-crash") { hang() } else { SCB::sys_reset() }
}

/// Spins forever so a debugger can attach with `probe-rs attach --chip STM32H563ZITx <elf>` (not `cargo embed`,
/// which resets the chip on connect). Interrupts are off, so the watchdog has to be petted here.
fn hang() -> ! {
    use embassy_stm32::pac;
    use embassy_stm32::pac::iwdg::vals::Key;

    loop {
        pac::IWDG.kr().write(|w| w.set_key(Key::Reset));
    }
}

/// Saves a crash record on top of any earlier one that hasn't been cleared, and returns it with `count` filled in.
fn save_crash(mut info: CrashInfo) -> CrashInfo {
    info.count = read_record().map_or(1, |prev| prev.count.saturating_add(1));
    write_record(info);
    info
}

/// Reads the crash record, or `None` if RAM holds random junk instead of a record.
fn read_record() -> Option<CrashInfo> {
    // SAFETY: `CRASH` is correctly sized by the linker. validity of data is checked via `CRASH_MAGIC`
    let crash = unsafe { (&raw const CRASH).read_volatile().assume_init() };
    (crash.magic == CRASH_MAGIC).then_some(crash)
}

fn write_record(info: CrashInfo) {
    // SAFETY: no reference to `CRASH` exists and STM32H563 is single-core. the crash handlers can't be preempted,
    // and a crash landing mid-write from normal code can only garble the count.
    unsafe { (&raw mut CRASH).write_volatile(MaybeUninit::new(info)) };
}

/// Hashes the panic's file path to 32 bits so it fits in a CAN frame. A host script can hash every
/// path under shepherd-3/src/ (e.g. "shepherd-3/src/can.rs") the same way to map it back.
fn fnv1a(bytes: &[u8]) -> u32 {
    bytes.iter().fold(0x811c_9dc5u32, |h, &b| (h ^ b as u32).wrapping_mul(0x0100_0193))
}
