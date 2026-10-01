#![no_std]
#![no_main]

//! Initialize the bootloader, prepare firmware swaps, and hand control to the application.

use core::cell::RefCell;

use cortex_m::peripheral::SCB;
use embassy_boot_stm32::{
    AlignedBuffer, BlockingFirmwareUpdater, BootLoader, BootLoaderConfig, FirmwareUpdaterConfig,
};
use embassy_executor::Spawner;
use embassy_stm32::flash::Flash;
use embassy_stm32::rcc::{Hse, HseMode};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use {defmt_rtt as _, panic_probe as _};

mod app;
mod boot;
mod can;
mod commands;
mod config;
mod dispatch;
mod flash;
mod handlers;
mod types;

use can::CanHandler;
use types::Control;

unsafe extern "C" {
    // Absolute vector-table address supplied by memory.x.
    static __bootloader_active_address: u32;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();

    config.rcc.hse = Some(Hse {
        freq: Hertz(config::HSE_HZ),
        mode: HseMode::Oscillator,
    });

    let p = embassy_stm32::init(config);

    let regions = Flash::new_blocking(p.FLASH).into_blocking_regions();

    // Flash access stays in this task; partitions share their bank through RefCell.
    let bank1 = Mutex::<NoopRawMutex, _>::new(RefCell::new(regions.bank1_region));

    let bank2 = Mutex::<NoopRawMutex, _>::new(RefCell::new(regions.bank2_region));

    // ACTIVE lives in bank 1; DFU and persistent swap state live in bank 2.
    let config = BootLoaderConfig::from_linkerfile_blocking(&bank1, &bank2, &bank2);

    // Complete any pending software swap or rollback before inspecting the app.
    let boot = BootLoader::prepare::<_, _, _, { embassy_stm32::flash::MAX_ERASE_SIZE }>(config);

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&bank2, &bank2);

    // State writes need a scratch buffer aligned to the flash programming unit.
    let mut aligned = AlignedBuffer([0; embassy_stm32::flash::WRITE_SIZE]);

    let mut updater = BlockingFirmwareUpdater::new(config, aligned.as_mut());

    // Take the controller and pins selected in ecus.json.
    let mut can = include!(concat!(env!("OUT_DIR"), "/ecu_can_init.rs"));

    match boot::run(&mut can, &mut updater, &boot.state).await {
        Control::Reset => SCB::sys_reset(),
        Control::StartApp => {}
        Control::Continue => unreachable!(),
    }

    // The linker symbol's address is the value; do not dereference it.
    let start = core::ptr::addr_of!(__bootloader_active_address) as u32;

    // Stop bootloader interrupts before the app replaces the vector table and RAM.
    cortex_m::interrupt::disable();
    can.shutdown();

    // This final handoff ends the bootloader's use of the core peripherals.
    unsafe {
        let mut core = cortex_m::Peripherals::steal();
        core.SYST.disable_interrupt();
        core.SYST.disable_counter(); // Prevent new ticks during application startup.
        core.SYST.clear_current();
        // Mask every external IRQ, including the Embassy timer interrupt.
        for register in &core.NVIC.icer {
            register.write(u32::MAX);
        }
        // Discard events queued before shutdown.
        for register in &core.NVIC.icpr {
            register.write(u32::MAX);
        }
        SCB::clear_pendsv(); // Discard deferred work from the bootloader executor.
        SCB::clear_pendst(); // Clear any tick queued before SysTick was stopped.
        // Complete peripheral writes before changing interrupt delivery.
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // Individual IRQs are masked and pending system ticks are cleared.
        // cortex-m-rt expects PRIMASK clear; the app enables its own IRQs later.
        cortex_m::interrupt::enable();
        // StartApp is returned only after validating the active vectors.
        boot.load(start)
    }
}
