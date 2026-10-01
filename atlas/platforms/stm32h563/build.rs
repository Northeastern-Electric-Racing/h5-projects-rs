#[path = "../../config.rs"]
mod build_config;

use build_config::Config;
use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=../../config.rs");
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let config_path = manifest.join("../../ecus.json");
    println!("cargo:rerun-if-changed={}", config_path.display());
    println!("cargo:rerun-if-env-changed=BOOTLOADER_ECU");
    let config = Config::read(&config_path).unwrap_or_else(|e| panic!("{e}"));
    let name = env::var("BOOTLOADER_ECU")
        .expect("Set BOOTLOADER_ECU to an ECU name from ecus.json (for example, bms)");
    let (ecu, platform) = config.select(&name).unwrap_or_else(|e| panic!("{e}"));
    // Match Cargo's actual package identity rather than a duplicated package name.
    assert_eq!(
        platform.cargo_package,
        env::var("CARGO_PKG_NAME").unwrap(),
        "Selected ECU's cargo_package does not match the package being built"
    );
    platform
        .validate_h563(ecu)
        .unwrap_or_else(|e| panic!("{e}"));

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let hw = &ecu.hardware;
    let peripheral = &hw.can_peripheral;
    let rx = &hw.can_rx;
    let tx = &hw.can_tx;
    // Validation above limits these identifiers to supported peripheral and pin names.
    fs::write(out.join("ecu_can_hardware.rs"), format!(
        "use embassy_stm32::peripherals::{{{peripheral} as CanPeripheral, {rx} as CanRxPin, {tx} as CanTxPin}};\n\
         use embassy_stm32::pac::{peripheral} as CAN_REGS;\n\
         bind_interrupts!(struct Irqs {{\n\
             {peripheral}_IT0 => IT0InterruptHandler<CanPeripheral>;\n\
             {peripheral}_IT1 => IT1InterruptHandler<CanPeripheral>;\n\
         }});\n"
    )).unwrap();
    fs::write(
        out.join("ecu_can_init.rs"),
        format!("CanHandler::new(p.{peripheral}, p.{rx}, p.{tx})\n"),
    )
    .unwrap();

    fs::write(out.join("ecu_can.rs"), format!(
        "pub const REQUEST_ID: u16 = {:#x};\npub const RESPONSE_ID: u16 = {:#x};\npub const DATA_ID: u16 = {:#x};\npub const DEFAULT_BIT_RATE: u32 = {};\n",
        ecu.can.request_id, ecu.can.response_id, ecu.can.write_data_id, ecu.can.default_bit_rate,
    )).unwrap();

    let memory = &platform.memory;
    let mut regions = String::from("MEMORY\n{\n");
    for (name, region) in [
        ("FLASH", &memory.bootloader),
        ("ACTIVE", &memory.application),
        ("DFU", &memory.dfu),
        ("BOOTLOADER_STATE", &memory.state),
        ("RAM", &memory.ram),
    ] {
        regions.push_str(&format!(
            "  {name} : ORIGIN = {:#x}, LENGTH = {:#x}\n",
            region.address, region.size
        ));
    }
    regions.push_str("}\n");
    fs::write(out.join("memory-regions.x"), regions).unwrap();
    let (ram_start, ram_end) = memory.ram.bounds().unwrap();
    // Address checks and linker placement come from the same memory definitions.
    fs::write(out.join("ecu_memory.rs"), format!(
        "pub const APP_START: u32 = {:#x};\npub const APP_SIZE: u32 = {:#x};\npub const SRAM_START: u32 = {ram_start:#x};\npub const SRAM_END: u32 = {ram_end:#x};\npub const HSE_HZ: u32 = {};\n",
        memory.application.address, memory.application.size, ecu.hardware.hse_hz,
    )).unwrap();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
}
