use std::{env, path::PathBuf};

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rerun-if-env-changed=BOOTLOADER_ECU");
    println!(
        "cargo:rustc-check-cfg=cfg(ecu, values(\"bms\", \"vcu\", \"lightning\", \"msb_front\", \"msb_back\"))"
    );
    let name = match env::var("BOOTLOADER_ECU") {
        Ok(name) => name.to_ascii_lowercase(),
        Err(env::VarError::NotPresent) => "bms".into(),
        Err(error) => panic!("Invalid BOOTLOADER_ECU: {error}"),
    };
    assert!(
        matches!(
            name.as_str(),
            "bms" | "vcu" | "lightning" | "msb_front" | "msb_back"
        ),
        "Unknown ECU {name}; select bms, vcu, lightning, msb_front, or msb_back"
    );
    println!("cargo:rustc-cfg=ecu=\"{name}\"");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    // Resolve link.x's memory include explicitly instead of using dependency search order.
    println!(
        "cargo:rustc-link-arg-bins=--remap-inputs=*memory.x={}",
        manifest.join("memory.x").display()
    );
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-search={}", manifest.display());
    println!("cargo:rerun-if-changed=memory.x");
}
