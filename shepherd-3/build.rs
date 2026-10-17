use std::{env, fs, path::PathBuf};

fn main() {
    // Make this application's memory layout available when building from the workspace.
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Check if defmt_monitor is enabled
    // (need this to be able to use #[cfg(defmt_monitor)]
    let enabled = match std::env::var("DEFMT_MONITOR") {
        Ok(v) => !matches!(v.trim().to_ascii_lowercase().as_str(), "off" | "0" | "false" | "no"),
        Err(_) => true,
    };
    println!("cargo::rustc-check-cfg=cfg(defmt_monitor)");
    if enabled {
        println!("cargo::rustc-cfg=defmt_monitor");
    }
    println!("cargo::rerun-if-env-changed=DEFMT_MONITOR");
}
