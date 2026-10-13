fn main() {
    let memory_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../atlas/app/memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", memory_path.display());
    // Link at the application address, leaving Atlas and update storage reserved.
    println!("cargo:rustc-link-arg-bins=--remap-inputs=*memory.x={}", memory_path.display());
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
