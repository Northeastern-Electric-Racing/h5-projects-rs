fn main() {
    let memory_path = std::env::var("DEP_BOOT_CONTROL_MEMORY_X")
        .expect("Bootloader application dependency must provide its memory.x path");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", memory_path);
    // Link at the application address, leaving Atlas and update storage reserved.
    println!(
        "cargo:rustc-link-arg-bins=--remap-inputs=*memory.x={}",
        memory_path
    );
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
