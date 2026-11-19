use std::{env, fs, path::PathBuf};

fn main() {
    // Copy memory.x into OUT_DIR and add it to the linker search path.
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let memory = env::var_os("DEP_BOOT_CONTROL_MEMORY_X")
        .expect("boot-control must provide the application memory layout");
    fs::copy(memory, out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
