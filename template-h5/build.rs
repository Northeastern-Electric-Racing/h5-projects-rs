use std::path::PathBuf;

fn main() {
    let memory_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../atlas/app/memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", memory_path.display());
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    // Select the application layout even when Embassy's memory.x appears first.
    println!(
        "cargo:rustc-link-arg-bins=--remap-inputs=*memory.x={}",
        memory_path.display()
    );
}
