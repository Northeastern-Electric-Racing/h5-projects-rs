fn main() {
    let memory_path =
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../atlas/app/memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", memory_path.display());
    // Link at the application address, leaving Atlas and update storage reserved.
    println!(
        "cargo:rustc-link-arg-bins=--remap-inputs=*memory.x={}",
        memory_path.display()
    );
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
