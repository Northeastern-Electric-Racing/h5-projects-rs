// The shared parser also exposes validation used only by platform build scripts.
#[allow(dead_code)]
#[path = "../config.rs"]
mod build_config;

use std::{env, fs, path::PathBuf, process::ExitCode};

fn generate() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=../config.rs");
    println!("cargo:rerun-if-changed=../ecus.json");
    let manifest =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").ok_or("Missing manifest directory")?);
    let path = manifest.join("../ecus.json");
    let config = build_config::Config::read(&path)
        .map_err(|error| format!("{}: {error}", path.display()))?;

    // Generate every ECU so applications in one workspace can select independently.
    let mut source = String::from("match name {\n");
    for (name, ecu) in &config.ecus {
        source.push_str(&format!(
            "{name:?} => Self::new(StandardId::new({}).unwrap(), &{:?}),\n",
            ecu.can.application_boot_request_id, ecu.can.application_boot_request_data,
        ));
    }
    source.push_str("_ => None,\n}\n");
    let out = PathBuf::from(env::var_os("OUT_DIR").ok_or("Missing output directory")?);
    fs::write(out.join("ecu_requests.rs"), source)?;
    Ok(())
}

fn main() -> ExitCode {
    match generate() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}
