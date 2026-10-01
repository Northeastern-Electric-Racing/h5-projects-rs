//! Build an ECU's bootloader and optionally download it or run with probe logs.

use serde::Deserialize;
use std::{
    collections::BTreeMap,
    env, fs,
    io::{BufRead, BufReader},
    path::PathBuf,
    process::{Command, ExitCode, Stdio},
};

// Read only build selection here; the platform build script validates the full file.
#[derive(Deserialize)]
struct Config {
    schema_version: u32,
    ecus: BTreeMap<String, Ecu>,
    platforms: BTreeMap<String, Platform>,
}

#[derive(Deserialize)]
struct Ecu {
    platform: String,
}

#[derive(Deserialize)]
struct Platform {
    cargo_package: String,
    rust_target: String,
    chip: String,
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args().skip(1);
    let usage = "cargo {build-atlas|run-atlas|download-atlas} <ecu> [--offline]";
    let first = args.next().ok_or(usage)?;
    let (action, name) = match first.as_str() {
        "--run" => (Some("run"), args.next().ok_or(usage)?),
        "--download" => (Some("download"), args.next().ok_or(usage)?),
        _ => (None, first),
    };
    if name == "--help" || name == "-h" {
        println!("{usage}");
        return Ok(());
    }
    let name = name.to_ascii_lowercase();
    let mut offline = false;
    for arg in args {
        match arg.as_str() {
            "--offline" => offline = true,
            _ => return Err(format!("Unknown option: {arg}").into()),
        }
    }
    // ECU names become directory names; reject path components before invoking Cargo.
    if name.is_empty()
        || !name
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
    {
        return Err("ECU names must use letters, digits, or hyphens".into());
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_owned();
    let config: Config = serde_json::from_str(&fs::read_to_string(root.join("atlas/ecus.json"))?)?;
    if config.schema_version != 1 {
        return Err("Unsupported schema_version; expected 1".into());
    }
    let ecu = config.ecus.get(&name).ok_or_else(|| {
        format!(
            "Unknown ECU {name}; choose: {}",
            config.ecus.keys().cloned().collect::<Vec<_>>().join(", ")
        )
    })?;
    let platform = config
        .platforms
        .get(&ecu.platform)
        .ok_or_else(|| format!("Unknown platform: {}", ecu.platform))?;
    println!("Building {name} using {}", platform.cargo_package);

    let mut command = Command::new(env::var_os("CARGO").unwrap_or_else(|| "cargo".into()));
    command
        .current_dir(&root)
        .args([
            "build",
            "--release",
            "--package",
            &platform.cargo_package,
            "--target",
            &platform.rust_target,
        ])
        .arg("--target-dir")
        .arg(root.join("target").join(format!("{name}-bootloader")))
        .arg("--message-format=json-render-diagnostics")
        .env("BOOTLOADER_ECU", &name)
        .stdout(Stdio::piped());
    if offline {
        command.arg("--offline");
    }
    let mut child = command.spawn()?;
    let mut executable = None;
    // Cargo supplies the actual artifact path, so no target-specific filename is assumed.
    for line in BufReader::new(child.stdout.take().unwrap()).lines() {
        let line = line?;
        let Ok(message) = serde_json::from_str::<serde_json::Value>(&line) else {
            continue;
        };
        if message["reason"] == "compiler-artifact"
            && message["target"]["name"] == platform.cargo_package
            && let Some(path) = message["executable"].as_str()
        {
            executable = Some(path.to_owned());
        }
    }
    if !child.wait()?.success() {
        return Err("Bootloader build failed".into());
    }
    let executable = executable.ok_or("Cargo did not report a bootloader ELF")?;
    println!("ELF: {executable}");
    if let Some(action) = action {
        // Inherit the terminal so probe selection and firmware logs remain visible.
        let mut probe = Command::new("probe-rs");
        probe
            .current_dir(&root)
            .args([action, "--chip", &platform.chip]);
        if action == "download" {
            probe.args(["--verify", "--reset"]);
        }
        if !probe.arg(&executable).status()?.success() {
            return Err(format!("probe-rs {action} failed").into());
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("Error: {error}");
            ExitCode::FAILURE
        }
    }
}
