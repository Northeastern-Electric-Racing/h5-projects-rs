//! Build an ECU's bootloader and optionally download it or run with probe logs.

#[path = "../config.rs"]
mod build_config;

use std::{
    env,
    io::{BufRead, BufReader},
    path::PathBuf,
    process::{Command, ExitCode, Stdio},
};

const PACKAGE: &str = "atlas";
const TARGET: &str = "thumbv8m.main-none-eabihf";
// Probe-rs uses this device name for the STM32H563ZIT6.
const CHIP: &str = "STM32H563ZITx";

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    let usage = "cargo {build-atlas|run-atlas|download-atlas} <ecu> [--offline]";
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!("{usage}");
        return Ok(());
    }
    let mut args = args.into_iter();
    let first = args.next().ok_or(usage)?;
    let (action, name) = match first.as_str() {
        "--run" => (Some("run"), args.next().ok_or(usage)?),
        "--download" => (Some("download"), args.next().ok_or(usage)?),
        _ => (None, first),
    };
    let mut offline = false;
    for arg in args {
        match arg.as_str() {
            "--offline" => offline = true,
            _ => return Err(format!("Unexpected argument: {arg}; select exactly one ECU").into()),
        }
    }
    let name = name.to_ascii_lowercase();
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
    let config = build_config::Config::read(&root.join("atlas/ecus.json"))?;
    config.select(&name)?;
    println!("Building {name} using {PACKAGE}");

    let mut command = Command::new(env::var_os("CARGO").unwrap_or_else(|| "cargo".into()));
    command
        .current_dir(&root)
        .args([
            "build",
            "--release",
            "--package",
            PACKAGE,
            "--target",
            TARGET,
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
            && message["target"]["name"] == PACKAGE
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
        probe.current_dir(&root).args([action, "--chip", CHIP]);
        if action == "download" {
            probe.args(["--verify", "--reset"]);
        } else if action == "run" {
            probe.arg("--no-catch-reset");
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
