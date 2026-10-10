# Atlas CAN bootloader

CAN bootloader with per-ECU settings, firmware verification, and software
swap/rollback. All ECU builds use one bootloader implementation and memory layout.

## Layout

- `src/`: bootloader firmware.
- `app/`: application helper and application linker layout.
- `ecus.json`: ECU CAN IDs, bitrate, and CAN peripheral/pins.
- `config.rs`: configuration parsing and validation.
- `memory.x`: fixed flash/RAM regions and runtime bounds exported as linker symbols.
- `build.rs`: CAN configuration generation and linker setup.
- `tools/`: build and probe helper.

## Build and flash

Run from this directory, using `bms`, `vcu`, or another configured ECU name:

```sh
cargo build-atlas bms
cargo download-atlas bms  # Build, download, and reset through probe-rs.
cargo run-atlas bms       # Build and run with probe logs.
```

Use `cargo build-atlas --help` for usage.

Install `probe-rs` for download/run. The package, Rust target, and probe chip are
fixed in `tools/main.rs`. Firmware outputs go to
`../target/<ecu>-bootloader/<rust_target>/release/<cargo_package>`.

The build script generates CAN bindings and constants from the selected ECU.
Memory regions are defined directly in `memory.x`; runtime address checks use its
linker symbols.

## Update flow

1. Complete any pending swap or rollback.
2. Start a valid application immediately unless DFU entry was requested.
3. In update mode, accept the image into DFU storage and verify its CRC and vectors.
4. Activation records the swap request and resets after the CAN acknowledgement.
5. The next boot swaps the image into the application region. The application
   must confirm startup to prevent rollback on a subsequent reset.

An invalid application remains in update mode for recovery. A valid application
must persist a DFU request and reset to enter the bootloader.

The bootloader consumes that request on entry. Communication loss leaves the
current update session open; resetting before activation starts the existing valid
application. Retry from the application by requesting entry again. If clearing the
request fails, the bootloader logs an error and stays in update mode.

## Application integration

Add `atlas-app.workspace = true` to the application dependencies. Initialize CAN
as usual and allow the application's boot-request ID through its receive filters.
Pass the ID and payload explicitly; the helper does not read `ecus.json`.

```rust
let mut bootloader = atlas_app::Bootloader::new(p.FLASH, 0x013, &[0xB0, 0x07, 0x10, 0xAD]);
// Confirm only after required application initialization and checks succeed.
bootloader.confirm_boot()?;

// In the existing CAN receive loop; unrelated frames return immediately.
bootloader.handle_frame(&frame)?;
```

The helper owns FLASH and only writes the boot-state sector. A matching standard
data frame persists the request and resets; the bootloader answers host commands
afterward. Flash failures return an error without resetting. Projects that need
to stop outputs first can check `is_request(&frame)` before calling `handle_frame`.

Use `template-h5/build.rs` as the example for linking with `app/memory.x` at
`0x0802E000`. The application helper does not stage firmware or perform swaps.

The template includes CAN bootloader entry by default. Set the request ID, payload,
and bitrate directly in `template-h5/src/boot.rs` to match the host's ECU settings.
The receive filter and helper share the same request ID. CAN peripheral and pins
also remain in application code; no application settings are generated from JSON.

Flash Atlas for the same ECU first, then download the application ELF from
`target/thumbv8m.main-none-eabihf/release/template-h5` without a chip erase.
The template always links at the Atlas application address, `0x0802E000`.

## Adding an ECU

Add an entry to `ecus.json` with unique CAN IDs and supported hardware settings.
Use its name with the same commands. There is no platform selection.
