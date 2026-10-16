# Atlas CAN bootloader

CAN bootloader with per-ECU settings, firmware verification, and software
swap/rollback. All ECU builds use one bootloader implementation and memory layout.
The CAN bitrate is fixed at build time by `can_line_bitrate`; runtime bitrate changes
are not supported.

## Layout

- `src/`: bootloader firmware.
- `ecus.json`: shared line bitrate and boot-entry payload, ECU CAN IDs, and CAN peripheral/pins.
- `memory.x`: fixed flash/RAM regions and runtime bounds exported as linker symbols.
- `build.rs`: JSON parsing, validation, CAN configuration generation, and linker setup.

## Build and flash

From `atlas/`, select an ECU from `ecus.json`:

```sh
BOOTLOADER_ECU=bms cargo build --release
BOOTLOADER_ECU=bms cargo run --release  # Build, flash, and view probe logs.
```

To build, download, and reset without viewing logs, run from `atlas/`:

```sh
BOOTLOADER_ECU=bms cargo download
```

From the repository root, add `-p atlas` to `cargo build` or `cargo run`; the
`download` alias is available from `atlas/`. To retain multiple
ECU builds, add `--target-dir ../target/<ecu>-can-bootloader` when running from
`atlas/` and use the ELF beneath that directory. CI builds and lints every
configured ECU into a separate output directory.

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

Add `boot-control.workspace = true` to the application dependencies.
The workspace aliases the `boot-control-stm32h563` package as `boot-control`.
The helper and its `memory.x` live in `firmware-rs/bootloader-app/platforms/stm32h563`. Initialize CAN
as usual and allow the application's boot-request ID through its receive filters.
Pass the ID and payload explicitly; the helper does not read `ecus.json`.

```rust
let mut bootloader = boot_control::Bootloader::new(p.FLASH, 0x013, &[0xB0, 0x07, 0x10, 0xAD]);
// Confirm only after required application initialization and checks succeed.
bootloader.confirm_boot()?;

// In the existing CAN receive loop; unrelated frames return immediately.
bootloader.handle_frame(&frame)?;
```

The helper owns FLASH and only writes the boot-state sector. A matching standard
data frame persists the request and resets; the bootloader answers host commands
afterward. Flash failures return an error without resetting. Projects that need
to stop outputs first can check `is_request(&frame)` before calling `handle_frame`.

Applications must link with the helper's `memory.x` at `0x0802E000`.
Their build scripts read `DEP_BOOT_CONTROL_MEMORY_X` and pass
`--remap-inputs=*memory.x=<path>` to the linker, avoiding checkout-specific paths.
The application helper does not stage firmware or perform swaps. Flash Atlas for
the same ECU first, then download the application ELF without a chip erase.

## Adding an ECU

Add an entry to `ecus.json` with unique CAN IDs and supported hardware settings.
Use its name with the same commands. There is no platform selection.
