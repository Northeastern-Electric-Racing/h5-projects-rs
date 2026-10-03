# Atlas CAN bootloader

CAN bootloader with per-ECU settings, firmware verification, and software
swap/rollback. All ECU builds use one bootloader implementation and memory layout.

## Layout

- `src/`: bootloader firmware.
- `ecus.json`: ECU CAN IDs, bitrate, and CAN peripheral/pins.
- `config.rs`: configuration parsing and validation.
- `memory.x`: fixed flash/RAM regions and runtime bounds exported as linker symbols.
- `build.rs`: CAN configuration generation and linker setup.
- `tools/`: build and probe helper.

## Build and flash

Run from this directory, replacing `<ecu>` with `bms`, `vcu`, or another configured name:

```sh
cargo build-atlas <ecu>
cargo download-atlas <ecu>  # Build, download, and reset through probe-rs.
cargo run-atlas <ecu>       # Build and run with probe logs.
```

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

## Adding an ECU

Add an entry to `ecus.json` with unique CAN IDs and supported hardware settings.
Use its name with the same commands. There is no platform selection.
