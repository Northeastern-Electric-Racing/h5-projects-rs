# Atlas CAN bootloader

CAN bootloader with per-ECU settings, firmware verification, and software
swap/rollback. All ECU builds use one bootloader implementation and memory layout.
CAN IDs, frame formats, and bitrate come from Odyssey-Definitions’ `can-bootloader/ecus.json`
at build time. Runtime bitrate changes are not supported.

## Layout

- `src/`: bootloader firmware.
- `memory.x`: fixed flash/RAM regions and runtime bounds exported as linker symbols.
- `build.rs`: ECU selection and linker setup.

## Build and flash

Atlas defaults to BMS, including when building from the workspace root.
From `atlas/`, override it with `bms`, `vcu`, `lightning`, `msb_front`, or `msb_back`:

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
`atlas/` and use the ELF beneath that directory.

CAN constants are generated automatically for `BOOTLOADER_ECU`.
Pins and controller bindings live in `src/main.rs` and `src/can.rs`.
Use the same `ecus.json` on the flashing host.
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

The `boot-control` helper handles CAN boot requests and startup confirmation.
Create it with `boot_control::Bootloader::new(p.FLASH, request_id)`. The H563
helper handles the boot-state partition, scratch buffer, and reset.
Call `confirm_boot()` after required startup checks and `handle_frame()` from the
receive task. Configure CAN and its boot-request filter in the application as usual.
The fixed boot-request payload is `B0 07 10 AD`.

Applications use the shared `boot-control/platforms/stm32h563/memory.x` in
firmware-rs. The platform crate exports its path to application build scripts,
which copy it into `OUT_DIR` for linking.
The application helper does not stage firmware or perform swaps. Flash Atlas for
the same ECU first, then download the application ELF without a chip erase.

## Adding an ECU

Add the ECU to Odyssey-Definitions’ `ecus.json` and boot-entry definitions, then update the
config dependency. Add its name to `build.rs` and pins to `src/main.rs` and `src/can.rs`.
