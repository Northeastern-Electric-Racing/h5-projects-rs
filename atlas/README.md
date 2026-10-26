# Atlas CAN bootloader

CAN bootloader with per-ECU configuration, firmware verification, and software
swap/rollback. All ECU builds use one bootloader implementation and memory layout.
CAN IDs, frame formats, and bitrate come from Odyssey-Definitions’ `can-bootloader/ecus.json`
at build time. Runtime bitrate changes are not supported.

## Build and flash

1. From inside the `atlas/` project folder, select an ECU supported by `build.rs`.

2. Build the bootloader:

   ```sh
   BOOTLOADER_ECU=<ecu> cargo build --release
   ```

3. Download the bootloader:

   ```sh
   BOOTLOADER_ECU=<ecu> cargo download
   ```

   Use `BOOTLOADER_ECU=<ecu> cargo run --release` to flash and view probe logs instead.

Use the same `ecus.json` on the flashing host.

Probe-rs can still flash applications normally over the debug probe.
Flash Atlas first, then the application linked with the shared application layout.
Do not perform a full-chip erase when updating the application, since it also removes Atlas.

## Update flow

1. Firmware updates begin with a CAN boot-entry request sent to the running application.
2. Boot-control records the request in flash and resets the MCU. The bootloader
   clears the request and stays in update mode.
3. The host transfers the new firmware into DFU storage, leaving the current
   application intact. The bootloader verifies the image's CRC and startup addresses.
4. The host requests activation. The bootloader records the pending update,
   acknowledges the command over CAN, and resets.
5. The bootloader swaps the new image into the application region and starts it.
   The application confirms startup through boot-control; otherwise, the bootloader
   rolls back to the previous image on a subsequent reset.

If communication stops before activation, the bootloader stays in update mode.
Resetting the MCU starts the existing valid application, provided the entry request
was cleared successfully. If no valid application exists, the bootloader stays in
update mode to receive firmware. On normal boots without an update request, the
bootloader starts the valid application immediately.

## Application integration

Applications use `boot-control` from firmware-rs for boot confirmation and CAN bootloader entry:

```toml
[dependencies]
boot-control.workspace = true
```

Create a `boot-control` instance using the flash peripheral, confirm startup, and
pass it to the CAN task:

```rust
let mut bootloader = boot_control::Bootloader::new(p.FLASH);
// Confirm basic startup to prevent rollback on the next reset.
// Device initialization continues in application tasks.
bootloader
    .confirm_boot()
    .expect("Failed to confirm application startup");
can::start(spawner, can_resources, bootloader);
```

Calling `confirm_boot()` keeps the new firmware from being rolled back on the next
reset. The projects call it after basic startup, while device setup continues in tasks.

Add the ECU's boot-request ID to the CAN receive filter. In the receive task, use
the generated parser to identify the request and pass its data to `boot-control`:

```rust
match Msg::decode(&frame) {
    Some(Msg::MsbFrontBootRequest(_)) => bootloader
        .handle_request(frame.data())
        .unwrap_or_else(|_| defmt::error!("Failed to persist bootloader request")),
    _ => {}
}
```

Use the generated boot-request type for the selected ECU. Boot-control validates the boot-request data,
saves the boot-entry request, and resets only after a successful state write.

Use the shared application memory layout provided by boot-control in firmware-rs.

## Adding an ECU

Add the ECU to Odyssey-Definitions’ `ecus.json` and boot-entry definitions in `ajax.json`, then update the
config dependency. Add its name to `atlas/build.rs` and configure its CAN controller
and RX/TX pins in `atlas/src/main.rs` and `atlas/src/can.rs`.
