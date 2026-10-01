/* Generated from the selected ECU's platform in ecus.json. */
INCLUDE memory-regions.x

__bootloader_flash_address = ORIGIN(FLASH);
__bootloader_active_address = ORIGIN(ACTIVE);

__bootloader_active_start = ORIGIN(ACTIVE) - 0x08000000;
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - 0x08000000;

__bootloader_dfu_start = ORIGIN(DFU) - 0x08100000;
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - 0x08100000;

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - 0x08100000;
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - 0x08100000;
