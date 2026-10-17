MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 0x2E000
  ACTIVE : ORIGIN = 0x0802E000, LENGTH = 0xD2000
  DFU : ORIGIN = 0x08100000, LENGTH = 0xFA000
  BOOTLOADER_STATE : ORIGIN = 0x081FA000, LENGTH = 0x2000
  RAM : ORIGIN = 0x20000000, LENGTH = 0xA0000
}

__bootloader_active_address = ORIGIN(ACTIVE);
__bootloader_active_size = LENGTH(ACTIVE);
__bootloader_sram_start = ORIGIN(RAM);
__bootloader_sram_end = ORIGIN(RAM) + LENGTH(RAM);

__bootloader_active_start = ORIGIN(ACTIVE) - 0x08000000;
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - 0x08000000;

__bootloader_dfu_start = ORIGIN(DFU) - 0x08100000;
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - 0x08100000;

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - 0x08100000;
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - 0x08100000;
