//! ECU configuration source used by platform build scripts; not generated output.

use serde::Deserialize;
use std::{
    collections::{BTreeMap, HashSet},
    fs,
    path::Path,
};

type Result<T> = std::result::Result<T, String>;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub schema_version: u32,
    pub supported_bit_rates: Vec<u32>,
    pub platforms: BTreeMap<String, Platform>,
    pub ecus: BTreeMap<String, Ecu>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Platform {
    pub cargo_package: String,
    pub rust_target: String,
    pub chip: String,
    pub memory: Memory,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Memory {
    pub bootloader: Region,
    pub application: Region,
    pub dfu: Region,
    pub state: Region,
    pub ram: Region,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Region {
    #[serde(deserialize_with = "unsigned")]
    pub address: u32,
    #[serde(deserialize_with = "unsigned")]
    pub size: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ecu {
    pub label: String,
    pub platform: String,
    pub hardware: Hardware,
    pub can: Can,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Hardware {
    pub hse_hz: u32,
    pub can_peripheral: String,
    pub can_rx: String,
    pub can_tx: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Can {
    pub default_bit_rate: u32,
    #[serde(deserialize_with = "unsigned")]
    pub request_id: u16,
    #[serde(deserialize_with = "unsigned")]
    pub response_id: u16,
    #[serde(deserialize_with = "unsigned")]
    pub write_data_id: u16,
    #[serde(deserialize_with = "unsigned")]
    pub application_boot_request_id: u16,
    #[serde(deserialize_with = "bytes")]
    pub application_boot_request_data: Vec<u8>,
}

// JSON uses decimal numbers or quoted hexadecimal strings for these fields.
#[derive(Deserialize)]
#[serde(untagged)]
enum Unsigned {
    Decimal(u64),
    Hex(String),
}

impl Unsigned {
    fn value<T: TryFrom<u64>>(self) -> Result<T> {
        let value = match self {
            Self::Decimal(value) => value,
            Self::Hex(text) => {
                let digits = text
                    .strip_prefix("0x")
                    .or_else(|| text.strip_prefix("0X"))
                    .ok_or_else(|| format!("Hex value needs a 0x prefix: {text}"))?;
                if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_hexdigit()) {
                    return Err(format!("Invalid hex value: {text}"));
                }
                u64::from_str_radix(digits, 16)
                    .map_err(|_| format!("Hex value overflows: {text}"))?
            }
        };
        T::try_from(value).map_err(|_| format!("Value {value:#x} exceeds field range"))
    }
}

fn unsigned<'de, D: serde::Deserializer<'de>, T: TryFrom<u64>>(
    deserializer: D,
) -> std::result::Result<T, D::Error> {
    Unsigned::deserialize(deserializer)?
        .value()
        .map_err(serde::de::Error::custom)
}

fn bytes<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> std::result::Result<Vec<u8>, D::Error> {
    Vec::<Unsigned>::deserialize(deserializer)?
        .into_iter()
        .map(|value| value.value().map_err(serde::de::Error::custom))
        .collect()
}

impl Region {
    pub fn bounds(&self) -> Result<(u32, u32)> {
        let start = self.address;
        let size = self.size;
        let end = start.checked_add(size).ok_or("Memory region overflows")?;
        if size == 0 {
            return Err("Memory region cannot be empty".into());
        }
        Ok((start, end))
    }
}

impl Config {
    pub fn read(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let config: Self =
            serde_json::from_str(&text).map_err(|e| format!("Invalid ECU configuration: {e}"))?;
        config.validate()?;
        Ok(config)
    }

    pub fn select(&self, name: &str) -> Result<(&Ecu, &Platform)> {
        let ecu = self.ecus.get(&name.to_ascii_lowercase()).ok_or_else(|| {
            format!(
                "Unknown ECU {name}; choose: {}",
                self.ecus.keys().cloned().collect::<Vec<_>>().join(", ")
            )
        })?;
        Ok((ecu, &self.platforms[&ecu.platform]))
    }

    fn validate(&self) -> Result<()> {
        if self.schema_version != 1 {
            return Err("Unsupported schema_version; expected 1".into());
        }
        if self.ecus.is_empty() {
            return Err("No ECUs configured".into());
        }
        // Position in this array is the existing SET_BAUD_RATE wire code.
        if self.supported_bit_rates != [125_000, 250_000, 500_000, 1_000_000] {
            return Err("supported_bit_rates must match the protocol bitrate codes".into());
        }
        for platform in self.platforms.values() {
            let m = &platform.memory;
            let regions = [&m.bootloader, &m.application, &m.dfu, &m.state, &m.ram];
            let mut bounds = Vec::new();
            for region in regions {
                let (start, end) = region.bounds()?;
                if bounds.iter().any(|&(s, e)| start < e && s < end) {
                    return Err("Memory regions overlap".into());
                }
                bounds.push((start, end));
            }
        }
        let mut ids = HashSet::new();
        for (name, ecu) in &self.ecus {
            if ecu.label.trim().is_empty() {
                return Err(format!("ECU label cannot be empty: {name}"));
            }
            // Names also become artifact filenames; keep them portable and unambiguous.
            if name.is_empty()
                || !name
                    .bytes()
                    .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
            {
                return Err(format!(
                    "ECU name must use lowercase letters, digits, or hyphens: {name}"
                ));
            }
            if !self.platforms.contains_key(&ecu.platform) {
                return Err(format!("Unknown platform for {name}: {}", ecu.platform));
            }
            if !self.supported_bit_rates.contains(&ecu.can.default_bit_rate) {
                return Err(format!("Unsupported default bitrate for {name}"));
            }
            for id in [
                ecu.can.request_id,
                ecu.can.response_id,
                ecu.can.write_data_id,
                ecu.can.application_boot_request_id,
            ] {
                if id > 0x7ff {
                    return Err(format!("CAN ID {id:#x} must fit in 11 bits"));
                }
                if !ids.insert(id) {
                    return Err(format!("Duplicate CAN ID {id:#x}"));
                }
            }
            if !(1..=8).contains(&ecu.can.application_boot_request_data.len()) {
                return Err(format!("Invalid application boot request for {name}"));
            }
        }
        Ok(())
    }
}

impl Platform {
    /// Match the bank-relative flash implementation compiled by this platform package.
    pub fn validate_h563(&self, ecu: &Ecu) -> Result<()> {
        if self.rust_target != "thumbv8m.main-none-eabihf" || self.chip != "STM32H563ZITx" {
            return Err("Selected platform does not match the STM32H563ZI package".into());
        }
        let hw = &ecu.hardware;
        hw.validate_h563_can()?;
        if hw.hse_hz != 25_000_000 {
            return Err("This platform currently supports a 25 MHz HSE oscillator".into());
        }
        let m = &self.memory;
        for (region, bank_start, bank_end) in [
            (&m.bootloader, 0x0800_0000, 0x0810_0000),
            (&m.application, 0x0800_0000, 0x0810_0000),
            (&m.dfu, 0x0810_0000, 0x0820_0000),
            (&m.state, 0x0810_0000, 0x0820_0000),
        ] {
            let (start, end) = region.bounds()?;
            if start < bank_start || end > bank_end || start % 8192 != 0 || end % 8192 != 0 {
                return Err(
                    "Flash partitions must be bank-contained and aligned to 8 KiB sectors".into(),
                );
            }
        }
        if m.bootloader.address != 0x0800_0000 {
            return Err("Bootloader must start at 0x08000000".into());
        }
        let (ram_start, ram_end) = m.ram.bounds()?;
        if ram_start < 0x2000_0000
            || ram_end > 0x200a_0000
            || ram_start % 8 != 0
            || ram_end % 8 != 0
        {
            return Err("RAM must fit H563 SRAM and be 8-byte aligned".into());
        }
        if m.dfu.size < m.application.size + 8192 {
            return Err("DFU needs at least one extra erase sector beyond the application".into());
        }
        // Reserve progress entries for both the swap and rollback, in flash write units.
        let state_needed = (2 + 4 * (m.application.size / 8192)) * 16;
        if m.state.size < state_needed {
            return Err("State partition is too small for swap progress".into());
        }
        Ok(())
    }
}

impl Hardware {
    /// Accept CAN pins exposed by the pinned STM32H563ZI driver metadata.
    fn validate_h563_can(&self) -> Result<()> {
        let (rx_pins, tx_pins): (&[&str], &[&str]) = match self.can_peripheral.as_str() {
            "FDCAN1" => (
                &["PA11", "PB8", "PD0", "PE0"],
                &["PA12", "PB7", "PB9", "PD1", "PD5", "PE1"],
            ),
            "FDCAN2" => (&["PB5", "PB12", "PD9"], &["PA10", "PB6", "PB13"]),
            _ => {
                return Err(format!(
                    "Unsupported H563 CAN peripheral: {}; choose FDCAN1 or FDCAN2",
                    self.can_peripheral
                ));
            }
        };
        if !rx_pins.contains(&self.can_rx.as_str()) {
            return Err(format!(
                "Invalid RX pin {} for {}; choose {}",
                self.can_rx,
                self.can_peripheral,
                rx_pins.join(", ")
            ));
        }
        if !tx_pins.contains(&self.can_tx.as_str()) {
            return Err(format!(
                "Invalid TX pin {} for {}; choose {}",
                self.can_tx,
                self.can_peripheral,
                tx_pins.join(", ")
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const FIXTURE: &str = include_str!("ecus.json");

    #[test]
    fn selects_both_ecus_with_distinct_ids() {
        let config: Config = serde_json::from_str(FIXTURE).unwrap();
        config.validate().unwrap();
        for name in ["BMS", "vcu"] {
            let (ecu, platform) = config.select(name).unwrap();
            platform.validate_h563(ecu).unwrap();
        }
        assert_eq!(config.select("bms").unwrap().0.can.request_id, 0x10);
        assert_eq!(config.select("vcu").unwrap().0.can.request_id, 0x14);
        assert_eq!(
            config
                .select("bms")
                .unwrap()
                .0
                .can
                .application_boot_request_data,
            [0xb0, 7, 0x10, 0xad]
        );
        assert!(config.select("missing").is_err());
    }

    fn modified(pointer: &str, value: serde_json::Value) -> serde_json::Value {
        let mut config: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
        *config.pointer_mut(pointer).unwrap() = value;
        config
    }

    #[test]
    fn accepts_decimal_numbers_alongside_hex_strings() {
        let mut value: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
        value["ecus"]["bms"]["can"]["request_id"] = serde_json::json!(16);
        value["ecus"]["bms"]["can"]["application_boot_request_data"] =
            serde_json::json!([176, "0X07", 16, "0xad"]);
        value["platforms"]["stm32h563zi"]["memory"]["bootloader"]["address"] =
            serde_json::json!(0x08000000);
        let config: Config = serde_json::from_value(value).unwrap();
        config.validate().unwrap();
        let (ecu, platform) = config.select("bms").unwrap();
        platform.validate_h563(ecu).unwrap();
        assert_eq!(ecu.can.request_id, 16);
        assert_eq!(ecu.can.application_boot_request_data, [176, 7, 16, 173]);
    }

    #[test]
    fn rejects_invalid_configuration_and_unimplemented_hardware() {
        for (pointer, value) in [
            ("/schema_version", serde_json::json!(2)),
            ("/ecus/vcu/can/request_id", serde_json::json!(16)),
            ("/ecus/bms/can/request_id", serde_json::json!(2048)),
            ("/ecus/bms/platform", serde_json::json!("unknown")),
            (
                "/ecus/bms/can/application_boot_request_data",
                serde_json::json!([]),
            ),
            (
                "/platforms/stm32h563zi/memory/application/address",
                serde_json::json!(0x08000000),
            ),
            (
                "/platforms/stm32h563zi/memory/application/size",
                serde_json::json!(0xD2001),
            ),
            (
                "/platforms/stm32h563zi/memory/dfu/size",
                serde_json::json!(0xD2000),
            ),
            (
                "/platforms/stm32h563zi/memory/state/address",
                serde_json::json!(0x08200000),
            ),
            ("/ecus/bms/hardware/can_rx", serde_json::json!("PA0")),
        ] {
            let config: Config = serde_json::from_value(modified(pointer, value)).unwrap();
            let result = config.validate().and_then(|()| {
                let (ecu, platform) = config.select("bms")?;
                platform.validate_h563(ecu)
            });
            assert!(result.is_err(), "{pointer}");
        }
    }

    #[test]
    fn rejects_wrong_types_overflow_and_unknown_fields() {
        for (pointer, value) in [
            ("/ecus/bms/can/request_id", serde_json::json!("0xGG")),
            ("/ecus/bms/can/request_id", serde_json::json!("010")),
            ("/ecus/bms/can/request_id", serde_json::json!("0x10000")),
            ("/ecus/bms/can/request_id", serde_json::json!("0x")),
            (
                "/ecus/bms/can/application_boot_request_data",
                serde_json::json!(["0x100"]),
            ),
            ("/ecus/bms/can/request_id", serde_json::json!(-1)),
            ("/ecus/bms/can/request_id", serde_json::json!(65536)),
            (
                "/platforms/stm32h563zi/memory/bootloader/address",
                serde_json::json!(0x100000000_u64),
            ),
            (
                "/ecus/bms/can/application_boot_request_data",
                serde_json::json!([256]),
            ),
        ] {
            assert!(
                serde_json::from_value::<Config>(modified(pointer, value)).is_err(),
                "{pointer}"
            );
        }
        let mut unknown: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
        unknown["unknown"] = serde_json::json!(true);
        assert!(serde_json::from_value::<Config>(unknown).is_err());
    }

    #[test]
    fn accepts_supported_can_peripherals_and_pin_mappings() {
        for (peripheral, rx_pins, tx_pins) in [
            (
                "FDCAN1",
                &["PA11", "PB8", "PD0", "PE0"][..],
                &["PA12", "PB7", "PB9", "PD1", "PD5", "PE1"][..],
            ),
            (
                "FDCAN2",
                &["PB5", "PB12", "PD9"][..],
                &["PA10", "PB6", "PB13"][..],
            ),
        ] {
            for rx in rx_pins {
                for tx in tx_pins {
                    let mut config: Config = serde_json::from_str(FIXTURE).unwrap();
                    let hw = &mut config.ecus.get_mut("bms").unwrap().hardware;
                    hw.can_peripheral = peripheral.into();
                    hw.can_rx = (*rx).into();
                    hw.can_tx = (*tx).into();
                    let (ecu, platform) = config.select("bms").unwrap();
                    platform.validate_h563(ecu).unwrap();
                }
            }
        }
    }

    #[test]
    fn rejects_can_pins_from_another_peripheral_or_direction() {
        for (peripheral, rx, tx) in [
            ("FDCAN3", "PD9", "PB13"),
            ("FDCAN1", "PD9", "PA12"),
            ("FDCAN1", "PA11", "PB13"),
            ("FDCAN2", "PA11", "PB13"),
            ("FDCAN2", "PB13", "PD9"),
        ] {
            let hw = Hardware {
                hse_hz: 25_000_000,
                can_peripheral: peripheral.into(),
                can_rx: rx.into(),
                can_tx: tx.into(),
            };
            assert!(hw.validate_h563_can().is_err());
        }
    }
}
