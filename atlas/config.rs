//! ECU configuration source used by build scripts; not generated output.

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
    pub ecus: BTreeMap<String, Ecu>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ecu {
    pub label: String,
    pub hardware: Hardware,
    pub can: Can,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Hardware {
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

impl Config {
    pub fn read(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let config: Self =
            serde_json::from_str(&text).map_err(|e| format!("Invalid ECU configuration: {e}"))?;
        config.validate()?;
        Ok(config)
    }

    pub fn select(&self, name: &str) -> Result<&Ecu> {
        let ecu = self.ecus.get(&name.to_ascii_lowercase()).ok_or_else(|| {
            format!(
                "Unknown ECU {name}; choose: {}",
                self.ecus.keys().cloned().collect::<Vec<_>>().join(", ")
            )
        })?;
        Ok(ecu)
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
            ecu.hardware
                .validate_h563_can()
                .map_err(|e| format!("ecus.{name}.hardware: {e}"))?;
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
            assert!(config.select(name).is_ok());
        }
        assert_eq!(config.select("bms").unwrap().can.request_id, 0x10);
        assert_eq!(config.select("vcu").unwrap().can.request_id, 0x14);
        assert_eq!(
            config
                .select("bms")
                .unwrap()
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
        let config: Config = serde_json::from_value(value).unwrap();
        config.validate().unwrap();
        let ecu = config.select("bms").unwrap();
        assert_eq!(ecu.can.request_id, 16);
        assert_eq!(ecu.can.application_boot_request_data, [176, 7, 16, 173]);
    }

    #[test]
    fn rejects_invalid_configuration_and_unimplemented_hardware() {
        for (pointer, value) in [
            ("/schema_version", serde_json::json!(2)),
            ("/ecus/vcu/can/request_id", serde_json::json!(16)),
            ("/ecus/bms/can/request_id", serde_json::json!(2048)),
            ("/ecus/bms/can/default_bit_rate", serde_json::json!(123)),
            (
                "/ecus/bms/can/application_boot_request_data",
                serde_json::json!([]),
            ),
            ("/ecus/bms/hardware/can_rx", serde_json::json!("PA0")),
        ] {
            let config: Config = serde_json::from_value(modified(pointer, value)).unwrap();
            let result = config.validate();
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
                    config.validate().unwrap();
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
                can_peripheral: peripheral.into(),
                can_rx: rx.into(),
                can_tx: tx.into(),
            };
            assert!(hw.validate_h563_can().is_err());
        }
    }
}
