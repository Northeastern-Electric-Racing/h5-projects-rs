use defmt::info;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::{Peri, peripherals};
use embassy_time::Timer;
use variant_count::VariantCount;

use crate::adc::{self, AdcMuxData};
use crate::efuses;

const V_REF: f32 = 3.3;
const MAX_TWELVE_BIT_RESOUTION: u16 = 4095;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum EfuseControlState {
    EfuseOn,
    EfuseOff,
    EfuseAuto,
}

#[derive(Clone, Copy, PartialEq, Eq, VariantCount)]
pub enum EfuseId {
    EfuseDashboard,
    EfuseBrake,
    EfuseShutdown,
    EfuseLV,
    EfuseRadfan,
    EfuseFanbatt,
    EfusePump1,
    EfusePump2,
    EfuseBattbox,
    EfuseMC,
    EfuseSpare,
}

enum PredicateOperation {
    GREATER,
    LESS,
    GREQ,
    LEQ,
    EQ,
}

pub struct AutoPredicate {
    value1: f32,
    value2: f32,
    operation: PredicateOperation,
}

pub struct Efuse {
    efuse_id: EfuseId,
    en_pin: Output<'static>,
    er_pin: Input<'static>,
    scale: f32,
    control_state: EfuseControlState,
    auto_on_predicate: Option<AutoPredicate>,
    auto_off_predicate: Option<AutoPredicate>,
}

const GAIN_IMON: f32 = 27.9e-6;

/// Volts-per-amp scale factor for a given IMON sense resistor (ohms).
const fn scale(r_imon: f32) -> f32 {
    1.0 / (GAIN_IMON * r_imon * 1000.0)
}

pub struct EfusePins {
    pub dashboard_en: Peri<'static, peripherals::PD0>,
    pub dashboard_er: Peri<'static, peripherals::PD1>,
    pub brake_en: Peri<'static, peripherals::PD8>,
    pub brake_er: Peri<'static, peripherals::PD9>,
    pub shutdown_en: Peri<'static, peripherals::PG6>,
    pub shutdown_er: Peri<'static, peripherals::PG7>,
    pub lv_en: Peri<'static, peripherals::PD6>,
    pub lv_er: Peri<'static, peripherals::PD7>,
    pub radfan_en: Peri<'static, peripherals::PG4>,
    pub radfan_er: Peri<'static, peripherals::PG5>,
    pub fanbatt_en: Peri<'static, peripherals::PD10>,
    pub fanbatt_er: Peri<'static, peripherals::PD11>,
    pub pump1_en: Peri<'static, peripherals::PD12>,
    pub pump1_er: Peri<'static, peripherals::PD13>,
    pub pump2_en: Peri<'static, peripherals::PD14>,
    pub pump2_er: Peri<'static, peripherals::PD15>,
    pub battbox_en: Peri<'static, peripherals::PF2>,
    pub battbox_er: Peri<'static, peripherals::PF3>,
    pub mc_en: Peri<'static, peripherals::PF4>,
    pub mc_er: Peri<'static, peripherals::PF5>,
    pub spare_en: Peri<'static, peripherals::PG10>,
    pub spare_er: Peri<'static, peripherals::PG11>,
}

const ER_PULL: Pull = Pull::Up;
const EN_SPEED: Speed = Speed::Low;

impl Efuse {
    pub fn new(
        efuse_id: EfuseId,
        en_pin: Output<'static>,
        er_pin: Input<'static>,
        scale_factor: f32,
        default_state: EfuseControlState,
        auto_on_predicate: Option<AutoPredicate>,
        auto_off_predicate: Option<AutoPredicate>,
    ) -> Self {
        Efuse {
            efuse_id: efuse_id,
            en_pin: en_pin,
            er_pin: er_pin,
            scale: scale(scale_factor),
            control_state: default_state,
            auto_on_predicate,
            auto_off_predicate,
        }
    }

    pub fn init_all(pins: EfusePins) -> [Efuse; EfuseId::VARIANT_COUNT] {
        [
            Efuse::new(
                EfuseId::EfuseDashboard,
                Output::new(pins.dashboard_en, Level::Low, EN_SPEED),
                Input::new(pins.dashboard_er, ER_PULL),
                39.0,
                EfuseControlState::EfuseOn,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfuseBrake,
                Output::new(pins.brake_en, Level::Low, EN_SPEED),
                Input::new(pins.brake_er, ER_PULL),
                200.0,
                EfuseControlState::EfuseAuto,
                Some(AutoPredicate {
                    value1: 0.0,
                    value2: 0.0,
                    operation: PredicateOperation::EQ,
                }),
                Some(AutoPredicate {
                    value1: 0.0,
                    value2: 0.0,
                    operation: PredicateOperation::EQ,
                }),
            ),
            Efuse::new(
                EfuseId::EfuseShutdown,
                Output::new(pins.shutdown_en, Level::High, EN_SPEED),
                Input::new(pins.shutdown_er, ER_PULL),
                110.0,
                EfuseControlState::EfuseOn,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfuseLV,
                Output::new(pins.lv_en, Level::High, EN_SPEED),
                Input::new(pins.lv_er, ER_PULL),
                39.0,
                EfuseControlState::EfuseOn,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfuseRadfan,
                Output::new(pins.radfan_en, Level::Low, EN_SPEED),
                Input::new(pins.radfan_er, ER_PULL),
                56.0,
                EfuseControlState::EfuseAuto,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfuseFanbatt,
                Output::new(pins.fanbatt_en, Level::Low, EN_SPEED),
                Input::new(pins.fanbatt_er, ER_PULL),
                27.0,
                EfuseControlState::EfuseAuto,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfusePump1,
                Output::new(pins.pump1_en, Level::Low, EN_SPEED),
                Input::new(pins.pump1_er, ER_PULL),
                47.0,
                EfuseControlState::EfuseAuto,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfusePump2,
                Output::new(pins.pump2_en, Level::Low, EN_SPEED),
                Input::new(pins.pump2_er, ER_PULL),
                47.0,
                EfuseControlState::EfuseAuto,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfuseBattbox,
                Output::new(pins.battbox_en, Level::High, EN_SPEED),
                Input::new(pins.battbox_er, ER_PULL),
                56.0,
                EfuseControlState::EfuseOn,
                None,
                None,
            ),
            Efuse::new(
                EfuseId::EfuseMC,
                Output::new(pins.mc_en, Level::Low, EN_SPEED),
                Input::new(pins.mc_er, ER_PULL),
                56.0,
                EfuseControlState::EfuseOn,
                None,
                None,
            ),
            // Spare has no IMON sense resistor and no ADC channel, so its scale
            // is never used: `AdcMux::get_efuse_data` returns None for it.
            Efuse::new(
                EfuseId::EfuseSpare,
                Output::new(pins.spare_en, Level::Low, EN_SPEED),
                Input::new(pins.spare_er, ER_PULL),
                0.0,
                EfuseControlState::EfuseAuto,
                Some(AutoPredicate {
                    value1: 0.0,
                    value2: 0.0,
                    operation: PredicateOperation::EQ,
                }),
                Some(AutoPredicate {
                    value1: 0.0,
                    value2: 0.0,
                    operation: PredicateOperation::EQ,
                }),
            ),
        ]
    }

    pub fn get_id(&self) -> EfuseId {
        self.efuse_id
    }

    pub fn enable(&mut self) {
        self.en_pin.set_high();
    }

    pub fn disable(&mut self) {
        self.en_pin.set_low();
    }

    pub fn update_control_state(&mut self, new_control_state: EfuseControlState) {
        self.control_state = new_control_state;
    }

    pub fn get_control_state(&self) -> EfuseControlState {
        self.control_state
    }

    pub fn get_data(&self, adc_data: &AdcMuxData) -> Option<(f32, f32)> {
        if let Some(data) = adc_data.get_efuse_data(self.efuse_id) {
            let voltage = (data / MAX_TWELVE_BIT_RESOUTION) as f32 * V_REF;
            let current = voltage * self.scale;
            Some((voltage, current))
        } else {
            None
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.en_pin.is_set_high()
    }

    pub fn is_faulted(&self) -> bool {
        self.er_pin.is_low()
    }
}

const EFUSE_PERIOD_MS: u64 = 100;

#[embassy_executor::task]
pub async fn efuse_task(pins: EfusePins) {
    let mut _efuses: [Efuse; _] = Efuse::init_all(pins);

    loop {
        for efuse in &mut _efuses {
            match efuse.get_control_state() {
                EfuseControlState::EfuseOff => {
                    efuse.disable();
                }
                EfuseControlState::EfuseOn => {
                    efuse.enable();
                }
                EfuseControlState::EfuseAuto => {
                    let turn_on = match &efuse.auto_on_predicate {
                        Some(auto_mode_predicate) => match auto_mode_predicate.operation {
                            PredicateOperation::LESS => {
                                auto_mode_predicate.value1 < auto_mode_predicate.value2
                            }
                            PredicateOperation::GREATER => {
                                auto_mode_predicate.value1 > auto_mode_predicate.value2
                            }
                            PredicateOperation::GREQ => {
                                auto_mode_predicate.value1 >= auto_mode_predicate.value2
                            }
                            PredicateOperation::LEQ => {
                                auto_mode_predicate.value1 <= auto_mode_predicate.value2
                            }
                            PredicateOperation::EQ => {
                                auto_mode_predicate.value1 == auto_mode_predicate.value2
                            }
                        },
                        None => true,
                    };

                    let turn_off = match &efuse.auto_off_predicate {
                        Some(auto_mode_predicate) => match auto_mode_predicate.operation {
                            PredicateOperation::LESS => {
                                auto_mode_predicate.value1 < auto_mode_predicate.value2
                            }
                            PredicateOperation::GREATER => {
                                auto_mode_predicate.value1 > auto_mode_predicate.value2
                            }
                            PredicateOperation::GREQ => {
                                auto_mode_predicate.value1 >= auto_mode_predicate.value2
                            }
                            PredicateOperation::LEQ => {
                                auto_mode_predicate.value1 <= auto_mode_predicate.value2
                            }
                            PredicateOperation::EQ => {
                                auto_mode_predicate.value1 == auto_mode_predicate.value2
                            }
                        },
                        None => true,
                    };

                    if turn_on {
                        efuse.enable();
                    } else if turn_off {
                        efuse.disable();
                    }
                }
            }
        }

        let adc = adc::data().await; // lock, copy, unlock
        for efuse in &_efuses {
            if let Some((voltage, current)) = efuse.get_data(&adc) {
                info!("Voltage: {}, Current: {}", voltage, current);
            }
        }
    }
}
