use embassy_stm32::{gpio::{Input, Output}};
use crate::adc::{Adc1Channels, AdcMux};

const V_REF: f32 = 3.3; 
const MAX_TWELVE_BIT_RESOUTION: u16 = 4095;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum EfuseControlState {
    EfuseOn,
    EfuseOff,
    EfuseAuto
}

#[derive(Clone, Copy, PartialEq, Eq)]
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

pub struct Efuse {
    efuse_id: EfuseId,
    en_pin: Output<'static>,
    er_pin: Input<'static>,
    scale: f32,
    control_state: EfuseControlState,
}

const GAIN_IMON: f32 = 27.9e-6;

/// Volts-per-amp scale factor for a given IMON sense resistor (ohms).
const fn scale(r_imon: f32) -> f32 {
    1.0 / (GAIN_IMON * r_imon * 1000.0)
}

impl Efuse {
    pub fn new(efuse_id: EfuseId, en_pin: Output<'static>, er_pin: Input<'static>, scale_factor: f32,  default_state: EfuseControlState, adc_channel: Adc1Channels) -> Self { 
        Efuse { efuse_id: efuse_id, en_pin: en_pin, er_pin: er_pin, scale: scale(scale_factor), control_state: default_state }
    }

    pub fn get_id(&self) -> EfuseId {
        self.efuse_id
    }

    pub fn enable(&mut self)  {
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

    pub fn get_data(self, adc_mux: &AdcMux) -> Option<(f32, f32)> {
        if let Some(data) = adc_mux.get_efuse_data(self.efuse_id) {
            let voltage = (data / MAX_TWELVE_BIT_RESOUTION) as f32 * V_REF;
            let current = voltage * self.scale;
            Some((voltage, current))
        } else {
            None
        }
    }

    pub fn is_enabled(self) -> bool {
        self.en_pin.is_set_high()
    }

    pub fn is_faulted(self) -> bool {
        self.er_pin.is_high()
    }
}

