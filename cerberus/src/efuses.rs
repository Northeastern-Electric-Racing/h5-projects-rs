use embassy_stm32::gpio::{Input, Output};

#[derive(PartialEq, Eq)]
pub enum EfuseControlState {
    EfuseOn,
    EfuseOff,
    EfuseAuto
}

pub struct Efuse {
    en_pin: Output<'static>,
    er_pin: Input<'static>,
    scale: f32,
    control_state: EfuseControlState
}

const GAIN_IMON: f32 = 27.9e-6;

/// Volts-per-amp scale factor for a given IMON sense resistor (ohms).
const fn scale(r_imon: f32) -> f32 {
    1.0 / (GAIN_IMON * r_imon * 1000.0)
}

impl Efuse {
    fn new(en_pin: Output<'static>, er_pin: Input<'static>, scale_factor: f32,  default_state: EfuseControlState) -> Self { 
        Efuse { en_pin: en_pin, er_pin: er_pin, scale: scale(scale_factor), control_state: default_state }
    }

    fn enable(&mut self)  {
        self.en_pin.set_high();
    }

    fn disable(&mut self) {
        self.en_pin.set_low();
    }

    fn update_control_state(&mut self, new_control_state: EfuseControlState) {
        self.control_state = new_control_state;
    }

    fn get_control_state(self) -> EfuseControlState {
        self.control_state
    }
}