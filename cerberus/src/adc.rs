use embassy_stm32::{gpio::Output};
use variant_count::VariantCount;

use crate::efuses::{EfuseId};

#[derive(PartialEq, Eq)]
enum AdcMuxSel {
    SelLow,
    SelHigh
}

#[derive(VariantCount)]
pub enum Adc1Channels {
    Adc1Channel3,  // EF_DASH_ADC
    Adc1Channel0,  // Mux 1
    Adc1Channel5,  // Mux 3
    Adc1Channel9,  // Mux 4
    Adc1Channel6,  // EF_FANBATT_ADC
    Adc1Channel2,  // EF_PUMP1_ADC
    Adc1Channel13, // EF_PUMP2_ADC
    Adc1Channel18, // EF_MC_ADC
    Adc1Channel15, // Mux 2
}

#[derive(VariantCount)]
enum MuxDataIndex { 
    Sel1High, // BREAKLIGHT_ADC
    Sel1Low,  // BATTBOX_ADC

    Sel2High, // LFIU_CURRENT_1
    Sel2Low,  // LFIU_CURRENT_2

    Sel3High, // LV_ADC
    Sel3Low,  // SHUTDOWN_ADC

    Sel4High, // RADFAN_ADC
    Sel4Low   // LV_BATT_ADC
}

pub struct AdcMux {
    mux_sel: AdcMuxSel,
    mux_buffer: [u16; MuxDataIndex::VARIANT_COUNT],
    sel_pins:[Output<'static>; MuxDataIndex::VARIANT_COUNT / 2], // half the size of the mux data options
    adc_buffer: [u16; Adc1Channels::VARIANT_COUNT],
}

impl AdcMux {
    fn new(sel_pins: [Output<'static>; MuxDataIndex::VARIANT_COUNT / 2], adc_buffer: [u16; Adc1Channels::VARIANT_COUNT]) -> Self {
        Self { mux_sel: AdcMuxSel::SelLow, mux_buffer: [0; MuxDataIndex::VARIANT_COUNT], sel_pins: sel_pins, adc_buffer: adc_buffer }
    }

    async fn switch_states(&mut self) {
        if self.mux_sel == AdcMuxSel::SelLow {
            self.sel_pins.iter_mut().for_each(|output| output.set_low());
            self.mux_buffer[MuxDataIndex::Sel1Low as usize] = self.adc_buffer[Adc1Channels::Adc1Channel0 as usize];
            self.mux_buffer[MuxDataIndex::Sel2Low as usize] = self.adc_buffer[Adc1Channels::Adc1Channel15 as usize];
            self.mux_buffer[MuxDataIndex::Sel3Low as usize] = self.adc_buffer[Adc1Channels::Adc1Channel5 as usize];
            self.mux_buffer[MuxDataIndex::Sel4Low as usize] = self.adc_buffer[Adc1Channels::Adc1Channel9 as usize];
            
            embassy_time::Timer::after_millis(10).await;
            self.mux_sel = AdcMuxSel::SelHigh;
        } else {
            self.sel_pins.iter_mut().for_each(|output| output.set_high());
            self.mux_buffer[MuxDataIndex::Sel1High as usize] = self.adc_buffer[Adc1Channels::Adc1Channel0 as usize];
            self.mux_buffer[MuxDataIndex::Sel2High as usize] = self.adc_buffer[Adc1Channels::Adc1Channel15 as usize];
            self.mux_buffer[MuxDataIndex::Sel3High as usize] = self.adc_buffer[Adc1Channels::Adc1Channel5 as usize];
            self.mux_buffer[MuxDataIndex::Sel4High as usize] = self.adc_buffer[Adc1Channels::Adc1Channel9 as usize];

            embassy_time::Timer::after_millis(10).await;
            self.mux_sel = AdcMuxSel::SelLow;
        }
    }   

    pub fn get_efuse_data(&self, efuse_id: EfuseId) -> Option<u16> {
        match efuse_id {
            EfuseId::EfuseDashboard => Some(self.adc_buffer[Adc1Channels::Adc1Channel3 as usize]),
            EfuseId::EfuseBrake => Some(self.mux_buffer[MuxDataIndex::Sel1High as usize]),
            EfuseId::EfuseShutdown => Some(self.mux_buffer[MuxDataIndex::Sel3Low as usize]),
            EfuseId::EfuseLV => Some(self.mux_buffer[MuxDataIndex::Sel3High as usize]),
            EfuseId::EfuseRadfan => Some(self.mux_buffer[MuxDataIndex::Sel4High as usize]),
            EfuseId::EfuseFanbatt => Some(self.adc_buffer[Adc1Channels::Adc1Channel6 as usize]),
            EfuseId::EfusePump1 => Some(self.adc_buffer[Adc1Channels::Adc1Channel2 as usize]),
            EfuseId::EfusePump2 => Some(self.adc_buffer[Adc1Channels::Adc1Channel13 as usize]),
            EfuseId::EfuseBattbox => Some(self.mux_buffer[MuxDataIndex::Sel1Low as usize]),
            EfuseId::EfuseMC => Some(self.adc_buffer[Adc1Channels::Adc1Channel18 as usize]),
            EfuseId::EfuseSpare => None,
        }
    }
}