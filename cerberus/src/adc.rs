use embassy_stm32::gpio::{AnyPin, Output};
use variant_count::VariantCount;

#[derive(PartialEq, Eq)]
enum AdcMuxSel {
    SelLow,
    SelHigh
}

#[derive(VariantCount)]
enum Adc1_channels {
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

struct AdcMux {
    mux_sel: AdcMuxSel,
    mux_buffer: [u16; MuxDataIndex::VARIANT_COUNT],
    sel_pins:[Output<'static>; MuxDataIndex::VARIANT_COUNT / 2], // half the size of the mux data options
    adc_buffer: [u16; Adc1_channels::VARIANT_COUNT],
}

impl AdcMux {
    fn new(sel_pins: [Output<'static>; MuxDataIndex::VARIANT_COUNT / 2], adc_buffer: [u16; Adc1_channels::VARIANT_COUNT]) -> Self {
        Self { mux_sel: AdcMuxSel::SelLow, mux_buffer: [0; MuxDataIndex::VARIANT_COUNT], sel_pins: sel_pins, adc_buffer: adc_buffer }
    }

    async fn switch_states(&mut self) {
        if self.mux_sel == AdcMuxSel::SelLow {
            self.sel_pins.into_iter().map(|output | output.set_low());
            self.mux_buffer[MuxDataIndex::Sel1Low] = self.adc_buffer[Adc1_channels::Adc1Channel0];
            self.mux_buffer[MuxDataIndex::Sel2Low] = self.adc_buffer[Adc1_channels::Adc1Channel15];
            self.mux_buffer[MuxDataIndex::Sel3Low] = self.adc_buffer[Adc1_channels::Adc1Channel5];
            self.mux_buffer[MuxDataIndex::Sel4Low] = self.adc_buffer[Adc1_channels::Adc1Channel9];
            
            embassy_time::Timer::after_millis(10);
            self.mux_sel = AdcMuxSel::SelHigh;
        } else {
            self.sel_pins.into_iter().map(|output | output.set_high());
            self.mux_buffer[MuxDataIndex::Sel1High] = self.adc_buffer[Adc1_channels::Adc1Channel0];
            self.mux_buffer[MuxDataIndex::Sel2High] = self.adc_buffer[Adc1_channels::Adc1Channel15];
            self.mux_buffer[MuxDataIndex::Sel3High] = self.adc_buffer[Adc1_channels::Adc1Channel5];
            self.mux_buffer[MuxDataIndex::Sel4High] = self.adc_buffer[Adc1_channels::Adc1Channel9];

            embassy_time::Timer::after_millis(10);
            self.mux_sel = AdcMuxSel::SelLow;
        }
    }
}