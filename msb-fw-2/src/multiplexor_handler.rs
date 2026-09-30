//! Handler for the two TMUX1134 analog multiplexers on the MSB.
//!
//! Each TMUX1134 has four 2:1 switches. `SELx` picks whether `Dx` is connected to `SxA` or `SxB`,
//! and every `Dx` line runs through a 100 Ω / 100 nF RC filter into an ADC pin. Sensors never
//! touch the SEL pins directly; they ask the handler to read a (mux, channel, input) source,
//! which switches the mux, waits for the RC filter to settle, then samples the ADC.
//!
//! | Mux | SEL pins  | D1               | D2              | D3              | D4              |
//! |-----|-----------|------------------|-----------------|-----------------|-----------------|
//! | U18 | PC6..PC9  | PC0  (ADC1 ch10) | PC2 (ADC1 ch12) | PC3 (ADC1 ch13) | PA0 (ADC1 ch0)  |
//! | U19 | PF6..PF9  | PA3  (ADC2 ch15) | PF13 (ADC2 ch2) | PF14 (ADC2 ch6) | PF12 (ADC1 ch6) |
//!
//! U18 inputs: `SxA` = STRAIN_OUT1..4, `SxB` = LPF1..4.
//! U19 inputs: S1A = THERMOCOUPLE, S1B = LPF5, S2A/B = LPF6/LPF7, S3A = LPF8, S3B = ADC_EXT1,
//! S4A/B = ADC_EXT2/ADC_EXT3.

use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, BorrowedAdcChannel, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{ADC1, ADC2};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

/// Settling time after a SEL change. The D-side RC filter has τ = 10 µs; ~9τ settles to 12 bits.
const MUX_SETTLE_US: u64 = 100;

/// ADC sample time, matching the C firmware (92.5 cycles).
const SAMPLE_TIME: SampleTime = SampleTime::Cycles925;

/// The mux handler shared between all analog sensor tasks.
///
/// Hold the lock across the whole select → settle → read sequence so another task cannot flip
/// the SEL pins mid-read.
pub type SharedMux = Mutex<ThreadModeRawMutex, MuxHandler<'static>>;

/// Which TMUX1134 on the board.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MuxId {
    /// U18 (`MUX2` in the C firmware): strain gauges / LPF1..4.
    U18,
    /// U19 (`MUX1` in the C firmware): thermocouple / LPF5..8 / external ADCs.
    U19,
}

/// One of the four switches inside a TMUX1134.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MuxChannel {
    Ch1 = 0,
    Ch2 = 1,
    Ch3 = 2,
    Ch4 = 3,
}

impl MuxChannel {
    pub const ALL: [MuxChannel; 4] = [MuxChannel::Ch1, MuxChannel::Ch2, MuxChannel::Ch3, MuxChannel::Ch4];
}

/// Which input a switch connects to its `Dx` output.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MuxInput {
    A,
    B,
}

/// Maps an input to the SEL pin level that selects it.
///
/// Taken from the C firmware, which reads the LPF (`SxB`) sensors with every SEL pin driven low.
/// If bench testing shows the opposite, this is the only place to change.
const fn sel_level(input: MuxInput) -> Level {
    match input {
        MuxInput::A => Level::High,
        MuxInput::B => Level::Low,
    }
}

/// A single TMUX1134's four SEL lines.
struct Mux<'d> {
    sel: [Output<'d>; 4],
    /// Last selected input per channel; `None` until first set, so the first read always settles.
    state: [Option<MuxInput>; 4],
}

impl<'d> Mux<'d> {
    fn new(sel1: Peri<'d, impl Pin>, sel2: Peri<'d, impl Pin>, sel3: Peri<'d, impl Pin>, sel4: Peri<'d, impl Pin>) -> Self {
        Self {
            sel: [
                Output::new(sel1, Level::Low, Speed::Low),
                Output::new(sel2, Level::Low, Speed::Low),
                Output::new(sel3, Level::Low, Speed::Low),
                Output::new(sel4, Level::Low, Speed::Low),
            ],
            state: [None; 4],
        }
    }

    /// Drives the SEL pin for `channel`. Returns `true` if the selection changed.
    fn set(&mut self, channel: MuxChannel, input: MuxInput) -> bool {
        let i = channel as usize;
        if self.state[i] == Some(input) {
            return false;
        }
        self.sel[i].set_level(sel_level(input));
        self.state[i] = Some(input);
        true
    }
}

/// The ADC channel a mux output lands on. U19 D1 (PA3) is shared by ADC1/2 but read on ADC2, as in
/// the C firmware.
enum MuxOutput<'d> {
    Adc1(BorrowedAdcChannel<'d, ADC1>),
    Adc2(BorrowedAdcChannel<'d, ADC2>),
}

/// Owns both muxes, both ADCs, and the eight mux output pins.
pub struct MuxHandler<'d> {
    u18: Mux<'d>,
    u19: Mux<'d>,
    adc1: Adc<'d, ADC1, Blocking>,
    adc2: Adc<'d, ADC2, Blocking>,
    u18_out: [MuxOutput<'d>; 4],
    u19_out: [MuxOutput<'d>; 4],
}

/// Pins used by the [`MuxHandler`].
pub struct MuxPins<'d> {
    pub u18_sel1: Peri<'d, embassy_stm32::peripherals::PC6>,
    pub u18_sel2: Peri<'d, embassy_stm32::peripherals::PC7>,
    pub u18_sel3: Peri<'d, embassy_stm32::peripherals::PC8>,
    pub u18_sel4: Peri<'d, embassy_stm32::peripherals::PC9>,
    pub u19_sel1: Peri<'d, embassy_stm32::peripherals::PF6>,
    pub u19_sel2: Peri<'d, embassy_stm32::peripherals::PF7>,
    pub u19_sel3: Peri<'d, embassy_stm32::peripherals::PF8>,
    pub u19_sel4: Peri<'d, embassy_stm32::peripherals::PF9>,
    pub u18_d1: Peri<'d, embassy_stm32::peripherals::PC0>,
    pub u18_d2: Peri<'d, embassy_stm32::peripherals::PC2>,
    pub u18_d3: Peri<'d, embassy_stm32::peripherals::PC3>,
    pub u18_d4: Peri<'d, embassy_stm32::peripherals::PA0>,
    pub u19_d1: Peri<'d, embassy_stm32::peripherals::PA3>,
    pub u19_d2: Peri<'d, embassy_stm32::peripherals::PF13>,
    pub u19_d3: Peri<'d, embassy_stm32::peripherals::PF14>,
    pub u19_d4: Peri<'d, embassy_stm32::peripherals::PF12>,
}

impl<'d> MuxHandler<'d> {
    pub fn new(adc1: Adc<'d, ADC1, Blocking>, adc2: Adc<'d, ADC2, Blocking>, pins: MuxPins<'d>) -> Self {
        Self {
            u18: Mux::new(pins.u18_sel1, pins.u18_sel2, pins.u18_sel3, pins.u18_sel4),
            u19: Mux::new(pins.u19_sel1, pins.u19_sel2, pins.u19_sel3, pins.u19_sel4),
            adc1,
            adc2,
            u18_out: [
                MuxOutput::Adc1(pins.u18_d1.degrade_adc()),
                MuxOutput::Adc1(pins.u18_d2.degrade_adc()),
                MuxOutput::Adc1(pins.u18_d3.degrade_adc()),
                MuxOutput::Adc1(pins.u18_d4.degrade_adc()),
            ],
            u19_out: [
                MuxOutput::Adc2(pins.u19_d1.degrade_adc()),
                MuxOutput::Adc2(pins.u19_d2.degrade_adc()),
                MuxOutput::Adc2(pins.u19_d3.degrade_adc()),
                MuxOutput::Adc1(pins.u19_d4.degrade_adc()),
            ],
        }
    }

    fn mux(&mut self, mux: MuxId) -> &mut Mux<'d> {
        match mux {
            MuxId::U18 => &mut self.u18,
            MuxId::U19 => &mut self.u19,
        }
    }

    /// Selects `input` on one channel, waiting for the output to settle if it changed.
    pub async fn select(&mut self, mux: MuxId, channel: MuxChannel, input: MuxInput) {
        if self.mux(mux).set(channel, input) {
            Timer::after_micros(MUX_SETTLE_US).await;
        }
    }

    /// Selects `input` on every channel of both muxes (like the C `adc_switchMuxStates`).
    pub async fn select_all(&mut self, input: MuxInput) {
        let mut changed = false;
        for channel in MuxChannel::ALL {
            changed |= self.u18.set(channel, input);
            changed |= self.u19.set(channel, input);
        }
        if changed {
            Timer::after_micros(MUX_SETTLE_US).await;
        }
    }

    /// Selects a source and returns the raw 12-bit ADC reading of it.
    pub async fn read(&mut self, mux: MuxId, channel: MuxChannel, input: MuxInput) -> u16 {
        self.select(mux, channel, input).await;
        self.sample(mux, channel)
    }

    /// Reads every input of every channel on both muxes: all `A` inputs, then all `B` inputs,
    /// settling once per switch.
    pub async fn read_all(&mut self) -> MuxSnapshot {
        let mut snapshot = MuxSnapshot::default();
        for input in [MuxInput::A, MuxInput::B] {
            self.select_all(input).await;
            for mux in [MuxId::U18, MuxId::U19] {
                for channel in MuxChannel::ALL {
                    let raw = self.sample(mux, channel);
                    snapshot.set(mux, channel, input, raw);
                }
            }
        }
        snapshot
    }

    /// Samples a mux output as currently selected.
    fn sample(&mut self, mux: MuxId, channel: MuxChannel) -> u16 {
        let out = match mux {
            MuxId::U18 => &mut self.u18_out[channel as usize],
            MuxId::U19 => &mut self.u19_out[channel as usize],
        };
        match out {
            MuxOutput::Adc1(ch) => self.adc1.blocking_read(ch, SAMPLE_TIME),
            MuxOutput::Adc2(ch) => self.adc2.blocking_read(ch, SAMPLE_TIME),
        }
    }
}

/// Raw 12-bit readings of every mux source, indexed by `[mux][channel][input]`.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct MuxSnapshot {
    raw: [[[u16; 2]; 4]; 2],
}

impl MuxSnapshot {
    const fn index(mux: MuxId, channel: MuxChannel, input: MuxInput) -> (usize, usize, usize) {
        let m = match mux {
            MuxId::U18 => 0,
            MuxId::U19 => 1,
        };
        let i = match input {
            MuxInput::A => 0,
            MuxInput::B => 1,
        };
        (m, channel as usize, i)
    }

    fn set(&mut self, mux: MuxId, channel: MuxChannel, input: MuxInput, raw: u16) {
        let (m, c, i) = Self::index(mux, channel, input);
        self.raw[m][c][i] = raw;
    }

    /// Raw reading of one source.
    pub fn get(&self, mux: MuxId, channel: MuxChannel, input: MuxInput) -> u16 {
        let (m, c, i) = Self::index(mux, channel, input);
        self.raw[m][c][i]
    }
}
