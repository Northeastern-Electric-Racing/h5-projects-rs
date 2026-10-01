use embassy_futures::join::join;
use embassy_stm32::adc::{Adc, AdcChannel, BorrowedAdcChannel, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{ADC1, ADC2, GPDMA1_CH0, GPDMA1_CH1};
use embassy_stm32::{Peri, bind_interrupts, dma};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker, Timer};

/// Settling time after a SEL change. The D-side RC filter has τ = 10 µs; ~9τ settles to 12 bits.
const MUX_SETTLE_US: u64 = 100;

/// ADC sample time, matching the C firmware (92.5 cycles).
const SAMPLE_TIME: SampleTime = SampleTime::Cycles925;

/// How often [`mux_scan_task`] scans every mux input.
const SCAN_PERIOD: Duration = Duration::from_millis(10);

/// Max number of sensor tasks that can subscribe to [`MUX_SNAPSHOT`]. Raise as sensors are added.
pub const MAX_SENSOR_TASKS: usize = 8;

/// Latest scan of every mux input, published by [`mux_scan_task`].
pub static MUX_SNAPSHOT: Watch<ThreadModeRawMutex, MuxSnapshot, MAX_SENSOR_TASKS> = Watch::new();

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL0 => dma::InterruptHandler<GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<GPDMA1_CH1>;
});

/// Mux outputs converted by ADC1, in DMA scan order.
const ADC1_SCAN: [(MuxId, MuxChannel); 5] = [
    (MuxId::U18, MuxChannel::Ch1), // PC0, ch10
    (MuxId::U18, MuxChannel::Ch2), // PC2, ch12
    (MuxId::U18, MuxChannel::Ch3), // PC3, ch13
    (MuxId::U18, MuxChannel::Ch4), // PA0, ch0
    (MuxId::U19, MuxChannel::Ch4), // PF12, ch6
];

/// Mux outputs converted by ADC2, in DMA scan order. PA3 is shared by ADC1/2 but read on ADC2, as
/// in the C firmware.
const ADC2_SCAN: [(MuxId, MuxChannel); 3] = [
    (MuxId::U19, MuxChannel::Ch1), // PA3, ch15
    (MuxId::U19, MuxChannel::Ch2), // PF13, ch2
    (MuxId::U19, MuxChannel::Ch3), // PF14, ch6
];

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

/// Owns both muxes, both ADCs with their DMA channels, and the eight mux output pins.
pub struct MuxHandler<'d> {
    u18: Mux<'d>,
    u19: Mux<'d>,
    adc1: Adc<'d, ADC1, Blocking>,
    adc2: Adc<'d, ADC2, Blocking>,
    dma1: Peri<'d, GPDMA1_CH0>,
    dma2: Peri<'d, GPDMA1_CH1>,
    /// Channels in [`ADC1_SCAN`] order.
    adc1_ch: [BorrowedAdcChannel<'d, ADC1>; ADC1_SCAN.len()],
    /// Channels in [`ADC2_SCAN`] order.
    adc2_ch: [BorrowedAdcChannel<'d, ADC2>; ADC2_SCAN.len()],
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
    pub fn new(
        adc1: Adc<'d, ADC1, Blocking>,
        adc2: Adc<'d, ADC2, Blocking>,
        dma1: Peri<'d, GPDMA1_CH0>,
        dma2: Peri<'d, GPDMA1_CH1>,
        pins: MuxPins<'d>,
    ) -> Self {
        Self {
            u18: Mux::new(pins.u18_sel1, pins.u18_sel2, pins.u18_sel3, pins.u18_sel4),
            u19: Mux::new(pins.u19_sel1, pins.u19_sel2, pins.u19_sel3, pins.u19_sel4),
            adc1,
            adc2,
            dma1,
            dma2,
            adc1_ch: [
                pins.u18_d1.degrade_adc(),
                pins.u18_d2.degrade_adc(),
                pins.u18_d3.degrade_adc(),
                pins.u18_d4.degrade_adc(),
                pins.u19_d4.degrade_adc(),
            ],
            adc2_ch: [pins.u19_d1.degrade_adc(), pins.u19_d2.degrade_adc(), pins.u19_d3.degrade_adc()],
        }
    }

    /// Selects `input` on every channel of both muxes (like the C `adc_switchMuxStates`), waiting
    /// for the outputs to settle if anything changed.
    async fn select_all(&mut self, input: MuxInput) {
        let mut changed = false;
        for channel in MuxChannel::ALL {
            changed |= self.u18.set(channel, input);
            changed |= self.u19.set(channel, input);
        }
        if changed {
            Timer::after_micros(MUX_SETTLE_US).await;
        }
    }

    /// Converts every mux output as currently selected, with both ADCs scanning over DMA at once.
    async fn scan(&mut self) -> ([u16; ADC1_SCAN.len()], [u16; ADC2_SCAN.len()]) {
        let mut buf1 = [0u16; ADC1_SCAN.len()];
        let mut buf2 = [0u16; ADC2_SCAN.len()];

        let seq1 = self.adc1_ch.iter_mut().map(|ch| (ch.reborrow_adc(), SAMPLE_TIME));
        let seq2 = self.adc2_ch.iter_mut().map(|ch| (ch.reborrow_adc(), SAMPLE_TIME));
        join(
            self.adc1.read_sequence(self.dma1.reborrow(), Irqs, seq1, None, &mut buf1),
            self.adc2.read_sequence(self.dma2.reborrow(), Irqs, seq2, None, &mut buf2),
        )
        .await;

        (buf1, buf2)
    }

    /// Reads every input of every channel on both muxes: all `A` inputs, then all `B` inputs,
    /// settling once per switch.
    pub async fn read_all(&mut self) -> MuxSnapshot {
        let mut snapshot = MuxSnapshot::EMPTY;
        for input in [MuxInput::A, MuxInput::B] {
            self.select_all(input).await;
            let (buf1, buf2) = self.scan().await;
            for (&(mux, channel), &raw) in ADC1_SCAN.iter().zip(buf1.iter()) {
                snapshot.set(mux, channel, input, raw);
            }
            for (&(mux, channel), &raw) in ADC2_SCAN.iter().zip(buf2.iter()) {
                snapshot.set(mux, channel, input, raw);
            }
        }
        snapshot.timestamp = Instant::now();
        snapshot
    }
}

/// Owns the [`MuxHandler`] and publishes a full scan to [`MUX_SNAPSHOT`] every [`SCAN_PERIOD`].
///
/// Sensor tasks never touch the muxes or ADCs; they subscribe to [`MUX_SNAPSHOT`] and pick out
/// their sources with [`MuxSnapshot::get`].
#[embassy_executor::task]
pub async fn mux_scan_task(mut mux: MuxHandler<'static>) {
    let sender = MUX_SNAPSHOT.sender();
    let mut ticker = Ticker::every(SCAN_PERIOD);

    loop {
        sender.send(mux.read_all().await);
        ticker.next().await;
    }
}

/// Raw 12-bit readings of every mux source, indexed by `[mux][channel][input]`.
#[derive(Clone, Copy, defmt::Format)]
pub struct MuxSnapshot {
    raw: [[[u16; 2]; 4]; 2],
    /// When the scan finished.
    pub timestamp: Instant,
}

impl MuxSnapshot {
    const EMPTY: Self = Self { raw: [[[0; 2]; 4]; 2], timestamp: Instant::from_ticks(0) };

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
