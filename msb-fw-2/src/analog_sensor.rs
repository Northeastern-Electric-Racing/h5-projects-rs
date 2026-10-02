//! Shared plumbing for sensors read through the analog muxes.
//!
//! A sensor describes where its inputs are wired and how to turn volts into a physical value by
//! implementing [`AnalogSensor`]; [`run`] does the rest (subscribe to [`MUX_SNAPSHOT`], convert,
//! log, publish). Embassy tasks can't be generic, so each sensor still has a one-line task:
//!
//! ```ignore
//! #[embassy_executor::task]
//! pub async fn strain_gauge_task() {
//!     analog_sensor::run::<StrainGauges, NUM_STRAIN_GAUGES>().await
//! }
//! ```

use crate::multiplexor_handler::{MUX_SNAPSHOT, MuxSource};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

/// ADC reference voltage.
pub const MAX_VOLTS: f32 = 3.3;
const MAX_ADC_VAL_12B: f32 = 4095.0;

/// Max number of tasks that can subscribe to one sensor's [`SensorData`].
pub const MAX_DATA_RECEIVERS: usize = 2;

/// Where a sensor publishes its latest readings, one per input.
pub type SensorData<R, const N: usize> = Watch<ThreadModeRawMutex, [R; N], MAX_DATA_RECEIVERS>;

/// Converts a raw 12-bit ADC reading to volts.
pub fn raw_to_volts(raw: u16) -> f32 {
    f32::from(raw) * MAX_VOLTS / MAX_ADC_VAL_12B
}

/// A sensor with `N` inputs read from the mux snapshot.
pub trait AnalogSensor<const N: usize> {
    /// How often [`run`] converts and publishes.
    const POLL_PERIOD: Duration;
    /// Where each input is wired.
    const SOURCES: [MuxSource; N];
    /// Label for each input, passed to [`log`](Self::log).
    const NAMES: [&'static str; N];

    type Reading: Copy + 'static;

    /// Volts → physical value for input `index`.
    fn convert(index: usize, raw: u16, volts: f32) -> Self::Reading;

    /// Logs one input's reading.
    fn log(name: &'static str, reading: &Self::Reading);

    /// Where readings are published.
    fn data() -> &'static SensorData<Self::Reading, N>;
}

/// Polls sensor `S` forever: every [`AnalogSensor::POLL_PERIOD`], takes the latest mux snapshot,
/// converts and logs each input, and publishes the readings.
pub async fn run<S: AnalogSensor<N>, const N: usize>() -> ! {
    let mut rx = MUX_SNAPSHOT.receiver().expect("Too many MUX_SNAPSHOT receivers.");
    let sender = S::data().sender();
    let mut ticker = Ticker::every(S::POLL_PERIOD);

    loop {
        // Latest scan of every mux input; waits only until the first scan lands.
        let snapshot = rx.get().await;

        let readings: [S::Reading; N] = core::array::from_fn(|i| {
            let (id, channel, input) = S::SOURCES[i];
            let raw = snapshot.get(id, channel, input);
            S::convert(i, raw, raw_to_volts(raw))
        });
        for (name, reading) in S::NAMES.iter().zip(readings.iter()) {
            S::log(name, reading);
        }
        sender.send(readings);

        ticker.next().await;
    }
}
