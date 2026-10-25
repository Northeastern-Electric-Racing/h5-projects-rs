/// Rolling mean over the last `N` samples, with the mean cached rather than recomputed on read.
pub struct MovingAverage<const N: usize> {
    samples: [f32; N],
    index: usize,
    average: f32,
    seeded: bool,
}

impl<const N: usize> MovingAverage<N> {
    pub const fn new() -> Self {
        Self { samples: [0.0; N], index: 0, average: 0.0, seeded: false }
    }

    /// Refills the entire window with `value`, so the average is `value` immediately.
    pub const fn reset(&mut self, value: f32) {
        self.samples = [value; N];
        self.index = 0;
        self.average = value;
        self.seeded = true;
    }

    pub fn push(&mut self, sample: f32) {
        if !self.seeded {
            self.reset(sample);
            return;
        }

        self.samples[self.index] = sample;
        self.index = (self.index + 1) % N;

        self.average = self.samples.iter().sum::<f32>() / N as f32;
    }

    pub const fn get(&self) -> f32 {
        self.average
    }
}

impl<const N: usize> Default for MovingAverage<N> {
    fn default() -> Self {
        Self::new()
    }
}
