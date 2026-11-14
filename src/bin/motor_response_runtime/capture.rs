//! Fixed-memory periodic capture. No formatting, IO, allocation or safety decisions.
pub const DRY_CAPTURE_US: u64 = 10_000_000;
const SAMPLE_PERIOD_US: u64 = 100_000;
pub struct Capture<T: Copy, const N: usize> {
    samples: [Option<T>; N],
    cursor: usize,
    count: usize,
    next_us: u64,
    pub overwritten: u32,
}
impl<T: Copy, const N: usize> Capture<T, N> {
    pub fn new() -> Self {
        assert!(N > 0);
        Self {
            samples: [None; N],
            cursor: 0,
            count: 0,
            next_us: 0,
            overwritten: 0,
        }
    }
    pub fn record(&mut self, now: u64, sample: T) {
        if now < self.next_us {
            return;
        }
        self.next_us = now.saturating_add(SAMPLE_PERIOD_US);
        if self.count == N {
            self.overwritten = self.overwritten.saturating_add(1);
        } else {
            self.count += 1;
        }
        self.samples[self.cursor] = Some(sample);
        self.cursor = (self.cursor + 1) % N;
    }
    pub fn recent(&self) -> impl Iterator<Item = T> + '_ {
        let start = if self.count == N { self.cursor } else { 0 };
        (0..self.count).filter_map(move |i| self.samples[(start + i) % N])
    }
}
