//! Conservative trailing-window accounting, never a resetting/overwriting bucket.
pub const WINDOW_US: u64 = 1_500_000;
pub const CAPACITY: usize = 2;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Interval {
    pub start_us: u64,
    /// None means an acknowledged pulse is still active. It cannot expire.
    pub end_us: Option<u64>,
}
impl Interval {
    fn expires(self) -> Option<u64> {
        self.end_us.and_then(|end| end.checked_add(WINDOW_US))
    }
    fn retained(self, now: u64) -> bool {
        self.expires().is_none_or(|expiry| now < expiry)
    }
    fn duration(self, now: u64) -> u64 {
        self.end_us.unwrap_or(now).saturating_sub(self.start_us)
    }
}
#[allow(dead_code)] // Raw fields are included in deferred telemetry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub window_us: u64,
    pub retained: [Option<Interval>; CAPACITY],
    pub pulses_used: u8,
    /// Full acknowledged interval durations, not prorated partial overlaps.
    pub on_us: u64,
    pub recovery_at_us: Option<u64>,
    pub recovery_in_us: Option<u64>,
}
pub struct Ledger {
    entries: [Option<Interval>; CAPACITY],
}
impl Ledger {
    pub fn new() -> Self {
        Self {
            entries: [None; CAPACITY],
        }
    }
    pub fn expire(&mut self, now: u64) {
        for entry in &mut self.entries {
            if entry.is_some_and(|p| !p.retained(now)) {
                *entry = None;
            }
        }
    }
    /// Only successful assist-command completion may create an interval.
    /// No unexpired entry can be overwritten, even on a malformed call.
    pub fn begin(&mut self, now: u64) -> Result<(), ()> {
        self.expire(now);
        if self.entries.iter().flatten().any(|p| p.end_us.is_none()) {
            return Err(());
        }
        let slot = self.entries.iter_mut().find(|p| p.is_none()).ok_or(())?;
        *slot = Some(Interval {
            start_us: now,
            end_us: None,
        });
        Ok(())
    }
    pub fn end(&mut self, now: u64) -> Result<(), ()> {
        let pulse = self
            .entries
            .iter_mut()
            .flatten()
            .find(|p| p.end_us.is_none())
            .ok_or(())?;
        if now < pulse.start_us {
            return Err(());
        }
        pulse.end_us = Some(now);
        Ok(())
    }
    pub fn snapshot(&self, now: u64) -> Snapshot {
        let retained = self.entries.map(|p| p.filter(|p| p.retained(now)));
        let pulses_used = retained.iter().flatten().count() as u8;
        let on_us = retained
            .iter()
            .flatten()
            .fold(0u64, |sum, p| sum.saturating_add(p.duration(now)));
        let recovery_at_us = retained.iter().flatten().filter_map(|p| p.expires()).min();
        Snapshot {
            window_us: WINDOW_US,
            retained,
            pulses_used,
            on_us,
            recovery_at_us,
            recovery_in_us: recovery_at_us.map(|t| t.saturating_sub(now)),
        }
    }
}
