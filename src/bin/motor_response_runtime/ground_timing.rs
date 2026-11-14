//! Wall-clock attribution, not CPU profiling. No logging, allocation or thresholds.
#[derive(Clone, Copy, Debug, Default)]
pub struct Gap {
    pub interval_us: u64,
    pub previous_work_us: u64,
    pub summary_log_us: u64,
    pub sensor_poll_us: u64,
    pub motor_drive_us: u64,
    pub outside_work_and_log_us: u64,
}
#[derive(Default)]
pub struct GroundTiming {
    pub poll_us: u64,
    pub drive_us: u64,
    pub summary_us: u64,
    pub max_poll_us: u64,
    pub max_drive_us: u64,
    pub max_summary_us: u64,
    pub worst: Gap,
}
impl GroundTiming {
    /// Previous work stops BEFORE the summary write; avoid double-counting it.
    pub fn begin(&mut self, interval_us: u64, previous_work_us: u64) {
        if interval_us > self.worst.interval_us {
            self.worst = Gap {
                interval_us,
                previous_work_us,
                summary_log_us: self.summary_us,
                sensor_poll_us: self.poll_us,
                motor_drive_us: self.drive_us,
                outside_work_and_log_us: interval_us
                    .saturating_sub(previous_work_us.saturating_add(self.summary_us)),
            };
        }
        self.poll_us = 0;
        self.drive_us = 0;
        self.summary_us = 0;
    }
    pub fn poll(&mut self, us: u64) {
        self.poll_us = us;
        self.max_poll_us = self.max_poll_us.max(us);
    }
    pub fn drive(&mut self, us: u64) {
        self.drive_us = us;
        self.max_drive_us = self.max_drive_us.max(us);
    }
    pub fn summary(&mut self, us: u64) {
        self.summary_us = us;
        self.max_summary_us = self.max_summary_us.max(us);
    }
}
