//! Logical servo calibration and nonblocking tickle trajectory; no position feedback.
pub const CENTER_DEG: i32 = 90;
pub const LEFT_SIGN: i32 = -1;
pub const MIN_PULSE_US: u32 = 1000;
pub const MAX_PULSE_US: u32 = 2000;
pub const PERIOD_US: u32 = 20_000;
pub const OFFSET_DEG: i32 = 10;
pub const STEP_INTERVAL_MS: u64 = 15;
pub const IDLE_OFFSET_DEG: i32 = 5;
pub const IDLE_STEP_MS: u64 = 60;
pub fn pulse_us(angle: i32) -> u32 {
    assert!((0..=180).contains(&angle));
    MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * angle as u32 / 180
}
/// Holds center PWM after completion, avoiding repeated detach/recenter jumps.
pub struct Motion {
    current: i32,
    phase: u8,
    due: u64,
    idle_motion: bool,
    idle_direction: i32,
    idle_due: u64,
    sequence: u32,
}
impl Motion {
    pub fn new() -> Self {
        Self {
            current: CENTER_DEG,
            phase: 3,
            due: 0,
            idle_motion: false,
            idle_direction: LEFT_SIGN,
            idle_due: 25_000,
            sequence: 1,
        }
    }
    /// Tickle interrupts idle movement; repeated tickle events never queue.
    pub fn trigger(&mut self, now: u64) {
        if self.phase == 3 || self.idle_motion {
            self.idle_motion = false;
            self.phase = 0;
            self.due = now;
        }
    }
    fn schedule_idle(&mut self, now: u64) {
        self.sequence = self.sequence.wrapping_mul(1664525).wrapping_add(1013904223);
        self.idle_due = now + 20_000 + (self.sequence % 10_001) as u64;
    }
    pub fn update(&mut self, now: u64) -> Option<i32> {
        if self.phase == 3 && now >= self.idle_due {
            self.idle_motion = true;
            self.idle_direction = -self.idle_direction;
            self.phase = 0;
            self.due = now;
        }
        if self.phase == 3 || now < self.due {
            return None;
        }
        let target = if self.idle_motion {
            if self.phase == 0 {
                CENTER_DEG + self.idle_direction * IDLE_OFFSET_DEG
            } else {
                CENTER_DEG
            }
        } else {
            match self.phase {
                0 => CENTER_DEG + LEFT_SIGN * OFFSET_DEG,
                1 => CENTER_DEG - LEFT_SIGN * OFFSET_DEG,
                _ => CENTER_DEG,
            }
        };
        if !(0..=180).contains(&target) || !(0..=180).contains(&self.current) {
            self.phase = 3;
            self.current = CENTER_DEG;
            return Some(CENTER_DEG);
        }
        self.current += (target - self.current).signum();
        self.due = now
            + if self.idle_motion {
                IDLE_STEP_MS
            } else {
                STEP_INTERVAL_MS
            };
        if self.current == target {
            self.phase += 1;
            if self.idle_motion && self.phase == 1 {
                self.due = now + 350;
            }
            if (self.idle_motion && self.phase == 2) || self.phase == 3 {
                self.phase = 3;
                self.idle_motion = false;
                self.schedule_idle(now);
            }
        }
        Some(self.current)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn idle_moves_and_touch_interrupts_smoothly() {
        let mut m = Motion::new();
        assert_eq!(m.update(24_999), None);
        assert_eq!(m.update(25_000), Some(91));
        assert_eq!(m.update(25_060), Some(92));
        m.trigger(25_061);
        assert_eq!(m.update(25_061), Some(91));
        assert!(!m.idle_motion);
        let mut m = Motion::new();
        for now in (25_000..27_000).step_by(10) {
            m.update(now);
        }
        assert_eq!(m.current, CENTER_DEG);
        assert_eq!(m.phase, 3);
        assert!((46_000..57_000).contains(&m.idle_due));
    }
    #[test]
    fn single_pass_no_queue() {
        let mut m = Motion::new();
        m.trigger(0);
        let mut values = [0; 40];
        for (i, v) in values.iter_mut().enumerate() {
            m.trigger(i as u64 * STEP_INTERVAL_MS);
            *v = m.update(i as u64 * STEP_INTERVAL_MS).unwrap();
        }
        assert_eq!(values[9], 80);
        assert_eq!(values[29], 100);
        assert_eq!(values[39], 90);
        assert_eq!(m.update(1300), None);
        assert_eq!(pulse_us(90), 1500);
        assert_eq!(pulse_us(80), 1444);
        assert_eq!(pulse_us(100), 1555);
    }
}
