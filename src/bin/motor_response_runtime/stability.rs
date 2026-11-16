//! Streaming diagnostics only. No motor handles, allocation, logging, or control decisions.
use super::{
    events::Snapshot,
    policy::{DisarmReason, ResponseState},
    restart_assist::Trace,
};
const BANDS: [f32; 4] = [1.0, 2.0, 3.0, 5.0];
const CROSSING_NOISE_DEG: f32 = 0.25;
const NEAR_UPRIGHT_DEG: f32 = 2.0;
const KICK_RECORDS: usize = 128;

pub struct Stability {
    pub active_us: u64,
    pub within_us: [u64; 4],
    pub unknown_pose_us: u64,
    pub longest_near_us: u64,
    pub max_abs_pitch: f32,
    pub max_abs_rate: f32,
    pub zero_crossings: u32,
    pub overshoot_like: u32,
    pub correction_sign_changes: u32,
    pub zero_us: u64,
    pub nonzero_us: u64,
    pub kick_count: u32,
    pub kick_on_us: u64,
    pub max_kick_us: u64,
    pub kick_timing_estimated: bool,
    pub missing_kick_records: u32,
    pub disarm: Option<DisarmReason>,
    pub active_missed_deadlines: u32,
    pub clock_errors: u32,
    pub invalid_samples: u32,
    start_us: Option<u64>,
    last_us: Option<u64>,
    finished: bool,
    pose: Option<(u64, f32)>,
    sample_us: Option<u64>,
    near_run_us: u64,
    pitch_side: i16,
    side_correction: bool,
    command_side: i16,
    previous_zero: bool,
    open_kick: Option<u64>,
    sampled_kick_us: u64,
    durations: [u32; KICK_RECORDS],
    duration_count: usize,
    missed_base: u32,
}
impl Default for Stability {
    fn default() -> Self {
        Self {
            active_us: 0,
            within_us: [0; 4],
            unknown_pose_us: 0,
            longest_near_us: 0,
            max_abs_pitch: 0.0,
            max_abs_rate: 0.0,
            zero_crossings: 0,
            overshoot_like: 0,
            correction_sign_changes: 0,
            zero_us: 0,
            nonzero_us: 0,
            kick_count: 0,
            kick_on_us: 0,
            max_kick_us: 0,
            kick_timing_estimated: false,
            missing_kick_records: 0,
            disarm: None,
            active_missed_deadlines: 0,
            clock_errors: 0,
            invalid_samples: 0,
            start_us: None,
            last_us: None,
            finished: false,
            pose: None,
            sample_us: None,
            near_run_us: 0,
            pitch_side: 0,
            side_correction: false,
            command_side: 0,
            previous_zero: true,
            open_kick: None,
            sampled_kick_us: 0,
            durations: [0; KICK_RECORDS],
            duration_count: 0,
            missed_base: 0,
        }
    }
}
impl Stability {
    pub fn has_started(&self) -> bool {
        self.start_us.is_some()
    }
    /// Timestamp comes from the existing neutral-acceptance transition, once only.
    pub fn start(&mut self, us: u64) {
        if self.start_us.is_none() && !self.finished {
            self.start_us = Some(us);
            self.last_us = Some(us);
        }
    }
    pub fn kick_durations_us(&self) -> &[u32] {
        &self.durations[..self.duration_count]
    }
    fn unknown(&mut self) {
        self.pose = None;
        self.near_run_us = 0;
        self.pitch_side = 0;
        self.side_correction = false;
    }
    fn advance(&mut self, now: u64) -> bool {
        let Some(last) = self.last_us else {
            return false;
        };
        if now < last {
            self.clock_errors = self.clock_errors.saturating_add(1);
            return false;
        }
        let dt = now - last;
        self.active_us = self.active_us.saturating_add(dt);
        if self.previous_zero {
            self.zero_us = self.zero_us.saturating_add(dt);
        } else {
            self.nonzero_us = self.nonzero_us.saturating_add(dt);
        }
        if self.open_kick.is_some() {
            self.sampled_kick_us = self.sampled_kick_us.saturating_add(dt);
        }
        let mut known = 0;
        if let Some((sample, pitch)) = self.pose {
            known = now
                .min(sample.saturating_add(crate::balance::MAX_SAMPLE_GAP_US))
                .saturating_sub(last)
                .min(dt);
            for (i, limit) in BANDS.iter().enumerate() {
                if pitch.abs() <= *limit {
                    self.within_us[i] = self.within_us[i].saturating_add(known);
                }
            }
            if pitch.abs() <= NEAR_UPRIGHT_DEG {
                self.near_run_us = self.near_run_us.saturating_add(known);
                self.longest_near_us = self.longest_near_us.max(self.near_run_us);
            } else {
                self.near_run_us = 0;
            }
        }
        self.unknown_pose_us = self.unknown_pose_us.saturating_add(dt - known);
        if known < dt {
            self.unknown();
        }
        self.last_us = Some(now);
        true
    }
    fn record_duration(&mut self, duration: u64) {
        self.max_kick_us = self.max_kick_us.max(duration);
        if self.duration_count < KICK_RECORDS {
            self.durations[self.duration_count] = duration.min(u64::from(u32::MAX)) as u32;
            self.duration_count += 1;
        } else {
            self.missing_kick_records = self.missing_kick_records.saturating_add(1);
        }
    }
    fn kicks(&mut self, now: u64, applied: bool, trace: Option<Trace>, count: u32) {
        if count < self.kick_count {
            self.missing_kick_records = self.missing_kick_records.saturating_add(1);
            return;
        }
        let delta = count - self.kick_count;
        if delta > 1 {
            // Never invent durations for edges absent from observations.
            self.missing_kick_records = self
                .missing_kick_records
                .saturating_add(delta - u32::from(applied) + u32::from(self.open_kick.is_some()));
            self.open_kick = None;
        } else if delta == 1 && !applied {
            // A new pulse both started and ended between snapshots. Only the
            // newest completed duration is available; an older open edge is lost.
            if self.open_kick.take().is_some() {
                self.missing_kick_records = self.missing_kick_records.saturating_add(1);
            }
            if let Some(t) = trace {
                self.record_duration(t.last_pulse_us);
            } else {
                self.missing_kick_records = self.missing_kick_records.saturating_add(1);
            }
        } else if (!applied || delta == 1)
            && let Some(start) = self.open_kick.take()
        {
            self.record_duration(trace.map_or(now.saturating_sub(start), |t| t.last_pulse_us));
        }
        if applied && self.open_kick.is_none() {
            self.open_kick = Some(trace.map_or(now, |t| t.us.saturating_sub(t.pulse_us)));
        }
        self.kick_count = count;
        if let Some(t) = trace {
            self.kick_on_us = t.total_on_us;
            self.max_kick_us = self.max_kick_us.max(t.last_pulse_us).max(t.pulse_us);
        } else {
            self.kick_on_us = self.sampled_kick_us;
            self.kick_timing_estimated |= count > 0;
        }
    }
    pub fn observe(&mut self, s: &Snapshot) {
        if self.finished {
            return;
        }
        if !self.has_started() {
            if s.state != ResponseState::Active {
                self.missed_base = s.missed;
                return;
            }
            self.start(s.us); // Runtime normally supplies the exact transition first.
        }
        if !self.advance(s.us) {
            return;
        }
        self.active_missed_deadlines = s.missed.saturating_sub(self.missed_base);
        let active = s.state == ResponseState::Active;
        if s.sensor_fault.is_some() {
            self.unknown();
        } else if let Some(ts) = s.sample_us
            && self.sample_us != Some(ts)
        {
            if ts > s.us || self.sample_us.is_some_and(|old| ts < old) {
                self.invalid_samples = self.invalid_samples.saturating_add(1);
                self.unknown();
            } else {
                self.sample_us = Some(ts);
                if let (Some(pitch), Some(rate)) = (s.pitch, s.rate)
                    && pitch.is_finite()
                    && rate.is_finite()
                    && s.us - ts <= crate::balance::MAX_SAMPLE_GAP_US
                {
                    self.max_abs_pitch = self.max_abs_pitch.max(pitch.abs());
                    self.max_abs_rate = self.max_abs_rate.max(rate.abs());
                    self.pose = Some((ts, pitch));
                    if active {
                        let side = if pitch >= CROSSING_NOISE_DEG {
                            1
                        } else if pitch <= -CROSSING_NOISE_DEG {
                            -1
                        } else {
                            0
                        };
                        if side != 0 {
                            if self.pitch_side != 0 && side != self.pitch_side {
                                self.zero_crossings = self.zero_crossings.saturating_add(1);
                                if self.side_correction {
                                    self.overshoot_like = self.overshoot_like.saturating_add(1);
                                }
                                self.side_correction = false;
                            }
                            self.pitch_side = side;
                        }
                    }
                } else {
                    self.invalid_samples = self.invalid_samples.saturating_add(1);
                    self.unknown();
                }
            }
        }
        let applied = active && s.kick && s.commanded != (0, 0);
        let count = s.reversal.map_or(
            self.kick_count + u32::from(applied && self.open_kick.is_none()),
            |t| u32::from(t.kicks),
        );
        self.kicks(s.us, applied, s.assist, count);
        if active {
            let side = s.commanded.1.signum(); // Existing physical mapping (-u,+u).
            if side != 0 {
                if self.command_side != 0 && side != self.command_side {
                    self.correction_sign_changes = self.correction_sign_changes.saturating_add(1);
                }
                self.command_side = side;
                self.side_correction |= side == self.pitch_side;
            }
        }
        self.previous_zero = s.commanded == (0, 0);
        if !active {
            self.disarm = s.disarm;
            self.finished = true;
        }
    }
    /// Called immediately after existing real shutdown, before any serial dump.
    pub fn finish(
        &mut self,
        now: u64,
        reason: Option<DisarmReason>,
        missed: u32,
        trace: Option<Trace>,
        count: u8,
    ) {
        if !self.finished && self.has_started() {
            self.advance(now);
            self.kicks(now, false, trace, u32::from(count));
            self.active_missed_deadlines = missed.saturating_sub(self.missed_base);
        }
        self.disarm = reason;
        self.finished = true;
    }
}
