//! Observed body trajectory, NOT a stall detector or wheel-speed estimate.
use super::{Input, STOP_RESERVE_US};
pub const MIN_ANGLE: f32 = 6.0;
pub const CUTOFF_MARGIN: f32 = 3.0;
pub const MIN_RATE: f32 = 3.0; // Fast qualification only, not pulse cancellation.
pub const TREND_US: u64 = 100_000;
pub const MIN_GROWTH: f32 = 0.5;
pub const SLOW_ANGLE: f32 = 8.0;
pub const SLOW_TREND_US: u64 = 500_000;
pub const SLOW_GROWTH: f32 = 0.2;
pub const PROBE_US: u64 = 200_000;
pub const PROBE_INCREMENT: f32 = 5.0;
pub const IMPROVING_RATE: f32 = 0.0;
pub const IMPROVING_ANGLE: f32 = 0.1;
pub const REARM_US: u64 = 200_000;
pub const REARM_DROP: f32 = 0.5;
pub const MAX_EVIDENCE_GAP_US: u64 = 20_000;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Watching,
    Trend,
    DriveProbe,
    ZeroWait,
    Qualifying,
    Pulse,
    AwaitImprovement,
    Spent,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    Demand,
    Improving,
    EvidenceGap,
    Resources,
    Margin,
    NoInitialAssist,
    Fault,
    Completed,
    Rearmed,
    ProbeCapped,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Telemetry {
    pub phase: Phase,
    pub reason: Option<Reason>,
    pub attempted: bool,
    pub pulse_applied: bool,
    pub zero_wait_us: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Ordinary,
    Probe,
    Zero,
    Permit,
    Cancel,
}
pub struct Recovery {
    pub telemetry: Telemetry,
    trend: Option<(u64, f32)>,
    previous: Option<(u64, f32)>,
    direction: i16,
    peak: f32,
    probe_since: u64,
    improvement: Option<(u64, f32)>,
}
impl Recovery {
    pub fn new() -> Self {
        Self {
            telemetry: Telemetry {
                phase: Phase::Watching,
                reason: None,
                attempted: false,
                pulse_applied: false,
                zero_wait_us: super::DEAD_TIME_US,
            },
            trend: None,
            previous: None,
            direction: 0,
            peak: 0.0,
            probe_since: 0,
            improvement: None,
        }
    }
    pub fn finish(&mut self, reason: Reason) {
        self.telemetry.phase = if reason == Reason::Fault {
            Phase::Spent
        } else {
            Phase::AwaitImprovement
        };
        self.telemetry.reason = Some(reason);
        self.trend = None;
        self.improvement = None;
    }
    fn reject(&mut self, reason: Reason) -> Decision {
        self.trend = None;
        self.telemetry.reason = Some(reason);
        if self.telemetry.attempted {
            self.finish(reason);
            Decision::Cancel
        } else {
            self.telemetry.phase = Phase::Watching;
            Decision::Ordinary
        }
    }
    pub fn acknowledge(&mut self, assisted: bool) {
        if matches!(self.telemetry.phase, Phase::ZeroWait | Phase::Qualifying) && assisted {
            self.telemetry.phase = Phase::Pulse;
            self.telemetry.pulse_applied = true;
            self.telemetry.reason = None;
        } else if self.telemetry.phase == Phase::Pulse && !assisted {
            self.finish(Reason::Completed);
        }
    }
    fn observe_improvement(&mut self, i: Input, fresh: bool) {
        if !fresh {
            return;
        }
        let angle = i.pitch.abs();
        let continuous = self
            .previous
            .is_some_and(|(t, _)| i.sample_us.saturating_sub(t) <= MAX_EVIDENCE_GAP_US);
        let toward = i.pitch * i.rate < 0.0 && self.previous.is_some_and(|(_, p)| angle <= p);
        let upright =
            angle <= crate::balance::ARM_ANGLE_DEG && i.rate.abs() <= crate::balance::ARM_RATE_DPS;
        if i.now_us.saturating_sub(i.sample_us) > MAX_EVIDENCE_GAP_US
            || !continuous
            || !(toward || upright)
        {
            self.improvement = None;
        } else {
            let (since, start) = *self.improvement.get_or_insert((i.sample_us, angle));
            if i.sample_us.saturating_sub(since) >= REARM_US
                && (upright || start - angle >= REARM_DROP)
            {
                // Observed improvement only; no ledger/cooldown changes and no immediate assist.
                self.telemetry = Telemetry {
                    phase: Phase::Watching,
                    reason: Some(Reason::Rearmed),
                    attempted: false,
                    pulse_applied: false,
                    zero_wait_us: self.telemetry.zero_wait_us,
                };
                self.trend = None;
                self.improvement = None;
                self.peak = angle;
            }
        }
        self.previous = Some((i.sample_us, angle));
    }
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        i: Input,
        fresh: bool,
        resources: bool,
        ordinary: bool,
        initial: bool,
        zero_since: Option<u64>,
        zero_us: u64,
        dwell_us: u64,
        pulse_us: u64,
    ) -> Decision {
        self.telemetry.zero_wait_us = zero_us;
        if self.telemetry.phase == Phase::Spent {
            return Decision::Ordinary;
        }
        if self.telemetry.phase == Phase::AwaitImprovement {
            self.observe_improvement(i, fresh);
            return Decision::Ordinary;
        }
        let busy = self.telemetry.attempted;
        let sign = i.limited_pd.signum();
        let angle = i.pitch.abs();
        if i.limited_pd.unsigned_abs() <= 2
            || sign == 0
            || (busy && sign != self.direction)
            || i.pitch.signum() != f32::from(sign)
        {
            return self.reject(Reason::Demand);
        }
        if !initial {
            return self.reject(Reason::NoInitialAssist);
        }
        if i.now_us.saturating_sub(i.sample_us) > MAX_EVIDENCE_GAP_US {
            return self.reject(Reason::EvidenceGap);
        }
        if i.rate * f32::from(sign) < IMPROVING_RATE {
            return self.reject(Reason::Improving);
        }
        if !resources && self.telemetry.phase != Phase::Pulse {
            return self.reject(Reason::Resources);
        }
        let zero_left = if busy {
            zero_since.map_or(zero_us, |t| {
                zero_us.saturating_sub(i.now_us.saturating_sub(t))
            })
        } else {
            zero_us
        };
        // Compare actual 1%-resolution driver duties after the ordinary ceiling.
        let probe_duty =
            (i.mapped_pwm.abs() + PROBE_INCREMENT).min(super::settings::MAPPING.maximum_duty);
        let effective_increase = probe_duty as i16 - i.mapped_pwm.abs() as i16;
        let useful_probe = effective_increase >= 1;
        let probe_left = if !useful_probe {
            0
        } else if self.telemetry.phase == Phase::DriveProbe {
            PROBE_US.saturating_sub(i.now_us.saturating_sub(self.probe_since))
        } else if !busy {
            PROBE_US
        } else {
            0
        };
        let horizon = if self.telemetry.phase == Phase::Pulse {
            STOP_RESERVE_US
        } else {
            probe_left + zero_left + dwell_us + pulse_us + STOP_RESERVE_US
        };
        let projected = angle + i.rate.abs() * (horizon as f32 / 1_000_000.0);
        if angle < MIN_ANGLE || projected >= crate::balance::CONTROL_FALL_ANGLE_DEG - CUTOFF_MARGIN
        {
            return self.reject(Reason::Margin);
        }
        if fresh {
            if let Some((t, _)) = self.previous
                && i.sample_us.saturating_sub(t) > MAX_EVIDENCE_GAP_US
            {
                self.previous = Some((i.sample_us, angle));
                self.peak = angle;
                return self.reject(Reason::EvidenceGap);
            }
            if self.direction != sign && !busy {
                self.trend = None;
                self.peak = angle;
                self.direction = sign;
            }
            self.peak = self.peak.max(angle);
            self.previous = Some((i.sample_us, angle));
            // Ignore sub-0.1deg sample noise; cumulative improvement from peak cancels.
            if self.peak - angle >= IMPROVING_ANGLE {
                self.peak = angle;
                return self.reject(Reason::Improving);
            }
        }
        if busy {
            if self.telemetry.phase == Phase::DriveProbe {
                if useful_probe && i.now_us.saturating_sub(self.probe_since) < PROBE_US {
                    return Decision::Probe;
                }
                self.telemetry.phase = Phase::ZeroWait;
                if !useful_probe {
                    self.telemetry.reason = Some(Reason::ProbeCapped);
                }
            }
            if self.telemetry.phase == Phase::Pulse {
                return Decision::Permit;
            }
            if zero_since.is_none_or(|t| i.now_us.saturating_sub(t) < zero_us) {
                self.telemetry.phase = Phase::ZeroWait;
                return Decision::Zero;
            }
            self.telemetry.phase = Phase::Qualifying;
            return Decision::Permit;
        }
        if !ordinary || !fresh || i.rate * f32::from(sign) < 0.0 {
            self.trend = None;
            return Decision::Ordinary;
        }
        let (since, start) = *self.trend.get_or_insert((i.sample_us, angle));
        self.telemetry.phase = Phase::Trend;
        self.telemetry.reason = None;
        let elapsed = i.sample_us.saturating_sub(since);
        let growth = angle - start;
        let fast =
            i.rate * f32::from(sign) >= MIN_RATE && elapsed >= TREND_US && growth >= MIN_GROWTH;
        let slow = angle >= SLOW_ANGLE && elapsed >= SLOW_TREND_US && growth >= SLOW_GROWTH;
        if !(fast || slow) {
            return Decision::Ordinary;
        }
        self.telemetry.attempted = true;
        if useful_probe {
            self.telemetry.phase = Phase::DriveProbe;
            self.probe_since = i.now_us;
            Decision::Probe
        } else {
            self.telemetry.phase = Phase::ZeroWait;
            self.telemetry.reason = Some(Reason::ProbeCapped);
            Decision::Zero
        }
    }
}
