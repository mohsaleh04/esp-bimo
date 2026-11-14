//! Independent interval oracle plus integrated arbiter tests, all host-only.
use super::*;
use restart_assist::{Assist, Config, Input, Reason, State};
use rolling_budget::{Ledger, WINDOW_US};

#[test]
fn ledger_expiry_is_end_plus_window_with_exact_microsecond_boundaries() {
    let mut b = Ledger::new();
    b.begin(0).unwrap();
    b.end(40_000).unwrap();
    let before = b.snapshot(1_539_999);
    assert_eq!(before.pulses_used, 1);
    assert_eq!(before.on_us, 40_000);
    assert_eq!(before.recovery_at_us, Some(1_540_000));
    assert_eq!(before.recovery_in_us, Some(1));
    assert_eq!(b.snapshot(1_540_000).pulses_used, 0);
    assert_eq!(b.snapshot(1_540_001).on_us, 0);
}
#[test]
fn partial_overlaps_are_charged_in_full_and_fixed_boundaries_do_not_reset() {
    let mut b = Ledger::new();
    b.begin(0).unwrap();
    b.end(40_000).unwrap();
    b.begin(1_040_000).unwrap();
    b.end(1_088_000).unwrap();
    for t in [1_499_999, 1_500_000, 1_500_001, 1_539_999] {
        let w = b.snapshot(t);
        assert_eq!(w.pulses_used, 2);
        assert_eq!(w.on_us, 88_000);
    }
    let w = b.snapshot(1_540_000);
    assert_eq!(w.pulses_used, 1);
    assert_eq!(w.on_us, 48_000);
    assert_eq!(w.recovery_at_us, Some(2_588_000));
    assert_eq!(b.snapshot(2_587_999).pulses_used, 1);
    assert_eq!(b.snapshot(2_588_000).pulses_used, 0);
}
#[test]
fn active_and_bucket_crossing_pulses_cannot_expire_early() {
    let mut b = Ledger::new();
    b.begin(1_490_000).unwrap();
    let w = b.snapshot(20_000_000);
    assert_eq!(w.pulses_used, 1);
    assert!(w.on_us > 150_000);
    assert_eq!(w.recovery_at_us, None); // A missing end ACK is never silently forgotten.
    assert!(b.begin(20_000_000).is_err());
    b.end(1_540_000).unwrap();
    assert_eq!(b.snapshot(3_039_999).on_us, 50_000);
    assert_eq!(b.snapshot(3_040_000).pulses_used, 0);
}
#[test]
fn full_ledger_cannot_overwrite_unexpired_entries() {
    let mut b = Ledger::new();
    b.begin(0).unwrap();
    b.end(40_000).unwrap();
    b.begin(1_040_000).unwrap();
    b.end(1_088_000).unwrap();
    let original = b.snapshot(1_400_000);
    assert!(b.begin(1_400_000).is_err());
    assert_eq!(original, b.snapshot(1_400_000));
    assert!(b.begin(1_539_999).is_err());
    b.begin(1_540_000).unwrap();
    let w = b.snapshot(1_540_000);
    assert_eq!(w.pulses_used, 2);
    assert!(
        w.retained
            .iter()
            .flatten()
            .any(|p| p.start_us == 1_040_000 && p.end_us == Some(1_088_000))
    );
    assert!(b.begin(1_540_001).is_err());
}
#[test]
fn expiry_addition_overflow_does_not_create_free_credit() {
    let mut b = Ledger::new();
    b.begin(u64::MAX - 100).unwrap();
    b.end(u64::MAX - 50).unwrap();
    b.expire(u64::MAX);
    assert_eq!(b.snapshot(u64::MAX).pulses_used, 1);
    assert_eq!(b.snapshot(u64::MAX).recovery_at_us, None);
}

// This history is independent of the policy's ledger: it records only actual
// accepted assist edges from the same arbiter/driver interface used in firmware.
#[derive(Clone, Copy)]
struct ActualPulse {
    start: u64,
    end: Option<u64>,
}
struct Trial {
    r: Response,
    m: MemoryMotors,
    g: output::MotorOutput,
    timing: policy::LoopTiming,
    now: u64,
    history: Vec<ActualPulse>,
    was_assisting: bool,
}
impl Trial {
    fn new() -> Self {
        let mut s = Self {
            r: control_profile::response_with_restart(true, true).unwrap(),
            m: Default::default(),
            g: output::MotorOutput::with_policy(output::AssistPolicy::Rolling),
            timing: Default::default(),
            now: 0,
            history: Vec::new(),
            was_assisting: false,
        };
        s.g.confirm_zero(&mut s.m, || 0);
        for t in (0..=500_000).step_by(10_000) {
            s.step_at(t, 0.0);
        }
        s.step_at(520_000, 0.0);
        assert_eq!(s.r.state(), ResponseState::Active);
        s
    }
    fn step_at(&mut self, now: u64, pd: f32) -> restart_assist::Trace {
        let dt = if now == self.now {
            0.01
        } else {
            (now - self.now) as f32 / 1_000_000.0
        };
        self.now = now;
        self.r.tick(now);
        let late = self.timing.begin(now, settings::MAX_LOOP_GAP_US);
        control_profile::enforce(&mut self.r, &mut self.m, late);
        let o = ground::sample(&mut self.m, &mut self.r, pd, 0.0, true, dt, now);
        control_profile::enforce(&mut self.r, &mut self.m, false);
        if self.r.state() == ResponseState::Active && !self.r.finished && !self.m.armed {
            self.m.arm();
        }
        let _ = self.g.drive(
            &mut self.m,
            &mut self.r,
            o,
            output::Observation {
                sample_us: now,
                pitch: pd,
                rate: 0.0,
            },
            || now,
        );
        let trace = self.g.trace(now).unwrap();
        if trace.applied && !self.was_assisting {
            self.history.push(ActualPulse {
                start: now,
                end: None,
            });
        }
        if !trace.applied && self.was_assisting {
            self.history.last_mut().unwrap().end = Some(now);
        }
        self.was_assisting = trace.applied;
        self.assert_bounds_at(now);
        if let Some(window) = trace.window {
            assert!(window.pulses_used <= 2);
            assert!(window.on_us <= 300_000);
        }
        trace
    }
    fn step(&mut self, pd: f32) -> restart_assist::Trace {
        self.step_at(self.now + 10_000, pd)
    }
    fn hold(&mut self, n: usize, pd: f32) -> restart_assist::Trace {
        let mut t = self.g.trace(self.now).unwrap();
        for _ in 0..n {
            t = self.step(pd);
        }
        t
    }
    fn until(&mut self, t: u64, pd: f32) -> restart_assist::Trace {
        while t - self.now > 10_000 {
            self.step(pd);
        }
        if self.now < t {
            self.step_at(t, pd)
        } else {
            self.g.trace(t).unwrap()
        }
    }
    fn start(&mut self, sign: f32) -> u64 {
        for _ in 0..20 {
            if self.step(3.0 * sign).applied {
                return self.now;
            }
        }
        panic!("no eligible startup; {:?}", self.g.trace(self.now));
    }
    fn two_short_pulses(&mut self) {
        let first = self.start(1.0);
        self.until(first + 30_000, 3.0);
        self.step(0.0); // 40ms
        self.hold(40, 0.0);
        let second = self.start(-1.0);
        self.until(second + 38_000, -3.0);
        self.step_at(second + 48_000, 0.0);
        assert_eq!(self.history.len(), 2);
        assert_eq!(self.g.trace(self.now).unwrap().pulses_remaining, 0);
    }
    fn assert_bounds_at(&self, t: u64) {
        let left = t.saturating_sub(WINDOW_US);
        let mut count = 0;
        let mut on = 0;
        for p in &self.history {
            if p.start > t {
                continue;
            }
            let end = p.end.unwrap_or(t).min(t);
            if p.start == t || end > left || (end == p.start && p.start > left) {
                count += 1;
            }
            on += end.saturating_sub(p.start.max(left));
        }
        assert!(count <= 2, "{count} overlapping pulses at {t}");
        assert!(on <= 300_000, "{on} assist us at {t}");
        for pair in self.history.windows(2) {
            assert!(pair[1].start - pair[0].end.unwrap() >= 300_000);
        }
        for p in &self.history {
            if let Some(end) = p.end {
                assert!(end - p.start <= 150_000);
            }
        }
    }
}
#[test]
fn repeated_restarts_recover_credit_and_obey_every_trailing_window() {
    let mut s = Trial::new();
    for n in 0..20 {
        s.hold(110, 0.0);
        s.hold(20, if n % 2 == 0 { 3.0 } else { -3.0 });
    }
    assert!(s.history.len() > 2);
    assert!(s.history.len() <= 2 * (s.now / WINDOW_US + 1) as usize);
    s.r.stop();
    s.step(0.0);
    assert_eq!(s.m.outputs, (0, 0));
    // Continuous rolling usage changes slope only at pulse starts/ends and those
    // endpoints plus W. Check both sides of every such boundary, independently.
    for p in &s.history {
        for endpoint in [p.start, p.end.unwrap()] {
            for base in [endpoint, endpoint + WINDOW_US] {
                for t in [base.saturating_sub(1), base, base + 1] {
                    s.assert_bounds_at(t);
                }
            }
        }
    }
    assert_eq!(s.timing.missed_deadlines, 0);
}
#[test]
fn recovering_credit_does_not_fire_a_held_request_even_during_reversal_zero() {
    let mut s = Trial::new();
    s.two_short_pulses();
    let recovery = s.history[0].end.unwrap() + WINDOW_US;
    s.until(recovery - 50_000, 0.0);
    s.step(3.0); // Full ledger denies a new positive request.
    s.until(recovery - 1, 3.0);
    let before = s.g.trace(s.now).unwrap();
    assert_eq!(before.pulses_remaining, 0);
    assert_eq!(s.m.outputs, (0, 0));
    let t = s.until(recovery, 3.0);
    assert_eq!(t.pulses_remaining, 1);
    assert!(t.fresh_request_required);
    assert!(!t.applied);
    s.hold(110, 3.0);
    assert_eq!(s.history.len(), 2);
    assert_eq!(s.m.outputs, (-31, 31));
    assert_eq!(
        s.g.trace(s.now).unwrap().suppressed,
        Some(Reason::FreshRequestRequired)
    );
    s.hold(12, 0.0);
    s.start(1.0);
    assert_eq!(s.history.len(), 3);
}
#[test]
fn neutral_sign_chatter_and_state_toggles_do_not_reset_budget() {
    let mut s = Trial::new();
    s.two_short_pulses();
    let intervals = s.g.trace(s.now).unwrap().window.unwrap().retained;
    for n in 0..40 {
        s.step(match n % 3 {
            0 => 0.0,
            1 => 3.0,
            _ => -3.0,
        });
    }
    let t = s.g.trace(s.now).unwrap();
    assert_eq!(t.window.unwrap().retained, intervals);
    assert_eq!(t.pulses_remaining, 0);
    assert_eq!(s.history.len(), 2);
    s.r.stop();
    s.step(3.0);
    assert_eq!(s.m.outputs, (0, 0));
    s.hold(600, 0.0);
    s.m.arm();
    s.step(3.0);
    assert!(!s.m.armed);
    assert_eq!(s.m.outputs, (0, 0));
    assert_eq!(s.r.state(), ResponseState::Disarmed);
}
#[test]
fn sustained_stall_like_nonzero_demand_never_generates_periodic_assists() {
    let mut s = Trial::new();
    s.start(1.0);
    s.hold(2000, 3.0);
    assert_eq!(s.history.len(), 1);
    let t = s.g.trace(s.now).unwrap();
    assert_eq!(t.pulses_remaining, 2);
    assert!(!t.applied);
    assert_eq!(s.m.outputs, (-31, 31)); // No claim that the real motor would move.
}
#[test]
fn shared_reversal_guard_still_requires_100ms_before_recovered_assist() {
    let mut s = Trial::new();
    s.start(1.0);
    s.hold(10, 3.0);
    s.hold(110, 0.0);
    let zero = s.now + 10_000;
    s.step(-3.0);
    for _ in 0..9 {
        s.step(-3.0);
        assert_eq!(s.m.outputs, (0, 0));
    }
    s.step(-3.0);
    assert_eq!(s.now - zero, 100_000);
    assert_eq!(s.m.outputs, (65, -65));
}
fn input(t: u64, pd: i16) -> Input {
    Input {
        now_us: t,
        sample_us: t,
        active: true,
        fault: None,
        pitch: f32::from(pd),
        rate: 0.0,
        limited_pd: pd,
        mapped_pwm: pwm_mapping::map(settings::MAPPING, f32::from(pd)).unwrap(),
    }
}
fn pure_step(a: &mut Assist, t: u64, pd: i16) -> restart_assist::Trace {
    let proposal = a.update(input(t, pd));
    a.acknowledge(
        t,
        f32::from(proposal.proposed_pwm as i16),
        a.proposed_assist() && proposal.proposed_pwm != 0.0,
    )
}
#[test]
fn rolling_defaults_are_disabled_and_fixed_lifetime_option_is_preserved() {
    let mut a = Assist::new_rolling(Config::rolling_defaults());
    a.acknowledge(0, 0.0, false);
    for t in (0..=120_000).step_by(10_000) {
        pure_step(&mut a, t, 0);
    }
    let t = pure_step(&mut a, 130_000, 3);
    assert!(!t.applied);
    assert_eq!(t.suppressed, Some(Reason::Disabled));
    assert_eq!(
        output::select_policy(true, true, false, false),
        Some(output::AssistPolicy::Fixed)
    );
    assert_eq!(output::select_policy(false, true, true, false), None);
    assert_eq!(
        output::select_policy(false, true, true, true),
        Some(output::AssistPolicy::Rolling)
    );
    for reviewed in [false, true] {
        assert_eq!(output::select_policy(true, true, true, reviewed), None);
    }
}
#[test]
fn rolling_time_budget_can_exhaust_before_slots_and_recovers_without_reset() {
    let mut a = Assist::new_rolling(Config {
        enabled: true,
        total_us: 140_000,
        ..Config::rolling_defaults()
    });
    a.acknowledge(0, 0.0, false);
    for t in (0..=120_000).step_by(10_000) {
        pure_step(&mut a, t, 0);
    }
    for t in (130_000..=240_000).step_by(10_000) {
        pure_step(&mut a, t, 3);
    }
    let trace = a.status(240_000);
    assert_eq!(trace.state, State::BudgetExhausted);
    assert_eq!(trace.pulses_remaining, 1);
    assert_eq!(trace.window.unwrap().on_us, 80_000);
    assert_eq!(trace.suppressed, Some(Reason::WindowTimeLimit));
    for t in (250_000..=1_730_000).step_by(10_000) {
        pure_step(&mut a, t, 3);
    }
    let t = a.status(1_730_000);
    assert_eq!(t.pulses_remaining, 2);
    assert_eq!(t.time_remaining_us, 140_000);
    assert!(!t.applied);
    assert_eq!(t.suppressed, Some(Reason::FreshRequestRequired));
}
#[test]
fn rolling_faults_tilt_and_deadline_remain_permanent_after_credit_recovers() {
    for case in 0..4 {
        let mut s = Trial::new();
        s.start(1.0);
        match case {
            0 => {
                s.r.sensor_fail(policy::SensorFault::I2cRead);
                s.step(3.0);
            }
            1 => {
                s.step_at(s.now + 30_001, 3.0);
            }
            2 => {
                s.step(15.0);
            }
            _ => {
                s.m.reject = true;
                s.step(0.0);
            }
        }
        assert!(s.r.finished);
        assert_eq!(s.m.outputs, (0, 0));
        assert!(!s.m.armed);
        s.hold(600, 0.0);
        s.m.arm();
        s.step(3.0);
        assert_eq!(s.m.outputs, (0, 0));
        assert!(!s.m.armed);
        assert_eq!(s.g.trace(s.now).unwrap().state, State::Disarmed);
    }
}
#[test]
fn rolling_session_still_expires_at_30_seconds_without_automatic_rearming() {
    let mut s = Trial::new();
    let end = 520_000 + control_profile::ACTIVE_US;
    assert_eq!(control_profile::ACTIVE_US, 30_000_000);
    while s.now < end {
        s.step(0.0);
    }
    assert_eq!(s.r.disarm_reason, Some(policy::DisarmReason::ActiveTimeout));
    assert_eq!(s.m.outputs, (0, 0));
    s.m.arm();
    s.step(3.0);
    assert!(!s.m.armed);
}

#[test]
fn recovery_during_a_new_reversal_cannot_turn_the_denied_held_request_into_assist() {
    let mut s = Trial::new();
    s.two_short_pulses();
    let recovery = s.history[0].end.unwrap() + WINDOW_US;
    s.until(recovery - 50_000, -3.0);
    assert_eq!(s.m.outputs, (31, -31));
    s.step(3.0);
    assert_eq!(s.m.outputs, (0, 0));
    s.until(recovery, 3.0);
    assert_eq!(s.g.trace(s.now).unwrap().pulses_remaining, 1);
    s.hold(120, 3.0);
    let t = s.g.trace(s.now).unwrap();
    assert!(!t.applied);
    assert!(t.fresh_request_required);
    assert_eq!(s.history.len(), 2);
    assert_eq!(s.m.outputs, (-31, 31));
}

#[test]
fn rolling_assist_uses_robot_direction_for_requested_and_applied_duty() {
    for (pd, expected, motors) in [(3.0_f32, 70.0, (-70, 70)), (-3.0, -65.0, (65, -65))] {
        let mut s = Trial::new();
        s.start(pd.signum());
        let t = s.g.trace(s.now).unwrap();
        assert_eq!(t.requested_pwm, expected);
        assert_eq!(t.commanded_pwm, expected);
        assert_eq!(s.m.outputs, motors);
        // Continued pulse updates must retain the same directional duty.
        let t = s.hold(2, pd);
        assert!(t.applied);
        assert_eq!(t.commanded_pwm, expected);
    }
}

#[test]
fn rolling_timing_changes_do_not_relax_fixed_policy_cooldown() {
    assert_eq!(WINDOW_US, 1_500_000);
    assert_eq!(Config::rolling_defaults().cooldown_us, 300_000);
    assert_eq!(Config::default().cooldown_us, 1_000_000);
    let mut fixed = Assist::new(Config {
        cooldown_us: 300_000,
        ..Config::default()
    });
    fixed.acknowledge(0, 0.0, false);
    assert_eq!(
        pure_step(&mut fixed, 10_000, 0).suppressed,
        Some(Reason::Fault(restart_assist::Fault::InvalidConfig))
    );
}

#[test]
fn rolling_cooldown_expires_at_300ms_after_acknowledged_end() {
    let mut a = Assist::new_rolling(Config {
        enabled: true,
        ..Config::rolling_defaults()
    });
    a.acknowledge(0, 0.0, false);
    for t in (0..=120_000).step_by(10_000) {
        pure_step(&mut a, t, 0);
    }
    for t in (130_000..=160_000).step_by(10_000) {
        pure_step(&mut a, t, 3);
    }
    assert!(a.status(160_000).applied);
    assert!(!pure_step(&mut a, 170_000, 0).applied);
    for t in (180_000..=460_000).step_by(10_000) {
        pure_step(&mut a, t, 0);
    }
    assert_eq!(pure_step(&mut a, 469_999, 0).cooldown_remaining_us, 1);
    assert_eq!(pure_step(&mut a, 470_000, 0).cooldown_remaining_us, 0);
    for t in [480_000, 490_000, 500_000] {
        pure_step(&mut a, t, 3);
    }
    assert!(a.status(500_000).applied);
}
