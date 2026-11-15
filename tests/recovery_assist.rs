//! Synthetic observations only; never evidence of physical recovery.
use super::*;
use restart_assist::{Assist, Config, Input};
fn input(t: u64, pitch: f32, rate: f32) -> Input {
    let pd = balance::pd(balance::Settings::default(), pitch, rate).unwrap();
    Input {
        now_us: t,
        sample_us: t,
        active: true,
        fault: None,
        pitch,
        rate,
        limited_pd: pd,
        mapped_pwm: pwm_mapping::map(settings::MAPPING, f32::from(pd)).unwrap(),
    }
}
fn step(a: &mut Assist, t: u64, p: f32, r: f32) -> restart_assist::Trace {
    let o = a.update(input(t, p, r));
    a.acknowledge(
        t,
        f32::from(o.proposed_pwm as i16),
        a.proposed_assist() && o.proposed_pwm != 0.0,
    )
}
fn initialized(enabled: bool) -> Assist {
    initialized_sign(enabled, 1.0)
}
fn initialized_sign(enabled: bool, sign: f32) -> Assist {
    let mut a = Assist::new_rolling(Config {
        enabled: true,
        ..Config::rolling_defaults()
    });
    if enabled {
        a.enable_recovery();
    }
    a.acknowledge(0, 0.0, false);
    for t in (0..=120_000).step_by(10_000) {
        step(&mut a, t, 0.0, 0.0);
    }
    for t in (130_000..=600_000).step_by(10_000) {
        step(&mut a, t, sign * 3.0, 0.0);
    }
    a
}
#[test]
fn sustained_worsening_demand_can_qualify_one_shared_budget_recovery() {
    for sign in [1.0, -1.0] {
        let mut a = initialized_sign(true, sign);
        let mut zero_start = None;
        let mut recovery_start = None;
        for n in 0..55 {
            let t = 610_000 + n * 10_000;
            let o = step(&mut a, t, sign * (6.0 + n as f32 * 0.05), sign * 5.0);
            if o.recovery.unwrap().attempted && o.commanded_pwm == 0.0 {
                zero_start.get_or_insert(t);
            }
            if o.applied {
                recovery_start.get_or_insert(t);
                assert_eq!(o.commanded_pwm, if sign > 0.0 { 70.0 } else { -65.0 });
            }
        }
        assert!(
            recovery_start.is_some(),
            "sign={sign}, trace={:?}",
            a.status(1_150_000)
        );
        assert!(recovery_start.unwrap() - zero_start.unwrap() >= 120_000);
        assert_eq!(a.pulses_applied(), 2);
        assert_eq!(a.status(1_150_000).pulses_remaining, 0);
        a.enable_recovery(); // Repeated activation cannot reset the one-attempt latch.
        for n in 0..100 {
            step(&mut a, 1_160_000 + n * 10_000, sign * 8.8, sign * 5.0);
        }
        assert_eq!(a.pulses_applied(), 2); // Window recovery cannot grant another recovery.
    }
}
#[test]
fn feature_disabled_and_inadequate_trends_do_not_recover() {
    for (enabled, base, rate) in [
        (false, 6.0, 5.0),
        (true, 6.0, -5.0),
        (true, 4.0, 5.0),
        (true, 17.5, 20.0),
    ] {
        let mut a = initialized(enabled);
        for n in 0..55 {
            step(&mut a, 610_000 + n * 10_000, base + n as f32 * 0.005, rate);
        }
        assert_eq!(a.pulses_applied(), 1);
    }
}

fn pending() -> Assist {
    let mut a = initialized(true);
    for n in 0..32 {
        step(&mut a, 610_000 + n * 10_000, 6.0 + n as f32 * 0.05, 5.0);
    }
    assert!(a.status(920_000).recovery.unwrap().attempted);
    assert_eq!(a.status(920_000).commanded_pwm, 0.0);
    a
}
#[test]
fn pending_and_applied_recovery_cancel_without_immediate_rearming() {
    for active_pulse in [false, true] {
        for condition in 0..5 {
            let mut a = pending();
            let mut t = 920_000;
            let mut pitch = 7.55;
            if active_pulse {
                while !a.status(t).applied {
                    t += 10_000;
                    pitch += 0.05;
                    step(&mut a, t, pitch, 5.0);
                }
            }
            t += 10_000;
            let mut i = input(t, pitch + 0.05, 5.0);
            match condition {
                0 => {
                    i = input(t, pitch - 0.11, 5.0);
                } // accelerometer angle improving
                1 => {
                    i = input(t, pitch, -1.0);
                } // rate improving
                2 => {
                    i = input(t, 0.0, 0.0);
                } // neutral
                3 => {
                    i = input(t, -pitch, -5.0);
                } // opposite direction
                _ => {
                    i.sample_us = t - 20_001;
                } // stricter recovery freshness, not base fault
            }
            if condition == 1 {
                let first = a.update(i);
                if active_pulse {
                    assert!(a.proposed_assist());
                }
                assert_ne!(
                    first.recovery.unwrap().phase,
                    restart_assist::recovery::Phase::AwaitImprovement
                );
                let assisted = a.proposed_assist() && first.proposed_pwm != 0.0;
                a.acknowledge(t, f32::from(first.proposed_pwm as i16), assisted);
                t += 10_000;
                i = input(t, pitch, -1.0);
            }
            let o = a.update(i);
            assert!(!a.proposed_assist());
            assert!(o.proposed_pwm.abs() <= 50.0);
            a.acknowledge(t, f32::from(o.proposed_pwm as i16), false);
            let recovery = a.status(t).recovery.unwrap();
            assert_eq!(
                recovery.phase,
                if condition == 4 {
                    restart_assist::recovery::Phase::Spent
                } else {
                    restart_assist::recovery::Phase::AwaitImprovement
                }
            );
            assert!(recovery.attempted);
        }
    }
}
#[test]
fn margin_loss_cancels_qualification_and_retains_ordinary_command() {
    let mut a = pending();
    let o = step(&mut a, 930_000, 17.0, 20.0);
    assert_eq!(
        o.recovery.unwrap().reason,
        Some(restart_assist::recovery::Reason::Margin)
    );
    assert!(!o.applied);
    assert!(o.commanded_pwm > 0.0 && o.commanded_pwm <= 50.0);
    assert_eq!(a.pulses_applied(), 1);
}
#[test]
fn exhausted_budget_and_cooldown_never_trigger_forced_zero() {
    let mut a = initialized(true);
    // Spend the second ordinary Rolling pulse using an acknowledged neutral interval.
    for t in (610_000..=750_000).step_by(10_000) {
        step(&mut a, t, 0.0, 0.0);
    }
    for t in (760_000..=880_000).step_by(10_000) {
        step(&mut a, t, 3.0, 0.0);
    }
    assert_eq!(a.pulses_applied(), 2);
    for n in 0..35 {
        let o = step(&mut a, 890_000 + n * 10_000, 6.0 + n as f32 * 0.05, 5.0);
        assert!(!o.recovery.unwrap().attempted);
        assert!(!o.applied);
        assert!(o.commanded_pwm > 0.0);
    }
}
#[test]
fn irregular_cached_samples_cannot_supply_a_worsening_trend() {
    let mut a = initialized(true);
    for n in 0..40 {
        let t = 610_000 + n * 10_000;
        let mut i = input(t, 6.0 + n as f32 * 0.05, 5.0);
        i.sample_us = 610_000 + (n / 3) * 30_000;
        let o = a.update(i);
        a.acknowledge(t, f32::from(o.proposed_pwm as i16), a.proposed_assist());
    }
    assert!(!a.status(1_000_000).recovery.unwrap().attempted);
    assert_eq!(a.pulses_applied(), 1);
}
#[test]
fn faults_in_pending_recovery_permanently_zero_proposals() {
    for condition in 0..5 {
        let mut a = pending();
        let t = if condition == 0 { 950_001 } else { 930_000 };
        let mut i = input(t, 6.6, 5.0);
        match condition {
            1 => i.pitch = 21.0,
            2 => i.sample_us = t - 40_001,
            3 => i.fault = Some(restart_assist::Fault::Sensor),
            4 => i.active = false,
            _ => {}
        }
        let o = a.update(i);
        assert_eq!(o.state, restart_assist::State::Disarmed);
        assert_eq!(o.proposed_pwm, 0.0);
        a.acknowledge(t, 0.0, false);
        assert_eq!(
            step(&mut a, t + 10_000, 0.0, 0.0).state,
            restart_assist::State::Disarmed
        );
        assert_eq!(a.status(t + 10_000).commanded_pwm, 0.0);
    }
}

struct Integrated {
    r: Response,
    m: MemoryMotors,
    g: output::MotorOutput,
    now: u64,
    timing: policy::LoopTiming,
}
impl Integrated {
    fn new() -> Self {
        let mut s = Self {
            r: control_profile::response_with_restart(true, true).unwrap(),
            m: Default::default(),
            g: output::MotorOutput::with_policy(output::AssistPolicy::Rolling),
            now: 0,
            timing: Default::default(),
        };
        s.g.enable_recovery();
        s.g.confirm_zero(&mut s.m, || 0);
        for t in (0..=520_000).step_by(10_000) {
            s.at(t, 0.0, 0.0, true);
        }
        assert_eq!(s.r.state(), ResponseState::Active);
        for t in (530_000..=1_000_000).step_by(10_000) {
            s.at(t, 3.0, 0.0, true);
        }
        s
    }
    fn at(&mut self, t: u64, p: f32, rate: f32, corrected: bool) {
        let dt = if t == self.now {
            0.01
        } else {
            (t - self.now) as f32 / 1_000_000.0
        };
        self.now = t;
        self.r.tick(t);
        let late = self.timing.begin(t, settings::MAX_LOOP_GAP_US);
        control_profile::enforce(&mut self.r, &mut self.m, late);
        let o = ground::sample(&mut self.m, &mut self.r, p, rate, corrected, dt, t);
        control_profile::enforce(&mut self.r, &mut self.m, false);
        if self.r.state() == ResponseState::Active && !self.r.finished && !self.m.armed {
            self.m.arm();
        }
        let _ = self.g.drive(
            &mut self.m,
            &mut self.r,
            o,
            output::Observation {
                sample_us: t,
                pitch: p,
                rate,
            },
            || t,
        );
    }
}
#[test]
fn integrated_recovery_uses_actual_zero_interface_and_shared_ledger() {
    let mut s = Integrated::new();
    let mut zero = None;
    let mut applied = None;
    for n in 0..55 {
        let t = 1_010_000 + n * 10_000;
        s.at(t, 6.0 + n as f32 * 0.05, 5.0, true);
        let o = s.g.trace(t).unwrap();
        assert_eq!(s.m.outputs, balance::wheel_commands(o.commanded_pwm as i16));
        if o.recovery.unwrap().attempted && s.m.outputs == (0, 0) {
            zero.get_or_insert(t);
        }
        if o.recovery.unwrap().phase == restart_assist::recovery::Phase::Pulse {
            applied.get_or_insert(t);
            assert_eq!(s.m.outputs, (-70, 70));
        }
    }
    let required = if cfg!(feature = "balance-control-short-recovery-zero") {
        20_000
    } else {
        120_000
    };
    assert!(applied.unwrap() - zero.unwrap() >= required);
    assert_eq!(s.g.kicks(s.now), 2);
    assert_eq!(s.g.trace(s.now).unwrap().pulses_remaining, 0);
    s.r.stop();
    s.at(s.now + 10_000, 8.8, 5.0, true);
    assert!(!s.m.armed);
    assert_eq!(s.m.outputs, (0, 0));
}
#[test]
fn integrated_faults_during_zero_or_pulse_permanently_stop_real_interface() {
    for during_pulse in [false, true] {
        for condition in 0..7 {
            let mut s = Integrated::new();
            for n in 0..65 {
                s.at(1_010_000 + n * 10_000, 6.0 + n as f32 * 0.05, 5.0, true);
                let phase = s.g.trace(s.now).unwrap().recovery.unwrap().phase;
                if phase
                    == if during_pulse {
                        restart_assist::recovery::Phase::Pulse
                    } else {
                        restart_assist::recovery::Phase::ZeroWait
                    }
                {
                    break;
                }
            }
            let t = s.now + if condition == 0 { 30_001 } else { 10_000 };
            match condition {
                1 => s.r.sensor_fail(policy::SensorFault::I2cRead),
                2 => s.m.reject = true,
                3 => s.r.stop(),
                _ => {}
            }
            if condition == 4 {
                // Exact existing ACTIVE end; tick must terminate before a driver command.
                s.r.tick(30_520_000);
            }
            s.at(
                t,
                if condition == 5 {
                    21.0
                } else if condition == 6 {
                    f32::NAN
                } else {
                    7.5
                },
                5.0,
                true,
            );
            assert!(s.r.finished, "case={condition}");
            assert_eq!(s.m.outputs, (0, 0));
            assert!(!s.m.armed);
            s.m.reject = false;
            s.m.arm();
            s.at(t + 10_000, 0.0, 0.0, true);
            assert_eq!(s.m.outputs, (0, 0));
            assert!(!s.m.armed);
        }
    }
}

#[test]
fn factory_feature_is_rolling_only_and_dry_never_commands_motors() {
    let rolling = output::MotorOutput::with_policy(output::AssistPolicy::Rolling);
    assert_eq!(
        rolling.trace(0).unwrap().recovery.is_some(),
        cfg!(feature = "balance-control-recovery-assist")
    );
    let fixed = output::MotorOutput::with_policy(output::AssistPolicy::Fixed);
    assert!(fixed.trace(0).unwrap().recovery.is_none());
    let mut r = control_profile::response_with_restart(false, true).unwrap();
    let mut m = MemoryMotors::default();
    let mut g = rolling;
    g.enable_recovery();
    g.confirm_zero(&mut m, || 0);
    for n in 0..100 {
        let t = n * 10_000;
        let o = r.sample(6.0 + n as f32 * 0.05, 5.0, true, 0.01, t);
        g.drive(
            &mut m,
            &mut r,
            o,
            output::Observation {
                sample_us: t,
                pitch: 6.0,
                rate: 5.0,
            },
            || t,
        )
        .unwrap();
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
    }
}
#[test]
fn integrated_correction_expiry_during_recovery_is_a_permanent_fault() {
    let mut s = Integrated::new();
    for n in 0..65 {
        s.at(1_010_000 + n * 10_000, 6.0 + n as f32 * 0.05, 5.0, true);
        if s.g.trace(s.now).unwrap().recovery.unwrap().phase
            == restart_assist::recovery::Phase::Pulse
        {
            break;
        }
    }
    assert!(s.g.trace(s.now).unwrap().applied);
    for n in 0..6 {
        s.at(s.now + 10_000, 7.5 + n as f32 * 0.05, 5.0, false);
    }
    assert!(s.r.finished);
    assert_eq!(
        s.r.sensor_fault,
        Some(policy::SensorFault::CorrectionExpired)
    );
    assert_eq!(s.m.outputs, (0, 0));
    assert!(!s.m.armed);
}

#[test]
fn recovery_margin_and_evidence_boundaries_are_conservative() {
    use restart_assist::recovery::{Decision, Reason, Recovery};
    let mut r = Recovery::new();
    let mut i = input(0, 6.0, 5.0);
    assert_eq!(
        r.update(i, true, true, true, true, None, 100_000, 20_000, 150_000),
        Decision::Ordinary
    );
    for n in 1..=5 {
        i = input(n * 20_000, 6.0 + n as f32 * 0.1, 5.0);
        let d = r.update(i, true, true, true, true, None, 100_000, 20_000, 150_000);
        assert_eq!(
            d,
            if n == 5 {
                Decision::Probe
            } else {
                Decision::Ordinary
            }
        );
    }
    for angle in [16.36, 18.0, 20.99] {
        let mut r = Recovery::new();
        r.update(
            input(0, angle, 5.0),
            true,
            true,
            true,
            true,
            None,
            100_000,
            20_000,
            150_000,
        );
        assert_eq!(r.telemetry.reason, Some(Reason::Margin));
        assert!(!r.telemetry.attempted);
    }
    let mut r = Recovery::new();
    r.update(
        input(0, 6.0, 5.0),
        true,
        true,
        true,
        true,
        None,
        100_000,
        20_000,
        150_000,
    );
    r.update(
        input(20_001, 6.2, 5.0),
        true,
        true,
        true,
        true,
        None,
        100_000,
        20_000,
        150_000,
    );
    assert_eq!(r.telemetry.reason, Some(Reason::EvidenceGap));
    assert!(!r.telemetry.attempted);
}

#[test]
fn control_21_degree_boundary_stops_actual_outputs_for_all_authorities() {
    for mode in [
        output::AssistPolicy::Legacy,
        output::AssistPolicy::Fixed,
        output::AssistPolicy::Rolling,
    ] {
        for sign in [1.0, -1.0] {
            let mut s = Integrated::new();
            s.g = output::MotorOutput::with_policy(mode);
            s.g.confirm_zero(&mut s.m, || s.now);
            s.at(s.now + 10_000, sign * 20.999, 0.0, true);
            assert!(!s.r.finished);
            s.at(s.now + 10_000, sign * 21.0, 0.0, true);
            assert_eq!(
                s.r.disarm_reason,
                Some(policy::DisarmReason::Fault(balance::Fault::Tilt))
            );
            assert_eq!(s.m.outputs, (0, 0));
            assert!(!s.m.armed);
            s.at(s.now + 10_000, 0.0, 0.0, true);
            assert!(s.r.finished);
            assert_eq!(s.m.outputs, (0, 0));
        }
    }
    assert_eq!(
        control_profile::response(true)
            .unwrap()
            .control
            .tilt_limit(),
        21.0
    );
    assert_eq!(
        Response::new(
            balance::Settings::default(),
            settings::MAPPING,
            settings::KICK,
            70,
            true,
            1_000_000,
            10_000_000
        )
        .unwrap()
        .control
        .tilt_limit(),
        15.0
    );
    assert_eq!(
        balance::CONTROL_FALL_ANGLE_DEG - restart_assist::recovery::CUTOFF_MARGIN,
        18.0
    );
}
#[test]
fn recovery_now_accepts_12_to_18_margin_range_and_rejects_18() {
    use restart_assist::recovery::{Decision, Reason, Recovery};
    let mut r = Recovery::new();
    let mut decision = Decision::Ordinary;
    for n in 0..=10 {
        decision = r.update(
            input(n * 10_000, 13.0 + n as f32 * 0.05, 5.0),
            true,
            true,
            true,
            true,
            None,
            100_000,
            20_000,
            150_000,
        );
    }
    assert_eq!(decision, Decision::Probe);
    let mut r = Recovery::new();
    r.update(
        input(0, 18.0, 5.0),
        true,
        true,
        true,
        true,
        None,
        100_000,
        20_000,
        150_000,
    );
    assert_eq!(r.telemetry.reason, Some(Reason::Margin));
    assert!(!r.telemetry.attempted);
}

#[test]
fn slow_worsening_can_recover_after_a_bounded_ordinary_drive_probe() {
    for sign in [1.0, -1.0] {
        let mut a = initialized_sign(true, sign);
        let mut probe = false;
        let mut recovery = false;
        for n in 0..150 {
            let t = 610_000 + n * 10_000;
            let o = step(&mut a, t, sign * (8.0 + n as f32 * 0.005), sign * 0.5);
            if !o.applied
                && o.proposed_pwm.abs()
                    > input(t, sign * (8.0 + n as f32 * 0.005), sign * 0.5)
                        .mapped_pwm
                        .abs()
            {
                probe = true;
            }
            recovery |= o.applied;
        }
        assert!(probe);
        assert!(recovery);
        assert_eq!(a.pulses_applied(), 2);
    }
}
#[test]
fn falling_but_still_outward_rate_does_not_cancel_at_28ms() {
    let mut a = pending();
    let mut t = 920_000;
    let mut p = 7.55;
    while !a.status(t).applied {
        t += 10_000;
        p += 0.05;
        step(&mut a, t, p, 5.0);
    }
    let start = t;
    for n in 1..=9 {
        let o = step(&mut a, start + n * 10_000, p + n as f32 * 0.01, 1.0);
        assert!(o.applied, "pulse terminated at {}ms: {:?}", n * 10, o);
    }
    let o = step(&mut a, start + 100_000, p + 0.1, 1.0);
    assert!(!o.applied);
    assert_eq!(o.last_pulse_us, 100_000);
}

#[test]
fn observed_improvement_allows_new_deterioration_but_not_periodic_retry() {
    for sign in [1.0, -1.0] {
        let mut a = initialized_sign(true, sign);
        let mut t = 600_000;
        for n in 0..60 {
            t += 10_000;
            step(&mut a, t, sign * (6.0 + n as f32 * 0.05), sign * 5.0);
        }
        assert_eq!(a.pulses_applied(), 2);
        // Held worsening cannot reset eligibility, even when credits expire.
        for _ in 0..170 {
            t += 10_000;
            step(&mut a, t, sign * 9.0, sign * 1.0);
        }
        assert_eq!(a.pulses_applied(), 2);
        // Sustained measurable movement toward upright constitutes a new episode.
        for n in 0..50 {
            t += 10_000;
            step(&mut a, t, sign * (9.0 - n as f32 * 0.02), sign * (-2.0));
        }
        assert!(!a.status(t).recovery.unwrap().attempted);
        for n in 0..120 {
            t += 10_000;
            step(&mut a, t, sign * (8.02 + n as f32 * 0.005), sign * 0.5);
        }
        assert_eq!(a.pulses_applied(), 3);
        assert!(a.status(t).total_on_us <= 300_000);
    }
}
#[test]
fn probe_is_bounded_and_improving_motion_reduces_ordinary_drive() {
    let mut a = initialized(true);
    let mut probe_start = None;
    let mut zero_start = None;
    for n in 0..40 {
        let t = 610_000 + n * 10_000;
        let p = 6.0 + n as f32 * 0.05;
        let o = step(&mut a, t, p, 5.0);
        if o.recovery.unwrap().phase == restart_assist::recovery::Phase::DriveProbe {
            probe_start.get_or_insert(t);
            assert!(!o.applied);
            assert!(
                (o.proposed_pwm.abs() - (input(t, p, 5.0).mapped_pwm.abs() + 5.0).min(50.0)).abs()
                    < 0.001
            );
        }
        if o.commanded_pwm == 0.0 && o.recovery.unwrap().attempted {
            zero_start.get_or_insert(t);
        }
    }
    assert_eq!(zero_start.unwrap() - probe_start.unwrap(), 200_000);
    let o = step(&mut a, 1_010_000, 7.8, -5.0);
    let raw = input(1_010_000, 7.8, -5.0).mapped_pwm;
    assert!(!o.applied);
    assert!(o.commanded_pwm > 0.0 && o.commanded_pwm <= raw * 0.5);
}
#[test]
fn target_100ms_keeps_150ms_ceiling_and_checks_tighter_ack_reserve() {
    let mut a = initialized(true);
    assert_eq!(a.io_limit_us(), 20_000);
    let o = a.update(input(610_000, 6.0, 5.0));
    let ack = a.acknowledge(630_001, f32::from(o.proposed_pwm as i16), false);
    assert_eq!(ack.state, restart_assist::State::Disarmed);
    assert_eq!(
        ack.suppressed,
        Some(restart_assist::Reason::Fault(
            restart_assist::Fault::CommandDeadline
        ))
    );
}

#[test]
fn measured_loop_and_io_jitter_cannot_extend_pulse_beyond_150ms() {
    let mut a = Assist::new_rolling(Config {
        enabled: true,
        ..Config::rolling_defaults()
    });
    a.enable_recovery();
    a.acknowledge(0, 0.0, false);
    for t in (0..=120_000).step_by(10_000) {
        step(&mut a, t, 0.0, 0.0);
    }
    for t in [130_000, 140_000, 150_000] {
        step(&mut a, t, 3.0, 0.0);
    }
    assert!(a.status(150_000).applied);
    for t in [159_000, 189_000, 219_000, 249_000, 279_000] {
        let o = a.update(input(t, 3.0, 0.0));
        let assisted = a.proposed_assist();
        a.acknowledge(t + 20_000, f32::from(o.proposed_pwm as i16), assisted);
        assert_ne!(a.status(t + 20_000).state, restart_assist::State::Disarmed);
    }
    let o = a.status(299_000);
    assert!(!o.applied);
    assert_eq!(o.last_pulse_us, 149_000);
    assert!(o.window.unwrap().on_us <= 300_000);
}

fn fixed_mapped_step(a: &mut Assist, t: u64, p: f32, mapped: f32) -> restart_assist::Trace {
    let mut i = input(t, p, 5.0 * p.signum());
    i.mapped_pwm = mapped;
    let o = a.update(i);
    a.acknowledge(
        t,
        f32::from(o.proposed_pwm as i16),
        a.proposed_assist() && o.proposed_pwm != 0.0,
    )
}
#[test]
fn effective_probe_increase_controls_45_48_50_transition_timing() {
    use restart_assist::recovery::Phase;
    for mapped in [45.0_f32, 48.0, 50.0, -45.0, -48.0, -50.0] {
        let sign = mapped.signum();
        let magnitude = mapped.abs();
        let mut a = initialized_sign(true, sign);
        let mut probe = None;
        let mut zero = None;
        let mut pulse = None;
        for n in 0..55 {
            let t = 610_000 + n * 10_000;
            let o = fixed_mapped_step(&mut a, t, sign * (12.0 + n as f32 * 0.05), mapped);
            if o.recovery.unwrap().phase == Phase::DriveProbe {
                probe.get_or_insert(t);
                assert_eq!(o.commanded_pwm, sign * 50.0);
                assert_eq!(
                    o.commanded_pwm.abs() - magnitude,
                    if magnitude == 45.0 { 5.0 } else { 2.0 }
                );
            }
            if o.recovery.unwrap().attempted && o.commanded_pwm == 0.0 {
                zero.get_or_insert(t);
            }
            if o.applied {
                pulse.get_or_insert(t);
            }
        }
        let qualification = 710_000;
        if magnitude == 50.0 {
            assert!(probe.is_none());
            assert_eq!(zero, Some(qualification));
        } else {
            assert_eq!(probe, Some(qualification));
            assert_eq!(zero.unwrap() - probe.unwrap(), 200_000);
        }
        assert_eq!(pulse.unwrap() - zero.unwrap(), 120_000);
        assert_eq!(a.pulses_applied(), 2);
    }
}
#[test]
fn capped_probe_projection_excludes_200ms_but_useful_probe_keeps_it() {
    for mapped in [45.0, 48.0, 50.0] {
        let mut a = initialized(true);
        for n in 0..=10 {
            fixed_mapped_step(&mut a, 610_000 + n * 10_000, 15.7 + n as f32 * 0.05, mapped);
        }
        let o = a.status(710_000);
        assert_eq!(o.recovery.unwrap().attempted, mapped == 50.0);
        if mapped == 50.0 {
            assert_eq!(o.commanded_pwm, 0.0);
        } else {
            assert_eq!(
                o.recovery.unwrap().reason,
                Some(restart_assist::recovery::Reason::Margin)
            );
        }
    }
}

#[test]
fn probe_cap_reached_mid_probe_skips_remaining_wait_and_can_cancel() {
    use restart_assist::recovery::{Phase, Reason};
    for mapped in [45.0, 48.0, 50.0] {
        for fault in [false, true] {
            let mut a = initialized(true);
            for n in 0..=10 {
                fixed_mapped_step(&mut a, 610_000 + n * 10_000, 12.0 + n as f32 * 0.05, mapped);
            }
            if mapped < 50.0 {
                assert_eq!(a.status(710_000).recovery.unwrap().phase, Phase::DriveProbe);
            }
            let o = fixed_mapped_step(&mut a, 720_000, 12.55, 50.0);
            assert_eq!(o.commanded_pwm, 0.0);
            assert_eq!(o.recovery.unwrap().phase, Phase::ZeroWait);
            assert_eq!(o.recovery.unwrap().reason, Some(Reason::ProbeCapped));
            let mut i = input(730_000, if fault { 21.0 } else { 0.0 }, 0.0);
            if fault {
                i.mapped_pwm = 50.0;
            }
            let p = a.update(i);
            a.acknowledge(730_000, f32::from(p.proposed_pwm as i16), false);
            assert_eq!(p.proposed_pwm, 0.0);
            assert!(!p.applied);
            assert_eq!(a.pulses_applied(), 1);
            if fault {
                assert_eq!(p.state, restart_assist::State::Disarmed);
            } else {
                assert_eq!(p.recovery.unwrap().phase, Phase::AwaitImprovement);
            }
        }
    }
}
#[test]
fn capped_probe_integrated_zero_wait_and_fault_stop_actual_motor_interface() {
    use restart_assist::recovery::Phase;
    for fault in [false, true] {
        let mut s = Integrated::new();
        for n in 0..=10 {
            s.at(1_010_000 + n * 10_000, 15.0 + n as f32 * 0.05, 5.0, true);
        }
        assert_eq!(
            s.g.trace(s.now).unwrap().recovery.unwrap().phase,
            Phase::ZeroWait
        );
        assert_eq!(s.m.outputs, (0, 0));
        let zero_samples = if cfg!(feature = "balance-control-short-recovery-zero") {
            2
        } else {
            9
        };
        for n in 1..=zero_samples {
            s.at(1_110_000 + n * 10_000, 15.5 + n as f32 * 0.02, 5.0, true);
            assert_eq!(s.m.outputs, (0, 0));
            assert_eq!(s.g.kicks(s.now), 1);
        }
        s.at(s.now + 10_000, if fault { 21.0 } else { 0.0 }, 0.0, true);
        assert_eq!(s.m.outputs, (0, 0));
        assert_eq!(s.g.kicks(s.now), 1);
        if !fault {
            s.r.stop();
            s.at(s.now + 10_000, 0.0, 0.0, true);
        }
        assert!(s.r.finished);
        assert!(!s.m.armed);
    }
}

#[test]
fn short_recovery_zero_only_removes_same_direction_coast_not_dwell() {
    for sign in [1.0, -1.0] {
        for short in [false, true] {
            let mut a = initialized_sign(true, sign);
            if short {
                a.enable_short_recovery_zero();
            }
            let mut zero = None;
            let mut pulse = None;
            for n in 0..35 {
                let t = 610_000 + n * 10_000;
                let o = fixed_mapped_step(&mut a, t, sign * (12.0 + n as f32 * 0.05), sign * 50.0);
                if o.recovery.unwrap().attempted && o.commanded_pwm == 0.0 {
                    zero.get_or_insert(t);
                }
                if o.applied {
                    pulse.get_or_insert(t);
                }
            }
            assert_eq!(
                pulse.unwrap() - zero.unwrap(),
                if short { 30_000 } else { 120_000 }
            );
            assert_eq!(a.pulses_applied(), 2);
        }
    }
}

#[test]
fn short_recovery_zero_does_not_change_initial_start_or_cooldown() {
    for short in [false, true] {
        let mut early = Assist::new_rolling(Config {
            enabled: true,
            ..Config::rolling_defaults()
        });
        early.enable_recovery();
        if short {
            early.enable_short_recovery_zero();
        }
        early.acknowledge(0, 0.0, false);
        for t in (10_000..=90_000).step_by(10_000) {
            assert!(!step(&mut early, t, 3.0, 0.0).applied);
        }
        assert_eq!(early.pulses_applied(), 0);
        let mut a = Assist::new_rolling(Config {
            enabled: true,
            ..Config::rolling_defaults()
        });
        a.enable_recovery();
        if short {
            a.enable_short_recovery_zero();
        }
        a.acknowledge(0, 0.0, false);
        for t in (0..=120_000).step_by(10_000) {
            step(&mut a, t, 0.0, 0.0);
        }
        for t in [130_000, 140_000, 150_000] {
            let o = step(&mut a, t, 3.0, 0.0);
            assert_eq!(o.applied, t == 150_000);
        }
        for t in (160_000..=260_000).step_by(10_000) {
            step(&mut a, t, 3.0, 0.0);
        }
        for n in 0..28 {
            let o = fixed_mapped_step(&mut a, 270_000 + n * 10_000, 8.0 + n as f32 * 0.005, 50.0);
            assert!(!o.recovery.unwrap().attempted);
            assert!(!o.applied);
        }
        assert_eq!(a.pulses_applied(), 1);
    }
}
#[test]
fn short_recovery_zero_reversal_and_chatter_keep_the_real_100ms_interlock() {
    for chatter in [false, true] {
        let mut s = Integrated::new();
        s.g.enable_short_recovery_zero();
        for n in 0..=10 {
            s.at(1_010_000 + n * 10_000, 15.0 + n as f32 * 0.05, 5.0, true);
        }
        assert_eq!(s.m.outputs, (0, 0));
        assert_eq!(s.g.trace(s.now).unwrap().recovery.unwrap().zero_wait_us, 0);
        let reversal = s.now + 10_000;
        let mut first_opposite = None;
        for n in 0..20 {
            let sign = if chatter && n % 2 == 1 { 1.0 } else { -1.0 };
            s.at(
                reversal + n * 10_000,
                sign * (15.55 + n as f32 * 0.001),
                sign * 5.0,
                true,
            );
            if s.m.outputs.1 < 0 {
                first_opposite.get_or_insert(s.now);
            }
            if s.now - reversal < 100_000 {
                assert!(s.m.outputs.1 >= 0);
            }
        }
        if chatter {
            assert!(first_opposite.is_none());
            assert_eq!(s.g.kicks(s.now), 1);
        } else {
            assert!(first_opposite.unwrap() - reversal >= 100_000);
        }
        s.r.stop();
        s.at(s.now + 10_000, 0.0, 0.0, true);
        assert_eq!(s.m.outputs, (0, 0));
        assert!(!s.m.armed);
    }
}
#[test]
fn short_recovery_zero_cancellation_and_faults_never_complete_a_pending_kick() {
    for condition in 0..4 {
        let mut s = Integrated::new();
        s.g.enable_short_recovery_zero();
        for n in 0..=10 {
            s.at(1_010_000 + n * 10_000, 15.0 + n as f32 * 0.05, 5.0, true);
        }
        assert_eq!(s.m.outputs, (0, 0));
        match condition {
            0 => s.at(s.now + 10_000, 0.0, 0.0, true),
            1 => s.at(s.now + 10_000, 21.0, 5.0, true),
            2 => {
                s.r.sensor_fail(policy::SensorFault::I2cRead);
                s.at(s.now + 10_000, 15.6, 5.0, true);
            }
            _ => s.at(s.now + 30_001, 15.6, 5.0, true),
        }
        assert_eq!(s.g.kicks(s.now), 1);
        assert_eq!(s.m.outputs, (0, 0));
        if condition == 0 {
            s.r.stop();
            s.at(s.now + 10_000, 0.0, 0.0, true);
        }
        assert!(s.r.finished);
        assert!(!s.m.armed);
        s.m.arm();
        s.at(s.now + 10_000, 0.0, 0.0, true);
        assert_eq!(s.m.outputs, (0, 0));
        assert!(!s.m.armed);
    }
}
#[test]
fn short_recovery_zero_does_not_make_held_demand_or_budget_expiry_a_retry() {
    let mut a = initialized(true);
    a.enable_short_recovery_zero();
    let mut t = 600_000;
    for n in 0..55 {
        t += 10_000;
        fixed_mapped_step(&mut a, t, 12.0 + n as f32 * 0.02, 50.0);
    }
    // Slow growth here qualifies the existing 500ms path, not a fresh automatic retry.
    for _ in 0..60 {
        t += 10_000;
        fixed_mapped_step(&mut a, t, 13.1, 50.0);
    }
    assert_eq!(a.pulses_applied(), 2);
    for _ in 0..200 {
        t += 10_000;
        fixed_mapped_step(&mut a, t, 13.1, 50.0);
    }
    assert_eq!(a.pulses_applied(), 2);
    assert_eq!(a.status(t).pulses_remaining, 2);
    for n in 0..50 {
        t += 10_000;
        step(&mut a, t, 13.1 - n as f32 * 0.02, -2.0);
    }
    assert!(!a.status(t).recovery.unwrap().attempted);
    for n in 0..40 {
        t += 10_000;
        fixed_mapped_step(&mut a, t, 12.12 + n as f32 * 0.05, 50.0);
    }
    assert_eq!(a.pulses_applied(), 3);
    assert!(a.status(t).window.unwrap().on_us <= 300_000);
}

#[test]
fn fresh_100hz_with_1ms_cached_service_qualifies_fast_and_slow_recovery() {
    for (angle, rate, growth) in [(6.0, 5.0, 0.05), (8.0, 0.5, 0.005)] {
        let mut s = Integrated::new();
        let mut applied = false;
        for n in 0..150 {
            let t = 1_010_000 + n * 10_000;
            s.at(t, angle + n as f32 * growth, rate, true);
            applied |= s.g.trace(t).unwrap().recovery.unwrap().pulse_applied;
            for offset in (1_000..10_000).step_by(1_000) {
                s.g.service(&mut s.m, &mut s.r, || t + offset).unwrap();
                applied |=
                    s.g.trace(t + offset)
                        .unwrap()
                        .recovery
                        .unwrap()
                        .pulse_applied;
            }
            assert!(!s.r.finished);
        }
        assert!(applied, "rate={rate}; trace={:?}", s.g.trace(s.now));
        s.r.stop();
        s.at(s.now + 10_000, 0.0, 0.0, true);
        assert_eq!(s.m.outputs, (0, 0));
        assert!(!s.m.armed);
    }
}
#[test]
fn rejected_fresh_samples_do_not_create_a_sensor_gap() {
    use restart_assist::recovery::{Reason, Recovery};
    for rejected in 0..3 {
        let mut r = Recovery::new();
        for n in 0..=5 {
            let mut i = input(n * 10_000, 8.0 + n as f32 * 0.01, 5.0);
            if n > 0 && n < 5 {
                if rejected == 0 {
                    i.rate = -1.0;
                }
                if rejected == 1 {
                    i.pitch = 20.0;
                }
            }
            r.update(
                i,
                true,
                !(rejected == 2 && n > 0 && n < 5),
                true,
                true,
                None,
                100_000,
                20_000,
                150_000,
            );
        }
        assert_ne!(
            r.telemetry.reason,
            Some(Reason::EvidenceGap),
            "rejection={rejected}"
        );
    }
}
fn initial_running_kick(sign: f32) -> Assist {
    let mut a = Assist::new_rolling(Config {
        enabled: true,
        ..Config::rolling_defaults()
    });
    a.enable_recovery();
    a.acknowledge(0, 0.0, false);
    for t in (0..=120_000).step_by(10_000) {
        step(&mut a, t, 0.0, 0.0);
    }
    for t in [130_000, 140_000, 150_000] {
        step(&mut a, t, sign * 6.0, 0.0);
    }
    assert!(a.status(150_000).applied);
    a
}
#[test]
fn isolated_inward_rate_and_cached_repeats_do_not_cancel_running_kick() {
    for sign in [1.0, -1.0] {
        let mut a = initial_running_kick(sign);
        let i = input(160_000, sign * 6.001, -sign * 0.1);
        let o = a.update(i);
        assert!(a.proposed_assist(), "isolated rate: {o:?}");
        a.acknowledge(160_000, f32::from(o.proposed_pwm as i16), true);
        for t in (161_000..170_000).step_by(1_000) {
            let mut cached = i;
            cached.now_us = t;
            let o = a.update(cached);
            assert!(a.proposed_assist());
            a.acknowledge(t, f32::from(o.proposed_pwm as i16), true);
        }
        assert!(step(&mut a, 170_000, sign * 6.002, sign * 0.1).applied);
        assert!(step(&mut a, 180_000, sign * 6.003, -sign * 0.1).applied);
        let o = step(&mut a, 190_000, sign * 6.002, -sign * 0.1);
        assert!(!o.applied);
        assert_eq!(
            o.suppressed,
            Some(restart_assist::Reason::TrajectoryImproving)
        );
    }
}

#[test]
fn actual_received_gap_still_resets_evidence_after_rejected_samples() {
    use restart_assist::recovery::{Phase, Reason, Recovery};
    let mut r = Recovery::new();
    for n in 0..=3 {
        r.update(
            input(n * 10_000, 8.0 + n as f32 * 0.01, 5.0),
            true,
            n == 0,
            true,
            true,
            None,
            100_000,
            20_000,
            150_000,
        );
    }
    r.update(
        input(50_001, 8.05, 5.0),
        true,
        true,
        true,
        true,
        None,
        100_000,
        20_000,
        150_000,
    );
    assert_eq!(r.telemetry.reason, Some(Reason::EvidenceGap));
    r.update(
        input(60_001, 8.06, 5.0),
        true,
        true,
        true,
        true,
        None,
        100_000,
        20_000,
        150_000,
    );
    assert_eq!(r.telemetry.phase, Phase::Trend);
    assert_ne!(r.telemetry.reason, Some(Reason::EvidenceGap));
}
#[test]
fn measured_angle_reduction_can_end_a_kick_without_waiting_for_two_rates() {
    for sign in [1.0, -1.0] {
        let mut a = initial_running_kick(sign);
        let o = step(&mut a, 160_000, sign * 5.89, sign * 0.1);
        assert!(!o.applied);
        assert_eq!(
            o.suppressed,
            Some(restart_assist::Reason::TrajectoryImproving)
        );
        assert_eq!(o.last_pulse_us, 10_000);
    }
}
#[test]
fn cached_spike_does_not_extend_kick_past_target_or_hide_stale_data() {
    let mut a = initial_running_kick(1.0);
    for t in (160_000..=250_000).step_by(10_000) {
        let rate = if t == 190_000 { -0.1 } else { 0.1 };
        let o = step(&mut a, t, 6.001, rate);
        if t < 250_000 {
            assert!(o.applied);
        } else {
            assert!(!o.applied);
            assert_eq!(o.last_pulse_us, 100_000);
        }
    }
    let mut a = initial_running_kick(1.0);
    let mut cached = input(150_000, 6.0, 0.0);
    for t in (151_000..=191_000).step_by(1_000) {
        cached.now_us = t;
        let o = a.update(cached);
        if t > 190_000 {
            assert_eq!(o.state, restart_assist::State::Disarmed);
            assert_eq!(o.proposed_pwm, 0.0);
        }
        a.acknowledge(
            t,
            f32::from(o.proposed_pwm as i16),
            a.proposed_assist() && o.proposed_pwm != 0.0,
        );
    }
}

// Reported anchors only. Intermediate samples below are SYNTHETIC alternatives,
// not a reconstruction of the unavailable raw LIVE trace. Seed the confirmed
// post-pulse AwaitImprovement state to isolate its actual requalification rule.
const REPORTED_RECOVERY_US: u64 = 28_976_000;
const REPORTED_RECOVERY_END_US: u64 = REPORTED_RECOVERY_US + 47_500;
fn reported_post_pulse() -> restart_assist::recovery::Recovery {
    use restart_assist::recovery::{Reason, Recovery};
    let mut r = Recovery::new();
    r.telemetry.attempted = true;
    r.telemetry.pulse_applied = true;
    r.finish(Reason::Improving);
    // Synthetic improvement baseline uses the reported starting angle. The
    // actual angle at pulse end, and intermediate inward rates, were not supplied.
    r.update(
        input(REPORTED_RECOVERY_END_US, 8.89, -1.0),
        true,
        false,
        true,
        true,
        None,
        100_000,
        20_000,
        150_000,
    );
    r
}
#[test]
fn reported_angle_bounds_do_not_imply_a_200ms_requalification_interval() {
    use restart_assist::recovery::{Decision, Phase, Reason};
    for duration in [120_000_u64, 300_000] {
        let mut r = reported_post_pulse();
        let samples = duration / 10_000;
        let rate = -(8.89 - 6.72) / (duration as f32 / 1_000_000.0);
        let mut rearmed = false;
        for n in 1..=samples {
            let t = REPORTED_RECOVERY_END_US + n * 10_000;
            let angle = 8.89 - (8.89 - 6.72) * n as f32 / samples as f32;
            let i = input(t, angle, rate);
            // Resource rejection does not apply while observing improvement.
            r.update(i, true, false, true, true, None, 100_000, 20_000, 150_000);
            rearmed |= r.telemetry.reason == Some(Reason::Rearmed);
            for offset in (1_000..10_000).step_by(1_000) {
                let mut cached = i;
                cached.now_us = t + offset;
                r.update(
                    cached, false, false, true, true, None, 100_000, 20_000, 150_000,
                );
            }
        }
        assert_eq!(rearmed, duration == 300_000);
        let mut qualified = false;
        for n in 1..=60 {
            let t = REPORTED_RECOVERY_END_US + duration + n * 10_000;
            let angle = 6.72 + (12.27 - 6.72) * n as f32 / 60.0;
            let d = r.update(
                input(t, angle, 9.25),
                true,
                t >= REPORTED_RECOVERY_END_US + 300_000,
                true,
                true,
                None,
                100_000,
                20_000,
                150_000,
            );
            qualified |= matches!(d, Decision::Probe | Decision::Zero | Decision::Permit);
        }
        if duration == 120_000 {
            assert_eq!(r.telemetry.phase, Phase::AwaitImprovement);
            assert!(!qualified);
        } else {
            assert!(qualified);
        }
    }
}
#[test]
fn requalification_requires_uninterrupted_improvement_not_only_total_angle_drop() {
    use restart_assist::recovery::{Phase, Reason};
    for interruption in 0..3 {
        let mut r = reported_post_pulse();
        for n in 1..=30 {
            let mut t = REPORTED_RECOVERY_END_US + n * 10_000;
            if interruption == 2 && n >= 15 {
                t += 10_001;
            }
            let mut angle = 8.89 - (8.89 - 6.72) * n as f32 / 30.0;
            let mut rate = -7.2333;
            if n == 10 || n == 20 {
                if interruption == 0 {
                    rate = 0.1;
                }
                if interruption == 1 {
                    angle += 0.08;
                } // A small fresh uptick resets the existing policy.
            }
            r.update(
                input(t, angle, rate),
                true,
                true,
                true,
                true,
                None,
                100_000,
                20_000,
                150_000,
            );
            assert_ne!(r.telemetry.reason, Some(Reason::Rearmed));
        }
        assert_eq!(r.telemetry.phase, Phase::AwaitImprovement);
    }
}
#[test]
fn rearmed_same_direction_recovery_can_pass_fresh_request_latch_without_resetting_budget() {
    use restart_assist::recovery::{Phase, Reason};
    let mut a = initialized(true);
    let mut t = 600_000;
    for n in 0..60 {
        t += 10_000;
        step(&mut a, t, 6.0 + n as f32 * 0.05, 5.0);
    }
    assert_eq!(a.pulses_applied(), 2);
    for _ in 0..60 {
        t += 10_000;
        step(&mut a, t, 8.89, 1.0);
    }
    assert!(a.status(t).fresh_request_required);
    assert_eq!(a.status(t).pulses_remaining, 1);
    let mut saw_rearmed = false;
    for n in 1..=30 {
        t += 10_000;
        let o = step(&mut a, t, 8.89 - (8.89 - 6.72) * n as f32 / 30.0, -7.2333);
        saw_rearmed |= o.recovery.unwrap().reason == Some(Reason::Rearmed);
    }
    assert!(saw_rearmed);
    assert!(a.status(t).fresh_request_required); // Requalification does not clear/reset the latch.
    assert_ne!(a.status(t).recovery.unwrap().phase, Phase::AwaitImprovement);
    assert_eq!(a.status(t).pulses_remaining, 1);
    let mut applied = false;
    for n in 1..=60 {
        t += 10_000;
        let o = step(&mut a, t, 6.72 + (12.27 - 6.72) * n as f32 / 60.0, 9.25);
        if o.applied && !applied {
            assert_eq!(o.pulses_remaining, 0);
        }
        applied |= o.applied;
    }
    assert!(applied);
    assert_eq!(a.pulses_applied(), 3);
    assert!(a.status(t).window.unwrap().pulses_used <= 2);
    assert!(a.status(t).window.unwrap().on_us <= 300_000);
}
