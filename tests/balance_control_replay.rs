//! Host-only replay: MemoryMotors cannot access ESP peripherals or real motors.
#![allow(dead_code)]
#[path = "../src/balance.rs"]
pub(crate) mod balance;
#[path = "../src/bin/motor_response_runtime/bidirectional.rs"]
mod bidirectional;
#[path = "../src/bin/motor_response_runtime/control_profile.rs"]
mod control_profile;
#[path = "../src/bin/motor_response_runtime/ground.rs"]
mod ground;
#[path = "../src/motors/control.rs"]
pub(crate) mod motors;
#[path = "../src/bin/motor_response_runtime/output.rs"]
mod output;
#[path = "../src/bin/balance_motor_response_test/policy.rs"]
mod policy;
#[path = "../src/pwm_mapping.rs"]
pub(crate) mod pwm_mapping;
#[path = "../src/bin/motor_response_runtime/restart_assist.rs"]
mod restart_assist;
#[allow(dead_code)]
#[path = "../src/bin/motor_response_runtime/rolling_budget.rs"]
mod rolling_budget;
#[path = "../src/bin/motor_response_runtime/settings.rs"]
mod settings;
use motors::DualMotor;
use policy::{Response, ResponseState};
#[derive(Default)]
struct MemoryMotors {
    armed: bool,
    outputs: (i16, i16),
    reject: bool,
}
impl DualMotor for MemoryMotors {
    fn arm(&mut self) {
        self.armed = true;
    }
    fn command(&mut self, a: i16, b: i16) -> Result<(), motors::Disarmed> {
        if !self.armed || self.reject {
            return Err(motors::Disarmed);
        }
        self.outputs = (a, b);
        Ok(())
    }
    fn stop(&mut self) {
        self.outputs = (0, 0);
    }
    fn disarm(&mut self) {
        self.stop();
        self.armed = false;
    }
}
struct Replay {
    response: Response,
    interlock: bidirectional::Interlock,
    motors: MemoryMotors,
    metrics: control_profile::Behavior,
    timing: policy::LoopTiming,
}
impl Replay {
    fn new() -> Self {
        Self {
            response: control_profile::response(true).unwrap(),
            interlock: Default::default(),
            motors: Default::default(),
            metrics: Default::default(),
            timing: Default::default(),
        }
    }
    fn step(&mut self, t: u64, pitch: f32, rate: f32) -> Option<policy::Output> {
        self.step_with_correction(t, pitch, rate, true)
    }
    fn step_with_correction(
        &mut self,
        t: u64,
        pitch: f32,
        rate: f32,
        accel_used: bool,
    ) -> Option<policy::Output> {
        self.response.tick(t);
        let late = self.timing.begin(t, settings::MAX_LOOP_GAP_US);
        control_profile::enforce(&mut self.response, &mut self.motors, late);
        let o = ground::sample(
            &mut self.motors,
            &mut self.response,
            pitch,
            rate,
            accel_used,
            if self.timing.interval_us == 0 {
                0.01
            } else {
                self.timing.interval_us as f32 / 1_000_000.0
            },
            t,
        );
        control_profile::enforce(&mut self.response, &mut self.motors, false);
        if self.response.state() == ResponseState::Active && !self.response.finished {
            if !self.motors.armed {
                self.motors.arm();
            }
            let _ = self.interlock.drive(
                &mut self.motors,
                &mut self.response,
                o,
                settings::MOTOR_DUTY_CEILING,
                || t,
            );
        } else {
            self.motors.disarm();
            self.interlock.reset();
        }
        self.metrics
            .update(t, self.response.state(), Some(pitch), self.motors.outputs);
        o
    }
    fn ready(&mut self) {
        for t in (0..=500_000).step_by(10_000) {
            self.step(t, 0.0, 0.0);
        }
        self.step(520_000, 0.0, 0.0);
        assert_eq!(self.response.state(), ResponseState::Active);
        assert_eq!(self.motors.outputs, (0, 0));
    }
}
pub fn run() {
    for scenario in ["oscillation", "overshoot", "rapid"] {
        let mut r = Replay::new();
        r.ready();
        println!("scenario,time_us,pitch,rate,pd,mapped,requested,applied,interlock,state,reason");
        for t in (530_000..=1_520_000).step_by(10_000) {
            let elapsed = (t - 530_000) as f32 / 1_000_000.0;
            let (pitch, rate) = match scenario {
                "overshoot" => (3.0 + 18.0 * elapsed, 18.0),
                "rapid" => (if (t / 20_000) % 2 == 0 { 4.0 } else { -4.0 }, 0.0),
                _ => {
                    let phase = elapsed * 2.0 * core::f32::consts::PI * 2.0;
                    (
                        4.0 * libm::sinf(phase),
                        16.0 * core::f32::consts::PI * libm::cosf(phase),
                    )
                }
            };
            let o = r.step(t, pitch, rate);
            println!(
                "{scenario},{t},{pitch:.3},{rate:.3},{:?},{:?},{:?},{:?},{:?},{:?},{:?}",
                o.map(|o| o.limited_pd),
                o.map(|o| o.preview.mapped_pwm),
                o.map(|o| balance::wheel_commands(o.preview.final_pwm as i16)),
                r.motors.outputs,
                r.interlock.telemetry(t),
                r.response.state(),
                r.response.disarm_reason
            );
            if r.response.finished {
                break;
            }
        }
        r.motors.disarm();
        println!(
            "SUMMARY {scenario}: {:?}; reason={:?}; final={:?}; misses={}",
            r.metrics, r.response.disarm_reason, r.motors.outputs, r.timing.missed_deadlines
        );
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn repeated_corrections_have_acknowledged_zero_before_each_opposite_polarity() {
        let mut r = Replay::new();
        r.ready();
        let mut direction = 0;
        let mut zero_since = None;
        let mut changes = 0;
        for t in (530_000..1_520_000).step_by(10_000) {
            let pd = if ((t - 530_000) / 200_000) % 2 == 0 {
                4.0
            } else {
                -4.0
            };
            r.step(t, pd, 0.0);
            let applied = r.motors.outputs.1;
            if applied == 0 {
                zero_since.get_or_insert(t);
            } else {
                if direction != 0 && applied.signum() != direction {
                    assert!(t - zero_since.unwrap() >= 100_000);
                    changes += 1;
                    assert!(applied.abs() <= 50); // No second boost on any reversal.
                }
                direction = applied.signum();
                zero_since = None;
            }
            assert!(!r.response.finished);
            assert!(r.interlock.kicks <= 1);
        }
        assert!(changes >= 3);
        assert!(r.metrics.zero_crossings >= 3);
    }
    #[test]
    fn rapid_oscillation_remains_zero_after_initial_direction_until_stable() {
        let mut r = Replay::new();
        r.ready();
        r.step(530_000, 4.0, 0.0);
        for t in (540_000..900_000).step_by(10_000) {
            let pitch = if (t / 10_000) % 2 == 0 { -4.0 } else { 4.0 };
            r.step(t, pitch, 0.0);
            assert!(r.motors.outputs.1 >= 0);
            assert!(r.interlock.kicks <= 1);
        }
    }
    #[test]
    fn startup_kick_expires_cancels_and_cannot_restart() {
        for sign in [-1.0, 1.0] {
            let mut r = Replay::new();
            r.ready();
            for t in (530_000..680_000).step_by(10_000) {
                r.step(t, 3.0 * sign, 0.0);
                assert_eq!(r.motors.outputs.1, 70 * (sign as i16));
            }
            r.step(680_000, 3.0 * sign, 0.0);
            assert_eq!(r.motors.outputs.1, 31 * (sign as i16));
            for t in (690_000..1_010_000).step_by(10_000) {
                r.step(t, 0.0, 0.0);
                assert_eq!(r.motors.outputs, (0, 0));
            }
            r.step(1_010_000, 3.0 * sign, 0.0);
            assert_eq!(r.motors.outputs.1, 31 * (sign as i16));
            assert_eq!(r.interlock.kicks, 1);
        }
    }
    #[test]
    fn faults_including_before_readiness_are_terminal_and_stop_real_driver_interface() {
        for fault in [
            policy::SensorFault::I2cRead,
            policy::SensorFault::NoGravityReference,
            policy::SensorFault::CorrectionExpired,
            policy::SensorFault::InvalidGyro,
            policy::SensorFault::NonFiniteEstimate,
        ] {
            for active in [false, true] {
                let mut r = Replay::new();
                if active {
                    r.ready();
                    r.step(530_000, 4.0, 0.0);
                }
                r.response.sensor_fail(fault);
                control_profile::enforce(&mut r.response, &mut r.motors, false);
                assert!(r.response.finished);
                assert_eq!(r.motors.outputs, (0, 0));
                assert!(!r.motors.armed);
            }
        }
    }
    #[test]
    fn tilt_nan_deadline_and_driver_failures_never_rearm() {
        for case in 0..4 {
            let mut r = Replay::new();
            r.ready();
            r.step(530_000, 4.0, 0.0);
            match case {
                0 => {
                    r.step(540_000, 21.0, 0.0);
                }
                1 => {
                    r.step(540_000, f32::NAN, 0.0);
                }
                2 => {
                    r.step(560_001, 4.0, 0.0);
                }
                _ => {
                    r.motors.reject = true;
                    r.step(540_000, 4.0, 0.0);
                }
            }
            assert!(r.response.finished);
            assert_eq!(r.motors.outputs, (0, 0));
            r.step(570_000, 0.0, 0.0);
            assert_eq!(r.response.state(), ResponseState::Disarmed);
            assert_eq!(r.motors.outputs, (0, 0));
            assert!(!r.motors.armed);
        }
    }
    #[test]
    fn timeout_during_reversal_uses_current_active_window_and_is_permanent() {
        let mut r = Replay::new();
        r.ready();
        let deadline = 520_000 + control_profile::ACTIVE_US;
        for t in (530_000..deadline - 30_000).step_by(10_000) {
            r.step(t, 4.0, 0.0);
        }
        for t in (deadline - 30_000..deadline).step_by(10_000) {
            r.step(t, -4.0, 0.0);
            assert_eq!(r.motors.outputs, (0, 0));
        }
        r.step(deadline, -4.0, 0.0);
        assert_eq!(
            r.response.disarm_reason,
            Some(policy::DisarmReason::ActiveTimeout)
        );
        r.step(deadline + 10_000, 0.0, 0.0);
        assert!(!r.motors.armed);
        assert_eq!(r.motors.outputs, (0, 0));
    }
    #[test]
    fn metrics_use_elapsed_time_previous_output_and_fresh_angle_signs() {
        let mut b = control_profile::Behavior::default();
        b.update(0, ResponseState::Active, Some(3.0), (0, 0));
        b.update(10_000, ResponseState::Active, Some(0.0), (-31, 31));
        b.update(30_000, ResponseState::Active, Some(-4.0), (0, 0));
        b.update(60_000, ResponseState::Disarmed, None, (0, 0));
        b.update(90_000, ResponseState::Disarmed, Some(10.0), (0, 0));
        assert_eq!(b.elapsed_us, 60_000);
        assert_eq!(b.zero_us, 40_000);
        assert_eq!(b.longest_zero_us, 30_000);
        assert_eq!(b.max_abs_pitch, 4.0);
        assert_eq!(b.zero_crossings, 1);
    }
    #[test]
    fn dry_cannot_enter_active_or_energize_outputs() {
        let mut r = Replay::new();
        r.response = control_profile::response(false).unwrap();
        for t in (0..2_000_000).step_by(10_000) {
            r.step(t, 4.0, 0.0);
            assert_eq!(r.response.state(), ResponseState::DryRun);
            assert_eq!(r.motors.outputs, (0, 0));
        }
    }
    #[test]
    fn faults_during_reversal_wait_cancel_outputs_and_latch() {
        for cause in [
            policy::SensorFault::I2cRead,
            policy::SensorFault::CorrectionExpired,
        ] {
            let mut r = Replay::new();
            r.ready();
            r.step(530_000, 4.0, 0.0);
            r.step(540_000, -4.0, 0.0);
            assert_eq!(r.motors.outputs, (0, 0));
            assert_ne!(r.interlock.telemetry(540_000).pending_direction, 0);
            r.response.sensor_fail(cause);
            control_profile::enforce(&mut r.response, &mut r.motors, false);
            r.step(550_000, -4.0, 0.0);
            assert!(r.response.finished);
            assert!(!r.motors.armed);
            assert_eq!(r.motors.outputs, (0, 0));
        }
    }
    #[test]
    fn correction_and_freshness_deadlines_still_reject_at_existing_boundaries() {
        let mut r = Replay::new();
        r.ready();
        r.step(530_000, 4.0, 0.0);
        for t in (540_000..=580_000).step_by(10_000) {
            r.step_with_correction(t, 4.0, 0.0, false);
            assert!(!r.response.finished);
        }
        r.step_with_correction(580_001, 4.0, 0.0, false);
        assert_eq!(
            r.response.sensor_fault,
            Some(policy::SensorFault::CorrectionExpired)
        );
        assert_eq!(r.motors.outputs, (0, 0));
        let mut r = Replay::new();
        r.ready();
        r.step(530_000, 4.0, 0.0);
        r.response.tick(570_000);
        control_profile::enforce(&mut r.response, &mut r.motors, false);
        assert!(!r.response.finished);
        r.response.tick(570_001);
        control_profile::enforce(&mut r.response, &mut r.motors, false);
        assert!(r.response.finished);
        assert_eq!(r.motors.outputs, (0, 0));
    }
    #[test]
    fn pre_ready_deadline_fault_latches_and_both_tilt_signs_stop() {
        let mut r = Replay::new();
        r.step(0, 0.0, 0.0);
        r.step(30_001, 0.0, 0.0);
        assert!(r.response.finished);
        assert_eq!(r.motors.outputs, (0, 0));
        for sign in [-1.0, 1.0] {
            let mut r = Replay::new();
            r.ready();
            r.step(530_000, 4.0 * sign, 0.0);
            r.step(540_000, 21.0 * sign, 0.0);
            assert_eq!(
                r.response.disarm_reason,
                Some(policy::DisarmReason::Fault(balance::Fault::Tilt))
            );
            assert_eq!(r.motors.outputs, (0, 0));
            assert_eq!(r.metrics.max_abs_pitch, 21.0);
        }
    }
}

// Alternative policy ONLY in host replay. The production one-Kick gate is not
// bypassed or changed. MemoryMotors has no peripheral handles.
struct AssistReplay {
    policy: restart_assist::Assist,
    log: restart_assist::Recorder<64>,
    base: Replay,
    now: u64,
}
impl AssistReplay {
    fn new(config: restart_assist::Config) -> Self {
        let mut base = Replay::new();
        base.ready();
        let mut policy = restart_assist::Assist::new(config);
        let initial = policy.acknowledge(520_000, 0.0, false);
        let mut log = restart_assist::Recorder::new();
        log.record(initial);
        Self {
            policy,
            log,
            base,
            now: 520_000,
        }
    }
    fn step(
        &mut self,
        pitch: f32,
        rate: f32,
        injected: Option<restart_assist::Fault>,
    ) -> restart_assist::Trace {
        self.step_at(self.now + 10_000, pitch, rate, injected)
    }
    fn step_at(
        &mut self,
        now: u64,
        pitch: f32,
        rate: f32,
        injected: Option<restart_assist::Fault>,
    ) -> restart_assist::Trace {
        use restart_assist::{Fault, Input};
        let dt = now.saturating_sub(self.now) as f32 / 1_000_000.0;
        self.now = now;
        self.base.response.tick(now);
        let late = self.base.timing.begin(now, settings::MAX_LOOP_GAP_US);
        control_profile::enforce(&mut self.base.response, &mut self.base.motors, late);
        let o = ground::sample(
            &mut self.base.motors,
            &mut self.base.response,
            pitch,
            rate,
            true,
            dt,
            now,
        );
        control_profile::enforce(&mut self.base.response, &mut self.base.motors, false);
        let detected = if late {
            Some(Fault::Deadline)
        } else if self.base.response.finished {
            Some(match self.base.response.disarm_reason {
                Some(policy::DisarmReason::ActiveTimeout) => Fault::Timeout,
                Some(policy::DisarmReason::Fault(balance::Fault::Tilt)) => Fault::Tilt,
                _ => Fault::Sensor,
            })
        } else {
            None
        };
        let pd = o.map_or(0, |o| o.limited_pd);
        let t = self.policy.update(Input {
            now_us: now,
            sample_us: now,
            active: self.base.response.state() == ResponseState::Active,
            fault: injected.or(detected),
            pitch,
            rate,
            limited_pd: pd,
            mapped_pwm: o.map_or(0.0, |o| o.preview.mapped_pwm),
        });
        self.log.record(t);
        if t.state == restart_assist::State::Disarmed || self.base.response.finished {
            self.base.motors.disarm();
        } else {
            if !self.base.motors.armed {
                self.base.motors.arm();
            }
            let (a, b) = balance::wheel_commands(t.proposed_pwm as i16);
            if self.base.motors.command(a, b).is_err() {
                self.base.motors.disarm();
                let fault = self.policy.update(Input {
                    now_us: now,
                    sample_us: now,
                    active: true,
                    fault: Some(Fault::Driver),
                    pitch,
                    rate,
                    limited_pd: pd,
                    mapped_pwm: o.map_or(0.0, |o| o.preview.mapped_pwm),
                });
                self.log.record(fault);
            }
        }
        let actual = f32::from(self.base.motors.outputs.1);
        let assisted = self.policy.proposed_assist() && actual != 0.0;
        let ack = self.policy.acknowledge(now, actual, assisted);
        self.log.record(ack);
        ack
    }
    fn hold(&mut self, ticks: usize, pd: f32) -> restart_assist::Trace {
        let mut t = self.policy.status(self.now);
        for _ in 0..ticks {
            t = self.step(pd, 0.0, None);
        }
        t
    }
}
pub fn run_restart_assist() {
    let mut r = AssistReplay::new(restart_assist::Config {
        enabled: true,
        ..Default::default()
    });
    for pd in [0.0, 4.0, 0.0, -4.0, 0.0, 4.0] {
        r.hold(130, pd);
    }
    r.step(0.0, 0.0, Some(restart_assist::Fault::Disarmed));
    println!("OFFLINE ONLY: Restart Assist alternative policy; no motor peripherals");
    for t in r.log.recent() {
        println!("ASSIST {t:?}");
    }
    println!(
        "SUMMARY first_applied={:?} last_applied={:?} terminal={:?} overwritten={} actual_memory_outputs={:?}",
        r.log.first_applied,
        r.log.last_applied,
        r.log.terminal,
        r.log.overwritten,
        r.base.motors.outputs
    );
}
#[cfg(test)]
mod restart_tests {
    use super::*;
    use restart_assist::{Config, Fault, Reason, State};
    fn enabled() -> Config {
        Config {
            enabled: true,
            ..Default::default()
        }
    }
    fn start(r: &mut AssistReplay, sign: f32) {
        r.hold(12, 0.0);
        assert_eq!(r.step(3.0 * sign, 0.0, None).state, State::StartupCandidate);
        assert_eq!(r.base.motors.outputs, (0, 0));
        r.step(3.0 * sign, 0.0, None);
        let t = r.step(3.0 * sign, 0.0, None);
        assert!(t.requested && t.eligible && t.applied);
        assert_eq!(
            r.base.motors.outputs,
            (70 * (sign as i16), 70 * (sign as i16))
        );
    }
    #[test]
    fn explicitly_disabled_policy_and_initial_zero_interval_are_required() {
        let mut r = AssistReplay::new(Config {
            enabled: false,
            ..Default::default()
        });
        r.hold(12, 0.0);
        let t = r.step(3.0, 0.0, None);
        assert!(!t.applied);
        assert_eq!(t.suppressed, Some(Reason::Disabled));
        assert_eq!(r.base.motors.outputs, (31, 31));
        let mut r = AssistReplay::new(enabled());
        let t = r.step(3.0, 0.0, None);
        assert!(!t.applied);
        assert_eq!(t.suppressed, Some(Reason::ZeroInterval));
        r.hold(130, 3.0);
        assert!(r.log.first_applied.is_none()); // Held demand cannot become a late Kick.
    }
    #[test]
    fn repeated_start_stop_is_symmetric_and_pulse_count_bounded() {
        for sign in [-1.0, 1.0] {
            let mut r = AssistReplay::new(enabled());
            start(&mut r, sign);
            let t = r.step(0.0, 0.0, None);
            assert!(!t.applied);
            assert_eq!(r.base.motors.outputs, (0, 0));
            r.hold(110, 0.0);
            start(&mut r, sign);
            r.step(0.0, 0.0, None);
            r.hold(110, 0.0);
            let t = r.step(3.0 * sign, 0.0, None);
            assert_eq!(t.state, State::BudgetExhausted);
            assert!(!t.applied);
            assert_eq!(t.pulses_remaining, 0);
            assert_eq!(t.suppressed, Some(Reason::Budget));
        }
    }
    #[test]
    fn maximum_duration_has_scheduling_reserve_and_returns_to_ordinary_pwm() {
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        for _ in 0..8 {
            assert!(r.step(3.0, 0.0, None).applied);
        }
        let t = r.step(3.0, 0.0, None);
        assert!(!t.applied);
        assert_eq!(t.last_pulse_us, 90_000);
        assert!(t.last_pulse_us <= 150_000);
        assert_eq!(r.base.motors.outputs, (31, 31));
        assert_eq!(t.cooldown_remaining_us, 1_000_000);
    }
    #[test]
    fn cooldown_cannot_expire_into_a_delayed_assist_while_demand_is_held() {
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        r.step(0.0, 0.0, None);
        r.hold(12, 0.0);
        let t = r.step(3.0, 0.0, None);
        assert_eq!(t.suppressed, Some(Reason::Cooldown));
        assert!(!t.applied);
        let t = r.hold(150, 3.0);
        assert!(!t.applied);
        assert_eq!(t.pulses_remaining, 1);
    }
    #[test]
    fn reversal_during_assist_commands_zero_and_preserves_100ms_acknowledged_deadtime() {
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        let t = r.step(-3.0, 0.0, None);
        let zero_time = r.now;
        assert_eq!(t.suppressed, Some(Reason::DirectionConflict));
        assert_eq!(r.base.motors.outputs, (0, 0));
        for _ in 0..9 {
            r.step(-3.0, 0.0, None);
            assert_eq!(r.base.motors.outputs, (0, 0));
        }
        let t = r.step(-3.0, 0.0, None);
        assert_eq!(r.now - zero_time, 100_000);
        assert_eq!(r.base.motors.outputs, (-31, -31));
        assert!(!t.applied);
        assert_eq!(t.suppressed, Some(Reason::Cooldown));
    }
    #[test]
    fn rapid_candidate_direction_changes_never_trigger_pulses() {
        let mut r = AssistReplay::new(enabled());
        r.hold(12, 0.0);
        for i in 0..80 {
            let t = r.step(if i % 2 == 0 { 3.0 } else { -3.0 }, 0.0, None);
            assert!(!t.applied);
            assert_eq!(r.base.motors.outputs, (0, 0));
        }
        assert_eq!(r.policy.status(r.now).pulses_remaining, 2);
    }
    #[test]
    fn cumulative_time_budget_can_exhaust_before_pulse_count() {
        let mut r = AssistReplay::new(Config {
            total_us: 140_000,
            ..enabled()
        });
        start(&mut r, 1.0);
        r.hold(11, 3.0);
        let t = r.policy.status(r.now);
        assert_eq!(t.state, State::BudgetExhausted);
        assert!(!t.applied);
        assert_eq!(t.pulses_remaining, 1);
        assert_eq!(t.total_on_us, 80_000);
        assert_eq!(t.time_remaining_us, 60_000); // Reserved headroom cannot fund another pulse.
        r.hold(120, 0.0);
        assert!(!r.step(-3.0, 0.0, None).applied);
    }
    #[test]
    fn fault_stop_and_deadline_are_permanent_including_during_reversal() {
        for fault in [
            Fault::Sensor,
            Fault::Deadline,
            Fault::Driver,
            Fault::Disarmed,
        ] {
            let mut r = AssistReplay::new(enabled());
            start(&mut r, 1.0);
            r.step(-3.0, 0.0, None);
            r.step(-3.0, 0.0, Some(fault));
            assert_eq!(r.base.motors.outputs, (0, 0));
            assert!(!r.base.motors.armed);
            let t = r.hold(120, 0.0);
            assert_eq!(t.state, State::Disarmed);
            assert_eq!(t.suppressed, Some(Reason::Fault(fault)));
            assert!(!r.step(3.0, 0.0, None).applied);
        }
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        let t = r.step_at(r.now + 30_001, 3.0, 0.0, None);
        assert_eq!(t.state, State::Disarmed);
        assert_eq!(r.base.motors.outputs, (0, 0));
        assert_eq!(r.base.timing.missed_deadlines, 1);
    }
    #[test]
    fn tilt_cutoff_and_current_90_second_timeout_still_disarm() {
        for sign in [-1.0, 1.0] {
            let mut r = AssistReplay::new(enabled());
            start(&mut r, sign);
            let t = r.step(21.0 * sign, 0.0, None);
            assert_eq!(t.state, State::Disarmed);
            assert_eq!(r.base.motors.outputs, (0, 0));
        }
        let mut r = AssistReplay::new(enabled());
        while r.now < 520_000 + control_profile::ACTIVE_US {
            r.step(0.0, 0.0, None);
        }
        assert_eq!(
            r.base.response.disarm_reason,
            Some(policy::DisarmReason::ActiveTimeout)
        );
        assert_eq!(r.policy.status(r.now).state, State::Disarmed);
        assert_eq!(
            r.policy.status(r.now).suppressed,
            Some(Reason::Fault(Fault::Timeout))
        );
        assert_eq!(r.base.motors.outputs, (0, 0));
    }
    #[test]
    fn saturation_retains_applied_and_fault_context_and_never_counts_requests_as_application() {
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        r.hold(200, 0.0);
        assert!(r.log.overwritten > 0);
        assert!(r.log.first_applied.unwrap().applied);
        r.step(0.0, 0.0, Some(Fault::Sensor));
        let t = r.log.terminal.unwrap();
        assert!(!t.applied);
        assert_eq!(t.commanded_pwm, 0.0);
        assert_eq!(t.suppressed, Some(Reason::Fault(Fault::Sensor)));
    }
    fn input(now: u64, pd: i16) -> restart_assist::Input {
        restart_assist::Input {
            now_us: now,
            sample_us: now,
            active: true,
            fault: None,
            pitch: f32::from(pd),
            rate: 0.0,
            limited_pd: pd,
            mapped_pwm: pwm_mapping::map(settings::MAPPING, f32::from(pd)).unwrap(),
        }
    }
    #[test]
    fn no_assist_without_acknowledged_zero_or_fresh_candidate_sample() {
        let mut a = restart_assist::Assist::new(enabled());
        for t in (0..120_000).step_by(10_000) {
            a.update(input(t, 0));
        }
        assert!(!a.update(input(120_000, 3)).eligible); // Zero proposals were never acknowledged.
        let mut a = restart_assist::Assist::new(enabled());
        a.acknowledge(0, 0.0, false);
        for t in (0..=120_000).step_by(10_000) {
            let pd = if t < 120_000 { 0 } else { 3 };
            a.update(input(t, pd));
            a.acknowledge(t, 0.0, false);
        }
        for t in (130_000..=160_000).step_by(10_000) {
            let mut i = input(t, 3);
            i.sample_us = 120_000;
            let trace = a.update(i);
            assert!(!trace.eligible);
            assert_eq!(trace.suppressed, Some(Reason::NoFreshDemand));
            a.acknowledge(t, 0.0, false);
        }
        let mut i = input(160_001, 3);
        i.sample_us = 120_000;
        assert_eq!(a.update(i).suppressed, Some(Reason::Fault(Fault::Sensor)));
    }
    #[test]
    fn eligible_request_is_not_counted_as_applied_before_driver_ack() {
        let mut a = restart_assist::Assist::new(enabled());
        a.acknowledge(0, 0.0, false);
        for t in (0..=130_000).step_by(10_000) {
            a.update(input(t, if t < 120_000 { 0 } else { 3 }));
            a.acknowledge(t, 0.0, false);
        }
        let t = a.update(input(140_000, 3));
        assert!(t.requested && t.eligible);
        assert!(!t.applied);
        assert_eq!(t.pulses_remaining, 2);
        let t = a.acknowledge(141_000, 70.0, true);
        assert!(t.applied);
        assert_eq!(t.pulses_remaining, 1);
        assert_eq!(t.pulse_us, 0);
    }
    #[test]
    fn invalid_configuration_and_rejected_driver_commands_fail_closed() {
        for cfg in [
            Config {
                duty: 71.0,
                ..enabled()
            },
            Config {
                pulse_us: 150_001,
                ..enabled()
            },
            Config {
                cooldown_us: 999_999,
                ..enabled()
            },
            Config {
                max_pulses: 3,
                ..enabled()
            },
            Config {
                total_us: 300_001,
                ..enabled()
            },
            Config {
                zero_us: 99_999,
                ..enabled()
            },
        ] {
            let mut a = restart_assist::Assist::new(cfg);
            let t = a.update(input(0, 0));
            assert_eq!(t.state, State::Disarmed);
            assert_eq!(t.proposed_pwm, 0.0);
            assert_eq!(t.suppressed, Some(Reason::Fault(Fault::InvalidConfig)));
        }
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        r.base.motors.reject = true;
        let t = r.step(3.0, 0.0, None);
        assert_eq!(t.suppressed, Some(Reason::Fault(Fault::Driver)));
        assert_eq!(r.base.motors.outputs, (0, 0));
        assert!(!t.applied);
    }
    #[test]
    fn measured_jitter_cannot_use_reserved_pulse_headroom_to_request_more_assist() {
        let mut r = AssistReplay::new(enabled());
        start(&mut r, 1.0);
        let began = r.now;
        while r.policy.status(r.now).applied {
            r.step_at(r.now + 29_000, 3.0, 0.0, None);
            assert!(r.now - began <= 150_000);
        }
        let t = r.policy.status(r.now);
        assert_eq!(t.last_pulse_us, 116_000);
        assert!(t.total_on_us <= 150_000);
        assert_eq!(r.base.timing.missed_deadlines, 0);
    }
    #[test]
    fn pulse_reserve_covers_late_but_valid_driver_acknowledgements() {
        let mut a = restart_assist::Assist::new(enabled());
        a.acknowledge(0, 0.0, false);
        for t in (0..=130_000).step_by(10_000) {
            a.update(input(t, if t < 120_000 { 0 } else { 3 }));
            a.acknowledge(t, 0.0, false);
        }
        a.update(input(140_000, 3));
        a.acknowledge(140_000, 70.0, true);
        for (now, ack) in [(169_000, 198_000), (199_000, 228_000), (229_000, 258_000)] {
            let t = a.update(input(now, 3));
            assert!(t.eligible);
            a.acknowledge(ack, 70.0, true);
        }
        let t = a.update(input(259_000, 3));
        assert!(!t.eligible);
        assert_eq!(t.proposed_pwm as i16, 31);
        let t = a.acknowledge(288_000, 31.0, false);
        assert_eq!(t.last_pulse_us, 148_000);
        assert!(t.total_on_us <= 150_000);
        assert_ne!(t.state, State::Disarmed);
    }
}

#[cfg(test)]
mod arbiter_tests {
    use super::*;
    #[derive(Default)]
    struct Probe {
        motor: MemoryMotors,
        now: u64,
        writes: Vec<(u64, (i16, i16))>,
    }
    impl DualMotor for Probe {
        fn arm(&mut self) {
            self.motor.arm();
        }
        fn command(&mut self, a: i16, b: i16) -> Result<(), motors::Disarmed> {
            self.motor.command(a, b)?;
            self.writes.push((self.now, (a, b)));
            Ok(())
        }
        fn stop(&mut self) {
            self.motor.stop();
            self.writes.push((self.now, (0, 0)));
        }
        fn disarm(&mut self) {
            self.motor.disarm();
            self.writes.push((self.now, (0, 0)));
        }
    }
    struct Trial {
        r: Response,
        m: Probe,
        g: output::MotorOutput,
        t: u64,
        timing: policy::LoopTiming,
    }
    impl Trial {
        fn new(restart: bool) -> Self {
            let mut s = Self {
                r: control_profile::response_with_restart(true, restart).unwrap(),
                m: Default::default(),
                g: output::MotorOutput::new(restart),
                t: 0,
                timing: Default::default(),
            };
            s.g.confirm_zero(&mut s.m, || 0);
            for t in (0..=500_000).step_by(10_000) {
                s.step_at(t, 0.0, 0.0);
            }
            s.step_at(520_000, 0.0, 0.0);
            assert_eq!(s.r.state(), ResponseState::Active);
            s
        }
        fn step_at(&mut self, t: u64, pitch: f32, rate: f32) {
            let dt = if t == self.t {
                0.01
            } else {
                (t - self.t) as f32 / 1_000_000.0
            };
            self.t = t;
            self.m.now = t;
            self.r.tick(t);
            let late = self.timing.begin(t, settings::MAX_LOOP_GAP_US);
            control_profile::enforce(&mut self.r, &mut self.m, late);
            let o = ground::sample(&mut self.m, &mut self.r, pitch, rate, true, dt, t);
            control_profile::enforce(&mut self.r, &mut self.m, false);
            if self.r.state() == ResponseState::Active && !self.r.finished && !self.m.motor.armed {
                self.m.arm();
            }
            let _ = self.g.drive(
                &mut self.m,
                &mut self.r,
                o,
                output::Observation {
                    sample_us: t,
                    pitch,
                    rate,
                },
                || t,
            );
        }
        fn step(&mut self, pd: f32) {
            self.step_at(self.t + 10_000, pd, 0.0);
        }
        fn hold(&mut self, n: usize, pd: f32) {
            for _ in 0..n {
                self.step(pd);
            }
        }
        fn initial(&mut self, sign: f32) {
            self.hold(2, 3.0 * sign);
            assert_eq!(self.m.motor.outputs, (0, 0));
            self.step(3.0 * sign);
            assert_eq!(self.m.motor.outputs, (70 * sign as i16, 70 * sign as i16));
            assert_eq!(self.g.trace(self.t).unwrap().pulses_remaining, 1);
        }
    }
    #[test]
    fn integrated_initial_and_restart_share_two_pulses_then_third_is_denied() {
        for sign in [-1.0, 1.0] {
            let mut s = Trial::new(true);
            s.initial(sign);
            s.hold(10, 3.0 * sign);
            assert_eq!(s.m.motor.outputs.1, 31 * sign as i16);
            s.hold(110, 0.0);
            s.hold(3, 3.0 * sign);
            assert_eq!(s.m.motor.outputs.1, 70 * sign as i16);
            assert_eq!(s.g.trace(s.t).unwrap().pulses_remaining, 0);
            s.hold(10, 3.0 * sign);
            s.hold(110, 0.0);
            s.hold(15, 3.0 * sign);
            assert_eq!(s.m.motor.outputs.1, 31 * sign as i16);
            let t = s.g.trace(s.t).unwrap();
            assert_eq!(t.state, restart_assist::State::BudgetExhausted);
            assert!(t.total_on_us <= 300_000);
        }
    }
    #[test]
    fn reversal_never_applies_assist_until_shared_acknowledged_zero_deadtime() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        s.hold(10, 3.0);
        s.hold(110, 0.0);
        let zero = s.t + 10_000;
        s.step(-3.0);
        assert_eq!(s.m.motor.outputs, (0, 0));
        for _ in 0..9 {
            s.step(-3.0);
            assert_eq!(s.m.motor.outputs, (0, 0));
            assert_eq!(s.g.trace(s.t).unwrap().pulses_remaining, 1);
        }
        let t = s.g.trace(s.t).unwrap();
        assert!(t.proposed_pwm.abs() == 70.0);
        assert_eq!(t.approved_pwm, Some(0.0));
        assert_eq!(t.commanded_pwm, 0.0);
        s.step(-3.0);
        assert_eq!(s.t - zero, 100_000);
        assert_eq!(s.m.motor.outputs, (-70, -70));
        assert_eq!(s.g.trace(s.t).unwrap().pulses_remaining, 0);
        let before =
            s.m.writes
                .iter()
                .filter(|(t, _)| *t >= zero && *t < zero + 100_000);
        assert!(before.into_iter().all(|(_, c)| *c == (0, 0)));
    }
    #[test]
    fn faults_zero_actual_interface_and_no_rearm_even_if_external_caller_arms_driver() {
        for case in 0..5 {
            let mut s = Trial::new(true);
            s.initial(1.0);
            match case {
                0 => s.step_at(s.t + 10_000, 21.0, 0.0),
                1 => s.step_at(s.t + 30_001, 3.0, 0.0),
                2 => {
                    s.r.sensor_fail(policy::SensorFault::I2cRead);
                    s.step(3.0);
                }
                3 => {
                    s.m.motor.reject = true;
                    s.step(0.0);
                }
                _ => {
                    s.r.stop();
                    s.step(3.0);
                }
            }
            assert!(s.r.finished);
            assert!(!s.m.motor.armed);
            assert_eq!(s.m.motor.outputs, (0, 0));
            s.m.arm();
            s.step(0.0);
            s.step(3.0);
            assert_eq!(s.m.motor.outputs, (0, 0));
            assert!(!s.m.motor.armed);
        }
    }
    #[test]
    fn integrated_mapper_cannot_generate_a_separate_initial_kick() {
        let mut s = Trial::new(true);
        let t = s.t + 10_000;
        let o = s.r.sample(3.0, 0.0, true, 0.01, t).unwrap();
        assert!(!o.preview.kick_active);
        assert!((o.preview.final_pwm - 31.538462).abs() < 0.001);
        let mut forged = o;
        forged.preview.kick_active = true;
        forged.preview.final_pwm = 70.0;
        assert!(
            s.g.drive(
                &mut s.m,
                &mut s.r,
                Some(forged),
                output::Observation {
                    sample_us: t,
                    pitch: 3.0,
                    rate: 0.0
                },
                || t
            )
            .is_err()
        );
        assert!(s.r.finished);
        assert_eq!(s.m.motor.outputs, (0, 0));
    }
    #[test]
    fn ordinary_commands_stay_bounded_and_budget_not_reset_by_neutral() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        s.hold(10, 14.0);
        assert_eq!(s.m.motor.outputs.1, 48); // PD14 ordinary proportional mapping.
        s.hold(120, 0.0);
        s.hold(3, 3.0);
        s.hold(10, 3.0);
        s.hold(120, 0.0);
        for _ in 0..3 {
            s.hold(15, -14.0);
            s.hold(15, 14.0);
        }
        assert!(s.m.motor.outputs.1.abs() <= 50);
        assert_eq!(s.g.trace(s.t).unwrap().pulses_remaining, 0);
        s.step_at(s.t + 10_000, 14.0, 100.0); // PD saturates to +15, ordinary PWM to +50.
        assert_eq!(s.m.motor.outputs.1, 50);
        assert!(!s.g.trace(s.t).unwrap().applied);
    }
    #[test]
    fn legacy_default_keeps_original_one_kick_behavior() {
        let mut s = Trial::new(false);
        s.step(3.0);
        assert_eq!(s.m.motor.outputs, (70, 70));
        s.hold(110, 0.0);
        s.step(3.0);
        assert_eq!(s.m.motor.outputs, (31, 31));
        assert!(s.g.trace(s.t).is_none());
    }
    #[test]
    fn empty_polls_service_assist_expiration_without_renewing_sensor_freshness() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        s.hold(6, 3.0);
        assert_eq!(s.m.motor.outputs.1, 70);
        let last_sample = s.t;
        for offset in 1..=30_000 {
            if offset % 1000 != 0 {
                continue;
            }
            let now = last_sample + offset;
            s.m.now = now;
            s.g.service(&mut s.m, &mut s.r, || now).unwrap();
        }
        assert_eq!(s.m.motor.outputs.1, 31);
        assert!(!s.r.finished);
        s.g.service(&mut s.m, &mut s.r, || last_sample + 40_001)
            .unwrap();
        assert!(s.r.finished);
        assert_eq!(s.m.motor.outputs, (0, 0));
    }
    #[test]
    fn integrated_timeout_remains_ninety_seconds_and_is_terminal_during_deadtime() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        let deadline = 520_000 + control_profile::ACTIVE_US;
        assert_eq!(control_profile::ACTIVE_US, 90_000_000);
        while s.t < deadline - 30_000 {
            s.step(3.0);
        }
        while s.t < deadline - 10_000 {
            s.step(-3.0);
            assert_eq!(s.m.motor.outputs, (0, 0));
        }
        assert!(!s.r.finished);
        s.step(-3.0);
        assert_eq!(s.r.disarm_reason, Some(policy::DisarmReason::ActiveTimeout));
        assert_eq!(s.m.motor.outputs, (0, 0));
        assert!(!s.m.motor.armed);
        assert_eq!(
            s.g.trace(s.t).unwrap().state,
            restart_assist::State::Disarmed
        );
    }
    #[test]
    fn integrated_deadtime_starts_after_zero_driver_completion() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        s.hold(10, 3.0);
        s.hold(110, 0.0);
        let start = s.t + 10_000;
        let o = s.r.sample(-3.0, 0.0, true, 0.01, start).unwrap();
        let mut calls = 0;
        s.g.drive(
            &mut s.m,
            &mut s.r,
            Some(o),
            output::Observation {
                sample_us: start,
                pitch: -3.0,
                rate: 0.0,
            },
            || {
                calls += 1;
                if calls <= 2 { start } else { start + 1_200 }
            },
        )
        .unwrap();
        s.t = start;
        assert_eq!(s.m.motor.outputs, (0, 0));
        for _ in 0..10 {
            s.step(-3.0);
            assert_eq!(s.m.motor.outputs, (0, 0));
        }
        assert_eq!(s.t - start, 100_000);
        s.step_at(start + 101_200, -3.0, 0.0);
        assert_eq!(s.m.motor.outputs, (-70, -70));
        assert_eq!(s.g.trace(s.t).unwrap().pulses_remaining, 0);
    }
    #[test]
    fn integrated_fault_during_deadtime_and_rapid_reversals_cannot_rearm() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        for i in 0..60 {
            s.step(if i % 2 == 0 { -3.0 } else { 3.0 });
            assert!(s.m.motor.outputs.1 >= 0);
        }
        s.step(-3.0);
        assert_eq!(s.m.motor.outputs, (0, 0));
        s.r.sensor_fail(policy::SensorFault::InvalidGyro);
        s.step(-3.0);
        assert!(s.r.finished);
        assert_eq!(s.m.motor.outputs, (0, 0));
        assert_eq!(
            s.g.trace(s.t).unwrap().suppressed,
            Some(restart_assist::Reason::Fault(restart_assist::Fault::Sensor))
        );
        s.hold(120, 0.0);
        s.m.arm();
        s.step(3.0);
        assert!(!s.m.motor.armed);
    }
    #[test]
    fn approved_output_is_not_reported_as_commanded_when_driver_rejects_it() {
        let mut s = Trial::new(true);
        s.hold(2, 3.0);
        s.m.motor.reject = true;
        s.step(3.0);
        assert!(s.r.finished);
        assert_eq!(s.m.motor.outputs, (0, 0));
        let log = s.g.recorder().unwrap();
        assert!(log.first_applied.is_none());
        assert_eq!(log.terminal.unwrap().commanded_pwm, 0.0);
        assert_eq!(log.terminal.unwrap().approved_pwm, Some(0.0));
        assert!(log.recent().any(|t| t.proposed_pwm == 70.0
            && t.approved_pwm == Some(70.0)
            && t.commanded_pwm == 0.0
            && !t.applied));
    }
    #[test]
    fn integrated_correction_age_limit_cancels_assist_without_forgiving_fresh_gyro() {
        let mut s = Trial::new(true);
        s.initial(1.0);
        let corrected = s.t;
        for n in 1..=5 {
            let t = corrected + n * 10_000;
            s.m.now = t;
            let o = s.r.sample(3.0, 0.0, false, 0.01, t).unwrap();
            s.g.drive(
                &mut s.m,
                &mut s.r,
                Some(o),
                output::Observation {
                    sample_us: t,
                    pitch: 3.0,
                    rate: 0.0,
                },
                || t,
            )
            .unwrap();
            assert!(!s.r.finished);
        }
        s.g.service(&mut s.m, &mut s.r, || corrected + 50_001)
            .unwrap();
        assert_eq!(
            s.r.sensor_fault,
            Some(policy::SensorFault::CorrectionExpired)
        );
        assert_eq!(s.m.motor.outputs, (0, 0));
        assert!(!s.m.motor.armed);
        assert_eq!(
            s.g.trace(corrected + 50_001).unwrap().suppressed,
            Some(restart_assist::Reason::Fault(restart_assist::Fault::Sensor))
        );
    }
    #[test]
    fn overdue_approval_is_rejected_before_any_nonzero_driver_command() {
        let mut s = Trial::new(true);
        s.hold(2, 3.0);
        let start = s.t + 10_000;
        let o = s.r.sample(3.0, 0.0, true, 0.01, start).unwrap();
        let mut calls = 0;
        assert!(
            s.g.drive(
                &mut s.m,
                &mut s.r,
                Some(o),
                output::Observation {
                    sample_us: start,
                    pitch: 3.0,
                    rate: 0.0
                },
                || {
                    calls += 1;
                    if calls == 1 { start } else { start + 30_001 }
                }
            )
            .is_err()
        );
        assert!(s.r.finished);
        assert_eq!(s.m.motor.outputs, (0, 0));
        assert!(s.g.recorder().unwrap().first_applied.is_none());
        assert_eq!(
            s.r.disarm_reason,
            Some(policy::DisarmReason::Fault(balance::Fault::Timing))
        );
    }
}

/// Same authoritative firmware output path, with an in-memory driver only.
pub fn run_integrated_assist() {
    run_output_replay(output::AssistPolicy::Fixed);
}
pub fn run_rolling_assist() {
    run_output_replay(output::AssistPolicy::Rolling);
}
fn run_output_replay(selected: output::AssistPolicy) {
    let mut r = control_profile::response_with_restart(true, true).unwrap();
    let mut m = MemoryMotors::default();
    let mut g = output::MotorOutput::with_policy(selected);
    g.confirm_zero(&mut m, || 0);
    let mut timing = policy::LoopTiming::default();
    let rolling = selected == output::AssistPolicy::Rolling;
    let stop_at = if rolling {
        520_000 + control_profile::ACTIVE_US
    } else {
        5_000_000
    };
    let mut now = 0;
    for t in (0..=stop_at).step_by(10_000) {
        now = t;
        let pitch = if rolling {
            let cycle = t / 1_300_000;
            if t % 1_300_000 < 1_100_000 {
                0.0
            } else if cycle % 2 == 0 {
                3.0
            } else {
                -3.0
            }
        } else {
            match t {
                0..=699_999 => 0.0,
                700_000..=999_999 => 3.0,
                1_000_000..=2_499_999 => 0.0,
                2_500_000..=2_999_999 => -3.0,
                3_000_000..=4_499_999 => 0.0,
                _ => 3.0,
            }
        };
        let late = timing.begin(t, settings::MAX_LOOP_GAP_US);
        r.tick(t);
        control_profile::enforce(&mut r, &mut m, late);
        let o = ground::sample(&mut m, &mut r, pitch, 0.0, true, 0.01, t);
        if r.state() == ResponseState::Active && !m.armed {
            m.arm();
        }
        let _ = g.drive(
            &mut m,
            &mut r,
            o,
            output::Observation {
                sample_us: t,
                pitch,
                rate: 0.0,
            },
            || t,
        );
        if let Some(w) = g.trace(t).and_then(|t| t.window) {
            assert!(w.pulses_used <= 2);
            assert!(w.on_us <= 300_000);
        }
        if r.finished {
            break;
        }
    }
    r.stop();
    let stop = now + 10_000;
    let _ = g.drive(
        &mut m,
        &mut r,
        None,
        output::Observation {
            sample_us: stop,
            pitch: 0.0,
            rate: 0.0,
        },
        || stop,
    );
    println!("INTEGRATED OUTPUT REPLAY ONLY: real arbiter, memory driver, no hardware");
    if let Some(log) = g.recorder() {
        for t in log.recent() {
            println!("ARBITER {t:?}");
        }
        println!(
            "SUMMARY policy={selected:?} pulses={} first={:?} last={:?} terminal={:?} overwritten={} memory_outputs={:?} missed={}",
            g.kicks(stop),
            log.first_applied,
            log.last_applied,
            log.terminal,
            log.overwritten,
            m.outputs,
            timing.missed_deadlines
        );
    }
}

#[cfg(test)]
#[path = "rolling_assist.rs"]
mod rolling_tests;

#[cfg(test)]
#[path = "recovery_assist.rs"]
mod recovery_tests;

#[test]
fn ninety_second_active_window_starts_only_on_fresh_neutral_acceptance() {
    let mut r = control_profile::response(true).unwrap();
    for t in (0..=2_000_000).step_by(10_000) {
        r.sample(3.0, 0.0, true, 0.01, t);
        assert_ne!(r.state(), ResponseState::Active);
        assert!(!r.finished);
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    let start = 2_010_000;
    assert!(r.sample(0.0, 0.0, true, 0.01, start).is_none());
    assert_eq!(r.state(), ResponseState::Active);
    for t in ((start + 10_000)..(start + control_profile::ACTIVE_US)).step_by(10_000) {
        r.sample(0.0, 0.0, true, 0.01, t);
        assert_eq!(r.state(), ResponseState::Active);
        assert!(!r.finished);
    }
    r.sample(0.0, 0.0, true, 0.01, start + control_profile::ACTIVE_US);
    assert_eq!(r.disarm_reason, Some(policy::DisarmReason::ActiveTimeout));
    assert!(r.finished);
}

#[test]
fn ninety_second_telemetry_saturation_keeps_order_and_pinned_terminal_context() {
    #[path = "../src/bin/motor_response_runtime/capture.rs"]
    mod periodic_capture;
    let mut capture = periodic_capture::Capture::<u64, 64>::new();
    let mut log = restart_assist::Recorder::<64>::new();
    let a = restart_assist::Assist::new(restart_assist::Config::default());
    for t in (0..=control_profile::ACTIVE_US).step_by(1_000) {
        capture.record(t, t);
        let mut record = a.status(t);
        record.applied = t == 1_000 || t == 89_999_000;
        record.state = if t == control_profile::ACTIVE_US {
            restart_assist::State::Disarmed
        } else if record.applied {
            restart_assist::State::AssistActive
        } else {
            restart_assist::State::Idle
        };
        log.record(record);
    }
    assert_eq!(capture.recent().count(), 64);
    assert_eq!(capture.recent().next(), Some(83_700_000));
    assert_eq!(capture.recent().last(), Some(90_000_000));
    assert_eq!(capture.overwritten, 901 - 64);
    assert_eq!(log.recent().count(), 64);
    assert_eq!(log.overwritten, 90_001 - 64);
    assert_eq!(log.first_applied.unwrap().us, 1_000);
    assert_eq!(log.last_applied.unwrap().us, 89_999_000);
    assert_eq!(log.terminal.unwrap().us, 90_000_000);
    assert_eq!(
        log.recent().last().unwrap().state,
        restart_assist::State::Disarmed
    );
}
