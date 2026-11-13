#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[path = "../src/bin/balance_motor_response_test/policy.rs"]
mod policy;
#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;
use balance::{Fault, Settings, State, wheel_commands};
use policy::Response;
use pwm_mapping::{Config, KickConfig};
fn response(live: bool) -> Response {
    Response::new(Settings::default(), mapping(), kick(), 70, live, 1_000_000).unwrap()
}
fn mapping() -> Config {
    Config {
        enabled: true,
        deadband: 2.0,
        pd_output_limit: 15.0,
        minimum_duty: 30.0,
        maximum_duty: 50.0,
        slew_per_second: None,
    }
}
fn kick() -> KickConfig {
    KickConfig {
        enabled: true,
        kick_duty: 70.0,
        kick_duration_ms: 150,
        trigger_threshold: 4.0,
        rearm_delay_ms: 300,
        reversal_pause_ms: 100,
    }
}
fn arm(r: &mut Response) {
    for i in 1..=26 {
        assert!(r.sample(0.0, 0.0, true, 0.02, i * 20_000).is_none());
    }
    assert_eq!(r.control.state(), State::Active);
}
#[test]
fn calibration_pose_cannot_arm_and_live_requires_readiness() {
    let mut r = response(true);
    for i in 1..=40 {
        r.sample(34.0, 0.0, true, 0.02, i * 20_000);
    }
    assert_eq!(r.control.state(), State::Observe);
    assert!(!r.control.ready(800_000));
    assert!(Response::new(Settings::default(), mapping(), kick(), 50, true, 1_000_000).is_none());
    assert!(Response::new(Settings::default(), mapping(), kick(), 70, true, 3_000_000).is_none());
}
#[test]
fn arming_does_not_force_a_kick_requires_post_arm_neutral_then_request() {
    let mut r = response(true);
    arm(&mut r);
    assert!(r.sample(5.0, 0.0, true, 0.02, 540_000).is_none());
    assert!(r.sample(0.0, 0.0, true, 0.02, 560_000).is_none());
    let o = r.sample(5.0, 0.0, true, 0.02, 580_000).unwrap();
    assert!(o.preview.kick_active);
    assert_eq!(o.preview.final_pwm, 70.0);
    assert_eq!(wheel_commands(o.preview.final_pwm as i16), (-70, 70));
}
#[test]
fn zero_cancels_kick_and_reverse_preserves_electrical_signs() {
    let mut r = response(true);
    arm(&mut r);
    r.sample(0.0, 0.0, true, 0.02, 540_000);
    r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
    let zero = r.sample(0.0, 0.0, true, 0.02, 580_000).unwrap();
    assert!(!zero.preview.kick_active);
    assert_eq!(zero.preview.final_pwm, 0.0);
    let reverse = r.sample(-5.0, 0.0, true, 0.02, 600_000).unwrap();
    assert_eq!(reverse.preview.final_pwm, 0.0);
    for t in [620_000, 640_000, 660_000, 680_000] {
        r.sample(-5.0, 0.0, true, 0.02, t);
    }
    let reverse = r.sample(-5.0, 0.0, true, 0.02, 700_000).unwrap();
    assert!(reverse.preview.final_pwm < 0.0);
    let (a, b) = wheel_commands(reverse.preview.final_pwm as i16);
    assert!(a > 0 && b < 0);
}
#[test]
fn tilt_invalid_timing_and_stale_latch_finish_and_prevent_reactivation() {
    for fault in [Fault::Tilt, Fault::Invalid, Fault::Timing, Fault::Stale] {
        let mut r = response(true);
        arm(&mut r);
        r.sample(0.0, 0.0, true, 0.02, 540_000);
        r.sample(5.0, 0.0, true, 0.02, 560_000);
        match fault {
            Fault::Tilt => {
                r.sample(15.0, 0.0, true, 0.02, 580_000);
            }
            Fault::Invalid => {
                r.sample(f32::NAN, 0.0, true, 0.02, 580_000);
            }
            Fault::Timing => {
                r.sample(5.0, 0.0, true, 0.0, 580_000);
            }
            _ => r.tick(600_001),
        }
        assert!(r.finished);
        assert_eq!(r.control.state(), State::Fault(fault));
        assert!(r.sample(0.0, 0.0, true, 0.02, 620_000).is_none());
        assert!(r.sample(5.0, 0.0, true, 0.02, 640_000).is_none());
    }
}
#[test]
fn timed_trial_finishes_once_and_invalid_accel_disarms() {
    let mut r = response(true);
    arm(&mut r);
    for i in 27..=75 {
        r.sample(0.0, 0.0, true, 0.02, i * 20_000);
    }
    r.tick(1_520_000);
    assert!(r.finished);
    assert_eq!(r.control.state(), State::Fault(Fault::TrialExpired));
    assert!(r.sample(0.0, 0.0, true, 0.02, 1_540_000).is_none());
    let mut r = response(true);
    arm(&mut r);
    assert!(r.sample(0.0, 0.0, false, 0.02, 540_000).is_none());
    assert!(r.finished);
}
#[test]
fn preview_never_enters_active() {
    let mut r = response(false);
    for i in 1..=100 {
        r.sample(5.0, 0.0, true, 0.02, i * 20_000);
        assert_eq!(r.control.state(), State::DryRun);
    }
}

#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod motor_control;
mod motors {
    pub use super::motor_control::{Disarmed, DualMotor};
}
#[derive(Default)]
struct FakeMotor {
    armed: bool,
    output: (i16, i16),
}
impl motors::DualMotor for FakeMotor {
    fn arm(&mut self) {
        self.armed = true;
    }
    fn command(&mut self, a: i16, b: i16) -> Result<(), motors::Disarmed> {
        if !self.armed {
            return Err(motors::Disarmed);
        }
        self.output = (a, b);
        Ok(())
    }
    fn stop(&mut self) {
        self.output = (0, 0);
    }
    fn disarm(&mut self) {
        self.stop();
        self.armed = false;
    }
}
#[test]
fn physical_output_gate_disarms_on_fault_timeout_and_dry_even_with_old_kick() {
    use motors::DualMotor;
    for fault in [
        Fault::Tilt,
        Fault::Invalid,
        Fault::Timing,
        Fault::Stale,
        Fault::TrialExpired,
        Fault::Motor,
    ] {
        let mut r = response(true);
        arm(&mut r);
        r.sample(0.0, 0.0, true, 0.02, 540_000);
        let o = r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
        assert_eq!(o.limited_pd, 5);
        assert!(o.preview.mapped_pwm >= 30.0);
        let mut m = FakeMotor::default();
        m.arm();
        policy::drive(&mut m, &mut r, Some(o), 70, 560_000).unwrap();
        assert_eq!(m.output, (-70, 70));
        r.fail(fault);
        policy::drive(&mut m, &mut r, Some(o), 70, 560_000).unwrap(); // Stale kick cannot leak through.
        assert!(!m.armed);
        assert_eq!(m.output, (0, 0));
        assert!(r.sample(0.0, 0.0, true, 0.02, 580_000).is_none());
    }
    let mut r = response(false);
    let mut m = FakeMotor::default();
    m.arm();
    policy::drive(&mut m, &mut r, None, 70, 560_000).unwrap();
    assert!(!m.armed);
}

#[test]
fn stop_ends_one_shot_and_output_gate_rejects_over_ceiling() {
    use motors::DualMotor;
    let mut r = response(true);
    arm(&mut r);
    r.sample(0.0, 0.0, true, 0.02, 540_000);
    let o = r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
    let mut m = FakeMotor::default();
    m.arm();
    assert!(policy::drive(&mut m, &mut r, Some(o), 50, 560_000).is_err());
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
    r.control.request(balance::Request::Stop, 580_000).unwrap();
    r.tick(580_000);
    assert!(r.finished);
    assert!(r.sample(0.0, 0.0, true, 0.02, 600_000).is_none());
}

#[test]
fn normal_empty_polls_preserve_live_output_and_readiness_until_exact_freshness_limit() {
    use motors::DualMotor;
    let mut r = response(true);
    arm(&mut r);
    r.sample(0.0, 0.0, true, 0.02, 540_000);
    let o = r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
    let mut m = FakeMotor::default();
    m.arm();
    policy::drive(&mut m, &mut r, Some(o), 70, 560_000).unwrap();
    for now in (561_000..=600_000).step_by(1000) {
        r.tick(now); // Empty poll: do NOT call drive(None), reset filter, or resample.
        assert!(!r.finished);
        assert_eq!(r.control.state(), State::Active);
        assert!(r.control.sample_fresh(now));
        assert_eq!(m.output, (-70, 70));
    }
    policy::drive(&mut m, &mut r, Some(o), 70, 600_001).unwrap(); // Gate itself checks time.
    assert!(r.finished);
    assert_eq!(r.control.state(), State::Fault(Fault::Stale));
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
    let mut r = response(true);
    for i in 1..=26 {
        r.sample(0.0, 0.0, true, 0.02, i * 20_000);
    }
    assert!(r.control.ready(520_000));
    for now in 521_000..530_000 {
        r.tick(now);
        assert!(r.control.ready(now));
    }
}
#[test]
fn deadline_monitor_counts_once_per_loop_including_slow_empty_polls() {
    let mut timing = policy::LoopTiming::default();
    assert!(!timing.begin(0, 30_000));
    assert!(!timing.check_work(500, 30_000));
    assert!(!timing.begin(1500, 30_000));
    assert_eq!(timing.interval_us, 1500);
    assert!(!timing.check_work(31_500, 30_000)); // Exactly at limit permitted.
    assert!(timing.check_work(31_501, 30_000));
    assert_eq!(timing.missed_deadlines, 1);
    assert!(timing.check_work(32_000, 30_000));
    assert_eq!(timing.missed_deadlines, 1);
    assert!(timing.begin(32_001, 30_000));
    assert_eq!(timing.missed_deadlines, 2);
    let mut r = response(true);
    arm(&mut r);
    r.fail(Fault::Timing);
    assert!(r.finished);
    assert_eq!(r.control.state(), State::Fault(Fault::Timing));
}
#[test]
fn tilt_cutoff_blocks_physical_commands_but_dry_math_above_cutoff_stays_preview_only() {
    use motors::DualMotor;
    for pitch in [-34.0, -15.0, 15.0, 34.0] {
        let mut r = response(true);
        arm(&mut r);
        r.sample(0.0, 0.0, true, 0.02, 540_000);
        let old = r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
        let mut m = FakeMotor::default();
        m.arm();
        policy::drive(&mut m, &mut r, Some(old), 70, 560_000).unwrap();
        assert!(r.sample(pitch, 0.0, true, 0.02, 580_000).is_none());
        policy::drive(&mut m, &mut r, Some(old), 70, 580_000).unwrap();
        assert_eq!(r.control.state(), State::Fault(Fault::Tilt));
        assert!(!m.armed);
        assert_eq!(m.output, (0, 0));
        let mut dry = response(false);
        let preview = dry.sample(pitch, 0.0, true, 0.02, 20_000).unwrap();
        assert!(preview.preview.final_pwm.abs() > 0.0);
        m.arm();
        policy::drive(&mut m, &mut dry, Some(preview), 70, 20_000).unwrap();
        assert!(!m.armed);
        assert_eq!(m.output, (0, 0));
    }
}
#[test]
fn invalid_or_i2c_fault_disarms_even_while_last_sample_would_still_be_fresh() {
    use motors::DualMotor;
    let mut r = response(true);
    arm(&mut r);
    r.sample(0.0, 0.0, true, 0.02, 540_000);
    let old = r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
    let mut m = FakeMotor::default();
    m.arm();
    policy::drive(&mut m, &mut r, Some(old), 70, 560_000).unwrap();
    r.fail(Fault::Invalid); // Same entry point used for estimator and I2C failures.
    policy::drive(&mut m, &mut r, Some(old), 70, 561_000).unwrap();
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
    assert!(r.finished);
}
