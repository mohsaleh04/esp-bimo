#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[path = "../src/bin/balance_motor_response_test/policy.rs"]
mod policy;
#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;
use balance::{Fault, Settings, State, wheel_commands};
use policy::{DisarmReason, MAX_GYRO_ONLY_US, Response, ResponseState, SensorFault};
use pwm_mapping::{Config, KickConfig};
fn response(live: bool) -> Response {
    Response::new(
        Settings::default(),
        mapping(),
        kick(),
        70,
        live,
        1_000_000,
        10_000_000,
    )
    .unwrap()
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
    for i in 0..=25 {
        assert!(r.sample(0.0, 0.0, true, 0.02, i * 20_000).is_none());
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    assert!(r.sample(0.0, 0.0, true, 0.02, 520_000).is_none());
    assert_eq!(r.state(), ResponseState::Active);
    assert_eq!(r.state(), ResponseState::Active);
}
#[test]
fn calibration_pose_cannot_arm_and_live_requires_readiness() {
    let mut r = response(true);
    for i in 1..=40 {
        r.sample(34.0, 0.0, true, 0.02, i * 20_000);
    }
    assert_eq!(r.control.state(), State::Observe);
    assert!(!r.control.ready(800_000));
    assert!(
        Response::new(
            Settings::default(),
            mapping(),
            kick(),
            50,
            true,
            1_000_000,
            10_000_000
        )
        .is_none()
    );
    assert!(
        Response::new(
            Settings::default(),
            mapping(),
            kick(),
            70,
            true,
            policy::MAX_RESPONSE_WINDOW_US + 1,
            10_000_000
        )
        .is_none()
    );
}
#[test]
fn arming_does_not_force_a_kick_requires_post_ready_neutral_then_request() {
    let mut r = response(true);
    for i in 0..=25 {
        assert!(r.sample(3.88, 0.0, true, 0.02, i * 20_000).is_none());
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    assert_eq!(r.limited_pd, Some(3));
    assert!(!r.neutral_satisfied);
    assert!(r.sample(3.88, 0.0, true, 0.02, 520_000).is_none());
    assert_eq!(r.control.state(), State::Observe);
    assert!(r.sample(0.0, 0.0, true, 0.02, 540_000).is_none());
    assert_eq!(r.state(), ResponseState::Active);
    let o = r.sample(5.0, 0.0, true, 0.02, 560_000).unwrap();
    assert!(o.preview.kick_active);
    assert_eq!(o.preview.final_pwm, 70.0);
    assert_eq!(wheel_commands(o.preview.final_pwm as i16), (70, 70));
}
#[test]
fn zero_cancels_kick_and_reverse_commands_both_wheels_backwards() {
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
    assert!(a < 0 && b < 0);
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
        assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(fault)));
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
    assert_eq!(r.disarm_reason, Some(DisarmReason::ActiveTimeout));
    assert!(r.sample(0.0, 0.0, true, 0.02, 1_540_000).is_none());
    let mut r = response(true);
    for i in 0..=25 {
        r.sample(0.0, 0.0, true, 0.02, i * 20_000);
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    assert!(r.sample(0.0, 0.0, false, 0.02, 520_000).is_none());
    assert!(r.finished);
    assert_eq!(
        r.sensor_fault,
        Some(SensorFault::CorrectionRequiredForArming)
    );
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
        assert_eq!(m.output, (70, 70));
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
        assert_eq!(r.state(), ResponseState::Active);
        assert!(r.control.sample_fresh(now));
        assert_eq!(m.output, (70, 70));
    }
    policy::drive(&mut m, &mut r, Some(o), 70, 600_001).unwrap(); // Gate itself checks time.
    assert!(r.finished);
    assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Stale)));
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
    assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Timing)));
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
        assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Tilt)));
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

#[test]
fn ready_but_not_neutral_outlasts_active_window_then_hits_arming_timeout() {
    let mut r = Response::new(
        Settings::default(),
        mapping(),
        kick(),
        70,
        true,
        2_000_000,
        3_000_000,
    )
    .unwrap();
    r.begin(0);
    let mut m = FakeMotor::default();
    for i in 0..150 {
        let now = i * 20_000;
        assert!(r.sample(3.88, 0.0, true, 0.02, now).is_none());
        assert!(!r.finished);
        assert_eq!(r.limited_pd, Some(3));
        assert!(!r.neutral_satisfied);
        if now >= 500_000 {
            assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
            assert!(r.control.ready(now));
        }
        policy::drive(&mut m, &mut r, None, 70, now).unwrap();
        assert!(!m.armed);
        assert_eq!(m.output, (0, 0));
    }
    r.tick(3_000_000);
    assert!(r.finished);
    assert_eq!(r.state(), ResponseState::Disarmed);
    assert_eq!(r.disarm_reason, Some(DisarmReason::ArmingTimeout));
    assert!(r.sample(0.0, 0.0, true, 0.02, 3_020_000).is_none());
    let events = r.transitions();
    assert_eq!(events.len(), 2);
    let first = events[0].unwrap();
    assert_eq!(first.from, ResponseState::WaitingForReady);
    assert_eq!(first.to, ResponseState::ArmedWaitingForNeutral);
    assert_eq!(first.us, 500_000);
    assert!(first.reason.contains("neutral"));
    assert_eq!(events[1].unwrap().to, ResponseState::Disarmed);
}
#[test]
fn waiting_for_ready_is_also_bounded_even_without_any_samples() {
    let mut r = Response::new(
        Settings::default(),
        mapping(),
        kick(),
        70,
        true,
        2_000_000,
        1_000_000,
    )
    .unwrap();
    r.begin(0);
    r.tick(999_999);
    assert!(!r.finished);
    r.tick(1_000_000);
    assert_eq!(r.disarm_reason, Some(DisarmReason::ArmingTimeout));
    assert_eq!(r.state(), ResponseState::Disarmed);
}
#[test]
fn active_timer_starts_at_neutral_acceptance_not_initial_readiness() {
    let mut r = Response::new(
        Settings::default(),
        mapping(),
        kick(),
        70,
        true,
        5_000_000,
        10_000_000,
    )
    .unwrap();
    for i in 0..=150 {
        r.sample(3.88, 0.0, true, 0.02, i * 20_000);
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    assert!(!r.finished);
    // Readiness passed 2.5 seconds ago, but the ACTIVE timer has not started.
    assert!(r.sample(0.0, 0.0, true, 0.02, 3_020_000).is_none());
    assert_eq!(r.state(), ResponseState::Active);
    let event = r.transitions()[1].unwrap();
    assert_eq!(event.us, 3_020_000);
    assert!(event.reason.contains("starts NOW"));
    for i in 1..250 {
        r.sample(0.0, 0.0, true, 0.02, 3_020_000 + i * 20_000);
        assert!(!r.finished);
    }
    r.tick(8_020_000);
    assert!(r.finished);
    assert_eq!(r.disarm_reason, Some(DisarmReason::ActiveTimeout));
    assert_eq!(r.disarm_reason, Some(DisarmReason::ActiveTimeout));
    assert_eq!(r.transitions()[2].unwrap().to, ResponseState::Disarmed);
}
#[test]
fn neutral_does_not_bypass_current_readiness_or_waiting_safety_guards() {
    let mut r = response(true);
    for i in 0..=25 {
        r.sample(3.88, 0.0, true, 0.02, i * 20_000);
    }
    assert!(r.sample(0.0, 20.0, true, 0.02, 520_000).is_none()); // PD=1 is neutral, rate is unsafe.
    assert!(r.neutral_satisfied);
    assert!(!r.control.ready(520_000));
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    r.sample(15.0, 0.0, true, 0.02, 540_000);
    assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Tilt)));
    let mut r = response(true);
    for i in 0..=25 {
        r.sample(3.88, 0.0, true, 0.02, i * 20_000);
    }
    r.tick(540_001);
    assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Stale)));
}

#[test]
fn bounded_gyro_only_active_estimate_expires_at_absolute_limit_even_between_samples() {
    use motors::DualMotor;
    let mut r = response(true);
    arm(&mut r); // Last corrected gravity sample at 520ms.
    let mut m = FakeMotor::default();
    m.arm();
    let mut old = None;
    for t in [530_000, 540_000, 550_000, 560_000, 570_000] {
        let o = r.sample(5.0, 10.0, false, 0.01, t).unwrap();
        assert!(!r.finished);
        assert_eq!(r.accel_correction_age_us, Some(t - 520_000));
        policy::drive(&mut m, &mut r, Some(o), 70, t).unwrap();
        old = Some(o);
    }
    assert_eq!(MAX_GYRO_ONLY_US, 50_000);
    policy::drive(&mut m, &mut r, old, 70, 570_001).unwrap();
    assert!(r.finished);
    assert_eq!(r.sensor_fault, Some(SensorFault::CorrectionExpired));
    assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Invalid)));
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
    assert!(r.sample(0.0, 0.0, true, 0.01, 580_000).is_none()); // No recovery/rearming after fault.
}
#[test]
fn corrected_acceleration_resets_transient_age_but_never_rescues_overdue_data() {
    let mut r = response(true);
    arm(&mut r);
    r.sample(1.0, 1.0, false, 0.01, 530_000).unwrap();
    r.sample(1.0, 1.0, false, 0.01, 540_000).unwrap();
    r.sample(1.0, 1.0, true, 0.01, 550_000).unwrap();
    assert_eq!(r.accel_correction_age_us, Some(0));
    assert_eq!(r.sensor_fault, None);
    for t in [560_000, 570_000, 580_000, 590_000, 600_000] {
        r.sample(1.0, 1.0, false, 0.01, t).unwrap();
    }
    assert!(!r.finished);
    assert!(r.sample(1.0, 1.0, true, 0.01, 600_001).is_none()); // Correction arrives after the deadline.
    assert_eq!(r.sensor_fault, Some(SensorFault::CorrectionExpired));
}
#[test]
fn gyro_only_cannot_hide_stale_samples_tilt_nonfinite_estimate_or_deadline_fault() {
    for fault in [Fault::Stale, Fault::Tilt, Fault::Invalid, Fault::Timing] {
        let mut r = response(true);
        arm(&mut r);
        r.sample(1.0, 0.0, false, 0.01, 530_000).unwrap();
        match fault {
            Fault::Stale => r.tick(570_001),
            Fault::Tilt => {
                r.sample(15.0, 0.0, false, 0.01, 540_000);
            }
            Fault::Invalid => {
                r.sample(0.0, f32::NAN, false, 0.01, 540_000);
            }
            _ => r.fail(Fault::Timing),
        }
        assert!(r.finished);
        assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(fault)));
    }
}
#[test]
fn sensor_input_validation_and_all_sensor_failures_are_hard_stops() {
    use motors::DualMotor;
    use policy::input_fault;
    assert_eq!(input_fault([0.0, 0.0, 9.806], [0.0; 3], [0; 3]), None);
    assert_eq!(
        input_fault([f32::NAN, 0.0, 9.806], [0.0; 3], [0; 3]),
        Some(SensorFault::NonFiniteAccel)
    );
    for axis in 0..3 {
        let mut g = [0.0; 3];
        g[axis] = f32::INFINITY;
        assert_eq!(
            input_fault([0.0, 0.0, 9.806], g, [0; 3]),
            Some(SensorFault::InvalidGyro)
        );
        let mut raw = [0; 3];
        raw[axis] = i16::MAX;
        assert_eq!(
            input_fault([0.0, 0.0, 9.806], [0.0; 3], raw),
            Some(SensorFault::ClippedGyro)
        );
    }
    for cause in [
        SensorFault::I2cRead,
        SensorFault::InvalidGyro,
        SensorFault::ClippedGyro,
        SensorFault::NonFiniteAccel,
        SensorFault::FilterInvalid,
        SensorFault::FilterGyroPropagationExpired,
        SensorFault::NoGravityReference,
    ] {
        let mut r = response(true);
        arm(&mut r);
        let o = r.sample(5.0, 0.0, false, 0.01, 530_000).unwrap();
        let mut m = FakeMotor::default();
        m.arm();
        policy::drive(&mut m, &mut r, Some(o), 70, 530_000).unwrap();
        r.sensor_fail(cause);
        policy::drive(&mut m, &mut r, Some(o), 70, 531_000).unwrap();
        assert!(r.finished);
        assert_eq!(r.sensor_fault, Some(cause));
        assert!(!m.armed);
        assert_eq!(m.output, (0, 0));
        assert!(r.sample(0.0, 0.0, true, 0.01, 540_000).is_none());
    }
}

#[allow(dead_code)]
#[path = "../src/orientation.rs"]
mod orientation;
#[test]
fn real_filter_transient_is_accepted_but_bad_gyro_has_no_grace_period() {
    use orientation::{ComplementaryFilter, GRAVITY, PitchFrame, Vector};
    let up = Vector([0.0, 0.0, GRAVITY]);
    let frame =
        PitchFrame::from_poses(up, Vector([-0.342 * GRAVITY, 0.0, 0.94 * GRAVITY])).unwrap();
    let mut filter = ComplementaryFilter::new(0.5);
    filter.update(&frame, up, Vector::ZERO, 0.01).unwrap();
    let mut r = response(true);
    arm(&mut r);
    let accel = Vector([0.0, 0.0, 1.2 * GRAVITY]);
    for now in [530_000, 540_000, 550_000] {
        let est = filter
            .update(&frame, accel, Vector([0.0, 10.0, 0.0]), 0.01)
            .unwrap();
        assert!(!est.accel_used);
        assert!(est.pitch_deg.is_finite());
        assert!(
            r.sample(est.pitch_deg, est.rate_dps, est.accel_used, 0.01, now)
                .is_some()
        );
    }
    let bad_gyro = Vector([0.0, f32::NAN, 0.0]);
    assert_eq!(
        policy::input_fault(accel.0, bad_gyro.0, [0; 3]),
        Some(SensorFault::InvalidGyro)
    );
    assert!(filter.update(&frame, accel, bad_gyro, 0.01).is_none());
    r.sensor_fail(SensorFault::InvalidGyro);
    assert!(r.finished);
    assert_eq!(r.state(), ResponseState::Disarmed);
}

#[test]
fn five_second_response_survives_shared_two_second_cap_then_disarms_permanently() {
    use motors::DualMotor;
    let mut r = Response::new(
        Settings::default(),
        mapping(),
        kick(),
        70,
        true,
        5_000_000,
        10_000_000,
    )
    .unwrap();
    arm(&mut r); // ACTIVE begins at 520ms.
    let mut m = FakeMotor::default();
    m.arm();
    for i in 27..=275 {
        let now = i * 20_000;
        let o = r.sample(5.0, 0.0, true, 0.02, now);
        policy::drive(&mut m, &mut r, o, 70, now).unwrap();
        assert!(!r.finished);
        assert_eq!(r.state(), ResponseState::Active);
        if now > 2_520_000 {
            assert!(m.armed);
            assert!(m.output.0 > 0 && m.output.1 > 0);
        }
    }
    r.tick(5_519_999);
    assert!(!r.finished);
    policy::drive(&mut m, &mut r, None, 70, 5_520_000).unwrap();
    assert_eq!(r.disarm_reason, Some(DisarmReason::ActiveTimeout));
    assert!(r.finished);
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
    assert!(r.sample(0.0, 0.0, true, 0.02, 5_540_000).is_none());
    assert!(
        Response::new(
            Settings::default(),
            mapping(),
            kick(),
            70,
            true,
            policy::MAX_RESPONSE_WINDOW_US + 1,
            10_000_000
        )
        .is_none()
    );
}

#[test]
fn explicit_abort_is_terminal_without_changing_suspended_pre_ready_recovery() {
    let mut r = response(true);
    r.fail(Fault::Invalid);
    assert!(!r.finished);
    assert_eq!(r.state(), ResponseState::WaitingForReady);
    let mut r = response(true);
    r.abort(Fault::Tilt);
    assert!(r.finished);
    assert_eq!(r.state(), ResponseState::Disarmed);
    assert_eq!(r.disarm_reason, Some(DisarmReason::Fault(Fault::Tilt)));
    assert!(r.sample(0.0, 0.0, true, 0.01, 10_000).is_none());
}
