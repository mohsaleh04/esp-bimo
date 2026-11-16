#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[path = "../src/bin/motor_response_runtime/events.rs"]
mod events;
#[path = "../src/bin/motor_response_runtime/ground.rs"]
mod ground;
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod motors;
#[allow(dead_code)]
#[path = "../src/orientation.rs"]
mod orientation;
#[allow(dead_code)]
#[path = "../src/bin/balance_motor_response_test/policy.rs"]
mod policy;
#[allow(dead_code)]
#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;
#[allow(dead_code)]
#[path = "../src/bin/motor_response_runtime/restart_assist.rs"]
mod restart_assist;
#[allow(dead_code)]
#[path = "../src/bin/motor_response_runtime/rolling_budget.rs"]
mod rolling_budget;
#[allow(dead_code)]
#[path = "../src/bin/motor_response_runtime/settings.rs"]
mod settings;
use events::{EventLog, Snapshot};
use policy::{DisarmReason, ResponseState, SensorFault};
fn snapshot(us: u64) -> Snapshot {
    Snapshot {
        us,
        sample_us: Some(us),
        state: ResponseState::Active,
        pitch: Some(4.0),
        rate: Some(0.0),
        pd: Some(4),
        mapped: Some(33.0),
        kick: false,
        kick_ms: Some(0),
        requested: (-33, 33),
        reversal: None,
        commanded: (-33, 33),
        assist: None,
        last_energized: (us, -33, 33),
        accel_used: Some(true),
        rejection: None,
        correction_age_us: Some(0),
        sensor_fault: None,
        disarm: None,
        loop_us: 1500,
        work_us: 1200,
        missed: 0,
    }
}
#[test]
fn overflow_keeps_recent_order_plus_pinned_sensor_kick_and_terminal_context() {
    let mut log = EventLog::<3>::new();
    let mut s = snapshot(0);
    s.kick = true;
    s.commanded = (-70, 70);
    log.observe(s);
    s = snapshot(10);
    s.accel_used = Some(false);
    s.rejection = Some(orientation::AccelRejection::Magnitude {
        measured: 12.0,
        minimum: 8.33,
        maximum: 11.28,
    });
    s.correction_age_us = Some(10);
    log.observe(s);
    for us in 11..100 {
        let mut t = snapshot(us);
        t.commanded = (-(us as i16), us as i16);
        log.observe(t);
    }
    assert!(log.overwritten > 0);
    assert_eq!(log.kick_starts, 1);
    assert_eq!(log.kick_ends, 1);
    assert_eq!(log.last_kick_on.unwrap().us, 0);
    assert_eq!(log.last_kick_off.unwrap().us, 10);
    assert_eq!(log.rejection_onset.unwrap().us, 10);
    assert_eq!(log.last_rejection.unwrap().rejection, s.rejection);
    let mut fault = snapshot(100);
    fault.state = ResponseState::Disarmed;
    fault.commanded = (0, 0);
    fault.last_energized = (99, -99, 99);
    fault.sensor_fault = Some(SensorFault::CorrectionExpired);
    fault.disarm = Some(DisarmReason::Fault(balance::Fault::Invalid));
    fault.correction_age_us = Some(50_600);
    log.observe(fault);
    log.observe(snapshot(101)); // Cannot overwrite terminal or resume recording after final stop.
    let final_record = log.terminal.unwrap();
    assert_eq!(final_record.us, 100);
    assert_eq!(final_record.correction_age_us, Some(50_600));
    assert_eq!(final_record.last_energized, (99, -99, 99));
    assert_eq!(final_record.commanded, (0, 0));
    assert_eq!(log.before_terminal.unwrap().us, 99);
    let times: Vec<_> = log
        .recent()
        .map(|e| {
            assert_ne!(e.flags, 0);
            e.sample.us
        })
        .collect();
    assert_eq!(times, vec![98, 99, 100]);
}
#[test]
fn normal_empty_polls_do_not_fabricate_sensor_events_or_overwrite_measurement_time() {
    let mut log = EventLog::<3>::new();
    let mut s = snapshot(10);
    s.accel_used = Some(false);
    s.rejection = Some(orientation::AccelRejection::Sideways {
        measured: 4.0,
        maximum: 3.43,
    });
    log.observe(s);
    for us in 11..20 {
        let mut p = s;
        p.us = us;
        log.observe(p);
    }
    assert_eq!(log.recent().count(), 1);
    assert_eq!(log.last_rejection.unwrap().us, 10);
}
#[test]
fn every_ground_opt_in_and_configured_window_are_mandatory() {
    for bits in 0..32 {
        assert_eq!(
            ground::live_allowed(
                bits & 1 != 0,
                bits & 2 != 0,
                bits & 4 != 0,
                bits & 8 != 0,
                bits & 16 != 0
            ),
            bits == 31
        );
    }
    assert!(!ground::window_valid(0));
    assert!(ground::window_valid(1_000_000));
    assert!(ground::window_valid(30_000_000));
    assert!(!ground::window_valid(30_000_001));
}
fn preview(kick: bool, final_pwm: f32) -> pwm_mapping::Preview {
    pwm_mapping::Preview {
        mapped_pwm: 33.0,
        final_pwm,
        kick_active: kick,
        kick_elapsed_ms: 0,
    }
}
#[test]
fn ground_allows_one_kick_then_hold_and_zero_but_stops_next_kick_or_reversal() {
    let mut g = ground::GroundGuard::default();
    assert_eq!(g.check(4, preview(true, 70.0), 2.0), None);
    assert_eq!(g.check(4, preview(true, 70.0), 2.0), None);
    assert_eq!(g.check(4, preview(false, 33.0), 2.0), None);
    assert_eq!(g.check(0, preview(false, 0.0), 2.0), None);
    assert_eq!(
        g.check(4, preview(true, 70.0), 2.0),
        Some(ground::GroundStop::AdditionalKick)
    );
    let mut g = ground::GroundGuard::default();
    g.check(4, preview(true, 70.0), 2.0);
    assert_eq!(g.check(-1, preview(false, 0.0), 2.0), None); // Sub-deadband noise.
    assert_eq!(
        g.check(-4, preview(false, 0.0), 2.0),
        Some(ground::GroundStop::ReversalRequested)
    );
}
#[derive(Default)]
struct Fake {
    armed: bool,
    outputs: (i16, i16),
    reject: bool,
    command_calls: usize,
    disarm_calls: usize,
}
impl motors::DualMotor for Fake {
    fn arm(&mut self) {
        self.armed = true;
    }
    fn command(&mut self, a: i16, b: i16) -> Result<(), motors::Disarmed> {
        self.command_calls += 1;
        if !self.armed || self.reject {
            return Err(motors::Disarmed); // Deliberately retain old output on error to test caller shutdown.
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
        self.disarm_calls += 1;
    }
}
#[test]
fn ground_guard_veto_and_one_second_timeout_reach_physical_disarm_gate() {
    use motors::DualMotor;
    for veto in [false, true] {
        let mapping = pwm_mapping::Config {
            enabled: true,
            deadband: 2.0,
            pd_output_limit: 15.0,
            minimum_duty: 30.0,
            maximum_duty: 50.0,
            slew_per_second: None,
        };
        let kick = pwm_mapping::KickConfig {
            enabled: true,
            kick_duty: 70.0,
            kick_duration_ms: 150,
            trigger_threshold: 4.0,
            rearm_delay_ms: 300,
            reversal_pause_ms: 100,
        };
        let mut r = policy::Response::new(
            balance::Settings::default(),
            mapping,
            kick,
            70,
            true,
            1_000_000,
            10_000_000,
        )
        .unwrap();
        for i in 0..=26 {
            r.sample(0.0, 0.0, true, 0.02, i * 20_000);
        }
        let mut m = Fake::default();
        m.arm();
        let mut guard = ground::GroundGuard::default();
        let o = r.sample(4.0, 0.0, true, 0.02, 540_000).unwrap();
        assert_eq!(guard.check(o.limited_pd, o.preview, 2.0), None);
        policy::drive(&mut m, &mut r, Some(o), 70, 540_000).unwrap();
        assert_eq!(m.outputs, (-70, 70));
        if veto {
            assert!(guard.check(-4, preview(false, 0.0), 2.0).is_some());
            r.fail(balance::Fault::Motor);
        } else {
            for i in 28..=75 {
                r.sample(0.0, 0.0, true, 0.02, i * 20_000);
            }
            r.tick(1_520_000);
        }
        policy::drive(&mut m, &mut r, Some(o), 70, 1_520_000).unwrap();
        assert!(r.finished);
        assert!(!m.armed);
        assert_eq!(m.outputs, (0, 0));
        assert!(r.sample(0.0, 0.0, true, 0.02, 1_540_000).is_none());
    }
}

#[test]
fn event_counters_include_reversals_through_zero_and_actual_command_oscillations() {
    let mut log = EventLog::<2>::new();
    let mut s = snapshot(1);
    log.observe(s);
    s.us = 2;
    s.requested = (0, 0);
    s.commanded = (0, 0);
    log.observe(s);
    s.us = 3;
    s.requested = (33, -33);
    s.commanded = (33, -33);
    log.observe(s);
    assert_eq!(log.reversals, 1);
    assert_eq!(log.zero_transitions, 2);
    assert_eq!(log.command_changes, 2);
}

#[path = "../src/bin/motor_response_runtime/ground_timing.rs"]
mod ground_timing;
fn ground_response(live: bool) -> policy::Response {
    profile_response(live, false)
}
fn profile_response(live: bool, bidirectional: bool) -> policy::Response {
    let kick = pwm_mapping::KickConfig {
        enabled: true,
        kick_duty: 70.0,
        kick_duration_ms: 150,
        trigger_threshold: 4.0,
        rearm_delay_ms: 300,
        reversal_pause_ms: 100,
    };
    let kick = if bidirectional {
        bidirectional::kick_config(kick)
    } else {
        kick
    };
    policy::Response::new(
        balance::Settings::default(),
        pwm_mapping::Config {
            enabled: true,
            deadband: 2.0,
            pd_output_limit: 15.0,
            minimum_duty: 30.0,
            maximum_duty: 50.0,
            slew_per_second: None,
        },
        kick,
        70,
        live,
        1_000_000,
        10_000_000,
    )
    .unwrap()
}
fn active_ground(sign: i16) -> (policy::Response, Fake, ground::GroundGuard, policy::Output) {
    use motors::DualMotor;
    let mut r = ground_response(true);
    for i in 0..=26 {
        r.sample(0.0, 0.0, true, 0.02, i * 20_000);
    }
    assert_eq!(r.state(), ResponseState::Active); // Neutral accepted at 520ms.
    let mut m = Fake::default();
    m.arm();
    let mut g = ground::GroundGuard::default();
    let o = r
        .sample(f32::from(sign) * 4.0, 0.0, true, 0.01, 530_000)
        .unwrap();
    ground::drive(&mut m, &mut r, &mut g, Some(o), 70, 2.0, 530_000).unwrap();
    assert_eq!(m.outputs, (-70 * sign, 70 * sign));
    (r, m, g, o)
}
fn assert_locked(
    r: &mut policy::Response,
    m: &mut Fake,
    g: &mut ground::GroundGuard,
    old: policy::Output,
    now: u64,
) {
    use motors::DualMotor;
    ground::drive(m, r, g, Some(old), 70, 2.0, now).unwrap();
    assert!(r.finished);
    assert_eq!(r.state(), ResponseState::Disarmed);
    assert!(!m.armed);
    assert_eq!(m.outputs, (0, 0));
    assert!(m.disarm_calls > 0);
    let calls = m.command_calls;
    assert!(r.sample(0.0, 0.0, true, 0.01, now + 10_000).is_none());
    m.arm(); // Even an erroneous external re-arm cannot bypass the final policy gate.
    ground::drive(m, r, g, Some(old), 70, 2.0, now + 10_000).unwrap();
    assert!(!m.armed);
    assert_eq!(m.outputs, (0, 0));
    assert_eq!(m.command_calls, calls);
}
#[test]
fn ground_every_sensor_failure_permanently_disarms_the_driver() {
    for cause in [
        SensorFault::I2cRead,
        SensorFault::NoGravityReference,
        SensorFault::CorrectionExpired,
        SensorFault::NonFiniteAccel,
        SensorFault::InvalidGyro,
        SensorFault::ClippedGyro,
        SensorFault::NonFiniteEstimate,
        SensorFault::FilterInvalid,
        SensorFault::FilterGyroPropagationExpired,
        SensorFault::SampleTiming,
        SensorFault::FreshSampleTimeout,
    ] {
        let (mut r, mut m, mut g, old) = active_ground(1);
        r.sensor_fail(cause);
        assert_locked(&mut r, &mut m, &mut g, old, 540_000);
        assert_eq!(r.sensor_fault, Some(cause));
    }
}
#[test]
fn ground_nonfinite_pitch_rate_and_bad_dt_do_not_leave_old_kick_energized() {
    for (pitch, rate, dt) in [
        (f32::NAN, 0.0, 0.01),
        (f32::INFINITY, 0.0, 0.01),
        (0.0, f32::NAN, 0.01),
        (0.0, f32::NEG_INFINITY, 0.01),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, f32::NAN),
        (0.0, 0.0, 0.040001),
    ] {
        let (mut r, mut m, mut g, old) = active_ground(1);
        assert!(r.sample(pitch, rate, true, dt, 540_000).is_none());
        assert_locked(&mut r, &mut m, &mut g, old, 540_000);
    }
}
#[test]
fn ground_tilt_boundary_is_symmetric_and_actual_outputs_are_zero_at_cutoff() {
    for sign in [-1, 1] {
        let (mut r, mut m, mut g, _) = active_ground(sign);
        let o = r
            .sample(f32::from(sign) * 14.999, 0.0, true, 0.01, 540_000)
            .unwrap();
        ground::drive(&mut m, &mut r, &mut g, Some(o), 70, 2.0, 540_000).unwrap();
        assert!(!r.finished);
        assert_ne!(m.outputs, (0, 0));
        for tilt in [15.0, 15.001] {
            let (mut r, mut m, mut g, old) = active_ground(sign);
            assert!(
                r.sample(f32::from(sign) * tilt, 0.0, true, 0.01, 540_000)
                    .is_none()
            );
            assert_eq!(
                r.disarm_reason,
                Some(DisarmReason::Fault(balance::Fault::Tilt))
            );
            assert_locked(&mut r, &mut m, &mut g, old, 540_000);
        }
    }
}
#[test]
fn ground_correction_age_and_sample_freshness_have_independent_exact_boundaries() {
    let (mut r, mut m, mut g, old) = active_ground(1); // Last correction 530ms.
    for t in [540_000, 550_000, 560_000, 570_000, 580_000] {
        let o = r.sample(4.0, 0.0, false, 0.01, t).unwrap();
        ground::drive(&mut m, &mut r, &mut g, Some(o), 70, 2.0, t).unwrap();
        assert!(!r.finished);
    }
    assert_eq!(r.accel_correction_age_us, Some(50_000));
    ground::drive(&mut m, &mut r, &mut g, Some(old), 70, 2.0, 580_001).unwrap();
    assert_eq!(r.sensor_fault, Some(SensorFault::CorrectionExpired));
    assert_locked(&mut r, &mut m, &mut g, old, 580_001);
    let (mut r, mut m, mut g, old) = active_ground(1);
    ground::drive(&mut m, &mut r, &mut g, Some(old), 70, 2.0, 570_000).unwrap();
    assert!(!r.finished);
    ground::drive(&mut m, &mut r, &mut g, Some(old), 70, 2.0, 570_001).unwrap();
    assert_eq!(
        r.disarm_reason,
        Some(DisarmReason::Fault(balance::Fault::Stale))
    );
    assert_locked(&mut r, &mut m, &mut g, old, 570_001);
}
#[test]
fn ground_deadline_boundary_and_clock_rollback_physically_stop_and_lock() {
    let (mut r, mut m, mut g, old) = active_ground(1);
    let mut t = policy::LoopTiming::default();
    t.begin(530_000, 30_000);
    assert!(!ground::deadline(
        &mut m,
        &mut r,
        t.check_work(560_000, 30_000)
    ));
    assert!(!r.finished);
    assert_ne!(m.outputs, (0, 0));
    assert!(ground::deadline(
        &mut m,
        &mut r,
        t.check_work(560_001, 30_000)
    ));
    assert_eq!(m.outputs, (0, 0));
    assert!(!m.armed); // No second drive call needed for deadline stop.
    assert_eq!(
        r.disarm_reason,
        Some(DisarmReason::Fault(balance::Fault::Timing))
    );
    assert_locked(&mut r, &mut m, &mut g, old, 560_001);
    let (mut r, mut m, mut g, old) = active_ground(1);
    let mut t = policy::LoopTiming::default();
    t.begin(530_000, 30_000);
    assert!(ground::deadline(&mut m, &mut r, t.begin(560_222, 30_000)));
    assert_locked(&mut r, &mut m, &mut g, old, 560_222); // Reported 30.222ms interval, still hard-fails LIVE.
    let (mut r, mut m, mut g, old) = active_ground(1);
    ground::drive(&mut m, &mut r, &mut g, Some(old), 70, 2.0, 529_999).unwrap();
    assert_locked(&mut r, &mut m, &mut g, old, 540_000);
}
#[test]
fn ground_one_second_timeout_starts_at_neutral_and_cannot_rearm() {
    let (mut r, mut m, mut g, old) = active_ground(1);
    let mut last = old;
    for t in (540_000..=1_510_000).step_by(10_000) {
        let o = r.sample(4.0, 0.0, true, 0.01, t).unwrap();
        ground::drive(&mut m, &mut r, &mut g, Some(o), 70, 2.0, t).unwrap();
        last = o;
    }
    ground::drive(&mut m, &mut r, &mut g, Some(last), 70, 2.0, 1_519_999).unwrap();
    assert!(!r.finished);
    assert_ne!(m.outputs, (0, 0));
    ground::drive(&mut m, &mut r, &mut g, Some(last), 70, 2.0, 1_520_000).unwrap();
    assert_eq!(r.disarm_reason, Some(DisarmReason::ActiveTimeout));
    assert_locked(&mut r, &mut m, &mut g, old, 1_520_000);
}
#[test]
fn ground_second_kick_and_reversal_are_latched_without_caller_fault_handling() {
    for reverse in [false, true] {
        let (mut r, mut m, mut g, old) = active_ground(1);
        let zero = r.sample(0.0, 0.0, true, 0.01, 540_000).unwrap();
        ground::drive(&mut m, &mut r, &mut g, Some(zero), 70, 2.0, 540_000).unwrap();
        for t in (550_000..=850_000).step_by(10_000) {
            let zero = r.sample(0.0, 0.0, true, 0.01, t).unwrap();
            ground::drive(&mut m, &mut r, &mut g, Some(zero), 70, 2.0, t).unwrap();
        }
        let pd = if reverse { -4.0 } else { 4.0 };
        let o = r.sample(pd, 0.0, true, 0.01, 860_000).unwrap();
        assert!(ground::drive(&mut m, &mut r, &mut g, Some(o), 70, 2.0, 860_000).is_err());
        assert_eq!(
            g.stop_reason(),
            Some(if reverse {
                ground::GroundStop::ReversalRequested
            } else {
                ground::GroundStop::AdditionalKick
            })
        );
        assert_locked(&mut r, &mut m, &mut g, old, 860_000);
        assert!(g.check(4, preview(false, 30.0), 2.0).is_some()); // Guard itself retains the stop too.
    }
}
#[test]
fn ground_gate_latches_driver_errors_and_out_of_range_or_nonfinite_outputs() {
    for case in 0..3 {
        let (mut r, mut m, mut g, old) = active_ground(1);
        let mut bad = old;
        if case == 0 {
            m.reject = true;
        } else if case == 1 {
            bad.preview.final_pwm = 71.0;
        } else {
            bad.preview.final_pwm = f32::NAN;
        }
        assert!(ground::drive(&mut m, &mut r, &mut g, Some(bad), 70, 2.0, 540_000).is_err());
        assert_locked(&mut r, &mut m, &mut g, old, 540_000);
    }
}
#[test]
fn ground_dry_and_pre_active_paths_cannot_apply_a_calculated_command() {
    use motors::DualMotor;
    let (_, _, _, old) = active_ground(1);
    let mut r = ground_response(false);
    let mut m = Fake::default();
    let mut g = ground::GroundGuard::default();
    for t in [0, 10_000, 20_000] {
        let o = r.sample(30.0, 0.0, true, 0.01, t).unwrap();
        m.arm();
        ground::drive(&mut m, &mut r, &mut g, Some(o), 70, 2.0, t).unwrap();
        assert_eq!(r.state(), ResponseState::DryRun);
        assert!(!m.armed);
        assert_eq!(m.outputs, (0, 0));
    }
    let mut r = ground_response(true);
    r.begin(0);
    r.sensor_fail(SensorFault::NoGravityReference);
    for i in 0..=25 {
        let now = i * 20_000;
        r.sample(0.0, 0.0, true, 0.02, now);
        m.arm();
        ground::drive(&mut m, &mut r, &mut g, Some(old), 70, 2.0, now).unwrap();
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    assert!(r.sample(0.0, 20.0, true, 0.01, 510_000).is_none()); // Neutral PD, unsafe rate; no activation.
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    ground::drive(&mut m, &mut r, &mut g, Some(old), 70, 2.0, 510_000).unwrap();
    assert!(!m.armed);
}
#[test]
fn ground_timing_attributes_30222us_gap_without_counting_log_as_poll_work() {
    let mut trace = ground_timing::GroundTiming::default();
    trace.poll(1850);
    trace.drive(0);
    trace.summary(26_000);
    trace.begin(30_222, 3018);
    assert_eq!(trace.worst.sensor_poll_us, 1850);
    assert_eq!(trace.worst.motor_drive_us, 0);
    assert_eq!(trace.worst.previous_work_us, 3018);
    assert_eq!(trace.worst.summary_log_us, 26_000);
    assert_eq!(trace.worst.outside_work_and_log_us, 1204);
    assert_eq!(trace.max_summary_us, 26_000);
    assert_eq!(trace.poll_us, 0);
    trace.poll(21_000);
    trace.drive(1200);
    trace.begin(24_000, 22_000);
    assert_eq!(trace.worst.interval_us, 30_222);
    assert_eq!(trace.max_poll_us, 21_000);
    assert_eq!(trace.max_drive_us, 1200);
}

#[test]
fn ground_readiness_dwell_new_neutral_and_ten_second_wait_are_required_at_real_gate() {
    use motors::DualMotor;
    let forged = policy::Output {
        limited_pd: 4,
        preview: preview(true, 70.0),
    };
    let mut r = ground_response(true);
    r.begin(0);
    let mut m = Fake::default();
    let mut g = ground::GroundGuard::default();
    for t in (0..=480_000).step_by(20_000) {
        assert!(r.sample(0.0, 0.0, true, 0.02, t).is_none());
        m.arm();
        ground::drive(&mut m, &mut r, &mut g, Some(forged), 70, 2.0, t).unwrap();
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
        assert_eq!(r.state(), ResponseState::WaitingForReady);
    }
    assert!(r.sample(0.0, 0.0, true, 0.019999, 499_999).is_none());
    assert_eq!(r.state(), ResponseState::WaitingForReady);
    assert!(r.sample(0.0, 0.0, true, 0.000001, 500_000).is_none());
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    m.arm();
    ground::drive(&mut m, &mut r, &mut g, Some(forged), 70, 2.0, 500_000).unwrap();
    assert!(!m.armed);
    assert!(r.sample(0.0, 0.0, true, 0.01, 510_000).is_none());
    assert_eq!(r.state(), ResponseState::Active);
    m.arm();
    ground::drive(&mut m, &mut r, &mut g, None, 70, 2.0, 510_000).unwrap();
    assert_eq!(m.outputs, (0, 0)); // Acceptance frame never kicks.
    let mut r = ground_response(true);
    r.begin(0);
    for t in (0..10_000_000).step_by(20_000) {
        r.sample(3.88, 0.0, true, 0.02, t);
        m.arm();
        ground::drive(&mut m, &mut r, &mut g, Some(forged), 70, 2.0, t).unwrap();
        assert!(!m.armed);
        assert_eq!(m.outputs, (0, 0));
        assert!(!r.finished);
    }
    ground::drive(&mut m, &mut r, &mut g, Some(forged), 70, 2.0, 9_999_999).unwrap();
    assert!(!r.finished);
    ground::drive(&mut m, &mut r, &mut g, Some(forged), 70, 2.0, 10_000_000).unwrap();
    assert_eq!(r.disarm_reason, Some(DisarmReason::ArmingTimeout));
    assert_locked(&mut r, &mut m, &mut g, forged, 10_000_000);
}

#[test]
fn ground_pre_ready_live_tilt_and_nonfinite_estimates_abort_but_dry_stays_preview() {
    use motors::DualMotor;
    let old = policy::Output {
        limited_pd: 4,
        preview: preview(true, 70.0),
    };
    for pitch in [-15.001, -15.0, 15.0, 15.001, f32::NAN, f32::INFINITY] {
        let mut r = ground_response(true);
        r.begin(0);
        let mut m = Fake::default();
        let mut g = ground::GroundGuard::default();
        m.arm();
        m.command(-30, 30).unwrap();
        assert!(ground::sample(&mut m, &mut r, pitch, 0.0, true, 0.01, 10_000).is_none());
        assert_eq!(
            r.disarm_reason,
            Some(DisarmReason::Fault(if pitch.is_finite() {
                balance::Fault::Tilt
            } else {
                balance::Fault::Invalid
            }))
        );
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
        assert_locked(&mut r, &mut m, &mut g, old, 10_000);
    }
    for pitch in [-14.999, 14.999] {
        let mut r = ground_response(true);
        let mut m = Fake::default();
        assert!(ground::sample(&mut m, &mut r, pitch, 0.0, true, 0.01, 10_000).is_none());
        assert!(!r.finished);
        assert_eq!(r.state(), ResponseState::WaitingForReady);
    }
    let mut r = ground_response(false);
    let mut m = Fake::default();
    let mut g = ground::GroundGuard::default();
    let output = ground::sample(&mut m, &mut r, 30.0, 0.0, true, 0.01, 10_000);
    assert!(output.is_some());
    assert_eq!(r.state(), ResponseState::DryRun);
    ground::drive(&mut m, &mut r, &mut g, output, 70, 2.0, 10_000).unwrap();
    assert_eq!(m.outputs, (0, 0));
    assert!(!m.armed);
}
#[test]
fn ground_sample_entry_stops_active_fault_before_waiting_for_output_dispatch() {
    for (pitch, rate) in [(15.0, 0.0), (0.0, f32::NAN)] {
        let (mut r, mut m, _, _) = active_ground(1);
        assert!(ground::sample(&mut m, &mut r, pitch, rate, true, 0.01, 540_000).is_none());
        assert!(r.finished);
        assert!(!m.armed);
        assert_eq!(m.outputs, (0, 0));
    }
}

#[path = "../src/bin/motor_response_runtime/bidirectional.rs"]
mod bidirectional;
fn bidir_step(
    g: &mut bidirectional::Interlock,
    r: &mut policy::Response,
    m: &mut Fake,
    sign: i16,
    t: u64,
    kick: bool,
) {
    let mut o = r.sample(f32::from(sign) * 4.0, 0.0, true, 0.01, t).unwrap();
    // Inject a worst-case mapper request: the driver gate must independently enforce dead-time.
    o.preview.mapped_pwm = f32::from(sign) * 33.0;
    o.preview.final_pwm = f32::from(sign) * if kick { 70.0 } else { 33.0 };
    o.preview.kick_active = kick;
    g.drive(m, r, Some(o), 70, || t).unwrap();
}
#[test]
fn bidirectional_both_signs_zero_ack_deadtime_and_no_second_kick() {
    for sign in [-1, 1] {
        let (mut r, mut m, _, _) = active_ground(sign);
        let mut g = bidirectional::Interlock::default();
        bidir_step(&mut g, &mut r, &mut m, sign, 540_000, true);
        assert_eq!(m.outputs, (-70 * sign, 70 * sign));
        bidir_step(&mut g, &mut r, &mut m, -sign, 550_000, true);
        assert_eq!(m.outputs, (0, 0));
        assert_eq!(g.telemetry(550_000).pending_direction, -sign);
        for t in (560_000..650_000).step_by(10_000) {
            bidir_step(&mut g, &mut r, &mut m, -sign, t, true);
            assert_eq!(m.outputs, (0, 0));
        }
        bidir_step(&mut g, &mut r, &mut m, -sign, 650_000, true);
        assert_eq!(m.outputs, (33 * sign, -33 * sign));
        assert_eq!(g.kicks, 1);
        assert!(!g.kicking);
    }
}
#[test]
fn bidirectional_chatter_restarts_wait_and_deadband_stays_zero() {
    let (mut r, mut m, _, _) = active_ground(1);
    let mut g = bidirectional::Interlock::default();
    bidir_step(&mut g, &mut r, &mut m, 1, 540_000, true);
    for t in (550_000..850_000).step_by(10_000) {
        let sign = if (t / 10_000) % 2 == 0 { 1 } else { -1 };
        bidir_step(&mut g, &mut r, &mut m, sign, t, true);
        assert!(m.outputs == (0, 0) || m.outputs.1 > 0);
        assert!(g.kicks <= 1);
    }
    let o = r.sample(2.0, 0.0, true, 0.01, 850_000).unwrap();
    g.drive(&mut m, &mut r, Some(o), 70, || 850_000).unwrap();
    assert_eq!(m.outputs, (0, 0));
    bidir_step(&mut g, &mut r, &mut m, -1, 860_000, true);
    assert_eq!(m.outputs, (0, 0));
    assert_eq!(g.telemetry(860_000).zero_elapsed_us, 0);
}
#[test]
fn bidirectional_faults_during_wait_stop_actual_driver_and_latch() {
    for fault in [
        balance::Fault::Invalid,
        balance::Fault::Timing,
        balance::Fault::Stale,
        balance::Fault::Tilt,
        balance::Fault::Motor,
    ] {
        let (mut r, mut m, _, old) = active_ground(1);
        let mut g = bidirectional::Interlock::default();
        bidir_step(&mut g, &mut r, &mut m, 1, 540_000, true);
        bidir_step(&mut g, &mut r, &mut m, -1, 550_000, true);
        r.fail(fault);
        g.drive(&mut m, &mut r, Some(old), 70, || 560_000).unwrap();
        assert!(r.finished);
        assert!(!m.armed);
        assert_eq!(m.outputs, (0, 0));
        g.drive(&mut m, &mut r, Some(old), 70, || 570_000).unwrap();
        assert_eq!(m.outputs, (0, 0));
        assert!(!g.kicking);
    }
}
#[test]
fn bidirectional_timeout_during_wait_and_driver_error_disarm() {
    for driver_error in [false, true] {
        let (mut r, mut m, _, old) = active_ground(1);
        let mut g = bidirectional::Interlock::default();
        for t in (540_000..1_490_000).step_by(10_000) {
            bidir_step(&mut g, &mut r, &mut m, 1, t, false);
        }
        bidir_step(&mut g, &mut r, &mut m, -1, 1_490_000, false);
        assert_eq!(m.outputs, (0, 0));
        if driver_error {
            m.reject = true;
            let o = r.sample(-4.0, 0.0, true, 0.01, 1_500_000).unwrap();
            assert!(g.drive(&mut m, &mut r, Some(o), 70, || 1_500_000).is_err());
        } else {
            r.sample(-4.0, 0.0, true, 0.01, 1_510_000);
            g.drive(&mut m, &mut r, Some(old), 70, || 1_520_000)
                .unwrap();
            assert_eq!(r.disarm_reason, Some(DisarmReason::ActiveTimeout));
        }
        assert!(r.finished);
        assert!(!m.armed);
        assert_eq!(m.outputs, (0, 0));
    }
}
#[test]
fn ground_preparation_is_scoped_and_all_new_gates_disabled() {
    let ground = include_str!("../src/bin/balance_ground_contact_test.rs");
    let bidir = include_str!("../src/bin/balance_bidirectional_ground_test.rs");
    for s in [ground, bidir] {
        assert!(s.contains("preparation_secs: 30"));
    }
    for name in [
        "PHYSICAL_RESTRAINT_VERIFIED",
        "INDEPENDENT_POWER_CUTOFF_VERIFIED",
        "SUPPLY_LIMITS_VERIFIED",
        "SENSOR_FAULT_REVIEWED",
    ] {
        assert!(bidir.contains(&format!("const {name}: bool = false;")));
    }
    assert!(
        include_str!("../src/bin/balance_motor_response_test.rs").contains("preparation_secs: 15")
    );
}

#[test]
fn bidirectional_deadtime_begins_after_zero_io_not_before_it() {
    let (mut r, mut m, _, _) = active_ground(1);
    let mut g = bidirectional::Interlock::default();
    bidir_step(&mut g, &mut r, &mut m, 1, 540_000, true);
    let mut o = r.sample(-4.0, 0.0, true, 0.01, 550_000).unwrap();
    o.preview.mapped_pwm = -33.0;
    o.preview.final_pwm = -33.0;
    let mut calls = 0;
    g.drive(&mut m, &mut r, Some(o), 70, || {
        calls += 1;
        if calls == 1 { 550_000 } else { 551_200 }
    })
    .unwrap();
    assert_eq!(m.outputs, (0, 0));
    for t in (560_000..650_000).step_by(10_000) {
        bidir_step(&mut g, &mut r, &mut m, -1, t, false);
    }
    bidir_step(&mut g, &mut r, &mut m, -1, 651_199, false);
    assert_eq!(m.outputs, (0, 0));
    bidir_step(&mut g, &mut r, &mut m, -1, 651_200, false);
    assert_eq!(m.outputs, (33, -33));
}
#[test]
fn bidirectional_dry_preview_cannot_access_the_driver() {
    let mut r = ground_response(false);
    let mut g = bidirectional::Interlock::default();
    let o = r.sample(4.0, 0.0, true, 0.01, 0).unwrap();
    g.preview(o, 0);
    assert_eq!(g.applied, (-70, 70));
    g.reset();
    assert_eq!(g.applied, (0, 0));
    assert!(!g.kicking);
}

#[test]
fn bidirectional_recorder_pins_accepted_kick_and_zero_wait_through_overflow() {
    let mut log = EventLog::<1>::new();
    let mut s = snapshot(1);
    s.commanded = (-70, 70);
    s.reversal = Some(bidirectional::Telemetry {
        pending_direction: 0,
        zero_elapsed_us: 0,
        kicks: 1,
        kick_applied: true,
    });
    log.observe(s);
    s.us = 2;
    s.commanded = (0, 0);
    s.reversal = Some(bidirectional::Telemetry {
        pending_direction: -1,
        zero_elapsed_us: 0,
        kicks: 1,
        kick_applied: false,
    });
    log.observe(s);
    s.us = 100_002;
    s.commanded = (33, -33);
    s.reversal = Some(bidirectional::Telemetry {
        pending_direction: 0,
        zero_elapsed_us: 0,
        kicks: 1,
        kick_applied: false,
    });
    log.observe(s);
    for i in 0..10 {
        s.us += 1;
        s.requested = (i, -i);
        log.observe(s);
    }
    assert_eq!(log.first_applied.unwrap().commanded, (-70, 70));
    assert_eq!(log.last_interlock_zero.unwrap().commanded, (0, 0));
    assert_eq!(log.last_interlock_release.unwrap().commanded, (33, -33));
    assert!(
        log.last_applied_kick_on
            .unwrap()
            .reversal
            .unwrap()
            .kick_applied
    );
    assert!(
        !log.last_applied_kick_off
            .unwrap()
            .reversal
            .unwrap()
            .kick_applied
    );
    assert!(log.overwritten > 0);
}

#[test]
fn bidirectional_zero_or_absent_output_cancels_kick_at_driver_gate() {
    for absent in [false, true] {
        let (mut r, mut m, _, _) = active_ground(1);
        let mut g = bidirectional::Interlock::default();
        bidir_step(&mut g, &mut r, &mut m, 1, 540_000, true);
        let o = r.sample(0.0, 0.0, true, 0.01, 550_000).unwrap();
        g.drive(
            &mut m,
            &mut r,
            if absent { None } else { Some(o) },
            70,
            || 550_000,
        )
        .unwrap();
        assert_eq!(m.outputs, (0, 0));
        assert!(!g.kicking);
        bidir_step(&mut g, &mut r, &mut m, 1, 560_000, true);
        assert_eq!(m.outputs, (-33, 33));
        assert_eq!(g.kicks, 1);
    }
}

#[test]
fn bidirectional_neutral_acceptance_does_not_consume_first_post_arm_kick() {
    let (mut r, mut m, _, _) = active_ground(1);
    let mut g = bidirectional::Interlock::default();
    g.reset(); // No estimate during startup must not spend the Kick budget.
    g.drive(&mut m, &mut r, None, 70, || 530_000).unwrap();
    assert_eq!(m.outputs, (0, 0));
    assert_eq!(g.kicks, 0);
    bidir_step(&mut g, &mut r, &mut m, 1, 540_000, true);
    assert_eq!(m.outputs, (-70, 70));
    assert_eq!(g.kicks, 1);
}

// Real PD -> mapper/KickState -> bidirectional gate; only the GPIO driver is fake.
fn threshold_trial(bidir: bool) -> (policy::Response, Fake, bidirectional::Interlock) {
    use motors::DualMotor;
    let mut r = profile_response(true, bidir);
    for t in (0..=520_000).step_by(20_000) {
        assert!(r.sample(0.0, 0.0, true, 0.02, t).is_none());
    }
    assert_eq!(r.state(), ResponseState::Active);
    let mut m = Fake::default();
    m.arm();
    let mut g = bidirectional::Interlock::default();
    g.drive(&mut m, &mut r, None, 70, || 520_000).unwrap();
    assert_eq!(m.outputs, (0, 0));
    (r, m, g)
}
fn threshold_step(
    r: &mut policy::Response,
    m: &mut Fake,
    g: &mut bidirectional::Interlock,
    pd: i16,
    t: u64,
) -> policy::Output {
    let o = r.sample(f32::from(pd), 0.0, true, 0.01, t).unwrap();
    assert_eq!(o.limited_pd, pd);
    g.drive(m, r, Some(o), 70, || t).unwrap();
    assert!(!r.finished);
    o
}
#[test]
fn threshold_initial_plus_and_minus_two_stay_zero() {
    for pd in [-2, 2] {
        let (mut r, mut m, mut g) = threshold_trial(true);
        let o = threshold_step(&mut r, &mut m, &mut g, pd, 530_000);
        assert_eq!(o.preview.final_pwm, 0.0);
        assert!(!o.preview.kick_active);
        assert_eq!(m.outputs, (0, 0));
        assert_eq!(g.kicks, 0);
    }
}
#[test]
fn threshold_initial_plus_and_minus_three_and_four_get_one_symmetric_kick() {
    for pd in [-4, -3, 3, 4] {
        let (mut r, mut m, mut g) = threshold_trial(true);
        let o = threshold_step(&mut r, &mut m, &mut g, pd, 530_000);
        assert!(o.preview.kick_active);
        assert_eq!(m.outputs, (-70 * pd.signum(), 70 * pd.signum()));
        assert_eq!(g.kicks, 1);
        assert!(g.kicking);
    }
}
#[test]
fn threshold_other_profiles_still_require_four() {
    for pd in [-4, -3, 3, 4] {
        let (mut r, mut m, mut g) = threshold_trial(false);
        let o = threshold_step(&mut r, &mut m, &mut g, pd, 530_000);
        let duty = if pd.abs() == 4 { 70 } else { 31 };
        assert_eq!(o.preview.kick_active, pd.abs() == 4);
        assert_eq!(m.outputs, (-duty * pd.signum(), duty * pd.signum()));
    }
}
#[test]
fn threshold_kick_expires_at_150ms_to_ordinary_pwm_without_retrigger_while_held() {
    for sign in [-1, 1] {
        let (mut r, mut m, mut g) = threshold_trial(true);
        for t in (530_000..680_000).step_by(10_000) {
            threshold_step(&mut r, &mut m, &mut g, 3 * sign, t);
            assert_eq!(m.outputs, (-70 * sign, 70 * sign));
        }
        for t in (680_000..900_000).step_by(10_000) {
            let o = threshold_step(&mut r, &mut m, &mut g, 3 * sign, t);
            assert!(!o.preview.kick_active);
            assert!((o.preview.mapped_pwm.abs() - 31.538462).abs() < 0.0001);
            assert_eq!(m.outputs, (-31 * sign, 31 * sign));
            assert_eq!(g.kicks, 1);
        }
    }
}
#[test]
fn threshold_deadband_cancels_and_mapper_rearm_cannot_grant_second_applied_kick() {
    for sign in [-1, 1] {
        let (mut r, mut m, mut g) = threshold_trial(true);
        threshold_step(&mut r, &mut m, &mut g, 3 * sign, 530_000);
        for t in (540_000..=850_000).step_by(10_000) {
            let o = threshold_step(&mut r, &mut m, &mut g, 2 * sign, t);
            assert!(!o.preview.kick_active);
            assert!(!g.kicking);
            assert_eq!(m.outputs, (0, 0));
        }
        let o = threshold_step(&mut r, &mut m, &mut g, 3 * sign, 860_000);
        assert!(o.preview.kick_active); // Real mapper rearmed, final gate still vetoes boost.
        assert_eq!(m.outputs, (-31 * sign, 31 * sign));
        assert_eq!(g.kicks, 1);
        assert!(!g.kicking);
    }
}
#[test]
fn threshold_reversal_keeps_zero_deadtime_and_cooldown_has_no_delayed_kick() {
    for sign in [-1, 1] {
        let (mut r, mut m, mut g) = threshold_trial(true);
        threshold_step(&mut r, &mut m, &mut g, 3 * sign, 530_000);
        for t in (540_000..640_000).step_by(10_000) {
            threshold_step(&mut r, &mut m, &mut g, -3 * sign, t);
            assert_eq!(m.outputs, (0, 0));
            assert!(!g.kicking);
        }
        for t in (640_000..900_000).step_by(10_000) {
            let o = threshold_step(&mut r, &mut m, &mut g, -3 * sign, t);
            // Reversal occurs before the mapper's 300ms cooldown expires.
            // Keeping the request past cooldown must not cause a delayed Kick.
            assert!(!o.preview.kick_active);
            assert_eq!(m.outputs, (31 * sign, -31 * sign));
            assert_eq!(g.kicks, 1);
        }
    }
}

#[test]
fn threshold_three_cannot_bypass_readiness_and_new_neutral_acceptance() {
    use motors::DualMotor;
    let mut r = profile_response(true, true);
    let mut m = Fake::default();
    let mut g = bidirectional::Interlock::default();
    for t in (0..=500_000).step_by(20_000) {
        let o = r.sample(0.0, 0.0, true, 0.02, t);
        g.drive(&mut m, &mut r, o, 70, || t).unwrap();
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
        assert_eq!(g.kicks, 0);
    }
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    let o = r.sample(3.0, 0.0, true, 0.02, 520_000);
    assert!(o.is_none());
    g.drive(&mut m, &mut r, o, 70, || 520_000).unwrap();
    assert_eq!(r.state(), ResponseState::ArmedWaitingForNeutral);
    assert_eq!(m.outputs, (0, 0));
    assert!(!m.armed);
    let o = r.sample(0.0, 0.0, true, 0.01, 530_000);
    assert_eq!(r.state(), ResponseState::Active);
    assert!(o.is_none());
    m.arm();
    g.drive(&mut m, &mut r, o, 70, || 530_000).unwrap();
    assert_eq!(m.outputs, (0, 0));
    assert_eq!(g.kicks, 0);
    threshold_step(&mut r, &mut m, &mut g, 3, 540_000);
    assert_eq!(m.outputs, (-70, 70));
    assert_eq!(g.kicks, 1);
}

#[test]
fn threshold_reversal_after_mapper_cooldown_still_cannot_apply_another_kick() {
    for sign in [-1, 1] {
        let (mut r, mut m, mut g) = threshold_trial(true);
        for t in (530_000..850_000).step_by(10_000) {
            threshold_step(&mut r, &mut m, &mut g, 3 * sign, t);
        }
        for t in (850_000..950_000).step_by(10_000) {
            threshold_step(&mut r, &mut m, &mut g, -3 * sign, t);
            assert_eq!(m.outputs, (0, 0));
        }
        let o = threshold_step(&mut r, &mut m, &mut g, -3 * sign, 950_000);
        assert!(o.preview.kick_active); // Mapper requests a new pulse after cooldown.
        assert_eq!(m.outputs, (31 * sign, -31 * sign)); // Real final gate suppresses it.
        assert_eq!(g.kicks, 1);
        assert!(!g.kicking);
    }
}

#[path = "../src/bin/motor_response_runtime/capture.rs"]
mod capture;
#[test]
fn capture_saturation_keeps_recent_samples_and_terminal_fault_context() {
    let mut capture = capture::Capture::<Snapshot, 2>::new();
    let mut events = EventLog::<2>::new();
    for i in 0..100 {
        let mut s = snapshot(i * 100_000);
        s.requested = (i as i16, -(i as i16));
        capture.record(s.us, s);
        events.observe(s);
    }
    assert_eq!(capture.overwritten, 98);
    assert_eq!(
        capture.recent().map(|s| s.us).collect::<Vec<_>>(),
        vec![9_800_000, 9_900_000]
    );
    let mut terminal = snapshot(10_000_000);
    terminal.state = ResponseState::Disarmed;
    terminal.commanded = (0, 0);
    terminal.disarm = Some(DisarmReason::Fault(balance::Fault::Timing));
    terminal.missed = 95;
    events.observe(terminal);
    assert_eq!(events.terminal.unwrap().missed, 95);
    assert_eq!(events.terminal.unwrap().commanded, (0, 0));
    assert_eq!(events.terminal.unwrap().disarm, terminal.disarm);
}
#[test]
fn capture_rate_limit_never_backfills_or_resets_timing_counters() {
    let mut capture = capture::Capture::<u64, 2>::new();
    let mut timing = policy::LoopTiming::default();
    assert!(!timing.begin(0, 30_000));
    capture.record(0, 0);
    assert!(timing.begin(38_246, 30_000));
    capture.record(38_246, 38_246);
    assert_eq!(timing.missed_deadlines, 1);
    assert_eq!(timing.max_interval_us, 38_246);
    assert!(!timing.begin(39_246, 30_000));
    capture.record(39_246, 39_246);
    assert_eq!(capture.recent().collect::<Vec<_>>(), vec![0]);
    capture.record(100_000, 100_000);
    capture.record(900_000, 900_000);
    assert_eq!(capture.recent().collect::<Vec<_>>(), vec![100_000, 900_000]);
    assert_eq!(capture.overwritten, 1);
    assert_eq!(timing.missed_deadlines, 1);
    assert_eq!(timing.max_interval_us, 38_246);
}
#[test]
fn saturated_capture_cannot_block_deadline_disarm_or_terminal_stop() {
    let (mut r, mut m, _, old) = active_ground(1);
    let mut capture = capture::Capture::<u64, 1>::new();
    for i in 0..100 {
        capture.record(i * 100_000, i);
    }
    assert_eq!(capture.overwritten, 99);
    let mut timing = policy::LoopTiming::default();
    timing.begin(530_000, 30_000);
    let late = timing.begin(568_246, 30_000);
    assert!(ground::deadline(&mut m, &mut r, late));
    // Observe actual driver shutdown BEFORE reading/formatting the full recorder.
    assert_eq!(m.outputs, (0, 0));
    assert!(!m.armed);
    assert!(r.finished);
    assert_eq!(
        r.disarm_reason,
        Some(DisarmReason::Fault(balance::Fault::Timing))
    );
    assert_eq!(capture.recent().collect::<Vec<_>>(), vec![99]);
    policy::drive(&mut m, &mut r, Some(old), 70, 568_246).unwrap();
    assert_eq!(m.outputs, (0, 0));
    assert_eq!(timing.missed_deadlines, 1);
    r.stop();
    assert_eq!(
        r.disarm_reason,
        Some(DisarmReason::Fault(balance::Fault::Timing))
    );
}
#[test]
fn dry_capture_stop_is_permanent_and_output_gate_disarms_before_dump() {
    use motors::DualMotor;
    let mut r = ground_response(false);
    let o = r.sample(4.0, 0.0, true, 0.01, 0).unwrap();
    let mut m = Fake::default();
    m.arm();
    m.command(-70, 70).unwrap();
    r.stop();
    policy::drive(&mut m, &mut r, Some(o), 70, capture::DRY_CAPTURE_US).unwrap();
    assert_eq!(m.outputs, (0, 0));
    assert!(!m.armed);
    assert_eq!(r.disarm_reason, Some(DisarmReason::Stop));
    assert!(
        r.sample(0.0, 0.0, true, 0.01, capture::DRY_CAPTURE_US + 10_000)
            .is_none()
    );
    assert_eq!(r.state(), ResponseState::Disarmed);
}

#[path = "../src/bin/motor_response_runtime/stability.rs"]
mod stability;
#[test]
fn phase2_full_90_seconds_survives_record_overflow_without_losing_statistics() {
    let mut stats = stability::Stability::default();
    let mut events = EventLog::<2>::new();
    stats.start(500_000);
    for t in (500_000..=90_500_000).step_by(1_000) {
        let mut s = snapshot(t);
        s.sample_us = Some(500_000 + (t - 500_000) / 10_000 * 10_000);
        s.pitch = Some(0.5);
        s.rate = Some(0.0);
        s.commanded = (0, 0);
        if t == 90_500_000 {
            s.state = ResponseState::Disarmed;
            s.disarm = Some(DisarmReason::ActiveTimeout);
        }
        s.requested = if (t / 100_000) % 2 == 0 {
            (-1, 1)
        } else {
            (0, 0)
        }; // Stress event storage only.
        stats.observe(&s);
        events.observe(s);
    }
    assert_eq!(stats.active_us, 90_000_000);
    assert_eq!(stats.within_us, [90_000_000; 4]);
    assert_eq!(stats.longest_near_us, 90_000_000);
    assert_eq!(stats.zero_us, 90_000_000);
    assert_eq!(stats.nonzero_us, 0);
    assert_eq!(stats.zero_crossings, 0);
    assert_eq!(stats.disarm, Some(DisarmReason::ActiveTimeout));
    assert!(events.overwritten > 0);
    assert_eq!(events.terminal.unwrap().us, 90_500_000);
}
#[test]
fn phase2_band_timing_and_noise_crossing_are_active_only() {
    let mut stats = stability::Stability::default();
    let mut s = snapshot(0);
    s.state = ResponseState::DryRun;
    s.pitch = Some(20.0);
    stats.observe(&s);
    stats.start(100_000);
    for (n, p) in [0.1, -0.1, 0.2, 0.5, 1.5, 2.5, 4.0, 6.0, -0.5]
        .into_iter()
        .enumerate()
    {
        let t = 100_000 + n as u64 * 10_000;
        s = snapshot(t);
        s.pitch = Some(p);
        s.rate = Some(n as f32);
        stats.observe(&s);
    }
    s = snapshot(190_000);
    s.state = ResponseState::Disarmed;
    stats.observe(&s);
    assert_eq!(stats.active_us, 90_000);
    assert_eq!(stats.within_us, [50_000, 60_000, 70_000, 80_000]);
    assert_eq!(stats.zero_crossings, 1);
    assert_eq!(stats.overshoot_like, 1);
    assert_eq!(stats.max_abs_pitch, 6.0);
    assert_eq!(stats.max_abs_rate, 8.0);
}

fn phase2_kick_sample(us: u64, count: u8, applied: bool, total: u64, last: u64) -> Snapshot {
    let mut s = snapshot(us);
    s.pitch = Some(0.5);
    s.kick = applied;
    s.commanded = if applied { (-70, 70) } else { (0, 0) };
    s.reversal = Some(bidirectional::Telemetry {
        pending_direction: 0,
        zero_elapsed_us: 0,
        kicks: count,
        kick_applied: applied,
    });
    let a = restart_assist::Assist::new(restart_assist::Config::default());
    let mut t = a.status(us);
    t.applied = applied;
    t.pulse_us = 0;
    t.total_on_us = total;
    t.last_pulse_us = last;
    s.assist = Some(t);
    s
}
#[test]
fn phase2_uses_acknowledged_kick_durations_and_bounded_storage() {
    // Storage stress only: this synthetic event rate is NOT an allowed motor policy.
    let mut stats = stability::Stability::default();
    stats.start(0);
    for i in 0..130_u64 {
        stats.observe(&phase2_kick_sample(
            i * 2_000,
            (i + 1) as u8,
            true,
            i * 1_000,
            if i == 0 { 0 } else { 1_000 },
        ));
        stats.observe(&phase2_kick_sample(
            i * 2_000 + 1_000,
            (i + 1) as u8,
            false,
            (i + 1) * 1_000,
            1_000,
        ));
    }
    stats.finish(
        260_000,
        Some(DisarmReason::Stop),
        0,
        phase2_kick_sample(260_000, 130, false, 130_000, 1_000).assist,
        130,
    );
    assert_eq!(stats.kick_count, 130);
    assert_eq!(stats.kick_on_us, 130_000);
    assert_eq!(stats.max_kick_us, 1_000);
    assert!(!stats.kick_timing_estimated);
    assert_eq!(stats.kick_durations_us(), &[1_000; 128]);
    assert_eq!(stats.missing_kick_records, 2);
    assert_eq!(stats.zero_us + stats.nonzero_us, stats.active_us);
}
#[test]
fn phase2_detects_a_short_acknowledged_pulse_absent_from_periodic_history() {
    let mut stats = stability::Stability::default();
    stats.start(0);
    stats.observe(&phase2_kick_sample(0, 0, false, 0, 0));
    let mut s = phase2_kick_sample(1_000, 1, false, 300, 300);
    s.state = ResponseState::Disarmed;
    s.disarm = Some(DisarmReason::Fault(balance::Fault::Timing));
    s.missed = 1;
    stats.observe(&s);
    assert_eq!(stats.kick_count, 1);
    assert_eq!(stats.kick_on_us, 300);
    assert_eq!(stats.kick_durations_us(), &[300]);
    assert_eq!(stats.active_missed_deadlines, 1);
    assert_eq!(stats.disarm, s.disarm);
    stats.observe(&snapshot(10_000));
    assert_eq!(stats.active_us, 1_000); // Frozen after stop.
}
#[test]
fn phase2_clips_pose_freshness_and_handles_duplicate_backwards_and_invalid_times() {
    let mut stats = stability::Stability::default();
    stats.start(0);
    let mut s = snapshot(0);
    s.pitch = Some(0.5);
    s.commanded = (0, 0);
    stats.observe(&s);
    stats.observe(&s);
    s.us = 100_000; // Cached estimate is now stale: never charge the whole gap as upright.
    stats.observe(&s);
    assert_eq!(stats.active_us, 100_000);
    assert_eq!(stats.within_us, [40_000; 4]);
    assert_eq!(stats.unknown_pose_us, 60_000);
    assert_eq!(stats.longest_near_us, 40_000);
    s.us = 99_999;
    stats.observe(&s);
    assert_eq!(stats.clock_errors, 1);
    s.us = 100_000;
    s.sample_us = Some(100_001);
    stats.observe(&s);
    s.sample_us = Some(100_000);
    s.pitch = Some(f32::NAN);
    stats.observe(&s);
    stats.finish(110_000, Some(DisarmReason::Stop), 0, None, 0);
    assert_eq!(stats.active_us, 110_000);
    assert_eq!(stats.unknown_pose_us, 70_000);
    assert_eq!(stats.zero_us + stats.nonzero_us, 110_000);
    assert!(stats.invalid_samples >= 2);
    assert_eq!(stats.zero_crossings, 0);
}
#[test]
fn phase2_applied_direction_changes_ignore_neutral_intervals_and_dry() {
    let mut stats = stability::Stability::default();
    let mut s = snapshot(0);
    s.state = ResponseState::DryRun;
    stats.observe(&s);
    assert_eq!(stats.active_us, 0);
    assert!(!stats.has_started());
    stats.start(100_000);
    for (n, command) in [(-30, 30), (0, 0), (0, 0), (30, -30), (0, 0), (-30, 30)]
        .into_iter()
        .enumerate()
    {
        s = snapshot(100_000 + n as u64 * 10_000);
        s.commanded = command;
        stats.observe(&s);
    }
    stats.finish(160_000, Some(DisarmReason::Stop), 0, None, 0);
    assert_eq!(stats.correction_sign_changes, 2);
    assert_eq!(stats.zero_us, 30_000);
    assert_eq!(stats.nonzero_us, 30_000);
}

#[test]
fn phase2_legacy_timing_is_labelled_and_no_active_run_stays_empty() {
    assert!(core::mem::size_of::<stability::Stability>() < 1024);
    let mut empty = stability::Stability::default();
    empty.finish(100, Some(DisarmReason::Stop), 2, None, 0);
    assert_eq!(empty.active_us, 0);
    assert!(!empty.has_started());
    let mut stats = stability::Stability::default();
    let mut s = phase2_kick_sample(0, 1, true, 0, 0);
    s.assist = None;
    stats.observe(&s);
    s.us = 10_000;
    s.sample_us = Some(10_000);
    s.kick = false;
    s.commanded = (0, 0);
    stats.observe(&s);
    stats.finish(20_000, Some(DisarmReason::Stop), 0, None, 1);
    assert!(stats.kick_timing_estimated);
    assert_eq!(stats.kick_durations_us(), &[10_000]);
    assert_eq!(stats.kick_on_us, 10_000);
}
