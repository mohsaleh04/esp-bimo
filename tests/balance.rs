#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
use balance::*;

fn ready(live: bool) -> Controller {
    let mut c = Controller::new(Settings::default(), true, live).unwrap();
    for i in 0..=30 {
        c.sample(0.0, 0.0, i * 20_000);
    }
    c
}
#[test]
fn pd_and_physical_mapping_have_the_confirmed_feedback_sign() {
    let s = Settings::default();
    assert!(pd(s, 2.0, 0.0).unwrap() > 0);
    assert!(pd(s, -2.0, 0.0).unwrap() < 0);
    assert!(pd(s, 0.0, 20.0).unwrap() > 0);
    assert_eq!(wheel_commands(10), (10, 10));
    assert_eq!(wheel_commands(-10), (-10, -10));
    assert_eq!(wheel_commands(0), (0, 0));
}
#[test]
fn pd_saturates_symmetrically_without_a_minimum_duty_jump() {
    let s = Settings::default();
    assert_eq!(pd(s, 100.0, 100.0), Some(15));
    assert_eq!(pd(s, -100.0, -100.0), Some(-15));
    assert_eq!(pd(s, 0.1, 0.0), Some(0));
    assert_eq!(pd(s, 0.0, 0.0), Some(0));
    assert_eq!(pd(s, f32::NAN, 0.0), None);
}
#[test]
fn boot_is_observe_and_no_automatic_arm_occurs() {
    let c = ready(true);
    assert_eq!(c.state(), State::Observe);
    assert!(c.output(1.0, 0.0).is_none());
}
#[test]
fn arming_requires_calibration_build_gate_freshness_and_stable_upright() {
    assert_eq!(
        ready(false).request(Request::Arm, 600_000),
        Err("live build feature disabled")
    );
    let mut c = Controller::new(Settings::default(), false, true).unwrap();
    for i in 0..=30 {
        c.sample(0.0, 0.0, i * 20_000);
    }
    assert!(c.request(Request::Arm, 600_000).is_err());
    let mut c = Controller::new(Settings::default(), true, true).unwrap();
    c.sample(0.0, 0.0, 0);
    assert!(c.request(Request::Arm, 0).is_err());
    let mut c = ready(true);
    c.sample(6.0, 0.0, 620_000);
    assert!(c.request(Request::Arm, 620_000).is_err());
    let mut c = ready(true);
    c.sample(0.0, 6.0, 620_000);
    assert!(c.request(Request::Arm, 620_000).is_err());
    assert!(ready(true).request(Request::Arm, 650_001).is_err());
}
#[test]
fn dry_run_never_enters_active_and_stop_requires_a_new_request() {
    let mut c = ready(false);
    c.request(Request::Dry, 600_000).unwrap();
    assert_eq!(c.state(), State::DryRun);
    assert!(c.output(2.0, 0.0).is_some());
    c.request(Request::Stop, 600_000).unwrap();
    assert_eq!(c.state(), State::Observe);
    c.sample(0.0, 0.0, 620_000);
    assert_eq!(c.state(), State::Observe);
}
#[test]
fn faults_latch_and_need_fresh_upright_dwell_then_deliberate_rearming() {
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.sample(15.1, 0.0, 620_000);
    assert_eq!(c.state(), State::Fault(Fault::Tilt));
    assert!(c.request(Request::Arm, 620_000).is_err());
    for i in 0..=30 {
        c.sample(0.0, 0.0, 640_000 + i * 20_000);
    }
    assert_eq!(c.state(), State::Fault(Fault::Tilt));
    assert!(c.request(Request::Arm, 1_240_000).is_err());
    c.request(Request::Reset, 1_240_000).unwrap();
    for i in 0..=30 {
        c.sample(0.0, 0.0, 1_260_000 + i * 20_000);
    }
    c.request(Request::Arm, 1_860_000).unwrap();
    assert_eq!(c.state(), State::Active);
}
#[test]
fn stale_invalid_time_and_trial_expiry_stop_active_control() {
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.tick(640_001);
    assert_eq!(c.state(), State::Fault(Fault::Stale));
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.sample(f32::NAN, 0.0, 620_000);
    assert_eq!(c.state(), State::Fault(Fault::Invalid));
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.sample(0.0, 0.0, 600_000);
    assert_eq!(c.state(), State::Fault(Fault::Timing));
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    for i in 1..=100 {
        c.sample(0.0, 0.0, 600_000 + i * 20_000);
        c.tick(600_000 + i * 20_000);
    }
    assert_eq!(c.state(), State::Fault(Fault::TrialExpired));
}
#[test]
fn console_commands_are_exact_and_overflow_does_not_arm() {
    let mut p = Console::default();
    for byte in b"ARM\r" {
        assert_eq!(p.byte(*byte), None);
    }
    assert_eq!(p.byte(b'\n'), Some(Request::Arm));
    for byte in b"XARM\n" {
        assert_ne!(p.byte(*byte), Some(Request::Arm));
    }
    for _ in 0..30 {
        p.byte(b'A');
    }
    assert_eq!(p.byte(b'\n'), Some(Request::Invalid));
    assert_eq!(p.byte(b'!'), Some(Request::Stop));
}

#[test]
fn denied_request_is_fail_closed_even_from_an_active_state() {
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    assert!(c.request(Request::Arm, 700_000).is_err());
    assert_eq!(c.state(), State::Observe);
    assert!(c.output(1.0, 0.0).is_none());
}
#[test]
fn settings_reject_unsafe_or_nonfinite_values_and_tilt_limit_is_inclusive() {
    let mut s = Settings {
        output_limit: 31,
        ..Default::default()
    };
    assert!(Controller::new(s, true, true).is_none());
    s = Settings::default();
    s.kp = f32::NAN;
    assert!(Controller::new(s, true, true).is_none());
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.sample(15.0, 0.0, 620_000);
    assert_eq!(c.state(), State::Fault(Fault::Tilt));
}

#[test]
fn repeated_arm_cannot_extend_an_active_trial() {
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.sample(0.0, 0.0, 620_000);
    assert!(c.request(Request::Arm, 620_000).is_err());
    assert_eq!(c.state(), State::Observe);
    assert!(!c.ready(620_000));
    assert!(c.output(0.0, 0.0).is_none());
}

#[test]
fn forward_calibration_pose_stays_observe_and_can_enter_dry() {
    let mut c = Controller::new(Settings::default(), true, false).unwrap();
    c.sample(34.0, 0.0, 0);
    assert_eq!(c.state(), State::Observe);
    assert!(!c.ready(0));
    c.request(Request::Dry, 0).unwrap();
    assert_eq!(c.state(), State::DryRun);
    assert_eq!(c.output(34.0, 0.0), Some(15));
    c.sample(-34.0, 10.0, 20_000);
    assert_eq!(c.state(), State::DryRun);
    assert_eq!(c.output(-34.0, 10.0), Some(-15));
}
#[test]
fn dry_survives_invalid_stale_and_gap_data_but_does_not_output_invalid_values() {
    let mut c = ready(false);
    c.request(Request::Dry, 600_000).unwrap();
    c.sample(f32::NAN, 0.0, 620_000);
    assert_eq!(c.state(), State::DryRun);
    assert_eq!(c.output(1.0, 0.0), None);
    c.sample(34.0, 0.0, 640_000);
    assert_eq!(c.output(34.0, 0.0), Some(15));
    c.tick(680_001);
    assert_eq!(c.state(), State::DryRun);
    assert_eq!(c.output(34.0, 0.0), None);
    c.sample(34.0, 0.0, 700_000);
    assert_eq!(c.output(34.0, 0.0), Some(15));
    c.sample(33.0, 0.0, 760_000);
    assert_eq!(c.state(), State::DryRun);
    assert_eq!(c.output(33.0, 0.0), Some(15));
}
#[test]
fn live_fault_requires_explicit_reset_even_after_upright_recovery() {
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.sample(34.0, 0.0, 620_000);
    assert_eq!(c.state(), State::Fault(Fault::Tilt));
    for i in 0..=30 {
        c.sample(0.0, 0.0, 640_000 + i * 20_000);
    }
    assert!(!c.ready(1_240_000));
    assert!(c.request(Request::Dry, 1_240_000).is_err());
    assert!(c.request(Request::Arm, 1_240_000).is_err());
    c.request(Request::Reset, 1_240_000).unwrap();
    assert_eq!(c.state(), State::Observe);
    assert!(c.request(Request::Arm, 1_240_000).is_err());
    for i in 0..=30 {
        c.sample(0.0, 0.0, 1_260_000 + i * 20_000);
    }
    c.request(Request::Arm, 1_860_000).unwrap();
}
#[test]
fn invalid_console_command_is_reported_and_cannot_clear_a_live_fault() {
    let mut p = Console::default();
    let mut result = None;
    for b in b"BOGUS\n" {
        result = p.byte(*b);
    }
    assert_eq!(result, Some(Request::Invalid));
    let mut c = ready(true);
    c.request(Request::Arm, 600_000).unwrap();
    c.fault(Fault::Invalid);
    assert!(c.request(Request::Invalid, 620_000).is_err());
    assert_eq!(c.state(), State::Fault(Fault::Invalid));
}

#[test]
fn gyro_only_observation_outputs_pd_but_cannot_establish_live_readiness() {
    let mut c = ready(true);
    c.request(Request::Dry, 600_000).unwrap();
    c.sample(1.0, 0.0, 620_000);
    c.block_arming();
    assert_eq!(c.output(1.0, 0.0), Some(1));
    assert!(!c.ready(620_000));
    assert!(c.request(Request::Arm, 620_000).is_err());
}

#[test]
fn automatic_dry_startup_needs_no_upright_pose_or_serial_and_never_grants_live() {
    // The standalone diagnostic explicitly disables live permission, independent of features.
    let mut c = Controller::new(Settings::default(), true, false).unwrap();
    c.request(Request::Dry, 0).unwrap();
    assert_eq!(c.state(), State::DryRun);
    assert_eq!(c.output(34.0, 0.0), None); // No fresh estimate yet.
    c.sample(34.0, 2.0, 20_000);
    assert_eq!(c.output(34.0, 2.0), Some(15));
    for i in 0..=30 {
        c.sample(0.0, 0.0, 40_000 + i * 20_000);
    }
    assert_eq!(c.state(), State::DryRun); // Upright readiness never auto-arms.
    assert!(c.request(Request::Arm, 640_000).is_err());
    assert_ne!(c.state(), State::Active);
}

#[test]
fn control_profile_cutoff_is_21_inclusive_and_latched_without_changing_default() {
    assert_eq!(FALL_ANGLE_DEG, 15.0);
    assert_eq!(CONTROL_FALL_ANGLE_DEG, 21.0);
    for sign in [1.0, -1.0] {
        let mut c = Controller::new(Settings::default(), true, true)
            .unwrap()
            .with_tilt_limit(CONTROL_FALL_ANGLE_DEG)
            .unwrap();
        for t in (0..=600_000).step_by(20_000) {
            c.sample(0.0, 0.0, t);
        }
        c.request(Request::Arm, 600_000).unwrap();
        c.sample(sign * 20.999, 0.0, 620_000);
        assert_eq!(c.state(), State::Active);
        c.sample(sign * 21.0, 0.0, 640_000);
        assert_eq!(c.state(), State::Fault(Fault::Tilt));
        c.sample(0.0, 0.0, 660_000);
        assert_eq!(c.state(), State::Fault(Fault::Tilt));
    }
    for limit in [f32::NAN, f32::INFINITY, 0.0, 21.001] {
        assert!(
            Controller::new(Settings::default(), true, true)
                .unwrap()
                .with_tilt_limit(limit)
                .is_none()
        );
    }
}
