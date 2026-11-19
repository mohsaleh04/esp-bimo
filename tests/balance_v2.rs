#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[path = "../src/balance_v2/controller.rs"]
mod controller;
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod motor_control;
#[allow(dead_code)]
#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;

use controller::{Controller, Fault};

#[test]
fn pd_uses_angle_and_rate_with_deadband_and_signed_wheel_commands() {
    let mut controller = Controller::new(0);
    let neutral = controller.preview(1.6, 0.0, true, 10_000).unwrap();
    assert_eq!(neutral.pd, 1);
    assert_eq!(neutral.wheels, [0, 0]);
    for sign in [1.0, -1.0] {
        let output = controller
            .preview(sign * 2.5, sign * 13.0, true, 20_000)
            .unwrap();
        assert_eq!(output.pd, (sign * 4.0) as i16);
        assert_eq!(output.wheels, [(sign * 21.0) as i16; 2]);
        assert_eq!(
            output.direction_inputs,
            if sign > 0.0 {
                [(false, true), (true, false)]
            } else {
                [(true, false), (false, true)]
            }
        );
    }
}

#[test]
fn pd_and_pwm_saturate_in_both_directions_below_tilt_cutoff() {
    let mut controller = Controller::new(0);
    for sign in [1.0, -1.0] {
        let output = controller
            .preview(sign * 20.0, sign * 200.0, true, 10_000)
            .unwrap();
        assert_eq!(output.pd, (sign * 10.0) as i16);
        assert_eq!(output.wheels, [(sign * 45.0) as i16; 2]);
    }
}

#[test]
fn empty_polls_do_not_refresh_sample_age_and_timeout_latches() {
    let mut controller = Controller::new(0);
    assert_eq!(controller.fresh_sample(10_000), Ok(None));
    assert!(controller.fresh_sample(20_000).unwrap().is_some());
    assert_eq!(controller.check_freshness(60_000), Ok(()));
    assert_eq!(controller.check_freshness(60_001), Err(Fault::StaleSample));
    assert_eq!(controller.fresh_sample(65_000), Err(Fault::StaleSample));
    assert_eq!(
        controller.preview(0.0, 0.0, true, 65_000),
        Err(Fault::StaleSample)
    );
    assert_eq!(
        Controller::new(0).check_freshness(40_001),
        Err(Fault::StaleSample)
    );
}

#[test]
fn duplicate_sample_time_tilt_and_invalid_estimate_latch_faults() {
    let mut controller = Controller::new(0);
    controller.fresh_sample(1).unwrap();
    assert_eq!(controller.fresh_sample(1), Err(Fault::Timing));

    for (pitch, rate, expected) in [
        (25.0, 0.0, Fault::Tilt),
        (-25.0, 0.0, Fault::Tilt),
        (0.0, f32::NAN, Fault::InvalidEstimate),
    ] {
        let mut controller = Controller::new(0);
        assert_eq!(controller.preview(pitch, rate, true, 10_000), Err(expected));
        assert_eq!(controller.preview(0.0, 0.0, true, 20_000), Err(expected));
    }
}

#[test]
fn gravity_correction_age_is_bounded_and_first_fault_wins() {
    let mut controller = Controller::new(0);
    controller.preview(0.0, 0.0, true, 10_000).unwrap();
    assert_eq!(controller.check_correction(60_000), Ok(()));
    assert_eq!(
        controller.check_correction(60_001),
        Err(Fault::CorrectionExpired)
    );
    assert_eq!(controller.fail(Fault::SensorRead), Fault::CorrectionExpired);
}

#[test]
fn sensor_faults_and_missing_gravity_reference_block_further_previews() {
    let mut controller = Controller::new(0);
    assert_eq!(
        controller.preview(0.0, 0.0, false, 10_000),
        Err(Fault::InvalidEstimate)
    );
    for reason in [Fault::SensorRead, Fault::InvalidSensor] {
        let mut controller = Controller::new(0);
        controller.fail(reason);
        assert_eq!(controller.fresh_sample(10_000), Err(reason));
        assert_eq!(controller.preview(0.0, 0.0, true, 10_000), Err(reason));
    }
}
