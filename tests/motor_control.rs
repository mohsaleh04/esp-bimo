// The HAL-backed trait implementation is validated in target builds.
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
use control::{Direction, DutyLimit};

#[test]
fn zero_and_zero_limit_are_deenergized() {
    let limit = DutyLimit::new(30);
    assert_eq!(limit.map(0).direction, Direction::Stopped);
    assert_eq!(limit.map(0).duty_percent, 0);
    let stopped = DutyLimit::new(0).map(100);
    assert_eq!(stopped.direction, Direction::Stopped);
    assert_eq!(stopped.duty_percent, 0);
}

#[test]
fn signs_choose_electrical_polarity_and_magnitude_is_clamped() {
    let limit = DutyLimit::new(30);
    assert_eq!(limit.map(15).direction, Direction::Positive);
    assert_eq!(limit.map(-15).direction, Direction::Negative);
    assert_eq!(limit.map(15).duty_percent, 15);
    assert_eq!(limit.map(-15).duty_percent, 15);
    for command in [31, 100, i16::MAX, -31, -100, i16::MIN] {
        assert_eq!(limit.map(command).duty_percent, 30);
    }
}

#[test]
fn limits_clamp_and_channels_map_independently() {
    assert_eq!(DutyLimit::new(101).map(200).duty_percent, 100);
    assert_eq!(DutyLimit::new(255).map(200).duty_percent, 100);
    let limit = DutyLimit::new(100);
    let a = limit.map(100);
    let b = limit.map(-7);
    assert_eq!((a.direction, a.duty_percent), (Direction::Positive, 100));
    assert_eq!((b.direction, b.duty_percent), (Direction::Negative, 7));
}

#[test]
fn gate_starts_disarmed_and_rejects_movement_until_armed() {
    let mut gate = control::CommandGate::default();
    assert_eq!(gate.check(1, 0), Err(control::Disarmed));
    assert_eq!(gate.check(0, -1), Err(control::Disarmed));
    assert_eq!(gate.check(0, 0), Ok(()));
    gate.arm();
    assert_eq!(gate.check(10, -10), Ok(()));
    gate.disarm();
    assert_eq!(gate.check(10, -10), Err(control::Disarmed));
}

#[test]
fn input_truth_table_never_requests_both_direction_pins_high() {
    assert_eq!(Direction::Stopped.inputs(), (false, false));
    assert_eq!(Direction::Positive.inputs(), (true, false));
    assert_eq!(Direction::Negative.inputs(), (false, true));
}

#[test]
fn diagnostic_limit_and_effective_pwm_duty_agree() {
    // Earlier test asked for 75 but its 40% ceiling allowed only 40%.
    assert_eq!(DutyLimit::new(40).map(75).duty_percent, 40);
    let limit = DutyLimit::new(50);
    assert_eq!(limit.map(50).duty_percent, 50);
    assert_eq!(limit.map(0).direction, Direction::Stopped);
    assert_eq!(limit.map(0).duty_percent, 0);
}

#[test]
fn bridge_direction_inverts_a_once_and_preserves_b_and_stop() {
    use control::Channel;

    for (direction, a, b) in [
        (Direction::Positive, (false, true), (true, false)),
        (Direction::Negative, (true, false), (false, true)),
        (Direction::Stopped, (false, false), (false, false)),
    ] {
        assert_eq!(Channel::A.direction_inputs(direction), a);
        assert_eq!(Channel::B.direction_inputs(direction), b);
    }
}

#[test]
fn disarm_rejects_every_nonzero_command_and_accepts_zero() {
    let mut gate = control::CommandGate::default();
    assert!(gate.check(1, 0).is_err());
    gate.arm();
    assert!(gate.check(1, 0).is_ok());
    gate.disarm();
    assert!(gate.check(0, 1).is_err());
    assert!(gate.check(1, 1).is_err());
    assert!(gate.check(0, 0).is_ok());
}
