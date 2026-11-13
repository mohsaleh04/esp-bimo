// The HAL-backed trait implementation is validated in target builds.
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
use control::{Direction, DutyLimit};

#[test]
fn zero_and_zero_limit_are_deenergized() {
    let limit = DutyLimit::new(30).unwrap();
    assert_eq!(limit.map(0).direction, Direction::Stopped);
    assert_eq!(limit.map(0).duty_percent, 0);
    let stopped = DutyLimit::new(0).unwrap().map(100);
    assert_eq!(stopped.direction, Direction::Stopped);
    assert_eq!(stopped.duty_percent, 0);
}

#[test]
fn signs_choose_electrical_polarity_and_magnitude_is_clamped() {
    let limit = DutyLimit::new(30).unwrap();
    assert_eq!(limit.map(15).direction, Direction::Positive);
    assert_eq!(limit.map(-15).direction, Direction::Negative);
    assert_eq!(limit.map(15).duty_percent, 15);
    assert_eq!(limit.map(-15).duty_percent, 15);
    for command in [31, 100, i16::MAX, -31, -100, i16::MIN] {
        assert_eq!(limit.map(command).duty_percent, 30);
    }
}

#[test]
fn limits_validate_and_channels_map_independently() {
    assert!(DutyLimit::new(101).is_none());
    assert!(DutyLimit::new(255).is_none());
    let limit = DutyLimit::new(100).unwrap();
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
