#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
use balance::*;

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
