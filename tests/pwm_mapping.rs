#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;
use pwm_mapping::{Config, map};
fn config() -> Config {
    Config {
        deadband: 2.0,
        pd_output_limit: 15.0,
        minimum_duty: 25.0,
        maximum_duty: 50.0,
    }
}
#[test]
fn rejects_nonfinite_pd() {
    for raw in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert_eq!(map(config(), raw), None);
    }
}
fn close(actual: f32, expected: f32) {
    assert!((actual - expected).abs() < 0.0001, "{actual} != {expected}");
}
#[test]
fn proportional_boundaries_midpoint_and_signs() {
    let c = config();
    assert_eq!(map(c, 2.0), Some(0.0));
    assert_eq!(map(c, -2.0), Some(0.0));
    for (pd, pwm) in [
        (2.001, 25.0 + 0.001 * 25.0 / 13.0),
        (8.5, 37.5),
        (15.0, 50.0),
        (80.0, 50.0),
    ] {
        close(map(c, pd).unwrap(), pwm);
        close(map(c, -pd).unwrap(), -pwm);
    }
}
#[test]
fn degenerate_and_nonfinite_ranges_are_rejected() {
    for c in [
        Config {
            pd_output_limit: 2.0,
            ..config()
        },
        Config {
            pd_output_limit: 1.0,
            ..config()
        },
        Config {
            pd_output_limit: f32::INFINITY,
            ..config()
        },
        Config {
            minimum_duty: 50.0,
            ..config()
        },
        Config {
            maximum_duty: f32::NAN,
            ..config()
        },
    ] {
        assert!(!c.valid());
        assert_eq!(map(c, 5.0), None);
    }
}
