#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;
use pwm_mapping::{Config, map, step};
fn config() -> Config {
    Config {
        enabled: true,
        deadband: 2.0,
        pd_output_limit: 15.0,
        minimum_duty: 25.0,
        maximum_duty: 50.0,
        slew_per_second: None,
    }
}
#[test]
fn exact_zero_and_deadband_stop_even_after_nonzero_output() {
    let c = config();
    for raw in [0.0, -0.0, 1.0, -2.0, 2.0] {
        assert_eq!(map(c, raw), Some(0.0));
        assert_eq!(step(c, raw, 45.0, 0.02), Some(0.0));
    }
}
#[test]
fn symmetric_floor_saturation_and_sign() {
    let c = config();
    for raw in [3.0, 15.0, 42.0, 1000.0] {
        let positive = map(c, raw).unwrap();
        assert!(positive > 0.0 && positive <= 50.0);
        assert_eq!(map(c, -raw), Some(-positive));
    }
    close(map(c, 3.0).unwrap(), 25.0 + 25.0 / 13.0);
    assert_eq!(map(c, 1000.0), Some(50.0));
}
#[test]
fn disabled_is_identity_except_ceiling_and_never_adds_a_floor() {
    let c = Config {
        enabled: false,
        slew_per_second: Some(1.0),
        ..config()
    };
    assert_eq!(step(c, 1.0, 0.0, 0.02), Some(1.0));
    assert_eq!(map(c, -80.0), Some(-50.0));
}
#[test]
fn slew_uses_measured_time_and_never_retains_the_wrong_sign() {
    let c = Config {
        slew_per_second: Some(100.0),
        ..config()
    };
    assert_eq!(step(c, 10.0, 0.0, 0.02), Some(2.0));
    assert_eq!(step(c, 10.0, 0.0, 0.04), Some(4.0));
    assert_eq!(step(c, -10.0, 40.0, 0.02), Some(-2.0));
    assert_eq!(step(c, 0.0, 40.0, 0.02), Some(0.0));
}
#[test]
fn invalid_configuration_input_and_timing_are_rejected() {
    assert_eq!(map(config(), f32::NAN), None);
    assert_eq!(
        map(
            Config {
                maximum_duty: 100.0,
                ..config()
            },
            5.0
        ),
        None
    );
    assert_eq!(
        map(
            Config {
                minimum_duty: 55.0,
                ..config()
            },
            5.0
        ),
        None
    );
    assert_eq!(step(config(), 5.0, 0.0, 0.2), None);
}

use pwm_mapping::{KickConfig, KickState};
fn kick() -> KickConfig {
    KickConfig {
        enabled: true,
        kick_duty: 50.0,
        kick_duration_ms: 50,
        trigger_threshold: 4.0,
        rearm_delay_ms: 300,
        reversal_pause_ms: 100,
    }
}
#[test]
fn kick_zero_start_expiration_and_held_request() {
    let mut s = KickState::default();
    assert_eq!(
        s.update(config(), kick(), 0.0, 0.02, 0).unwrap().final_pwm,
        0.0
    );
    let start = s.update(config(), kick(), 5.0, 0.02, 20).unwrap();
    assert!(start.kick_active);
    close(start.mapped_pwm, 25.0 + 3.0 * 25.0 / 13.0);
    assert_eq!(start.final_pwm, 50.0);
    assert_eq!(
        s.update(config(), kick(), 5.0, 0.02, 69)
            .unwrap()
            .kick_elapsed_ms,
        49
    );
    let expired = s.update(config(), kick(), 5.0, 0.02, 70).unwrap();
    assert!(!expired.kick_active);
    close(expired.final_pwm, map(config(), 5.0).unwrap());
    assert!(
        !s.update(config(), kick(), 5.0, 0.02, 500)
            .unwrap()
            .kick_active
    );
}
#[test]
fn kick_sign_symmetry_and_zero_cancels_immediately() {
    for sign in [-1.0, 1.0] {
        let mut s = KickState::default();
        assert_eq!(
            s.update(config(), kick(), sign * 5.0, 0.02, 0)
                .unwrap()
                .final_pwm,
            sign * 50.0
        );
        let zero = s.update(config(), kick(), 0.0, 0.02, 10).unwrap();
        assert_eq!(zero.final_pwm, 0.0);
        assert!(!zero.kick_active);
    }
}
#[test]
fn rapid_reversal_pauses_at_zero_and_respects_cooldown() {
    let mut s = KickState::default();
    s.update(config(), kick(), 5.0, 0.02, 0).unwrap();
    assert_eq!(
        s.update(config(), kick(), -5.0, 0.02, 10)
            .unwrap()
            .final_pwm,
        0.0
    );
    assert_eq!(
        s.update(config(), kick(), -5.0, 0.02, 109)
            .unwrap()
            .final_pwm,
        0.0
    );
    let reverse = s.update(config(), kick(), -5.0, 0.02, 110).unwrap();
    close(reverse.final_pwm, map(config(), -5.0).unwrap());
    assert!(!reverse.kick_active); // Original kick still in cooldown.
    assert!(
        !s.update(config(), kick(), -5.0, 0.02, 400)
            .unwrap()
            .kick_active
    );
}
#[test]
fn stable_reversal_can_kick_after_pause_and_cooldown() {
    let mut s = KickState::default();
    s.update(config(), kick(), 5.0, 0.02, 0).unwrap();
    assert_eq!(
        s.update(config(), kick(), -5.0, 0.02, 400)
            .unwrap()
            .final_pwm,
        0.0
    );
    assert_eq!(
        s.update(config(), kick(), 5.0, 0.02, 450)
            .unwrap()
            .final_pwm,
        0.0
    );
    assert_eq!(
        s.update(config(), kick(), -5.0, 0.02, 460)
            .unwrap()
            .final_pwm,
        0.0
    );
    assert_eq!(
        s.update(config(), kick(), -5.0, 0.02, 559)
            .unwrap()
            .final_pwm,
        0.0
    );
    assert_eq!(
        s.update(config(), kick(), -5.0, 0.02, 560)
            .unwrap()
            .final_pwm,
        -50.0
    );
}
#[test]
fn noisy_pd_cannot_repeatedly_kick_and_zero_dwell_rearms() {
    let mut s = KickState::default();
    for t in 0..20 {
        let raw = if t % 2 == 0 { 1.0 } else { 3.0 };
        assert!(
            !s.update(config(), kick(), raw, 0.02, t * 20)
                .unwrap()
                .kick_active
        );
    }
    assert!(
        s.update(config(), kick(), 5.0, 0.02, 400)
            .unwrap()
            .kick_active
    );
    s.update(config(), kick(), 0.0, 0.02, 420).unwrap();
    assert!(
        !s.update(config(), kick(), 5.0, 0.02, 440)
            .unwrap()
            .kick_active
    );
    s.update(config(), kick(), 0.0, 0.02, 500).unwrap();
    assert!(
        s.update(config(), kick(), 5.0, 0.02, 800)
            .unwrap()
            .kick_active
    );
}
#[test]
fn reset_invalid_data_and_disabled_preview_clear_kick_history() {
    let mut s = KickState::default();
    s.update(config(), kick(), 5.0, 0.02, 0).unwrap();
    s.reset();
    assert!(
        s.update(config(), kick(), 5.0, 0.02, 20)
            .unwrap()
            .kick_active
    );
    assert!(s.update(config(), kick(), f32::NAN, 0.02, 30).is_none());
    assert!(
        s.update(config(), kick(), 5.0, 0.02, 40)
            .unwrap()
            .kick_active
    );
    let disabled = KickConfig {
        enabled: false,
        ..kick()
    };
    assert!(
        !s.update(config(), disabled, 5.0, 0.02, 60)
            .unwrap()
            .kick_active
    );
    assert!(
        s.update(config(), kick(), 5.0, 0.02, 80)
            .unwrap()
            .kick_active
    );
    assert!(s.update(config(), kick(), 5.0, 0.0, 90).is_none());
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
#[test]
fn kick_trigger_uses_limited_pd_and_expiration_returns_proportional_output() {
    let mut s = KickState::default();
    let k = KickConfig {
        trigger_threshold: 16.0,
        ..kick()
    };
    let p = s.update(config(), k, 1000.0, 0.02, 0).unwrap();
    assert!(!p.kick_active); // Clamped to 15, below 16, despite huge raw request.
    for sign in [-1.0, 1.0] {
        s.reset();
        assert!(
            s.update(config(), kick(), sign * 8.5, 0.02, 0)
                .unwrap()
                .kick_active
        );
        let p = s.update(config(), kick(), sign * 8.5, 0.02, 50).unwrap();
        assert!(!p.kick_active);
        close(p.final_pwm, sign * 37.5);
    }
}

#[test]
fn kick_review_fast_deadband_chatter_does_not_rekick_but_ordinary_pwm_can_switch() {
    let mut state = KickState::default();
    let mut starts = 0;
    let mut was = false;
    for i in 0..100 {
        let pd = if i % 2 == 0 { 4.0 } else { 2.0 };
        let p = state.update(config(), kick(), pd, 0.01, i * 10).unwrap();
        if p.kick_active && !was {
            starts += 1;
        }
        was = p.kick_active;
    }
    assert_eq!(starts, 1); // 10ms zero periods cannot satisfy 300ms continuous rearm.
    state.reset();
    for i in 0..50 {
        let pd = if i % 2 == 0 { 3.0 } else { 2.0 };
        let p = state.update(config(), kick(), pd, 0.01, i * 10).unwrap();
        assert!(!p.kick_active);
        assert_eq!(p.final_pwm == 0.0, pd == 2.0); // Existing floor discontinuity, not repeated kicks.
    }
}
