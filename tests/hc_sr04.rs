#[path = "../src/hc_sr04.rs"]
mod hc_sr04;

use hc_sr04::{Outcome, measure};

#[test]
fn initial_high_skips_trigger() {
    let report = measure(|| (true, 0), || panic!("triggered with Echo already high"));
    assert_eq!(report.outcome, Outcome::EchoStuckHigh);
    assert_eq!(report.trigger_us, None);
    assert_eq!(report.wait_rise_us, None);
}

fn run(samples: &[(bool, u64)]) -> hc_sr04::Report {
    let mut samples = samples.iter().copied();
    measure(
        || samples.next().expect("unexpected sample"),
        || (100, u64::from(hc_sr04::TRIGGER_US) + 1),
    )
}

#[test]
fn missing_or_late_rise_times_out() {
    for level in [false, true] {
        let report = run(&[(false, 89), (false, 101), (level, 30_100)]);
        assert_eq!(report.outcome, Outcome::WaitRiseTimeout);
        assert_eq!(report.trigger_us, Some(11));
        assert_eq!(report.wait_rise_us, Some(30_000));
        assert_eq!(report.echo_high_us, None);
    }
}

#[test]
fn missing_or_late_fall_times_out() {
    for level in [true, false] {
        let report = run(&[(false, 89), (true, 150), (level, 30_150)]);
        assert_eq!(report.outcome, Outcome::WaitFallTimeout);
        assert_eq!(report.wait_rise_us, Some(50));
        assert_eq!(report.echo_high_us, Some(30_000));
    }
}

#[test]
fn converts_only_pulses_in_range() {
    for (width, expected) in [
        (0, Outcome::InvalidPulse),
        (116, Outcome::InvalidPulse),
        (117, Outcome::DistanceMm(20)),
        (1000, Outcome::DistanceMm(171)),
        (23_323, Outcome::DistanceMm(3999)),
        (23_324, Outcome::InvalidPulse),
    ] {
        let report = run(&[(false, 89), (true, 150), (false, 150 + width)]);
        assert_eq!(report.outcome, expected);
        assert!(!report.initial_echo_high);
        assert_eq!(report.trigger_us, Some(11));
        assert_eq!(report.wait_rise_us, Some(50));
        assert_eq!(report.echo_high_us, Some(width));
    }
}
