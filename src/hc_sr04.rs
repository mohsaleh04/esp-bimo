//! Timing and validation for the standalone HC-SR04 diagnostic.

pub const TRIGGER_US: u32 = 10;
const EDGE_TIMEOUT_US: u64 = 30_000;
// Echo widths corresponding to the 20–4000 mm operating range at 343 m/s.
const VALID_ECHO_US: core::ops::RangeInclusive<u64> = 117..=23_323;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    EchoStuckHigh,
    WaitRiseTimeout,
    WaitFallTimeout,
    InvalidPulse,
    DistanceMm(u64),
}

pub struct Report {
    pub initial_echo_high: bool,
    pub trigger_us: Option<u64>,
    pub wait_rise_us: Option<u64>,
    pub echo_high_us: Option<u64>,
    pub outcome: Outcome,
}

/// Sample Echo and its timestamp in microseconds on the same monotonic clock.
/// The trigger callback returns its falling-edge timestamp and HIGH duration.
pub fn measure(
    mut sample: impl FnMut() -> (bool, u64),
    trigger: impl FnOnce() -> (u64, u64),
) -> Report {
    let (initial_echo_high, _) = sample();
    let mut report = Report {
        initial_echo_high,
        trigger_us: None,
        wait_rise_us: None,
        echo_high_us: None,
        outcome: Outcome::EchoStuckHigh,
    };
    if initial_echo_high {
        return report;
    }

    let (trigger_end, trigger_us) = trigger();
    report.trigger_us = Some(trigger_us);
    let (rise, waited_us) = wait_for_level(&mut sample, true, trigger_end);
    report.wait_rise_us = Some(waited_us);
    let Some(rise) = rise else {
        report.outcome = Outcome::WaitRiseTimeout;
        return report;
    };

    let (fall, high_us) = wait_for_level(&mut sample, false, rise);
    report.echo_high_us = Some(high_us);
    report.outcome = if fall.is_none() {
        Outcome::WaitFallTimeout
    } else if VALID_ECHO_US.contains(&high_us) {
        Outcome::DistanceMm(high_us * 343 / 2000)
    } else {
        Outcome::InvalidPulse
    };
    report
}

fn wait_for_level(
    sample: &mut impl FnMut() -> (bool, u64),
    desired_high: bool,
    start_us: u64,
) -> (Option<u64>, u64) {
    loop {
        let (high, now_us) = sample();
        let elapsed_us = now_us - start_us;
        if elapsed_us >= EDGE_TIMEOUT_US {
            return (None, elapsed_us);
        }
        if high == desired_high {
            return (Some(now_us), elapsed_us);
        }
    }
}
