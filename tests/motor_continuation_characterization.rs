#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
use control::{Disarmed, DualMotor};
#[path = "../src/bin/motor_continuation_characterization/sequence.rs"]
mod sequence;
use sequence::Phase;
#[derive(Default)]
struct Motor {
    armed: bool,
    outputs: (i16, i16),
    fail: bool,
    history: Vec<(i16, i16)>,
}
impl DualMotor for Motor {
    fn arm(&mut self) {
        assert_eq!(self.outputs, (0, 0));
        self.armed = true;
    }
    fn stop(&mut self) {
        self.outputs = (0, 0);
    }
    fn disarm(&mut self) {
        self.stop();
        self.armed = false;
    }
    fn command(&mut self, a: i16, b: i16) -> Result<(), Disarmed> {
        if !self.armed || self.fail {
            return Err(Disarmed);
        }
        self.outputs = (a, b);
        self.history.push((a, b));
        Ok(())
    }
}
#[test]
fn six_stages_startup_then_hold_preserve_polarity_and_stop() {
    let mut m = Motor::default();
    for index in 0..6 {
        let (forward, startup, hold) = sequence::step(index).unwrap();
        assert_eq!(forward, index < 3);
        assert_eq!(startup, if forward { 70 } else { 65 });
        assert_eq!(hold, [50, 40, 30][index % 3]);
        let sign = if forward { 1 } else { -1 };
        for (phase, duty) in [(Phase::Startup, startup), (Phase::Hold, hold)] {
            let expected = (-sign * i16::from(duty), sign * i16::from(duty));
            assert_eq!(sequence::apply(&mut m, index, phase, true, 0), Ok(expected));
            assert_eq!(m.outputs, expected);
        }
        assert_eq!(m.history.len(), 2 * (index + 1));
        sequence::shutdown(&mut m);
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
    }
    assert!(sequence::step(6).is_none());
}
#[test]
fn dry_timeout_bad_stage_and_driver_failures_stop_both_outputs() {
    for phase in [Phase::Startup, Phase::Hold] {
        for (index, live, time, fail) in [
            (0, false, 0, false),
            (0, true, sequence::TOTAL_MS, false),
            (6, true, 0, false),
            (0, true, 0, true),
        ] {
            let mut m = Motor {
                armed: true,
                outputs: (-50, 50),
                fail,
                ..Default::default()
            };
            let _ = sequence::apply(&mut m, index, phase, live, time);
            assert_eq!(m.outputs, (0, 0));
            assert!(!m.armed);
        }
    }
    let mut m = Motor::default();
    assert!(sequence::apply(&mut m, 0, Phase::Hold, true, 0).is_err());
    assert_eq!(m.outputs, (0, 0));
}
#[test]
fn timing_and_independent_default_off_source_authorization() {
    assert_eq!(sequence::STARTUP_MS, 100);
    assert_eq!(sequence::HOLD_MS, 1000);
    assert_eq!(sequence::PAUSE_MS, 5000);
    assert_eq!(sequence::PREPARATION_MS, 15000);
    assert_eq!(sequence::TOTAL_MS, 100000);
    let source = include_str!("../src/bin/motor_continuation_characterization.rs");
    assert!(source.contains("const MOTOR_POWER_AND_RESTRAINT_VERIFIED: bool = false;"));
    assert!(
        source
            .contains("cfg!(feature = \"motor-kick-live\") && MOTOR_POWER_AND_RESTRAINT_VERIFIED")
    );
}

#[test]
fn timeout_boundary_cancels_an_ongoing_startup_before_hold() {
    let mut m = Motor::default();
    sequence::apply(&mut m, 0, Phase::Startup, true, sequence::TOTAL_MS - 1).unwrap();
    assert_eq!(m.outputs, (-70, 70));
    assert!(sequence::apply(&mut m, 0, Phase::Hold, true, sequence::TOTAL_MS).is_err());
    assert_eq!(m.outputs, (0, 0));
    assert!(!m.armed);
    assert_eq!(m.history, vec![(-70, 70)]);
}
