#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
use control::{Disarmed, DualMotor};
#[path = "../src/bin/motor_kick_diagnostic/sequence.rs"]
mod sequence;
use sequence::{Phase, Settings, apply};
fn settings() -> Settings {
    Settings {
        ceiling: 50,
        kick_duty: 50,
        kick_ms: 50,
        hold_duty: 30,
        hold_ms: 1000,
        repetitions: 1,
    }
}
#[derive(Default)]
struct Motor {
    armed: bool,
    commands: Vec<(i16, i16)>,
    output: (i16, i16),
}
impl DualMotor for Motor {
    fn arm(&mut self) {
        self.armed = true;
    }
    fn command(&mut self, a: i16, b: i16) -> Result<(), Disarmed> {
        if !self.armed {
            return Err(Disarmed);
        }
        self.output = (a, b);
        self.commands.push((a, b));
        Ok(())
    }
    fn stop(&mut self) {
        self.output = (0, 0);
    }
    fn disarm(&mut self) {
        self.stop();
        self.armed = false;
    }
}
#[test]
fn bounds_reject_candidate_above_ceiling_and_invalid_values() {
    assert!(settings().valid());
    for s in [
        Settings {
            ceiling: 30,
            ..settings()
        },
        Settings {
            ceiling: 100,
            ..settings()
        },
        Settings {
            hold_duty: 51,
            ..settings()
        },
        Settings {
            kick_ms: 0,
            ..settings()
        },
        Settings {
            hold_ms: 2001,
            ..settings()
        },
        Settings {
            repetitions: 0,
            ..settings()
        },
    ] {
        assert!(!s.valid());
    }
    let mut m = Motor::default();
    m.arm();
    assert!(apply(&mut m, true, Phase::Kick(50), 30, true).is_err());
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
}
#[test]
fn ordered_phases_and_polarity_end_in_shutdown_for_both_motors() {
    assert_eq!(
        settings().phases(),
        [
            (Phase::Kick(50), 50),
            (Phase::Hold(30), 1000),
            (Phase::Stop, 0)
        ]
    );
    for right in [true, false] {
        let mut m = Motor::default();
        for (phase, _) in settings().phases() {
            apply(&mut m, right, phase, 50, true).unwrap();
        }
        assert_eq!(
            m.commands,
            if right {
                vec![(-50, 0), (-30, 0)]
            } else {
                vec![(0, 50), (0, 30)]
            }
        );
        assert!(!m.armed);
        assert_eq!(m.output, (0, 0));
        assert!(m.command(1, 0).is_err());
    }
}
#[test]
fn preview_never_arms_and_final_stop_is_unconditional() {
    let mut m = Motor::default();
    for (phase, _) in settings().phases() {
        apply(&mut m, true, phase, 50, false).unwrap();
        assert!(!m.armed);
    }
    assert!(m.commands.is_empty());
    m.arm();
    m.command(20, 0).unwrap();
    apply(&mut m, true, Phase::Stop, 0, false).unwrap();
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
}
#[test]
fn rejected_hold_disarms_and_does_not_reenable_motor() {
    let mut m = Motor::default();
    apply(&mut m, true, Phase::Kick(50), 50, true).unwrap();
    assert!(apply(&mut m, true, Phase::Hold(60), 50, true).is_err());
    assert!(!m.armed);
    assert_eq!(m.output, (0, 0));
}
