#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
use control::{Disarmed, DualMotor};
#[path = "../src/bin/motor_duty_characterization/sequence.rs"]
mod sequence;
#[derive(Default)]
struct Motor {
    armed: bool,
    outputs: (i16, i16),
    fail: bool,
    commands: usize,
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
        if self.fail || !self.armed {
            return Err(Disarmed);
        }
        self.commands += 1;
        self.outputs = (a, b);
        Ok(())
    }
}
#[test]
fn sequence_commands_both_motors_once_per_stage_with_neutral_between() {
    let mut m = Motor::default();
    for index in 0..6 {
        let (forward, duty) = sequence::step(index).unwrap();
        assert_eq!(forward, index < 3);
        assert_eq!(duty, [40, 50, 60][index % 3]);
        sequence::apply(&mut m, index, true, 0).unwrap();
        let u = if forward {
            i16::from(duty)
        } else {
            -i16::from(duty)
        };
        assert_eq!(m.outputs, (u, u));
        assert_eq!(m.commands, index + 1);
        sequence::shutdown(&mut m);
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
    }
    assert!(sequence::step(6).is_none());
}
#[test]
fn dry_timeout_invalid_step_and_driver_error_leave_actual_outputs_zero() {
    for (index, live, elapsed, fail) in [
        (0, false, 0, false),
        (0, true, sequence::TOTAL_MS, false),
        (6, true, 0, false),
        (0, true, 0, true),
    ] {
        let mut m = Motor {
            armed: true,
            outputs: (60, 60),
            fail,
            commands: 0,
        };
        let _ = sequence::apply(&mut m, index, live, elapsed);
        assert_eq!(m.outputs, (0, 0));
        assert!(!m.armed);
    }
    assert_eq!(sequence::ON_MS, 1_000);
    const {
        assert!(
            sequence::PREPARATION_MS + 6 * (sequence::ON_MS + sequence::PAUSE_MS)
                < sequence::TOTAL_MS
        );
    }
}

#[test]
fn default_source_gate_is_off_and_balance_features_cannot_authorize_motion() {
    let source = include_str!("../src/bin/motor_duty_characterization.rs");
    assert!(source.contains("const MOTOR_POWER_AND_RESTRAINT_VERIFIED: bool = false;"));
    assert!(
        source
            .contains("cfg!(feature = \"motor-kick-live\") && MOTOR_POWER_AND_RESTRAINT_VERIFIED")
    );
    assert!(!source.contains("cfg!(feature = \"balance"));
}
