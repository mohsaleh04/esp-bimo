#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
#[path = "../src/bin/tb6612_diagnostics/sequence.rs"]
mod sequence;

use control::Direction;
use sequence::{Confirmation, Stage, selected_stage};

#[test]
fn pulse_requires_feature_explicit_stage_and_no_both_feature() {
    for stage in Stage::ALL {
        assert_eq!(selected_stage(true, false, Some(stage)), Some(stage));
        assert_eq!(selected_stage(false, false, Some(stage)), None);
        assert_eq!(selected_stage(true, true, Some(stage)), None);
    }
    assert_eq!(selected_stage(true, false, None), None);
}

#[test]
fn each_stage_selects_exactly_one_wheel_and_direction() {
    use Direction::*;
    assert_eq!(
        Stage::ALL.map(Stage::directions),
        [
            [Positive, Stopped],
            [Negative, Stopped],
            [Stopped, Positive],
            [Stopped, Negative],
        ]
    );
}

#[test]
fn confirmation_requires_one_exact_terminated_line() {
    for end in [b'\r', b'\n'] {
        let mut confirmation = Confirmation::default();
        for byte in b"RUN" {
            assert_eq!(confirmation.push(*byte), None);
        }
        assert_eq!(confirmation.push(end), Some(true));
        assert_eq!(confirmation.push(end), Some(false));
    }
    for input in [b"\n".as_slice(), b"RU\n", b"run\n", b"RUNX\n", b" RUN\n"] {
        let mut confirmation = Confirmation::default();
        assert_eq!(
            input.iter().find_map(|&byte| confirmation.push(byte)),
            Some(false)
        );
    }
}
