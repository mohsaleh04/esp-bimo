//! Host tests for the production TB6612 command state and transition planner.
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod control;
#[path = "../src/motors/tb6612_transition.rs"]
mod tb6612_transition;

use control::{Disarmed, DutyLimit};
use tb6612_transition::{Action, CommandState};

#[test]
fn classifies_effective_duty_changes_and_channel_topology() {
    use Action::*;
    let limit = DutyLimit::new(70);
    let cases = [
        ([0, 0], [20, -20], Reconfigure),
        ([20, -20], [20, -20], Unchanged),
        ([70, -70], [100, i16::MIN], Unchanged),
        ([20, -20], [30, -20], UpdateCompare([true, false])),
        ([20, -20], [20, -30], UpdateCompare([false, true])),
        ([20, -20], [30, -40], UpdateCompare([true, true])),
        ([0, -20], [0, -40], UpdateCompare([false, true])),
        ([20, -20], [-20, -20], Reconfigure),
        ([20, -20], [-20, 0], Reconfigure),
        ([20, -20], [20, 0], Reconfigure),
        ([20, 0], [0, -20], Reconfigure),
        ([20, 0], [20, -20], Reconfigure),
        ([20, -20], [0, 0], Stop),
        ([0, 0], [0, 0], Stop),
    ];
    for (old, raw, expected) in cases {
        let mut state = CommandState::default();
        state.arm();
        let (action, effective) = state.plan(limit, old).unwrap();
        if action != Stop {
            state.record(effective);
        }
        assert_eq!(
            state.plan(limit, raw).unwrap().0,
            expected,
            "{old:?} -> {raw:?}"
        );
    }
}

#[test]
fn raw_gate_precedes_clamp_and_zero_is_allowed_while_disarmed() {
    let limit = DutyLimit::new(0);
    let mut state = CommandState::default();
    assert_eq!(state.plan(limit, [20, 0]), Err(Disarmed));
    assert_eq!(state.plan(limit, [0, 0]).unwrap().0, Action::Stop);
    state.arm();
    assert_eq!(state.plan(limit, [20, 0]).unwrap().0, Action::Stop);
}

#[test]
fn full_on_duty_changes_still_use_gated_reconfiguration() {
    let limit = DutyLimit::new(100);
    for (old, new) in [
        ([99, 0], [100, 0]),
        ([100, 0], [99, 0]),
        ([100, 20], [100, 30]),
    ] {
        let mut state = CommandState::default();
        state.arm();
        let (_, effective) = state.plan(limit, old).unwrap();
        state.record(effective);
        assert_eq!(state.plan(limit, new).unwrap().0, Action::Reconfigure);
    }
}

#[test]
fn stop_arm_and_disarm_clear_applied_state_and_keep_gate_semantics() {
    let limit = DutyLimit::new(70);
    let raw = [20, -20];
    let mut state = CommandState::default();
    state.arm();
    let (_, effective) = state.plan(limit, raw).unwrap();
    state.record(effective);
    assert_eq!(state.plan(limit, raw).unwrap().0, Action::Unchanged);

    state.stop();
    assert_eq!(state.plan(limit, raw).unwrap().0, Action::Reconfigure);
    state.record(effective);
    state.arm();
    assert_eq!(state.plan(limit, raw).unwrap().0, Action::Reconfigure);

    state.record(effective);
    state.disarm();
    assert_eq!(state.plan(limit, raw), Err(Disarmed));
    assert_eq!(state.plan(limit, [0, 0]).unwrap().0, Action::Stop);
    state.arm();
    assert_eq!(state.plan(limit, raw).unwrap().0, Action::Reconfigure);
}

#[test]
fn planning_does_not_record_a_command_before_output_succeeds() {
    let limit = DutyLimit::new(70);
    let mut state = CommandState::default();
    state.arm();
    let (_, effective) = state.plan(limit, [20, 0]).unwrap();
    state.record(effective);
    assert_eq!(
        state.plan(limit, [35, 0]).unwrap().0,
        Action::UpdateCompare([true, false])
    );
    assert_eq!(state.plan(limit, [20, 0]).unwrap().0, Action::Unchanged);
    state.disarm();
    assert_eq!(state.plan(limit, [35, 0]), Err(Disarmed));
    state.arm();
    assert_eq!(state.plan(limit, [35, 0]).unwrap().0, Action::Reconfigure);
}

#[test]
fn asymmetric_updates_preserve_the_active_channel_set() {
    use Action::*;
    let limit = DutyLimit::new(70);
    let cases = [
        ([25, 0], [25, 0], Unchanged),
        ([0, -35], [0, -35], Unchanged),
        ([25, -35], [25, -35], Unchanged),
        ([25, 0], [40, 0], UpdateCompare([true, false])),
        ([0, -35], [0, -45], UpdateCompare([false, true])),
        ([25, -35], [40, -35], UpdateCompare([true, false])),
        ([25, -35], [25, -45], UpdateCompare([false, true])),
        ([25, 0], [25, -35], Reconfigure),
        ([0, -35], [25, -35], Reconfigure),
        ([25, -35], [25, 0], Reconfigure),
        ([25, -35], [0, -35], Reconfigure),
        ([25, 0], [0, -35], Reconfigure),
        ([0, -35], [25, 0], Reconfigure),
    ];
    for (old, requested, expected) in cases {
        let mut state = CommandState::default();
        state.arm();
        let (_, effective) = state.plan(limit, old).unwrap();
        state.record(effective);
        assert_eq!(
            state.plan(limit, requested).unwrap().0,
            expected,
            "{old:?} -> {requested:?}"
        );
    }
}
