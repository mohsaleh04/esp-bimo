//! Hardware-independent selection of the TB6612 output path.
use super::control::{CommandGate, Disarmed, Drive, DutyLimit};

#[derive(Debug, PartialEq, Eq)]
pub(super) enum Action {
    Stop,
    Unchanged,
    UpdateCompare([bool; 2]),
    Reconfigure,
}

#[derive(Default)]
pub(super) struct CommandState {
    gate: CommandGate,
    last: Option<[Drive; 2]>,
}

impl CommandState {
    pub(super) fn arm(&mut self) {
        self.stop();
        self.gate.arm();
    }

    pub(super) fn stop(&mut self) {
        self.last = None;
    }

    pub(super) fn disarm(&mut self) {
        self.stop();
        self.gate.disarm();
    }

    pub(super) fn plan(
        &self,
        limit: DutyLimit,
        raw: [i16; 2],
    ) -> Result<(Action, [Drive; 2]), Disarmed> {
        self.gate.check(raw[0], raw[1])?;
        let next = raw.map(|command| limit.map(command));
        let active_channels = next.map(|drive| drive.duty_percent != 0);
        let action = match self.last {
            _ if active_channels == [false, false] => Action::Stop,
            Some(last) if last == next => Action::Unchanged,
            Some(last)
                // Starting or stopping either channel requires rerouting PWM.
                if last.map(|drive| drive.duty_percent != 0) == active_channels
                    && last.iter().zip(&next).all(|(old, new)| {
                        old.direction == new.direction
                            && old.duty_percent < 100
                            && new.duty_percent < 100
                    }) =>
            {
                Action::UpdateCompare([
                    last[0].duty_percent != next[0].duty_percent,
                    last[1].duty_percent != next[1].duty_percent,
                ])
            }
            _ => Action::Reconfigure,
        };
        Ok((action, next))
    }

    /// Record the accepted register request, which can still be pending at TEZ.
    pub(super) fn record(&mut self, effective: [Drive; 2]) {
        self.last = Some(effective);
    }
}
