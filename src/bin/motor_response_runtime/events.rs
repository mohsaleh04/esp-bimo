//! Fixed-memory flight recorder. Overwrites old events, never the terminal slot.
use super::policy::{DisarmReason, ResponseState, SensorFault};
use crate::orientation::AccelRejection;
#[allow(dead_code)] // Fields are deliberately retained for the deferred Debug dump.
#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub us: u64,
    pub sample_us: Option<u64>,
    pub state: ResponseState,
    pub pitch: Option<f32>,
    pub rate: Option<f32>,
    pub pd: Option<i16>,
    pub mapped: Option<f32>,
    pub kick: bool,
    pub kick_ms: Option<u64>,
    pub requested: (i16, i16),
    pub reversal: Option<super::bidirectional::Telemetry>,
    pub commanded: (i16, i16),
    pub last_energized: (u64, i16, i16),
    pub accel_used: Option<bool>,
    pub rejection: Option<AccelRejection>,
    pub correction_age_us: Option<u64>,
    pub sensor_fault: Option<SensorFault>,
    pub disarm: Option<DisarmReason>,
    pub loop_us: u64,
    pub work_us: u64,
    pub missed: u32,
}
// Bit flags allow simultaneous sensor/kick/command/state changes in one event.
pub const SENSOR: u8 = 1;
pub const KICK: u8 = 2;
pub const COMMAND: u8 = 4;
pub const STATE: u8 = 8;
pub const FAULT: u8 = 16;
#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub flags: u8,
    pub sample: Snapshot,
}
pub struct EventLog<const N: usize> {
    ring: [Option<Event>; N],
    cursor: usize,
    count: usize,
    previous: Option<Snapshot>,
    accel_used: Option<bool>,
    pub overwritten: u32,
    pub kick_starts: u32,
    pub kick_ends: u32,
    pub reversals: u32,
    requested_direction: i16,
    pub command_changes: u32,
    pub zero_transitions: u32,
    pub last_rejection: Option<Snapshot>,
    pub rejection_onset: Option<Snapshot>,
    pub last_kick_on: Option<Snapshot>,
    pub last_kick_off: Option<Snapshot>,
    pub terminal: Option<Snapshot>,
    pub before_terminal: Option<Snapshot>,
    pub first_applied: Option<Snapshot>,
    pub last_interlock_zero: Option<Snapshot>,
    pub last_interlock_release: Option<Snapshot>,
    pub last_applied_kick_on: Option<Snapshot>,
    pub last_applied_kick_off: Option<Snapshot>,
}
impl<const N: usize> EventLog<N> {
    pub fn new() -> Self {
        assert!(N > 0);
        Self {
            ring: [None; N],
            cursor: 0,
            count: 0,
            previous: None,
            accel_used: None,
            overwritten: 0,
            kick_starts: 0,
            kick_ends: 0,
            reversals: 0,
            requested_direction: 0,
            command_changes: 0,
            zero_transitions: 0,
            last_rejection: None,
            rejection_onset: None,
            last_kick_on: None,
            last_kick_off: None,
            terminal: None,
            before_terminal: None,
            first_applied: None,
            last_interlock_zero: None,
            last_interlock_release: None,
            last_applied_kick_on: None,
            last_applied_kick_off: None,
        }
    }
    pub fn observe(&mut self, s: Snapshot) {
        if self.terminal.is_some() {
            return;
        } // Never overwrite the fault context.
        let mut flags = 0;
        if s.reversal.is_some() && s.commanded != (0, 0) && self.first_applied.is_none() {
            self.first_applied = Some(s);
        }
        if let Some(r) = s.reversal {
            let previous = self.previous.and_then(|p| p.reversal);
            if r.pending_direction != 0
                && previous.is_none_or(|p| p.pending_direction != r.pending_direction)
            {
                self.last_interlock_zero = Some(s);
            }
            if r.pending_direction == 0
                && previous.is_some_and(|p| p.pending_direction != 0)
                && s.commanded != (0, 0)
            {
                self.last_interlock_release = Some(s);
            }
            if r.kick_applied && previous.is_none_or(|p| !p.kick_applied) {
                self.last_applied_kick_on = Some(s);
            } else if !r.kick_applied && previous.is_some_and(|p| p.kick_applied) {
                self.last_applied_kick_off = Some(s);
            }
        }
        if let Some(used) = s.accel_used {
            if self.accel_used != Some(used) {
                flags |= SENSOR;
                if !used {
                    self.rejection_onset = Some(s);
                }
            }
            self.accel_used = Some(used);
        }
        if s.rejection.is_some()
            && s.sample_us.is_some()
            && self.previous.is_none_or(|p| p.sample_us != s.sample_us)
        {
            self.last_rejection = Some(s);
        }
        if let Some(p) = self.previous {
            if p.kick != s.kick {
                flags |= KICK;
                if s.kick {
                    self.kick_starts = self.kick_starts.saturating_add(1);
                    self.last_kick_on = Some(s);
                } else {
                    self.kick_ends = self.kick_ends.saturating_add(1);
                    self.last_kick_off = Some(s);
                }
            }
            if p.commanded != s.commanded || p.requested != s.requested {
                flags |= COMMAND;
                if p.commanded != s.commanded {
                    self.command_changes = self.command_changes.saturating_add(1);
                    if (p.commanded == (0, 0)) != (s.commanded == (0, 0)) {
                        self.zero_transitions = self.zero_transitions.saturating_add(1);
                    }
                }
            }
            if p.reversal.map(|r| r.transition_key()) != s.reversal.map(|r| r.transition_key()) {
                flags |= STATE;
            }
            if p.state != s.state {
                flags |= STATE;
            }
            if p.sensor_fault != s.sensor_fault {
                flags |= SENSOR;
            }
        } else {
            flags |= STATE;
            if s.kick {
                flags |= KICK;
                self.kick_starts = 1;
                self.last_kick_on = Some(s);
            }
        }
        if s.requested.0 != 0 {
            let direction = s.requested.0.signum();
            if self.requested_direction != 0 && direction != self.requested_direction {
                self.reversals = self.reversals.saturating_add(1);
            }
            self.requested_direction = direction;
        }
        if s.state == ResponseState::Disarmed {
            flags |= FAULT;
            self.terminal = Some(s);
            self.before_terminal = self.previous;
        }
        if flags != 0 {
            if self.count == N {
                self.overwritten = self.overwritten.saturating_add(1);
            } else {
                self.count += 1;
            }
            self.ring[self.cursor] = Some(Event { flags, sample: s });
            self.cursor = (self.cursor + 1) % N;
        }
        self.previous = Some(s);
    }
    pub fn recent(&self) -> impl Iterator<Item = Event> + '_ {
        let start = if self.count == N { self.cursor } else { 0 };
        (0..self.count).filter_map(move |i| self.ring[(start + i) % N])
    }
}
