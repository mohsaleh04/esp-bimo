//! Driver-independent signed duty commands. Channels are A/B, never left/right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Stopped,
    Positive,
    Negative,
}

impl Direction {
    /// Logical input pair; both LOW means coast when enable is inactive.
    pub const fn inputs(self) -> (bool, bool) {
        match self {
            Self::Stopped => (false, false),
            Self::Positive => (true, false),
            Self::Negative => (false, true),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Drive {
    pub direction: Direction,
    pub duty_percent: u8,
}

/// Absolute duty ceiling, not a multiplier. Commands use signed percent units.
#[derive(Debug, Clone, Copy)]
pub struct DutyLimit(u8);

impl DutyLimit {
    pub const fn new(percent: u8) -> Option<Self> {
        if percent <= 100 {
            Some(Self(percent))
        } else {
            None
        }
    }

    pub fn map(self, command: i16) -> Drive {
        let duty = command.unsigned_abs().min(self.0 as u16) as u8;
        Drive {
            direction: if duty == 0 {
                Direction::Stopped
            } else if command > 0 {
                Direction::Positive
            } else {
                Direction::Negative
            },
            duty_percent: duty,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disarmed;

#[derive(Default)]
pub struct CommandGate {
    armed: bool,
}

impl CommandGate {
    pub fn arm(&mut self) {
        self.armed = true;
    }
    pub fn disarm(&mut self) {
        self.armed = false;
    }
    pub fn check(&self, a: i16, b: i16) -> Result<(), Disarmed> {
        if self.armed || (a == 0 && b == 0) {
            Ok(())
        } else {
            Err(Disarmed)
        }
    }
}

/// Common interface for L298N and future bridge implementations.
/// Positive/negative are electrical polarity; physical forward is uncalibrated.
pub trait DualMotor {
    fn arm(&mut self);
    fn command(&mut self, a: i16, b: i16) -> Result<(), Disarmed>;
    /// Coast/de-energize both bridges; remains armed for subsequent commands.
    fn stop(&mut self);
    /// Stop and reject further nonzero commands until explicitly armed again.
    fn disarm(&mut self);
}
