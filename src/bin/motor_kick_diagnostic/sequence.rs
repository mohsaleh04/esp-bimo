//! Small testable sequence; no HAL or timing dependencies.
use crate::{Disarmed, DualMotor};

#[derive(Clone, Copy)]
pub struct Settings {
    pub ceiling: u8,
    pub kick_duty: u8,
    pub kick_ms: u64,
    pub hold_duty: u8,
    pub hold_ms: u64,
    pub repetitions: u8,
}
impl Settings {
    pub fn valid(self) -> bool {
        self.ceiling > 0
            && self.ceiling <= 100
            && self.kick_duty > 0
            && self.kick_duty <= self.ceiling
            && self.hold_duty > 0
            && self.hold_duty <= self.ceiling
            && self.kick_ms > 0
            && self.kick_ms <= 250
            && self.hold_ms > 0
            && self.hold_ms <= 2_000
            && self.repetitions > 0
            && self.repetitions <= 5
    }
    pub fn phases(self) -> [(Phase, u64); 3] {
        [
            (Phase::Kick(self.kick_duty), self.kick_ms),
            (Phase::Hold(self.hold_duty), self.hold_ms),
            (Phase::Stop, 0),
        ]
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Kick(u8),
    Hold(u8),
    Stop,
}

/// Reject out-of-range duties rather than silently clamping. Stop is unconditional.
pub fn apply(
    motors: &mut impl DualMotor,
    right: bool,
    phase: Phase,
    ceiling: u8,
    live: bool,
) -> Result<(), Disarmed> {
    let duty = match phase {
        Phase::Stop => {
            motors.stop();
            motors.disarm();
            return Ok(());
        }
        Phase::Kick(d) | Phase::Hold(d) => d,
    };
    if duty == 0 || duty > ceiling || ceiling == 0 || ceiling >= 100 {
        motors.disarm();
        return Err(Disarmed);
    }
    if !live {
        motors.disarm();
        return Ok(());
    }
    if matches!(phase, Phase::Kick(_)) {
        motors.arm();
    }
    let d = i16::from(duty);
    let (a, b) = if right { (-d, 0) } else { (0, d) };
    if let Err(error) = motors.command(a, b) {
        motors.disarm();
        return Err(error);
    }
    Ok(())
}
