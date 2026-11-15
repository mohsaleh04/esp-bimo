//! One startup/hold pair per stage; no retry, feedback control or assist budget.
use crate::{Disarmed, DualMotor, balance};
pub const PREPARATION_MS: u64 = 15_000;
pub const STARTUP_MS: u64 = 100;
pub const HOLD_MS: u64 = 1_000;
pub const PAUSE_MS: u64 = 5_000;
pub const TOTAL_MS: u64 = 100_000;
pub const CEILING: u8 = 70;
#[derive(Clone, Copy, Debug)]
pub enum Phase {
    Startup,
    Hold,
}
pub fn step(index: usize) -> Option<(bool, u8, u8)> {
    (index < 6).then(|| {
        (
            index < 3,
            if index < 3 { 70 } else { 65 },
            [50, 40, 30][index % 3],
        )
    })
}
pub fn shutdown(motors: &mut impl DualMotor) {
    motors.stop();
    motors.disarm();
}
pub fn apply(
    motors: &mut impl DualMotor,
    index: usize,
    phase: Phase,
    live: bool,
    elapsed_ms: u64,
) -> Result<(i16, i16), Disarmed> {
    let Some((forward, startup, hold)) = step(index) else {
        shutdown(motors);
        return Err(Disarmed);
    };
    if !live || elapsed_ms >= TOTAL_MS {
        shutdown(motors);
        return if live { Err(Disarmed) } else { Ok((0, 0)) };
    }
    let duty = match phase {
        Phase::Startup => startup,
        Phase::Hold => hold,
    };
    if duty > CEILING {
        shutdown(motors);
        return Err(Disarmed);
    }
    let u = if forward {
        i16::from(duty)
    } else {
        -i16::from(duty)
    };
    let outputs = balance::wheel_commands(u);
    if matches!(phase, Phase::Startup) {
        shutdown(motors);
        motors.arm();
    }
    // Hold neither stops nor rearms: same direction, one paired command.
    if let Err(error) = motors.command(outputs.0, outputs.1) {
        shutdown(motors);
        return Err(error);
    }
    Ok(outputs)
}
