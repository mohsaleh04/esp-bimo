//! Fixed sequence only: no balancing, compensation, or repeated stall attempts.
use crate::{Disarmed, DualMotor, balance};
pub const PREPARATION_MS: u64 = 15_000;
pub const ON_MS: u64 = 1_000;
pub const PAUSE_MS: u64 = 5_000;
pub const TOTAL_MS: u64 = 100_000;
pub const CEILING: u8 = 60;

/// Both motors together: forward 40/50/60%, then backward 40/50/60%.
pub fn step(index: usize) -> Option<(bool, u8)> {
    (index < 6).then(|| (index < 3, [40, 50, 60][index % 3]))
}
pub fn shutdown(motors: &mut impl DualMotor) {
    motors.stop();
    motors.disarm();
}
pub fn apply(
    motors: &mut impl DualMotor,
    index: usize,
    live: bool,
    elapsed_ms: u64,
) -> Result<(), Disarmed> {
    shutdown(motors);
    let Some((forward, duty)) = step(index) else {
        return Err(Disarmed);
    };
    if elapsed_ms >= TOTAL_MS || duty > CEILING {
        return Err(Disarmed);
    }
    if !live {
        return Ok(());
    }
    let u = if forward {
        i16::from(duty)
    } else {
        -i16::from(duty)
    };
    let (left, right) = balance::wheel_commands(u);
    motors.arm();
    let result = motors.command(left, right);
    if result.is_err() {
        shutdown(motors);
    }
    result
}
