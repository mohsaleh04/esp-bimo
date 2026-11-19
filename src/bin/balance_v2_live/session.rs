//! One-shot motor authority for the V2 feedback loop.
use super::{controller::Fault, motor_control::DualMotor};
use crate::balance;

pub(super) const MOTOR_CEILING: u8 = super::controller::PWM_CEILING;
const ACTIVE_US: u64 = 180_000_000;
const ARMING_US: u64 = 10_000_000;
const REVERSAL_ZERO_US: u64 = 100_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Stop {
    ArmingTimeout,
    ActiveTimeout,
    Sensor(Fault),
    Motor,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum State {
    WaitingUpright,
    WaitingNeutral,
    Active,
    Finished(Stop),
}

pub(super) struct Session<'a, M: DualMotor> {
    motors: &'a mut M,
    state: State,
    started_at: u64,
    last_time: u64,
    upright_since: Option<u64>,
    active_since: u64,
    last_direction: i16,
    zero_since: Option<u64>,
    #[cfg(test)]
    last_command: i16,
}

impl<'a, M: DualMotor> Session<'a, M> {
    pub fn new(motors: &'a mut M, now: u64) -> Self {
        motors.disarm();
        Self {
            motors,
            state: State::WaitingUpright,
            started_at: now,
            last_time: now,
            upright_since: None,
            active_since: 0,
            last_direction: 0,
            zero_since: None,
            #[cfg(test)]
            last_command: 0,
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    #[cfg(test)]
    pub fn last_command(&self) -> i16 {
        self.last_command
    }

    pub fn finish(&mut self, reason: Stop) -> Stop {
        if let State::Finished(previous) = self.state {
            return previous;
        }
        self.motors.disarm();
        self.state = State::Finished(reason);
        reason
    }

    /// Called even on polls with no fresh IMU sample.
    pub fn tick(&mut self, now: u64) -> Result<(), Stop> {
        if let State::Finished(reason) = self.state {
            return Err(reason);
        }
        if now < self.last_time {
            return Err(self.finish(Stop::Sensor(Fault::Timing)));
        }
        self.last_time = now;
        if self.state == State::Active {
            if now - self.active_since >= ACTIVE_US {
                return Err(self.finish(Stop::ActiveTimeout));
            }
        } else if now - self.started_at >= ARMING_US {
            return Err(self.finish(Stop::ArmingTimeout));
        }
        Ok(())
    }

    /// Accept only fresh estimates already checked by the V2 controller.
    pub fn observe(
        &mut self,
        pitch: f32,
        rate: f32,
        corrected: bool,
        wheels: [i16; 2],
        mut clock: impl FnMut() -> u64,
        before_arm: impl FnOnce(),
    ) -> Result<(), Stop> {
        let now = clock();
        self.tick(now)?;
        let [left, right] = wheels;
        if left != right || left.unsigned_abs() > u16::from(MOTOR_CEILING) {
            return Err(self.finish(Stop::Motor));
        }

        if self.state != State::Active {
            let upright = corrected
                && pitch.abs() <= balance::ARM_ANGLE_DEG
                && rate.abs() <= balance::ARM_RATE_DPS;
            if !upright {
                self.upright_since = None;
                self.state = State::WaitingUpright;
                return Ok(());
            }
            if self.state == State::WaitingUpright {
                let since = *self.upright_since.get_or_insert(now);
                if now - since >= balance::UPRIGHT_DWELL_US {
                    self.state = State::WaitingNeutral;
                }
                return Ok(());
            }
            if left != 0 {
                return Ok(());
            }
            before_arm();
            self.motors.arm();
            self.active_since = now;
            self.state = State::Active;
        }

        // On a requested reversal, coast for at least 100 ms from a completed
        // zero command. The timer measures electrical coast, not wheel speed.
        let opposite =
            left != 0 && self.last_direction != 0 && left.signum() != self.last_direction;
        let pwm = if opposite
            && self
                .zero_since
                .is_none_or(|since| now - since < REVERSAL_ZERO_US)
        {
            0
        } else {
            left
        };
        let (a, b) = balance::wheel_commands(pwm);
        if self.motors.command(a, b).is_err() {
            return Err(self.finish(Stop::Motor));
        }
        let completed = clock();
        self.tick(completed)?;
        if pwm == 0 {
            self.zero_since.get_or_insert(completed);
        } else {
            self.zero_since = None;
            self.last_direction = pwm.signum();
        }
        #[cfg(test)]
        {
            self.last_command = pwm;
        }
        Ok(())
    }
}
