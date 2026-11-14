//! Command interlock only: zero PWM is NOT evidence of zero wheel speed.
use super::policy::{self, Output, Response, ResponseState};
use crate::{
    balance,
    motors::{Disarmed, DualMotor},
};
/// This profile alone may customize the shared Kick configuration.
pub fn kick_config(kick: crate::pwm_mapping::KickConfig) -> crate::pwm_mapping::KickConfig {
    crate::pwm_mapping::KickConfig {
        trigger_threshold: 3.0, // First integer PD request outside the +/-2 deadband.
        ..kick
    }
}
pub const DEAD_TIME_US: u64 = 100_000;
#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    pub pending_direction: i16,
    #[allow(dead_code)] // Retained in deferred Debug flight-recorder output.
    pub zero_elapsed_us: u64,
    pub kicks: u8,
    pub kick_applied: bool,
}
impl Telemetry {
    pub fn transition_key(self) -> (i16, u8, bool) {
        (self.pending_direction, self.kicks, self.kick_applied)
    }
}
#[derive(Default)]
pub struct Interlock {
    direction: i16,
    pending: i16,
    zero_since: Option<u64>,
    first_kick: bool,
    kick_closed: bool,
    pub kicks: u8,
    pub kicking: bool,
    pub applied: (i16, i16),
}
impl Interlock {
    pub fn telemetry(&self, now: u64) -> Telemetry {
        Telemetry {
            pending_direction: self.pending,
            zero_elapsed_us: self.zero_since.map_or(0, |t| now.saturating_sub(t)),
            kicks: self.kicks,
            kick_applied: self.kicking,
        }
    }
    pub fn reset(&mut self) {
        self.pending = 0;
        self.zero_since = None;
        self.kicking = false;
        // Preserve the one-Kick budget, but pre-arming empty polls consume nothing.
        self.kick_closed |= self.first_kick || self.direction != 0;
        self.applied = (0, 0);
    }
    fn plan(&mut self, output: &mut Output, now: u64) {
        let request = output.preview.mapped_pwm.signum() as i16;
        let request = if output.preview.mapped_pwm == 0.0 {
            0
        } else {
            request
        };
        if request == 0 {
            output.preview.final_pwm = 0.0;
            output.preview.kick_active = false;
            self.kicking = false;
            self.kick_closed |= self.first_kick || self.direction != 0;
            self.pending = 0;
            self.zero_since = None;
            return;
        }
        if self.first_kick && !output.preview.kick_active {
            self.kick_closed = true;
        }
        if self.direction != 0 && request != self.direction {
            self.kick_closed |= self.first_kick || self.direction != 0;
        }
        if request != 0 && self.direction != 0 && request != self.direction {
            if self.pending != request {
                self.pending = request;
                self.zero_since = None;
            }
            if self
                .zero_since
                .is_none_or(|t| now.saturating_sub(t) < DEAD_TIME_US)
            {
                output.preview.final_pwm = 0.0;
                output.preview.kick_active = false;
                self.kicking = false;
                return;
            }
        } else if self.pending != 0 {
            // Chatter cancels the pending reversal. A new opposite request must wait again.
            self.pending = 0;
            self.zero_since = None;
        }
        if output.preview.kick_active && !self.kick_closed {
            if !self.first_kick {
                self.first_kick = true;
            }
        } else if self.kick_closed && output.preview.kick_active {
            output.preview.final_pwm = output.preview.mapped_pwm;
            output.preview.kick_active = false;
        }
        self.kicking = output.preview.kick_active;
    }
    /// Called ONLY after the driver accepted this command. Timestamp is after IO.
    fn acknowledged(&mut self, pwm: f32, completed_us: u64) {
        self.applied = balance::wheel_commands(pwm as i16);
        if self.kicking && self.applied != (0, 0) && self.kicks == 0 {
            self.kicks = 1; // Count only an accepted nonzero Kick command.
        }
        if self.applied == (0, 0) {
            if self.pending != 0 && self.zero_since.is_none() {
                self.zero_since = Some(completed_us);
            }
        } else {
            self.direction = (pwm as i16).signum();
            self.pending = 0;
            self.zero_since = None;
        }
    }
    pub fn drive(
        &mut self,
        motors: &mut impl DualMotor,
        response: &mut Response,
        mut output: Option<Output>,
        ceiling: u8,
        mut clock: impl FnMut() -> u64,
    ) -> Result<(), Disarmed> {
        let now = clock();
        response.tick(now);
        if response.state() != ResponseState::Active || response.finished {
            motors.disarm();
            self.reset();
            return Ok(());
        }
        if output.is_some_and(|o| {
            !o.preview.mapped_pwm.is_finite()
                || !o.preview.final_pwm.is_finite()
                || o.preview.mapped_pwm.abs() > f32::from(ceiling)
                || o.preview.final_pwm.abs() > f32::from(ceiling)
        }) {
            response.fail(balance::Fault::Motor);
            motors.disarm();
            self.reset();
            return Err(Disarmed);
        }
        if let Some(o) = output.as_mut() {
            self.plan(o, now);
        } else {
            // Neutral acceptance has no Output yet: do not consume the first Kick.
            self.kick_closed |= self.first_kick || self.direction != 0;
            self.kicking = false;
            self.pending = 0;
            self.zero_since = None;
        }
        let result = policy::drive(motors, response, output, ceiling, now);
        if result.is_err() || response.finished {
            response.fail(balance::Fault::Motor);
            motors.disarm();
            self.reset();
        } else {
            self.acknowledged(output.map_or(0.0, |o| o.preview.final_pwm), clock());
        }
        result
    }
    /// Mathematical DRY preview, never receives or owns a hardware driver.
    pub fn preview(&mut self, mut output: Output, now: u64) {
        self.plan(&mut output, now);
        self.acknowledged(output.preview.final_pwm, now);
    }
}
