//! Command interlock only: zero PWM is NOT evidence of zero wheel speed.
use super::policy::{self, Output, Response, ResponseState};
use crate::{
    balance,
    motors::{Disarmed, DualMotor},
};
/// Bidirectional profiles share this override; other profiles keep the base threshold.
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
/// Shared electrical command guard, independent of any startup-pulse budget.
#[derive(Default)]
pub struct ZeroInterlock {
    direction: i16,
    pending: i16,
    zero_since: Option<u64>,
}
impl ZeroInterlock {
    pub fn approve(&mut self, direction: i16, now: u64) -> bool {
        if direction != 0 && self.direction != 0 && direction != self.direction {
            if self.pending != direction {
                self.pending = direction;
                self.zero_since = None;
            }
            return self
                .zero_since
                .is_some_and(|t| now.saturating_sub(t) >= DEAD_TIME_US);
        }
        if self.pending != 0 {
            self.clear_pending();
        }
        true
    }
    pub fn acknowledge(&mut self, pwm: i16, completed_us: u64) {
        if pwm == 0 {
            if self.pending != 0 && self.zero_since.is_none() {
                self.zero_since = Some(completed_us);
            }
        } else {
            self.direction = pwm.signum();
            self.clear_pending();
        }
    }
    #[allow(dead_code)] // Used by the integrated arbiter; standalone legacy harness also compiles this module.
    pub fn needs_zero_ack(&self) -> bool {
        self.pending != 0 && self.zero_since.is_none()
    }
    pub fn clear_pending(&mut self) {
        self.pending = 0;
        self.zero_since = None;
    }
    pub fn telemetry(&self, now: u64, kicks: u8, kicking: bool) -> Telemetry {
        Telemetry {
            pending_direction: self.pending,
            zero_elapsed_us: self.zero_since.map_or(0, |t| now.saturating_sub(t)),
            kicks,
            kick_applied: kicking,
        }
    }
}
#[derive(Default)]
pub struct Interlock {
    zero: ZeroInterlock,
    first_kick: bool,
    kick_closed: bool,
    pub kicks: u8,
    pub kicking: bool,
    pub applied: (i16, i16),
}
impl Interlock {
    pub fn telemetry(&self, now: u64) -> Telemetry {
        self.zero.telemetry(now, self.kicks, self.kicking)
    }
    pub fn reset(&mut self) {
        self.zero.clear_pending();
        self.kicking = false;
        // Preserve the one-Kick budget, but pre-arming empty polls consume nothing.
        self.kick_closed |= self.first_kick || self.zero.direction != 0;
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
            self.kick_closed |= self.first_kick || self.zero.direction != 0;
            self.zero.clear_pending();
            return;
        }
        if self.first_kick && !output.preview.kick_active {
            self.kick_closed = true;
        }
        if self.zero.direction != 0 && request != self.zero.direction {
            self.kick_closed |= self.first_kick || self.zero.direction != 0;
        }
        if !self.zero.approve(request, now) {
            output.preview.final_pwm = 0.0;
            output.preview.kick_active = false;
            self.kicking = false;
            return;
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
        self.zero.acknowledge(pwm as i16, completed_us);
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
            self.kick_closed |= self.first_kick || self.zero.direction != 0;
            self.kicking = false;
            self.zero.clear_pending();
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
