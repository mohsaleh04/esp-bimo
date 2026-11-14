//! Additional restraints for ONE tethered response. Never used by suspended tests.
use crate::pwm_mapping::Preview;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroundStop {
    ReversalRequested,
    AdditionalKick,
}
#[derive(Default)]
pub struct GroundGuard {
    direction: i16,
    was_kicking: bool,
    kicks: u8,
    stopped: Option<GroundStop>,
}
impl GroundGuard {
    pub fn stop_reason(&self) -> Option<GroundStop> {
        self.stopped
    }
    pub fn check(&mut self, pd: i16, preview: Preview, deadband: f32) -> Option<GroundStop> {
        if self.stopped.is_some() {
            return self.stopped;
        }
        if f32::from(pd).abs() > deadband {
            let sign = pd.signum();
            if self.direction != 0 && sign != self.direction {
                self.stopped = Some(GroundStop::ReversalRequested);
                return self.stopped;
            }
            if preview.final_pwm != 0.0 {
                self.direction = sign;
            }
        }
        if preview.kick_active && !self.was_kicking {
            if self.kicks != 0 {
                self.stopped = Some(GroundStop::AdditionalKick);
                return self.stopped;
            }
            self.kicks += 1;
        }
        self.was_kicking = preview.kick_active;
        None
    }
}
/// Source assertions are operator commitments, not measurements of physical safety.
#[allow(dead_code)] // Used by the ground wrapper; shared runtime also builds for suspended tests.
pub fn live_allowed(
    feature: bool,
    restraint: bool,
    cutoff: bool,
    supply: bool,
    sensor_review: bool,
) -> bool {
    feature && restraint && cutoff && supply && sensor_review
}

pub fn window_valid(window_us: u64) -> bool {
    window_us > 0 && window_us <= 30_000_000
}

/// Ground-only final gate: a veto/output error both latches policy AND calls the
/// physical driver. Callers cannot accidentally treat a rejection as recoverable.
pub fn drive(
    motors: &mut impl crate::motors::DualMotor,
    response: &mut super::policy::Response,
    guard: &mut GroundGuard,
    output: Option<super::policy::Output>,
    ceiling: u8,
    deadband: f32,
    now: u64,
) -> Result<(), crate::motors::Disarmed> {
    use super::policy::{self, ResponseState};
    response.tick(now);
    if response.state() != ResponseState::Active || response.finished {
        motors.disarm();
        return Ok(());
    }
    if guard.stop_reason().is_some()
        || output.is_some_and(|o| guard.check(o.limited_pd, o.preview, deadband).is_some())
    {
        response.fail(crate::balance::Fault::Motor);
        motors.disarm();
        return Err(crate::motors::Disarmed);
    }
    if let Err(error) = policy::drive(motors, response, output, ceiling, now) {
        response.fail(crate::balance::Fault::Motor);
        motors.disarm();
        return Err(error);
    }
    Ok(())
}

/// Deadline detection at any ground loop checkpoint stops the real outputs now.
pub fn deadline(
    motors: &mut impl crate::motors::DualMotor,
    response: &mut super::policy::Response,
    late: bool,
) -> bool {
    if late && response.supervising() {
        response.fail(crate::balance::Fault::Timing);
        motors.disarm();
        true
    } else {
        false
    }
}

/// Ground LIVE rejects unsafe estimates permanently even BEFORE readiness.
/// DRY keeps its intentional mathematical preview behavior. Calibration and
/// preparation never call this path and keep the driver disarmed throughout.
pub fn sample(
    motors: &mut impl crate::motors::DualMotor,
    response: &mut super::policy::Response,
    pitch: f32,
    rate: f32,
    accel_used: bool,
    dt: f32,
    now: u64,
) -> Option<super::policy::Output> {
    use super::policy::{ResponseState, SensorFault};
    response.tick(now);
    if !response.finished && response.state() != ResponseState::DryRun {
        if !pitch.is_finite() || !rate.is_finite() {
            response.sensor_fault = Some(SensorFault::NonFiniteEstimate);
            response.abort(crate::balance::Fault::Invalid);
        } else if pitch.abs() >= crate::balance::FALL_ANGLE_DEG {
            response.abort(crate::balance::Fault::Tilt);
        }
    }
    if response.finished {
        motors.disarm();
        return None;
    }
    let output = response.sample(pitch, rate, accel_used, dt, now);
    if response.finished {
        motors.disarm();
    }
    output
}
