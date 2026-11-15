//! Bounded continuous-control profile. Response owns ALL arming authority.
use super::{
    bidirectional,
    policy::{Response, ResponseState},
    settings,
};
use crate::{balance, motors::DualMotor};
pub const ACTIVE_US: u64 = 90_000_000;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Calibrating,
    Preparing,
    WaitingForReady,
    ArmedWaitingForNeutral,
    Active,
    Disarmed,
    DryRun,
}
impl Stage {
    pub fn from_response(state: ResponseState) -> Self {
        match state {
            ResponseState::WaitingForReady => Self::WaitingForReady,
            ResponseState::ArmedWaitingForNeutral => Self::ArmedWaitingForNeutral,
            ResponseState::Active => Self::Active,
            ResponseState::Disarmed => Self::Disarmed,
            ResponseState::DryRun => Self::DryRun,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Calibrating => "CALIBRATING",
            Self::Preparing => "PREPARING",
            Self::WaitingForReady => "WAITING_FOR_READY",
            Self::ArmedWaitingForNeutral => "ARMED_WAITING_FOR_NEUTRAL",
            Self::Active => "ACTIVE",
            Self::Disarmed => "DISARMED",
            Self::DryRun => "DRY_RUN (motors disabled)",
        }
    }
}
pub fn response(live: bool) -> Option<Response> {
    response_with_restart(live, false)
}
pub fn response_with_restart(live: bool, restart: bool) -> Option<Response> {
    let mut kick = bidirectional::kick_config(settings::KICK);
    if restart {
        kick.enabled = false;
    } // One shared authority; no separate initial Kick.
    Response::new(
        balance::Settings {
            output_limit: settings::PD_OUTPUT_LIMIT,
            ..Default::default()
        },
        settings::MAPPING,
        kick,
        settings::MOTOR_DUTY_CEILING,
        live,
        ACTIVE_US,
        settings::ARMING_TIMEOUT_US,
    )?
    .with_tilt_limit(balance::CONTROL_FALL_ANGLE_DEG)
}
/// Stricter new-profile policy: faults also latch before readiness. DRY is observation only.
pub fn enforce(response: &mut Response, motors: &mut impl DualMotor, late: bool) {
    if response.state() != ResponseState::DryRun && !response.finished {
        if late {
            response.abort(balance::Fault::Timing);
        } else if response.sensor_fault.is_some() {
            response.abort(balance::Fault::Invalid);
        } else if let balance::State::Fault(fault) = response.control.state() {
            response.abort(fault);
        }
    }
    if response.finished {
        motors.disarm();
    }
}
/// Software-observed metrics, NOT wheel feedback. Update every loop; provide
/// pitch only once per fresh estimate. Intervals use the PREVIOUS held command.
#[derive(Default, Debug)]
pub struct Behavior {
    pub samples: u32,
    pub zero_crossings: u32,
    pub max_abs_pitch: f32,
    pub elapsed_us: u64,
    pub zero_us: u64,
    pub longest_zero_us: u64,
    previous_time: Option<u64>,
    previous_enabled: bool,
    previous_zero: bool,
    zero_run_us: u64,
    pitch_sign: i16,
}
impl Behavior {
    pub fn update(
        &mut self,
        now: u64,
        state: ResponseState,
        pitch: Option<f32>,
        applied: (i16, i16),
    ) {
        if self.previous_enabled {
            let dt = self.previous_time.map_or(0, |t| now.saturating_sub(t));
            self.elapsed_us += dt;
            if self.previous_zero {
                self.zero_us += dt;
                self.zero_run_us += dt;
                self.longest_zero_us = self.longest_zero_us.max(self.zero_run_us);
            } else {
                self.zero_run_us = 0;
            }
        }
        let enabled = matches!(state, ResponseState::Active | ResponseState::DryRun);
        if (enabled || self.previous_enabled)
            && let Some(pitch) = pitch.filter(|p| p.is_finite())
        {
            self.samples += 1;
            self.max_abs_pitch = self.max_abs_pitch.max(pitch.abs());
            let sign = if pitch > 0.0 {
                1
            } else if pitch < 0.0 {
                -1
            } else {
                0
            };
            if sign != 0 {
                if self.pitch_sign != 0 && self.pitch_sign != sign {
                    self.zero_crossings += 1;
                }
                self.pitch_sign = sign;
            }
        }
        self.previous_time = Some(now);
        self.previous_enabled = enabled;
        self.previous_zero = applied == (0, 0);
    }
}
