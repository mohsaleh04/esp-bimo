//! Pure DRY controller. Its output is telemetry, with no motor handle.
use super::motor_control::{Channel, DutyLimit};
use crate::{balance, pwm_mapping};

const PD: balance::Settings = balance::Settings {
    kp: 1.2,
    kd: 0.08,
    output_limit: 10,
};
pub(super) const PWM_CEILING: u8 = 45;
const PWM: pwm_mapping::Config = pwm_mapping::Config {
    enabled: true,
    deadband: 1.0,
    pd_output_limit: PD.output_limit as f32,
    minimum_duty: 10.0,
    maximum_duty: PWM_CEILING as f32,
    slew_per_second: None,
};
const MAX_CORRECTION_AGE_US: u64 = 50_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Fault {
    SensorRead,
    InvalidSensor,
    InvalidEstimate,
    Timing,
    StaleSample,
    CorrectionExpired,
    Tilt,
    InvalidOutput,
}

#[derive(Debug, PartialEq, Eq)]
pub(super) struct Preview {
    pub pd: i16,
    pub wheels: [i16; 2],
    pub direction_inputs: [(bool, bool); 2],
}

impl Preview {
    /// Preview the TB6612 logical commands and direction pins without GPIO access.
    fn from_pwm(pd: i16, pwm: i16) -> Self {
        let (left, right) = balance::wheel_commands(pwm);
        let limit = DutyLimit::new(PWM_CEILING);
        let left_drive = limit.map(left);
        let right_drive = limit.map(right);
        Self {
            pd,
            wheels: [left, right],
            direction_inputs: [
                Channel::A.direction_inputs(left_drive.direction),
                Channel::B.direction_inputs(right_drive.direction),
            ],
        }
    }
}

pub(super) struct Controller {
    started_at: u64,
    last_sample: Option<u64>,
    last_correction: Option<u64>,
    fault: Option<Fault>,
}

impl Controller {
    pub fn new(now: u64) -> Self {
        Self {
            started_at: now,
            last_sample: None,
            last_correction: None,
            fault: None,
        }
    }

    /// Latch the first fault until reset.
    pub fn fail(&mut self, reason: Fault) -> Fault {
        *self.fault.get_or_insert(reason)
    }

    pub fn check_freshness(&mut self, now: u64) -> Result<(), Fault> {
        if let Some(reason) = self.fault {
            return Err(reason);
        }
        match now.checked_sub(self.last_sample.unwrap_or(self.started_at)) {
            None => Err(self.fail(Fault::Timing)),
            Some(age) if age > balance::MAX_SAMPLE_GAP_US => Err(self.fail(Fault::StaleSample)),
            Some(_) => Ok(()),
        }
    }

    /// Empty polls never call this method. Two fresh reads are needed for dt.
    pub fn fresh_sample(&mut self, now: u64) -> Result<Option<f32>, Fault> {
        self.check_freshness(now)?;
        match self.last_sample.replace(now) {
            Some(previous) if previous == now => Err(self.fail(Fault::Timing)),
            Some(previous) => Ok(Some((now - previous) as f32 * 1e-6)),
            None => Ok(None),
        }
    }

    pub fn check_correction(&mut self, now: u64) -> Result<(), Fault> {
        if let Some(reason) = self.fault {
            return Err(reason);
        }
        if let Some(previous) = self.last_correction {
            match now.checked_sub(previous) {
                None => return Err(self.fail(Fault::Timing)),
                Some(age) if age > MAX_CORRECTION_AGE_US => {
                    return Err(self.fail(Fault::CorrectionExpired));
                }
                Some(_) => {}
            }
        }
        Ok(())
    }

    pub fn preview(
        &mut self,
        pitch_deg: f32,
        rate_dps: f32,
        accel_used: bool,
        now: u64,
    ) -> Result<Preview, Fault> {
        self.check_freshness(now)?;
        self.check_correction(now)?;
        if !pitch_deg.is_finite() || !rate_dps.is_finite() {
            return Err(self.fail(Fault::InvalidEstimate));
        }
        if pitch_deg.abs() >= balance::CONTROL_FALL_ANGLE_DEG {
            return Err(self.fail(Fault::Tilt));
        }
        if accel_used {
            self.last_correction = Some(now);
        } else if self.last_correction.is_none() {
            return Err(self.fail(Fault::InvalidEstimate));
        }

        let pd =
            balance::pd(PD, pitch_deg, rate_dps).ok_or_else(|| self.fail(Fault::InvalidOutput))?;
        let pwm =
            pwm_mapping::map(PWM, f32::from(pd)).ok_or_else(|| self.fail(Fault::InvalidOutput))?;
        Ok(Preview::from_pwm(pd, pwm as i16))
    }
}
