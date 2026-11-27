//! Signed PD-to-PWM mapping.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Limited PD magnitude at or below this value maps to zero.
    pub deadband: f32,
    /// Must match the PD controller output limit; strictly greater than deadband.
    pub pd_output_limit: f32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,
}
impl Config {
    pub fn valid(self) -> bool {
        self.deadband.is_finite()
            && self.deadband >= 0.0
            && self.minimum_duty.is_finite()
            && self.minimum_duty >= 0.0
            && self.maximum_duty.is_finite()
            && self.minimum_duty < self.maximum_duty
            && self.pd_output_limit.is_finite()
            && self.pd_output_limit > self.deadband
            && (0.0..100.0).contains(&self.maximum_duty)
    }
}

/// Apply the deadband, then scale proportionally to the PWM range.
/// No startup pulse, hysteresis, or inferred wheel-motion state.
pub fn map(config: Config, raw: f32) -> Option<f32> {
    if !config.valid() || !raw.is_finite() {
        return None;
    }
    if raw.abs() <= config.deadband {
        return Some(0.0);
    }
    let fraction = (raw.abs().min(config.pd_output_limit) - config.deadband)
        / (config.pd_output_limit - config.deadband);
    let magnitude = config.minimum_duty + fraction * (config.maximum_duty - config.minimum_duty);
    Some(raw.signum() * magnitude.min(config.maximum_duty))
}
