//! Balance PD and arming limits. Units: degrees, degrees/s, signed duty percent.
//! Positive pitch/rate -> positive physical-forward effort -> logical (u,u), with channel polarity applied by the driver.
pub const ARM_ANGLE_DEG: f32 = 5.0;
pub const ARM_RATE_DPS: f32 = 5.0;
pub const TILT_CUTOFF_DEG: f32 = 25.0;
pub const UPRIGHT_DWELL_US: u64 = 500_000;
pub const MAX_SAMPLE_GAP_US: u64 = 40_000;

#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub kp: f32, // duty-percent / degree
    pub kd: f32, // duty-percent / (degree/second)
    pub output_limit: i16,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            kp: 1.0,
            kd: 0.08,
            output_limit: 15,
        }
    }
}
impl Settings {
    pub fn valid(self) -> bool {
        self.kp.is_finite()
            && self.kd.is_finite()
            && self.kp >= 0.0
            && self.kd >= 0.0
            && (0..=30).contains(&self.output_limit)
    }
}

/// Catch a forward lean by moving the wheels forward. This sign must still be
/// physically checked with suspended wheels before any supported floor trial.
pub fn pd(settings: Settings, pitch_deg: f32, rate_dps: f32) -> Option<i16> {
    if !settings.valid() || !pitch_deg.is_finite() || !rate_dps.is_finite() {
        return None;
    }
    let u = settings.kp * pitch_deg + settings.kd * rate_dps;
    if !u.is_finite() {
        return None;
    }
    // Truncate toward zero to the existing driver's 1% resolution. No minimum
    // duty jump, full-duty kick, integral term or saturation compensation.
    Some(u.clamp(
        -(settings.output_limit as f32),
        settings.output_limit as f32,
    ) as i16)
}
pub fn wheel_commands(forward_percent: i16) -> (i16, i16) {
    let u = forward_percent.clamp(-100, 100);
    (u, u) // A LEFT, B RIGHT: both request logical forward; driver inverts A once.
}
