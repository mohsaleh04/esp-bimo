//! One-axis body tilt estimation. No HAL, allocation, or motor control.
//! The physical pitch axis is learned from upright and forward-lean poses.
pub const GRAVITY: f32 = 9.806;

#[derive(Debug, Clone, Copy)]
pub struct Vector(pub [f32; 3]);

impl Vector {
    pub const ZERO: Self = Self([0.0; 3]);
    pub fn finite(self) -> bool {
        self.0.iter().all(|x| x.is_finite())
    }
    pub fn dot(self, b: Self) -> f32 {
        self.0.iter().zip(b.0).map(|(a, b)| a * b).sum()
    }
    pub fn scale(self, k: f32) -> Self {
        Self(self.0.map(|v| v * k))
    }
    pub fn norm(self) -> f32 {
        libm::sqrtf(self.dot(self))
    }
    pub fn unit(self) -> Option<Self> {
        let n = self.norm();
        if self.finite() && n > 1e-6 {
            Some(self.scale(1.0 / n))
        } else {
            None
        }
    }
    pub fn cross(self, b: Self) -> Self {
        let [x, y, z] = self.0;
        let [a, b, c] = b.0;
        Self([y * c - z * b, z * a - x * c, x * b - y * a])
    }
}
impl core::ops::Add for Vector {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self(core::array::from_fn(|i| self.0[i] + b.0[i]))
    }
}
impl core::ops::Sub for Vector {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self(core::array::from_fn(|i| self.0[i] - b.0[i]))
    }
}

fn gravity_valid(a: Vector) -> bool {
    a.finite() && (0.85 * GRAVITY..=1.15 * GRAVITY).contains(&a.norm())
}
fn wrap(deg: f32) -> f32 {
    let mut r = libm::fmodf(deg + 180.0, 360.0);
    if r < 0.0 {
        r += 360.0;
    }
    r - 180.0
}

/// Why an accelerometer update is unsuitable as a gravity-based pitch correction.
/// Values/limits are in m/s². This is not by itself an invalid gyro estimate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccelRejection {
    NonFinite,
    Magnitude {
        measured: f32,
        minimum: f32,
        maximum: f32,
    },
    Sideways {
        measured: f32,
        maximum: f32,
    },
    PitchPlane {
        measured: f32,
        minimum: f32,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct PitchFrame {
    /// Unit gravity vector measured while the body is held upright.
    pub up: Vector,
    /// Acceleration direction that increases during the known forward lean.
    /// This is not a claim that this vector points toward the robot's front.
    pub tilt: Vector,
    /// Gyro projection axis; positive angular rate increases forward lean.
    pub axis: Vector,
}

impl PitchFrame {
    pub fn from_poses(upright: Vector, forward: Vector) -> Option<Self> {
        if !gravity_valid(upright) || !gravity_valid(forward) {
            return None;
        }
        let up = upright.unit()?;
        let forward = forward.unit()?;
        let cross = forward.cross(up);
        let angle = libm::atan2f(cross.norm(), forward.dot(up)).to_degrees();
        if !(10.0..=45.0).contains(&angle) {
            return None;
        }
        let axis = cross.unit()?;
        let tilt = up.cross(axis).unit()?;
        Some(Self { up, tilt, axis })
    }

    /// Diagnostic form of the existing gates; thresholds/math are unchanged.
    pub fn accel_rejection(&self, a: Vector) -> Option<AccelRejection> {
        if !a.finite() {
            return Some(AccelRejection::NonFinite);
        }
        if !gravity_valid(a) {
            return Some(AccelRejection::Magnitude {
                measured: a.norm(),
                minimum: 0.85 * GRAVITY,
                maximum: 1.15 * GRAVITY,
            });
        }
        let sideways = a.dot(self.axis).abs();
        if sideways > 0.35 * GRAVITY {
            return Some(AccelRejection::Sideways {
                measured: sideways,
                maximum: 0.35 * GRAVITY,
            });
        }
        let x = a.dot(self.tilt);
        let z = a.dot(self.up);
        if x * x + z * z < (0.5 * GRAVITY) * (0.5 * GRAVITY) {
            return Some(AccelRejection::PitchPlane {
                measured: libm::sqrtf(x * x + z * z),
                minimum: 0.5 * GRAVITY,
            });
        }
        None
    }
    pub fn accel_angle(&self, a: Vector) -> Option<f32> {
        if self.accel_rejection(a).is_some() {
            return None;
        }
        Some(libm::atan2f(a.dot(self.tilt), a.dot(self.up)).to_degrees())
    }
    pub fn rate(&self, corrected_gyro_dps: Vector) -> f32 {
        corrected_gyro_dps.dot(self.axis)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Estimate {
    pub pitch_deg: f32,
    pub rate_dps: f32,
    pub accel_used: bool,
}

pub struct ComplementaryFilter {
    tau_s: f32,
    angle: Option<f32>,
    gyro_only_s: f32,
}

impl ComplementaryFilter {
    pub fn new(tau_s: f32) -> Self {
        assert!(tau_s.is_finite() && tau_s > 0.0);
        Self {
            tau_s,
            angle: None,
            gyro_only_s: 0.0,
        }
    }
    pub fn reset(&mut self) {
        self.angle = None;
        self.gyro_only_s = 0.0;
    }
    pub fn pitch_deg(&self) -> Option<f32> {
        self.angle
    }
    /// dt is measured between fresh sample reads, in seconds. No fixed-rate dt.
    /// Invalid input or >100 ms gaps invalidate the state; next valid gravity
    /// sample reinitializes it rather than integrating across missing history.
    pub fn update(
        &mut self,
        frame: &PitchFrame,
        accel: Vector,
        gyro_dps: Vector,
        dt: f32,
    ) -> Option<Estimate> {
        if !dt.is_finite() || dt <= 0.0 || dt > 0.1 || !gyro_dps.finite() {
            self.reset();
            return None;
        }
        let rate = frame.rate(gyro_dps);
        if !rate.is_finite() {
            self.reset();
            return None;
        }
        let accel_angle = frame.accel_angle(accel);
        let angle = if let Some(previous) = self.angle {
            let predicted = wrap(previous + rate * dt);
            if let Some(measured) = accel_angle {
                self.gyro_only_s = 0.0;
                let alpha = self.tau_s / (self.tau_s + dt);
                wrap(predicted + (1.0 - alpha) * wrap(measured - predicted))
            } else {
                self.gyro_only_s += dt;
                if self.gyro_only_s > 0.5 {
                    self.reset();
                    return None;
                }
                predicted
            }
        } else {
            accel_angle?
        };
        self.angle = Some(angle);
        Some(Estimate {
            pitch_deg: angle,
            rate_dps: rate,
            accel_used: accel_angle.is_some(),
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CalibrationResult {
    pub accel_mean: Vector,
    pub gyro_bias_dps: Vector,
}

/// Stationary policy based on the measured robot noise envelope, not nominal ODR.
pub mod calibration_limits {
    pub const MIN_DURATION_US: u64 = 3_000_000;
    // 120 samples gives >=40 Hz over 3 s; independent from the elapsed-time gate.
    pub const MIN_SAMPLES: u32 = 120;
    pub const MIN_G: f32 = 0.85;
    pub const MAX_G: f32 = 1.15;
    pub const MAX_DT_S: f32 = 0.05;
    pub const MAX_DRIFT: f32 = 0.05 * super::GRAVITY;
    pub const MAX_ACCEL_VARIANCE: f32 = (0.02 * super::GRAVITY) * (0.02 * super::GRAVITY);
    // Observed gyro variance up to 0.78: 50% margin, rounded upward to 1.2.
    pub const MAX_GYRO_VARIANCE: f32 = 1.2;
    // Gross-motion/fault guard only, not the stationary detector. Observed peaks
    // 4 dps + approximately five noise standard deviations fit below 10 dps.
    pub const MAX_GYRO_DPS: f32 = 10.0;
    // Explicit conservative bias-plausibility prior; NOT proof of stationarity.
    pub const MAX_BIAS_DPS: f32 = 5.0;
    pub const MAX_ACCEL_MEAN_SHIFT: f32 = 0.015 * super::GRAVITY;
    pub const MAX_GYRO_MEAN_SHIFT: f32 = 0.75;
}
use calibration_limits as limits;

#[derive(Debug, Clone, Copy)]
pub enum CalibrationRejection {
    Gravity { magnitude: f32, finite: bool },
    Gyro { magnitude: f32, finite: bool },
    Drift { distance: f32 },
    Variance { accel: [f32; 3], gyro: [f32; 3] },
    Timing { dt: f32 },
    Read,
    Motion { accel_shift: f32, gyro_shift: f32 },
    Bias { magnitude: f32 },
}
impl CalibrationRejection {
    pub fn index(self) -> usize {
        match self {
            Self::Gravity { .. } => 0,
            Self::Gyro { .. } => 1,
            Self::Drift { .. } => 2,
            Self::Variance { .. } => 3,
            Self::Timing { .. } => 4,
            Self::Read => 5,
            Self::Motion { .. } => 6,
            Self::Bias { .. } => 7,
        }
    }
}

struct Moments {
    count: u32,
    accel: Vector,
    gyro: Vector,
    accel_m2: Vector,
    gyro_m2: Vector,
}
impl Default for Moments {
    fn default() -> Self {
        Self {
            count: 0,
            accel: Vector::ZERO,
            gyro: Vector::ZERO,
            accel_m2: Vector::ZERO,
            gyro_m2: Vector::ZERO,
        }
    }
}
impl Moments {
    fn add(&mut self, a: Vector, g: Vector) {
        self.count += 1;
        for i in 0..3 {
            let da = a.0[i] - self.accel.0[i];
            self.accel.0[i] += da / self.count as f32;
            self.accel_m2.0[i] += da * (a.0[i] - self.accel.0[i]);
            let dg = g.0[i] - self.gyro.0[i];
            self.gyro.0[i] += dg / self.count as f32;
            self.gyro_m2.0[i] += dg * (g.0[i] - self.gyro.0[i]);
        }
    }
}

/// All samples must be fresh, stationary and timestamped. Constant gyro offset
/// is estimated as bias; centered noise, pose drift and gyro trends detect motion.
/// Constant slow yaw is fundamentally indistinguishable from bias in a six-axis
/// IMU, so supported stillness remains a required external condition.
pub struct Calibration {
    first_us: Option<u64>,
    last_us: Option<u64>,
    first_accel: Vector,
    all: Moments,
    early: Moments,
    late: Moments,
}
impl Default for Calibration {
    fn default() -> Self {
        Self {
            first_us: None,
            last_us: None,
            first_accel: Vector::ZERO,
            all: Moments::default(),
            early: Moments::default(),
            late: Moments::default(),
        }
    }
}
impl Calibration {
    pub fn progress(&self) -> (u32, f32) {
        (self.all.count, self.elapsed_us() as f32 * 1e-6)
    }
    fn elapsed_us(&self) -> u64 {
        match (self.first_us, self.last_us) {
            (Some(a), Some(b)) => b - a,
            _ => 0,
        }
    }
    pub fn read_error(&mut self) -> CalibrationRejection {
        *self = Self::default();
        CalibrationRejection::Read
    }
    fn reject(&mut self, reason: CalibrationRejection) -> Option<CalibrationRejection> {
        *self = Self::default();
        Some(reason)
    }
    /// Backward-compatible measured-dt API; new diagnostics use push_at directly.
    pub fn push(&mut self, accel: Vector, gyro: Vector, dt: f32) {
        self.push_report(accel, gyro, dt);
    }
    pub fn push_report(&mut self, a: Vector, g: Vector, dt: f32) -> Option<CalibrationRejection> {
        if !dt.is_finite() || dt <= 0.0 || dt > limits::MAX_DT_S {
            return self.reject(CalibrationRejection::Timing { dt });
        }
        let us = libm::roundf(dt * 1e6) as u64;
        let Some(now) = self.last_us.unwrap_or(0).checked_add(us) else {
            return self.reject(CalibrationRejection::Timing { dt });
        };
        self.push_at(a, g, now)
    }
    /// Monotonic microsecond timestamp of this fresh read. Duration is the
    /// actual first-to-last sample span, never inferred from sample count/ODR.
    pub fn push_at(&mut self, a: Vector, g: Vector, now: u64) -> Option<CalibrationRejection> {
        if let Some(previous) = self.last_us {
            let dt = if now >= previous {
                (now - previous) as f32 * 1e-6
            } else {
                -((previous - now) as f32) * 1e-6
            };
            if dt <= 0.0 || dt > limits::MAX_DT_S {
                return self.reject(CalibrationRejection::Timing { dt });
            }
        }
        if !a.finite() || !(limits::MIN_G * GRAVITY..=limits::MAX_G * GRAVITY).contains(&a.norm()) {
            return self.reject(CalibrationRejection::Gravity {
                magnitude: a.norm(),
                finite: a.finite(),
            });
        }
        if !g.finite() || g.norm() > limits::MAX_GYRO_DPS {
            return self.reject(CalibrationRejection::Gyro {
                magnitude: g.norm(),
                finite: g.finite(),
            });
        }
        let mut event = None;
        if self.all.count > 0 {
            let distance = (a - self.first_accel).norm();
            if distance > limits::MAX_DRIFT {
                event = self.reject(CalibrationRejection::Drift { distance });
            }
        }
        if self.first_us.is_none() {
            self.first_us = Some(now);
            self.first_accel = a;
        }
        self.last_us = Some(now);
        self.all.add(a, g);
        let elapsed = self.elapsed_us();
        if elapsed <= 1_000_000 {
            self.early.add(a, g);
        }
        if elapsed >= 2_000_000 {
            self.late.add(a, g);
        }
        if self.ready()
            && let Some(reason) = self.window_rejection()
        {
            return self.reject(reason);
        }
        event
    }
    fn ready(&self) -> bool {
        self.all.count >= limits::MIN_SAMPLES && self.elapsed_us() >= limits::MIN_DURATION_US
    }
    fn window_rejection(&self) -> Option<CalibrationRejection> {
        let n = (self.all.count - 1) as f32;
        let av = self.all.accel_m2.0.map(|v| v / n);
        let gv = self.all.gyro_m2.0.map(|v| v / n);
        if !av.iter().all(|v| *v <= limits::MAX_ACCEL_VARIANCE)
            || !gv.iter().all(|v| *v <= limits::MAX_GYRO_VARIANCE)
        {
            return Some(CalibrationRejection::Variance {
                accel: av,
                gyro: gv,
            });
        }
        let accel_shift = (self.late.accel - self.early.accel).norm();
        let gyro_shift = (self.late.gyro - self.early.gyro).norm();
        if accel_shift > limits::MAX_ACCEL_MEAN_SHIFT || gyro_shift > limits::MAX_GYRO_MEAN_SHIFT {
            return Some(CalibrationRejection::Motion {
                accel_shift,
                gyro_shift,
            });
        }
        let magnitude = self.all.gyro.norm();
        if magnitude > limits::MAX_BIAS_DPS {
            return Some(CalibrationRejection::Bias { magnitude });
        }
        None
    }
    pub fn result(&self) -> Option<CalibrationResult> {
        if self.ready() && self.window_rejection().is_none() {
            Some(CalibrationResult {
                accel_mean: self.all.accel,
                gyro_bias_dps: self.all.gyro,
            })
        } else {
            None
        }
    }
}
