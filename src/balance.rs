//! Small experimental PD policy. Units: degrees, degrees/s, signed duty percent.
//! Positive pitch/rate -> positive physical-forward effort -> logical (u,u), with channel polarity applied by the driver.
pub const KI: f32 = 0.0; // No integral state or automatic friction compensation.
pub const ARM_ANGLE_DEG: f32 = 5.0;
pub const ARM_RATE_DPS: f32 = 5.0;
pub const FALL_ANGLE_DEG: f32 = 15.0; // Existing standalone/response diagnostics.
pub const CONTROL_FALL_ANGLE_DEG: f32 = 25.0;
pub const UPRIGHT_DWELL_US: u64 = 500_000;
pub const MAX_SAMPLE_GAP_US: u64 = 40_000;
pub const MAX_ACTIVE_US: u64 = 180_000_000; // Short first experiments, never unattended.

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    Invalid,
    Stale,
    Timing,
    Tilt,
    TrialExpired,
    Motor,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Observe,
    DryRun,
    Active,
    Fault(Fault),
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Stop,
    Reset,
    Invalid,
    Dry,
    Arm,
}

pub struct Controller {
    state: State,
    settings: Settings,
    calibrated: bool,
    live_allowed: bool,
    last_sample: Option<u64>,
    upright_since: Option<u64>,
    active_since: Option<u64>,
    tilt_limit: f32,
}
impl Controller {
    pub fn new(settings: Settings, calibrated: bool, live_allowed: bool) -> Option<Self> {
        if !settings.valid() {
            return None;
        }
        Some(Self {
            state: State::Observe,
            settings,
            calibrated,
            live_allowed,
            last_sample: None,
            upright_since: None,
            active_since: None,
            tilt_limit: FALL_ANGLE_DEG,
        })
    }
    /// Select a profile cutoff before any control samples/arming.
    pub fn with_tilt_limit(mut self, limit: f32) -> Option<Self> {
        if self.last_sample.is_some()
            || matches!(self.state, State::Active | State::Fault(_))
            || !limit.is_finite()
            || !(ARM_ANGLE_DEG..=CONTROL_FALL_ANGLE_DEG).contains(&limit)
        {
            return None;
        }
        self.tilt_limit = limit;
        Some(self)
    }
    pub fn tilt_limit(&self) -> f32 {
        self.tilt_limit
    }
    pub fn state(&self) -> State {
        self.state
    }
    pub fn fault(&mut self, reason: Fault) {
        // Observation failures invalidate data; only live control latches a fault.
        if self.state == State::Active {
            self.state = State::Fault(reason);
        }
        self.last_sample = None;
        self.upright_since = None;
        self.active_since = None;
    }
    /// Called only for fresh, valid filter results, using the same measured clock.
    pub fn sample(&mut self, pitch: f32, rate: f32, now: u64) {
        if !pitch.is_finite() || !rate.is_finite() {
            self.fault(Fault::Invalid);
            return;
        }
        if let Some(last) = self.last_sample {
            if now <= last {
                self.fault(Fault::Timing);
                return;
            }
            if now - last > MAX_SAMPLE_GAP_US {
                if self.state == State::Active {
                    self.fault(Fault::Timing);
                    return;
                }
                self.upright_since = None;
            }
        }
        if self.state == State::Active && pitch.abs() >= self.tilt_limit {
            self.fault(Fault::Tilt);
            return;
        }
        self.last_sample = Some(now);
        if pitch.abs() <= ARM_ANGLE_DEG && rate.abs() <= ARM_RATE_DPS {
            if self.upright_since.is_none() {
                self.upright_since = Some(now);
            }
        } else {
            self.upright_since = None;
        }
        self.tick(now);
    }
    pub fn tick(&mut self, now: u64) {
        if let Some(last) = self.last_sample {
            if now < last {
                self.fault(Fault::Timing);
                return;
            }
            if now - last > MAX_SAMPLE_GAP_US {
                self.fault(Fault::Stale);
                return;
            }
        }
        if let Some(start) = self.active_since
            && (now < start || now - start >= MAX_ACTIVE_US)
        {
            self.fault(Fault::TrialExpired);
        }
    }
    /// A gyro-only estimate can be observed, but cannot establish live readiness.
    pub fn block_arming(&mut self) {
        self.upright_since = None;
    }
    pub fn sample_fresh(&self, now: u64) -> bool {
        self.last_sample
            .is_some_and(|t| now >= t && now - t <= MAX_SAMPLE_GAP_US)
    }
    pub fn ready(&self, now: u64) -> bool {
        !matches!(self.state, State::Fault(_))
            && self.calibrated
            && self
                .last_sample
                .is_some_and(|t| now >= t && now - t <= MAX_SAMPLE_GAP_US)
            && self
                .upright_since
                .is_some_and(|t| now >= t && now - t >= UPRIGHT_DWELL_US)
    }
    fn deny(&mut self) {
        if !matches!(self.state, State::Fault(_)) {
            self.state = State::Observe;
        }
        self.active_since = None;
        self.upright_since = None;
    }
    pub fn request(&mut self, request: Request, now: u64) -> Result<(), &'static str> {
        if matches!(request, Request::Stop | Request::Reset) {
            self.state = State::Observe;
            self.active_since = None;
            self.upright_since = None;
            return Ok(());
        }
        if matches!(self.state, State::Fault(_)) {
            return Err("fault latched; send STOP or RESET first");
        }
        if request == Request::Invalid {
            self.deny();
            return Err("unknown or overlong command; stopped");
        }
        if self.state == State::Active {
            self.deny();
            return Err(
                "active trial stopped; complete a new upright dwell before another request",
            );
        }
        if request == Request::Dry {
            if !self.calibrated {
                return Err("calibration required");
            }
            self.state = State::DryRun;
            return Ok(());
        }
        if request == Request::Arm && !self.live_allowed {
            self.deny();
            return Err("live build feature disabled");
        }
        if !self.ready(now) {
            self.deny();
            return Err("need valid calibration, fresh data and 0.5s upright/low-rate dwell");
        }
        self.state = if request == Request::Arm {
            State::Active
        } else {
            State::DryRun
        };
        self.active_since = if request == Request::Arm {
            Some(now)
        } else {
            None
        };
        Ok(())
    }
    pub fn output(&self, pitch: f32, rate: f32) -> Option<i16> {
        if self.last_sample.is_some() && matches!(self.state, State::DryRun | State::Active) {
            pd(self.settings, pitch, rate)
        } else {
            None
        }
    }
}

/// Small fixed-buffer console. `!` or space stops immediately without a newline.
/// Invalid/overlong commands are rejected and stop control. No input triggers movement.
#[derive(Default)]
pub struct Console {
    line: [u8; 8],
    len: usize,
    overflow: bool,
}
impl Console {
    pub fn byte(&mut self, byte: u8) -> Option<Request> {
        if byte == b'!' || byte == b' ' {
            *self = Self::default();
            return Some(Request::Stop);
        }
        if byte == b'\r' {
            return None;
        }
        if byte == b'\n' {
            let request = if self.overflow {
                Request::Invalid
            } else {
                match &self.line[..self.len] {
                    b"DRY" => Request::Dry,
                    b"ARM" => Request::Arm,
                    b"STOP" => Request::Stop,
                    b"RESET" => Request::Reset,
                    _ => Request::Invalid,
                }
            };
            *self = Self::default();
            return Some(request);
        }
        if self.len < self.line.len() {
            self.line[self.len] = byte;
            self.len += 1;
        } else {
            self.overflow = true;
        }
        None
    }
}
