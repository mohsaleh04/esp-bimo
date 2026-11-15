//! One-shot suspended-wheel orchestration. Estimation, PD and mapping stay shared.
use crate::balance::{self, Controller, Fault, Request, Settings, State};
use crate::pwm_mapping::{Config, KickConfig, KickState, Preview};

// ACTIVE only. Fresh gyro samples still have their separate <=40ms deadline.
pub const MAX_GYRO_ONLY_US: u64 = 50_000;
// Local to this diagnostic; shared balance controller retains its 2-second cap.
pub const MAX_RESPONSE_WINDOW_US: u64 = 30_000_000;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorFault {
    I2cRead,
    NonFiniteAccel,
    InvalidGyro,
    ClippedGyro,
    NonFiniteEstimate,
    CorrectionRequiredForArming,
    NoGravityReference,
    CorrectionExpired,
    FilterInvalid,
    FilterGyroPropagationExpired,
    SampleTiming,
    FreshSampleTimeout,
}

/// Acquisition must call this BEFORE filter propagation; no grace for bad gyro
/// or nonfinite acceleration. Exact ADC rails indicate possible gyro clipping.
pub fn input_fault(accel: [f32; 3], gyro: [f32; 3], raw_gyro: [i16; 3]) -> Option<SensorFault> {
    if accel.iter().any(|v| !v.is_finite()) {
        Some(SensorFault::NonFiniteAccel)
    } else if gyro.iter().any(|v| !v.is_finite()) {
        Some(SensorFault::InvalidGyro)
    } else if raw_gyro.iter().any(|&v| v == i16::MIN || v == i16::MAX) {
        Some(SensorFault::ClippedGyro)
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResponseState {
    DryRun,
    WaitingForReady,
    ArmedWaitingForNeutral,
    Active,
    Disarmed,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisarmReason {
    ArmingTimeout,
    ActiveTimeout,
    Fault(Fault),
    Stop,
}
#[derive(Clone, Copy)]
pub struct Transition {
    pub from: ResponseState,
    pub to: ResponseState,
    pub us: u64,
    pub reason: &'static str,
}
pub struct Response {
    pub control: Controller,
    last_accel_us: Option<u64>,
    pub accel_correction_age_us: Option<u64>,
    pub sensor_fault: Option<SensorFault>,
    settings: Settings,
    kick_state: KickState,
    mapping: Config,
    kick: KickConfig,
    live: bool,
    window_us: u64,
    arming_timeout_us: u64,
    wait_started: Option<u64>,
    started: Option<u64>,
    now: u64,
    phase: ResponseState,
    // Monotonic, one-shot path has at most three transitions; fixed storage.
    transitions: [Option<Transition>; 3],
    transition_count: usize,
    pub limited_pd: Option<i16>,
    pub neutral_satisfied: bool,
    pub disarm_reason: Option<DisarmReason>,
    pub finished: bool,
}
#[derive(Clone, Copy)]
pub struct Output {
    pub limited_pd: i16,
    pub preview: Preview,
}
impl Response {
    #[allow(dead_code)] // Shared policy: selected explicitly only by balance-control.
    pub fn with_tilt_limit(mut self, limit: f32) -> Option<Self> {
        self.control = self.control.with_tilt_limit(limit)?;
        Some(self)
    }
    pub fn new(
        settings: Settings,
        mapping: Config,
        kick: KickConfig,
        ceiling: u8,
        live: bool,
        window_us: u64,
        arming_timeout_us: u64,
    ) -> Option<Self> {
        if !mapping.valid()
            || !kick.valid(mapping)
            || mapping.pd_output_limit != f32::from(settings.output_limit)
            || ceiling == 0
            || ceiling >= 100
            || mapping.maximum_duty > f32::from(ceiling)
            || (kick.enabled && kick.kick_duty > f32::from(ceiling))
            || window_us == 0
            || window_us > MAX_RESPONSE_WINDOW_US
            || arming_timeout_us == 0
            || arming_timeout_us > 60_000_000
        {
            return None;
        }
        let mut control = Controller::new(settings, true, live)?;
        if !live {
            control.request(Request::Dry, 0).ok()?;
        }
        Some(Self {
            control,
            last_accel_us: None,
            accel_correction_age_us: None,
            sensor_fault: None,
            settings,
            kick_state: KickState::default(),
            mapping,
            kick,
            live,
            window_us,
            arming_timeout_us,
            wait_started: None,
            started: None,
            now: 0,
            phase: if live {
                ResponseState::WaitingForReady
            } else {
                ResponseState::DryRun
            },
            transitions: [None; 3],
            transition_count: 0,
            limited_pd: None,
            neutral_satisfied: false,
            disarm_reason: None,
            finished: false,
        })
    }
    pub fn state(&self) -> ResponseState {
        self.phase
    }
    pub fn transitions(&self) -> &[Option<Transition>] {
        &self.transitions[..self.transition_count]
    }
    pub fn supervising(&self) -> bool {
        matches!(
            self.phase,
            ResponseState::ArmedWaitingForNeutral | ResponseState::Active
        )
    }
    fn transition(&mut self, phase: ResponseState, reason: &'static str) {
        if self.phase == phase {
            return;
        }
        self.transitions[self.transition_count] = Some(Transition {
            from: self.phase,
            to: phase,
            us: self.now,
            reason,
        });
        self.transition_count += 1;
        self.phase = phase;
    }
    /// Start the bounded arming wait only AFTER calibration and preparation.
    pub fn begin(&mut self, now: u64) {
        if self.wait_started.is_none() {
            self.wait_started = Some(now);
        }
        self.now = now;
    }
    fn finish(&mut self, reason: DisarmReason) {
        if self.finished {
            return;
        }
        self.kick_state.reset();
        self.last_accel_us = None;
        self.neutral_satisfied = false;
        self.limited_pd = None;
        self.disarm_reason = Some(reason);
        self.finished = true;
        let text = match reason {
            DisarmReason::ArmingTimeout => {
                "arming timeout: readiness/post-ready neutral not achieved"
            }
            DisarmReason::ActiveTimeout => "ACTIVE response window expired",
            DisarmReason::Fault(_) => "safety fault; see disarm_reason",
            DisarmReason::Stop => "controller stopped; reset required",
        };
        self.transition(ResponseState::Disarmed, text);
    }
    /// Explicit terminal stop, including a completed motor-disabled capture.
    #[allow(dead_code)] // Other standalone policy consumers do not use bounded DRY capture.
    pub fn stop(&mut self) {
        self.finish(DisarmReason::Stop);
    }
    pub fn sensor_fail(&mut self, reason: SensorFault) {
        self.sensor_fault = Some(reason);
        self.fail(Fault::Invalid);
    }
    pub fn fail(&mut self, reason: Fault) {
        if reason == Fault::Stale {
            self.sensor_fault = Some(SensorFault::FreshSampleTimeout);
        }

        self.control.fault(reason);
        self.kick_state.reset();
        self.last_accel_us = None;
        self.neutral_satisfied = false;
        self.limited_pd = None;
        if self.supervising() {
            self.finish(DisarmReason::Fault(reason));
        }
    }
    /// Explicit diagnostic abort, including before readiness. Existing suspended
    /// policy keeps fail()'s recoverable pre-ready behavior; ground calls this
    /// only for its stricter terminal estimate/tilt guard.
    pub fn abort(&mut self, reason: Fault) {
        if self.finished {
            return;
        }
        self.fail(reason);
        if !self.finished {
            self.finish(DisarmReason::Fault(reason));
        }
    }
    pub fn tick(&mut self, now: u64) {
        if self.finished {
            return;
        }
        if now < self.now {
            self.fail(Fault::Timing);
            return;
        }
        self.begin(now);
        self.accel_correction_age_us = self.last_accel_us.map(|t| now.saturating_sub(t));
        self.control.tick(now);
        if let State::Fault(reason) = self.control.state() {
            if reason == Fault::Stale {
                self.sensor_fault = Some(SensorFault::FreshSampleTimeout);
            }
            self.finish(if reason == Fault::TrialExpired {
                DisarmReason::ActiveTimeout
            } else {
                DisarmReason::Fault(reason)
            });
            return;
        }
        // In shared DRY mode tick() invalidates stale samples without latching.
        // The response policy must latch/disarm before any other grace/timer check.
        if self.supervising() && !self.control.sample_fresh(now) {
            self.fail(Fault::Stale);
            return;
        }
        if self.live && self.phase == ResponseState::Active {
            match self.accel_correction_age_us {
                Some(age) if age <= MAX_GYRO_ONLY_US => {}
                Some(_) => {
                    self.sensor_fail(SensorFault::CorrectionExpired);
                    return;
                }
                None => {
                    self.sensor_fail(SensorFault::NoGravityReference);
                    return;
                }
            }
        }
        if let Some(start) = self.started {
            if self.control.state() != State::DryRun {
                self.finish(DisarmReason::Stop);
                return;
            }
            if now - start >= self.window_us {
                self.control.fault(Fault::TrialExpired);
                self.finish(DisarmReason::ActiveTimeout);
                return;
            }
        } else if self.live && now - self.wait_started.unwrap_or(now) >= self.arming_timeout_us {
            self.finish(DisarmReason::ArmingTimeout);
            return;
        }
        if !self.control.sample_fresh(now) {
            self.last_accel_us = None;
            self.kick_state.reset();
            self.neutral_satisfied = false;
            self.limited_pd = None;
            if self.supervising() {
                self.fail(Fault::Stale);
            }
        }
    }
    /// Fresh estimates only. Ready and neutral are separate requirements.
    pub fn sample(
        &mut self,
        pitch: f32,
        rate: f32,
        accel_used: bool,
        dt: f32,
        now: u64,
    ) -> Option<Output> {
        self.tick(now);
        if self.finished {
            return None;
        }
        if !dt.is_finite() || dt <= 0.0 || dt > 0.04 {
            self.sensor_fault = Some(SensorFault::SampleTiming);
            self.fail(Fault::Timing);
            return None;
        }
        if !pitch.is_finite() || !rate.is_finite() {
            self.sensor_fail(SensorFault::NonFiniteEstimate);
            return None;
        }
        if self.live && !accel_used && self.phase != ResponseState::Active {
            self.sensor_fail(SensorFault::CorrectionRequiredForArming);
            return None;
        }
        // This flag may be false ONLY for a finite-accel, valid fresh-gyro estimate.
        // The acquisition path hard-rejects nonfinite inputs and clipped gyro.
        if accel_used {
            self.last_accel_us = Some(now);
            self.accel_correction_age_us = Some(0);
            self.sensor_fault = None;
        }
        if self.supervising() && pitch.abs() >= self.control.tilt_limit() {
            self.fail(Fault::Tilt);
            return None;
        }
        self.control.sample(pitch, rate, now);
        if !accel_used {
            self.control.block_arming();
        }
        self.tick(now);
        if self.finished || !self.control.sample_fresh(now) {
            return None;
        }
        // Use the existing PD function even while the shared controller is OBSERVE.
        let limited_pd = balance::pd(self.settings, pitch, rate)?;
        self.limited_pd = Some(limited_pd);
        self.neutral_satisfied = f32::from(limited_pd).abs() <= self.mapping.deadband;
        if self.phase == ResponseState::WaitingForReady {
            if self.control.ready(now) {
                self.transition(
                    ResponseState::ArmedWaitingForNeutral,
                    "upright dwell passed; motors disarmed; waiting for a NEW neutral sample",
                );
            }
            return None; // Neutral must be seen on a later sample.
        }
        if self.phase == ResponseState::ArmedWaitingForNeutral {
            if self.neutral_satisfied
                && self.control.ready(now)
                // Reuse the controller's readiness/sensor tracking in DRY state.
                // Only Response::Active permits motors; its own timer owns the
                // diagnostic window without changing the shared 2-second cap.
                && self.control.request(Request::Dry, now).is_ok()
            {
                self.started = Some(now);
                self.kick_state.reset();
                self.transition(
                    ResponseState::Active,
                    "fresh post-ready neutral accepted; ACTIVE motor-command window starts NOW",
                );
            }
            return None; // Neutral acceptance frame still outputs zero; never a forced kick.
        }
        let Some(preview) = self.kick_state.update(
            self.mapping,
            self.kick,
            f32::from(limited_pd),
            dt,
            now / 1_000,
        ) else {
            self.fail(Fault::Invalid);
            return None;
        };
        Some(Output {
            limited_pd,
            preview,
        })
    }
}

/// Last output gate: unsafe/finished/DRY states always disarm, regardless of preview.
pub fn drive(
    motors: &mut impl crate::motors::DualMotor,
    response: &mut Response,
    output: Option<Output>,
    ceiling: u8,
    now: u64,
) -> Result<(), crate::motors::Disarmed> {
    // Recheck elapsed time here, even if caller has not ticked since acquisition.
    response.tick(now);
    if response.state() != ResponseState::Active || response.finished {
        motors.disarm();
        return Ok(());
    }
    let u = output.map_or(0.0, |o| o.preview.final_pwm);
    if !u.is_finite() || u.abs() > f32::from(ceiling) || ceiling >= 100 {
        motors.disarm();
        return Err(crate::motors::Disarmed);
    }
    let (a, b) = crate::balance::wheel_commands(u as i16);
    if let Err(e) = motors.command(a, b) {
        motors.disarm();
        return Err(e);
    }
    Ok(())
}

/// Polling cadence is independent of sample cadence. Count each late loop once.
#[derive(Default)]
pub struct LoopTiming {
    start: Option<u64>,
    pub interval_us: u64,
    pub work_us: u64,
    pub max_interval_us: u64,
    pub max_work_us: u64,
    pub missed_deadlines: u32,
    missed_this_loop: bool,
}
impl LoopTiming {
    pub fn begin(&mut self, now: u64, limit: u64) -> bool {
        let previous = self.start.replace(now);
        self.interval_us = previous.map_or(0, |t| now.saturating_sub(t));
        self.max_interval_us = self.max_interval_us.max(self.interval_us);
        self.work_us = 0;
        self.missed_this_loop = false;
        let late = previous.is_some_and(|t| now < t || now - t > limit);
        self.mark(late)
    }
    pub fn check_work(&mut self, now: u64, limit: u64) -> bool {
        self.work_us = self.start.map_or(0, |t| now.saturating_sub(t));
        self.max_work_us = self.max_work_us.max(self.work_us);
        let late = self.start.is_some_and(|t| now < t || now - t > limit);
        self.mark(late)
    }
    fn mark(&mut self, late: bool) -> bool {
        if late && !self.missed_this_loop {
            self.missed_deadlines = self.missed_deadlines.saturating_add(1);
            self.missed_this_loop = true;
        }
        self.missed_this_loop
    }
}
