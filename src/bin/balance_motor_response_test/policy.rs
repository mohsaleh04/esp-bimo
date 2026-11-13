//! One-shot suspended-wheel orchestration. Estimation, PD and mapping stay shared.
use crate::balance::{Controller, Fault, MAX_ACTIVE_US, Request, Settings, State};
use crate::pwm_mapping::{Config, KickConfig, KickState, Preview};

pub struct Response {
    pub control: Controller,
    kick_state: KickState,
    mapping: Config,
    kick: KickConfig,
    live: bool,
    window_us: u64,
    started: Option<u64>,
    neutral_seen: bool,
    pub finished: bool,
}
#[derive(Clone, Copy)]
pub struct Output {
    pub limited_pd: i16,
    pub preview: Preview,
}
impl Response {
    pub fn new(
        settings: Settings,
        mapping: Config,
        kick: KickConfig,
        ceiling: u8,
        live: bool,
        window_us: u64,
    ) -> Option<Self> {
        if !mapping.valid()
            || !kick.valid(mapping)
            || mapping.pd_output_limit != f32::from(settings.output_limit)
            || ceiling == 0
            || ceiling >= 100
            || mapping.maximum_duty > f32::from(ceiling)
            || (kick.enabled && kick.kick_duty > f32::from(ceiling))
            || window_us == 0
            || window_us > MAX_ACTIVE_US
        {
            return None;
        }
        let mut control = Controller::new(settings, true, live)?;
        if !live {
            control.request(Request::Dry, 0).ok()?;
        }
        Some(Self {
            control,
            kick_state: KickState::default(),
            mapping,
            kick,
            live,
            window_us,
            started: None,
            neutral_seen: false,
            finished: false,
        })
    }
    pub fn fail(&mut self, reason: Fault) {
        self.control.fault(reason);
        self.kick_state.reset();
        self.neutral_seen = false;
        if self.started.is_some() {
            self.finished = true;
        }
    }
    pub fn tick(&mut self, now: u64) {
        self.control.tick(now);
        if let Some(start) = self.started
            && (now < start || now - start >= self.window_us)
        {
            self.control.fault(Fault::TrialExpired);
        }
        if !self.control.sample_fresh(now) {
            self.kick_state.reset();
            self.neutral_seen = false;
        }
        if matches!(self.control.state(), State::Fault(_))
            || (self.started.is_some() && self.control.state() != State::Active)
        {
            self.kick_state.reset();
            self.neutral_seen = false;
            self.finished = true;
        }
    }
    /// Call only with fresh estimator results. A false accel_used cannot arm LIVE.
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
            self.fail(Fault::Timing);
            return None;
        }
        if self.live && !accel_used {
            self.fail(Fault::Invalid);
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
        if self.live && self.control.state() == State::Observe {
            if self.control.ready(now) && self.control.request(Request::Arm, now).is_ok() {
                self.started = Some(now);
                self.kick_state.reset();
                self.neutral_seen = false;
            }
            return None; // Arming frame always commands zero.
        }
        let limited_pd = self.control.output(pitch, rate)?;
        if self.live && !self.neutral_seen {
            self.neutral_seen = f32::from(limited_pd).abs() <= self.mapping.deadband;
            self.kick_state.reset();
            return None; // Must see neutral AFTER arming, then a meaningful request.
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
    if response.control.state() != State::Active || response.finished {
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
