//! Experimental signed PWM compensation. Pure math only; never touches motors.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub enabled: bool,
    /// Limited PD magnitude at or below this value maps to zero.
    pub deadband: f32,
    /// Must match the PD controller output limit; strictly greater than deadband.
    pub pd_output_limit: f32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,
    /// Percent points per second; None disables slew limiting.
    pub slew_per_second: Option<f32>,
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
            && self
                .slew_per_second
                .is_none_or(|v| v.is_finite() && v > 0.0)
    }
}

/// Disabled: identity with a safety ceiling. Enabled: deadband, then proportional scaling to the PWM range.
/// No startup pulse, hysteresis, or inferred wheel-motion state.
pub fn map(config: Config, raw: f32) -> Option<f32> {
    if !config.valid() || !raw.is_finite() {
        return None;
    }
    if raw == 0.0 || (config.enabled && raw.abs() <= config.deadband) {
        return Some(0.0);
    }
    let magnitude = if config.enabled {
        let fraction = (raw.abs().min(config.pd_output_limit) - config.deadband)
            / (config.pd_output_limit - config.deadband);
        config.minimum_duty + fraction * (config.maximum_duty - config.minimum_duty)
    } else {
        raw.abs()
    };
    Some(raw.signum() * magnitude.min(config.maximum_duty))
}

/// Apply once per fresh sample with measured dt. Zero stops immediately; reversal
/// discards the previous sign before ramping. Slew may pass below minimum duty.
pub fn step(config: Config, raw: f32, previous: f32, dt: f32) -> Option<f32> {
    let target = map(config, raw)?;
    if !previous.is_finite() || !dt.is_finite() || dt <= 0.0 || dt > 0.1 {
        return None;
    }
    if target == 0.0 {
        return Some(0.0);
    }
    let Some(rate) = config.slew_per_second.filter(|_| config.enabled) else {
        return Some(target);
    };
    let previous = if previous.signum() == target.signum() {
        previous.clamp(-config.maximum_duty, config.maximum_duty)
    } else {
        0.0
    };
    let change = rate * dt;
    Some(target.clamp(previous - change, previous + change))
}

/// Separate DRY-only kick ceiling; ordinary maximum_duty still caps normal PWM.
#[derive(Clone, Copy, Debug)]
pub struct KickConfig {
    pub enabled: bool,
    pub kick_duty: f32,
    pub kick_duration_ms: u64,
    pub trigger_threshold: f32,
    pub rearm_delay_ms: u64,
    pub reversal_pause_ms: u64,
}
impl KickConfig {
    pub fn valid(self, mapping: Config) -> bool {
        self.kick_duty.is_finite()
            && self.kick_duty > 0.0
            && self.kick_duty < 100.0
            && self.kick_duration_ms > 0
            && self.kick_duration_ms <= 250
            && self.trigger_threshold.is_finite()
            && self.trigger_threshold > mapping.deadband
            && self.rearm_delay_ms >= self.kick_duration_ms
            && self.reversal_pause_ms > 0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Preview {
    pub mapped_pwm: f32,
    pub final_pwm: f32,
    pub kick_active: bool,
    pub kick_elapsed_ms: u64,
}

/// Command history only: these fields do NOT describe measured wheel motion.
#[derive(Default)]
pub struct KickState {
    previous_pwm: f32,
    last_time: Option<u64>,
    direction: i8,
    stopped_since: Option<u64>,
    reversal: Option<(i8, u64)>,
    kick_started: Option<u64>,
    last_kick: Option<u64>,
    consumed: bool,
}
impl KickState {
    /// Call on stop/disarm/fault or invalid/stale orientation. No time is advanced.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(
        &mut self,
        mapping: Config,
        kick: KickConfig,
        raw: f32,
        dt: f32,
        now_ms: u64,
    ) -> Option<Preview> {
        if !kick.valid(mapping) || self.last_time.is_some_and(|t| now_ms < t) {
            self.reset();
            return None;
        }
        let Some(normal) = step(mapping, raw, self.previous_pwm, dt) else {
            self.reset();
            return None;
        };
        // Ordinary mapping and trigger decisions use the same bounded PD request.
        let raw = raw.clamp(-mapping.pd_output_limit, mapping.pd_output_limit);
        self.last_time = Some(now_ms);
        self.previous_pwm = normal;
        let mut result = Preview {
            mapped_pwm: normal,
            final_pwm: normal,
            kick_active: false,
            kick_elapsed_ms: 0,
        };
        if !mapping.enabled || !kick.enabled {
            // Disabled preview has no hidden kick history.
            self.reset();
            return Some(result);
        }
        if raw.abs() <= mapping.deadband {
            self.kick_started = None;
            self.reversal = None;
            self.previous_pwm = 0.0;
            let stopped = *self.stopped_since.get_or_insert(now_ms);
            if now_ms - stopped >= kick.rearm_delay_ms {
                self.consumed = false;
            }
            result.final_pwm = 0.0;
            return Some(result);
        }
        // Only continuous zero/deadband time rearms a same-direction request.
        if self
            .stopped_since
            .is_some_and(|t| now_ms - t >= kick.rearm_delay_ms)
        {
            self.consumed = false;
        }
        self.stopped_since = None;
        let sign = if raw > 0.0 { 1 } else { -1 };
        if self.reversal.is_some() || (self.direction != 0 && sign != self.direction) {
            self.kick_started = None;
            let (pending, since) = self.reversal.get_or_insert((sign, now_ms));
            if *pending != sign {
                *pending = sign;
                *since = now_ms;
            }
            if now_ms - *since < kick.reversal_pause_ms {
                self.previous_pwm = 0.0;
                result.final_pwm = 0.0;
                return Some(result);
            }
            self.reversal = None;
            self.direction = sign;
            self.consumed = false;
        }
        self.direction = sign;
        if !self.consumed && raw.abs() >= kick.trigger_threshold {
            // Consume the transition even during cooldown: no delayed kick while held.
            self.consumed = true;
            if self
                .last_kick
                .is_none_or(|t| now_ms - t >= kick.rearm_delay_ms)
            {
                self.kick_started = Some(now_ms);
                self.last_kick = Some(now_ms);
            }
        }
        if let Some(start) = self.kick_started {
            result.kick_elapsed_ms = now_ms - start;
            if result.kick_elapsed_ms < kick.kick_duration_ms {
                result.kick_active = true;
                result.final_pwm = f32::from(sign) * kick.kick_duty;
            } else {
                self.kick_started = None;
            }
        }
        Some(result)
    }
}
