//! Pure Restart Assist policy. Only the output arbiter may approve its proposals.
//! Acknowledgements describe accepted software commands, never measured wheel speed.
use super::{bidirectional::DEAD_TIME_US, rolling_budget, settings};
// One interval to schedule the next decision, one for acknowledged command IO.
const STOP_RESERVE_US: u64 = 2 * settings::MAX_LOOP_GAP_US;
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub enabled: bool,
    pub duty: f32,
    pub pulse_us: u64,
    pub cooldown_us: u64,
    pub max_pulses: u8,
    pub total_us: u64,
    pub zero_us: u64,
    pub candidate_us: u64,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            enabled: true,
            duty: settings::KICK.kick_duty,
            pulse_us: 150_000,
            cooldown_us: 1_000_000,
            max_pulses: 2,
            total_us: 300_000,
            zero_us: DEAD_TIME_US,
            candidate_us: 20_000,
        }
    }
}
impl Config {
    pub fn rolling_defaults() -> Self {
        Self {
            enabled: false,
            ..Self::default()
        }
    }
    fn valid(self) -> bool {
        self.duty.is_finite()
            && self.duty > 0.0
            && self.duty <= f32::from(settings::MOTOR_DUTY_CEILING)
            && self.pulse_us > STOP_RESERVE_US
            && self.pulse_us <= 150_000
            && self.cooldown_us >= 1_000_000
            && self.max_pulses > 0
            && self.max_pulses <= 2
            && self.total_us > STOP_RESERVE_US
            && self.total_us <= 300_000
            && self.zero_us >= DEAD_TIME_US
            && self.candidate_us >= 20_000
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Idle,
    StartupCandidate,
    AssistActive,
    Cooldown,
    BudgetExhausted,
    Disarmed,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    Sensor,
    Deadline,
    Tilt,
    Driver,
    Timeout,
    InvalidInput,
    InvalidConfig,
    Clock,
    Disarmed,
    Acknowledgement,
    Ledger,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    Disabled,
    Neutral,
    NoFreshDemand,
    ZeroInterval,
    CandidateDwell,
    Cooldown,
    Budget,
    WindowPulseLimit,
    WindowTimeLimit,
    FreshRequestRequired,
    ReversalDeadTime,
    DirectionConflict,
    PulseComplete,
    Fault(Fault),
}
#[derive(Clone, Copy)]
pub struct Input {
    pub now_us: u64,
    pub sample_us: u64,
    pub active: bool,
    pub fault: Option<Fault>,
    pub pitch: f32,
    pub rate: f32,
    pub limited_pd: i16,
    pub mapped_pwm: f32,
}
#[allow(dead_code)] // Timing fields are retained in deferred Debug telemetry.
#[derive(Clone, Copy, Debug)]
pub struct Trace {
    pub us: u64,
    pub state: State,
    pub requested: bool,
    pub eligible: bool,
    pub applied: bool,
    pub suppressed: Option<Reason>,
    pub pulse_us: u64,
    pub last_pulse_us: u64,
    pub cooldown_remaining_us: u64,
    pub pulses_remaining: u8,
    pub time_remaining_us: u64,
    pub total_on_us: u64,
    pub window: Option<rolling_budget::Snapshot>,
    pub fresh_request_required: bool,
    pub requested_pwm: f32,
    pub proposed_pwm: f32,
    pub approved_pwm: Option<f32>,
    pub commanded_pwm: f32,
}
pub struct Assist {
    config: Config,
    state: State,
    fault: Option<Fault>,
    last_update: Option<u64>,
    last_clock: Option<u64>,
    sample: Option<u64>,
    zero_since: Option<u64>,
    last_direction: i16,
    command: f32,
    assisting: bool,
    pulse_start: Option<u64>,
    last_pulse_us: u64,
    candidate: Option<(i16, u64)>,
    cooldown_end: u64,
    pulses: u8,
    on_us: u64,
    requested: bool,
    eligible: bool,
    reason: Option<Reason>,
    requested_pwm: f32,
    proposed_pwm: f32,
    proposed_assist: bool,
    rolling: Option<rolling_budget::Ledger>,
    blocked_demand: Option<i16>,
    last_demand: i16,
}
impl Assist {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            state: State::Idle,
            fault: None,
            last_update: None,
            last_clock: None,
            sample: None,
            zero_since: None,
            last_direction: 0,
            command: 0.0,
            assisting: false,
            pulse_start: None,
            last_pulse_us: 0,
            candidate: None,
            cooldown_end: 0,
            pulses: 0,
            on_us: 0,
            requested: false,
            eligible: false,
            reason: None,
            requested_pwm: 0.0,
            proposed_pwm: 0.0,
            proposed_assist: false,
            rolling: None,
            blocked_demand: None,
            last_demand: 0,
        }
    }
    pub fn new_rolling(config: Config) -> Self {
        let mut policy = Self::new(config);
        policy.rolling = Some(rolling_budget::Ledger::new());
        policy
    }
    fn signed_assist_duty(&self, sign: i16) -> f32 {
        // Robot motion: positive is forward; only rolling backward assist is reduced.
        let duty = if self.rolling.is_some() && sign < 0 {
            self.config.duty.min(65.0)
        } else {
            self.config.duty
        };
        f32::from(sign) * duty
    }
    fn usage(&self, now: u64) -> (u8, u64) {
        self.rolling
            .as_ref()
            .map_or((self.pulses, self.on_us), |ledger| {
                let w = ledger.snapshot(now);
                (w.pulses_used, w.on_us)
            })
    }
    fn advance(&mut self, now: u64) {
        if let Some(previous) = self.last_clock {
            if now < previous {
                self.fault.get_or_insert(Fault::Clock);
            }
            if self.assisting {
                self.on_us = self.on_us.saturating_add(now.saturating_sub(previous));
            }
        }
        self.last_clock = Some(now);
        if let Some(ledger) = self.rolling.as_mut() {
            ledger.expire(now);
        }
    }
    fn exhausted(&self, now: u64) -> bool {
        let (pulses, time) = self.usage(now);
        pulses >= self.config.max_pulses
            || time.saturating_add(STOP_RESERVE_US) >= self.config.total_us
    }
    fn resting_state(&self, now: u64) -> State {
        if self.exhausted(now) {
            State::BudgetExhausted
        } else if now < self.cooldown_end {
            State::Cooldown
        } else {
            State::Idle
        }
    }
    fn propose(&mut self, now: u64, pwm: f32, assist: bool, reason: Option<Reason>) -> Trace {
        self.proposed_pwm = pwm;
        self.proposed_assist = assist;
        self.reason = reason;
        self.status(now)
    }
    pub fn update(&mut self, i: Input) -> Trace {
        self.advance(i.now_us);
        if let Some(fault) = self.fault {
            self.state = State::Disarmed;
            self.eligible = false;
            return self.propose(i.now_us, 0.0, false, Some(Reason::Fault(fault)));
        }
        self.requested = false;
        self.eligible = false;
        if !self.config.valid() {
            self.fault.get_or_insert(Fault::InvalidConfig);
        }
        if self
            .last_update
            .is_some_and(|t| i.now_us.saturating_sub(t) > settings::MAX_LOOP_GAP_US)
        {
            self.fault.get_or_insert(Fault::Deadline);
        }
        self.last_update = Some(i.now_us);
        if let Some(f) = i.fault {
            self.fault.get_or_insert(f);
        }
        if !i.active {
            self.fault.get_or_insert(Fault::Disarmed);
        }
        if !i.pitch.is_finite()
            || !i.rate.is_finite()
            || !i.mapped_pwm.is_finite()
            || i.mapped_pwm.abs() > settings::MAPPING.maximum_duty
            || i.limited_pd.unsigned_abs() > settings::PD_OUTPUT_LIMIT as u16
        {
            self.fault.get_or_insert(Fault::InvalidInput);
        } else if i.pitch.abs() >= crate::balance::FALL_ANGLE_DEG {
            self.fault.get_or_insert(Fault::Tilt);
        }
        if i.sample_us > i.now_us
            || i.now_us.saturating_sub(i.sample_us) > crate::balance::MAX_SAMPLE_GAP_US
            || self.sample.is_some_and(|s| i.sample_us < s)
        {
            self.fault.get_or_insert(Fault::Sensor);
        }
        if let Some(fault) = self.fault {
            self.state = State::Disarmed;
            self.candidate = None;
            return self.propose(i.now_us, 0.0, false, Some(Reason::Fault(fault)));
        }
        let fresh = self.sample != Some(i.sample_us);
        self.sample = Some(i.sample_us);
        let sign = i.limited_pd.signum();
        let neutral = f32::from(i.limited_pd).abs() <= settings::MAPPING.deadband;
        if self.rolling.is_some() && fresh {
            // A denial is consumed for this held demand. Credit recovery is NOT
            // a new request. Neutral/new sign can start another attempt, but
            // cannot clear any ledger entry or waive zero/dwell/cooldown checks.
            if neutral || (self.last_demand != 0 && sign != self.last_demand) {
                self.blocked_demand = None;
            }
            self.last_demand = if neutral { 0 } else { sign };
        }
        if (neutral && i.mapped_pwm != 0.0)
            || (!neutral && (i.mapped_pwm == 0.0 || i.mapped_pwm.signum() != f32::from(sign)))
        {
            self.fault.get_or_insert(Fault::InvalidInput);
            self.state = State::Disarmed;
            return self.propose(
                i.now_us,
                0.0,
                false,
                Some(Reason::Fault(Fault::InvalidInput)),
            );
        }
        self.requested =
            !neutral && (self.zero_since.is_some() || self.candidate.is_some() || self.assisting);
        self.requested_pwm = if self.requested {
            self.signed_assist_duty(sign)
        } else {
            i.mapped_pwm
        };
        if neutral {
            self.candidate = None;
            self.state = self.resting_state(i.now_us);
            return self.propose(i.now_us, 0.0, false, Some(Reason::Neutral));
        }
        if self.rolling.is_some()
            && !self.assisting
            && (self.exhausted(i.now_us) || i.now_us < self.cooldown_end)
        {
            // Consume a denied attempt even while a reversal's zero interlock
            // masks its output. Recovery during that wait must not become a kick.
            self.blocked_demand = Some(sign);
            self.candidate = None;
        }
        if self.last_direction != 0
            && sign != self.last_direction
            && self
                .zero_since
                .is_none_or(|t| i.now_us.saturating_sub(t) < DEAD_TIME_US)
        {
            self.candidate = None;
            let reason = if self.assisting {
                Reason::DirectionConflict
            } else {
                Reason::ReversalDeadTime
            };
            self.state = self.resting_state(i.now_us);
            return self.propose(i.now_us, 0.0, false, Some(reason));
        }
        if self.assisting {
            let elapsed = self.pulse_start.map_or(0, |t| i.now_us.saturating_sub(t));
            // Reserve scheduling AND command-acknowledgement time. Otherwise
            // stopping only at the nominal deadline would exceed the bound.
            let remaining = self
                .config
                .pulse_us
                .saturating_sub(elapsed)
                .min(self.config.total_us.saturating_sub(self.usage(i.now_us).1));
            if remaining > STOP_RESERVE_US {
                self.state = State::AssistActive;
                self.eligible = true;
                return self.propose(i.now_us, self.signed_assist_duty(sign), true, None);
            }
            self.state = self.resting_state(i.now_us);
            return self.propose(i.now_us, i.mapped_pwm, false, Some(Reason::PulseComplete));
        }
        self.state = self.resting_state(i.now_us);
        if !self.config.enabled {
            return self.propose(i.now_us, i.mapped_pwm, false, Some(Reason::Disabled));
        }
        if self.exhausted(i.now_us) {
            let reason = if self.rolling.is_some() {
                self.blocked_demand = Some(sign);
                self.candidate = None;
                if self.usage(i.now_us).0 >= self.config.max_pulses {
                    Reason::WindowPulseLimit
                } else {
                    Reason::WindowTimeLimit
                }
            } else {
                Reason::Budget
            };
            return self.propose(i.now_us, i.mapped_pwm, false, Some(reason));
        }
        if i.now_us < self.cooldown_end {
            if self.rolling.is_some() {
                self.blocked_demand = Some(sign);
                self.candidate = None;
            }
            return self.propose(i.now_us, i.mapped_pwm, false, Some(Reason::Cooldown));
        }
        if self.rolling.is_some() && self.blocked_demand == Some(sign) {
            return self.propose(
                i.now_us,
                i.mapped_pwm,
                false,
                Some(Reason::FreshRequestRequired),
            );
        }
        if self
            .zero_since
            .is_none_or(|t| i.now_us.saturating_sub(t) < self.config.zero_us)
        {
            return self.propose(i.now_us, i.mapped_pwm, false, Some(Reason::ZeroInterval));
        }
        if !fresh {
            return self.propose(i.now_us, 0.0, false, Some(Reason::NoFreshDemand));
        }
        let (direction, since) = *self.candidate.get_or_insert((sign, i.now_us));
        if direction != sign {
            self.candidate = Some((sign, i.now_us));
        }
        self.state = State::StartupCandidate;
        if direction != sign || i.now_us.saturating_sub(since) < self.config.candidate_us {
            return self.propose(i.now_us, 0.0, false, Some(Reason::CandidateDwell));
        }
        self.eligible = true;
        self.propose(i.now_us, self.signed_assist_duty(sign), true, None)
    }
    /// Caller may acknowledge ONLY after successful driver completion. This module
    /// has no motor handle. No acknowledge is evidence of physical motion/stoppage.
    pub fn acknowledge(&mut self, now: u64, pwm: f32, assisted: bool) -> Trace {
        self.advance(now);
        if self
            .last_update
            .is_some_and(|t| now.saturating_sub(t) > settings::MAX_LOOP_GAP_US)
        {
            self.fault.get_or_insert(Fault::Deadline);
        }
        if !pwm.is_finite()
            || (pwm != 0.0 && (pwm != f32::from(self.proposed_pwm as i16) || self.fault.is_some()))
            || (pwm != 0.0 && assisted != self.proposed_assist)
            || (assisted && pwm == 0.0)
        {
            self.fault.get_or_insert(Fault::Acknowledgement);
        }
        if assisted && !self.assisting {
            if let Some(ledger) = self.rolling.as_mut()
                && ledger.begin(now).is_err()
            {
                self.fault.get_or_insert(Fault::Ledger);
            }
            self.pulses = self.pulses.saturating_add(1);
            self.pulse_start = Some(now);
            self.candidate = None;
        } else if !assisted && self.assisting {
            if let Some(ledger) = self.rolling.as_mut()
                && ledger.end(now).is_err()
            {
                self.fault.get_or_insert(Fault::Ledger);
            }
            self.last_pulse_us = self.pulse_start.map_or(0, |t| now.saturating_sub(t));
            self.pulse_start = None;
            self.cooldown_end = now.saturating_add(self.config.cooldown_us);
        }
        self.command = pwm;
        self.assisting = assisted; // Acknowledged output remains a fact even after a fault.
        if pwm == 0.0 {
            self.zero_since.get_or_insert(now);
        } else {
            self.zero_since = None;
            self.last_direction = (pwm as i16).signum();
        }
        self.state = if self.fault.is_some() {
            State::Disarmed
        } else if self.assisting {
            State::AssistActive
        } else if self.candidate.is_some() {
            State::StartupCandidate
        } else {
            self.resting_state(now)
        };
        if let Some(fault) = self.fault {
            self.reason = Some(Reason::Fault(fault));
            self.proposed_pwm = 0.0;
            self.proposed_assist = false;
            self.eligible = false;
        }
        self.status(now)
    }
    pub fn stop(&mut self, now: u64, fault: Fault) {
        self.advance(now);
        self.fault.get_or_insert(fault);
        self.state = State::Disarmed;
        self.candidate = None;
        self.eligible = false;
        self.proposed_pwm = 0.0;
        self.proposed_assist = false;
        self.reason = Some(Reason::Fault(self.fault.unwrap()));
    }
    pub fn status(&self, now: u64) -> Trace {
        Trace {
            us: now,
            state: self.state,
            requested: self.requested,
            eligible: self.eligible,
            applied: self.assisting,
            suppressed: self.reason,
            pulse_us: self.pulse_start.map_or(0, |t| now.saturating_sub(t)),
            last_pulse_us: self.last_pulse_us,
            cooldown_remaining_us: self.cooldown_end.saturating_sub(now),
            pulses_remaining: self.config.max_pulses.saturating_sub(self.usage(now).0),
            time_remaining_us: self.config.total_us.saturating_sub(self.usage(now).1),
            total_on_us: self.on_us,
            window: self.rolling.as_ref().map(|ledger| ledger.snapshot(now)),
            fresh_request_required: self.blocked_demand.is_some(),
            requested_pwm: self.requested_pwm,
            proposed_pwm: self.proposed_pwm,
            approved_pwm: None,
            commanded_pwm: self.command,
        }
    }
    pub fn pulses_applied(&self) -> u8 {
        self.pulses
    }
    pub fn proposed_assist(&self) -> bool {
        self.proposed_assist
    }
}
/// Rolling trace with pinned first/last applied pulse and terminal context.
/// Recording never formats, allocates, waits, or changes safety decisions.
pub struct Recorder<const N: usize> {
    ring: [Option<Trace>; N],
    cursor: usize,
    count: usize,
    pub overwritten: u32,
    pub first_applied: Option<Trace>,
    pub last_applied: Option<Trace>,
    pub terminal: Option<Trace>,
}
impl<const N: usize> Recorder<N> {
    pub fn new() -> Self {
        assert!(N > 0);
        Self {
            ring: [None; N],
            cursor: 0,
            count: 0,
            overwritten: 0,
            first_applied: None,
            last_applied: None,
            terminal: None,
        }
    }
    pub fn record(&mut self, t: Trace) {
        if t.applied {
            self.first_applied.get_or_insert(t);
            self.last_applied = Some(t);
        }
        if t.state == State::Disarmed {
            self.terminal = Some(t);
        }
        if self.count == N {
            self.overwritten = self.overwritten.saturating_add(1);
        } else {
            self.count += 1;
        }
        self.ring[self.cursor] = Some(t);
        self.cursor = (self.cursor + 1) % N;
    }
    pub fn recent(&self) -> impl Iterator<Item = Trace> + '_ {
        let start = if self.count == N { self.cursor } else { 0 };
        (0..self.count).filter_map(move |i| self.ring[(start + i) % N])
    }
}
