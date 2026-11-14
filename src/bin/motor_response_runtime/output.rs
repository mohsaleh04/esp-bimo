//! Single selected output authority: legacy one-Kick OR shared Restart Assist budget.
use super::{
    bidirectional::{Interlock, Telemetry, ZeroInterlock},
    policy::{self, Output, Response, ResponseState},
    restart_assist::{Assist, Config, Fault, Input, Reason, Recorder, State, Trace},
    settings,
};
use crate::{
    balance,
    motors::{Disarmed, DualMotor},
    pwm_mapping::Preview,
};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssistPolicy {
    Legacy,
    Fixed,
    Rolling,
}
impl AssistPolicy {
    pub fn label(self) -> &'static str {
        match self {
            Self::Legacy => "legacy: one Kick",
            Self::Fixed => "Restart Assist: two total pulses",
            Self::Rolling => "Rolling Restart Assist: two pulses per 5 seconds",
        }
    }
}
/// Conflicting features or an unreviewed selection never grant LIVE permission.
#[allow(dead_code)] // Shared runtime also compiles in the unchanged response binaries.
pub fn select_policy(
    fixed_feature: bool,
    fixed_reviewed: bool,
    rolling_feature: bool,
    rolling_reviewed: bool,
) -> Option<AssistPolicy> {
    match (fixed_feature, rolling_feature) {
        (true, true) => None,
        (true, false) => fixed_reviewed.then_some(AssistPolicy::Fixed),
        (false, true) => rolling_reviewed.then_some(AssistPolicy::Rolling),
        (false, false) => Some(AssistPolicy::Legacy),
    }
}
#[derive(Clone, Copy)]
pub struct Observation {
    pub sample_us: u64,
    pub pitch: f32,
    pub rate: f32,
}
#[allow(clippy::large_enum_variant)] // Fixed embedded storage; constructed before control, no heap.
pub enum MotorOutput {
    Legacy(Interlock),
    Restart(RestartOutput),
}
impl MotorOutput {
    #[allow(dead_code)] // Compatibility for fixed-policy host callers.
    pub fn new(restart: bool) -> Self {
        Self::with_policy(if restart {
            AssistPolicy::Fixed
        } else {
            AssistPolicy::Legacy
        })
    }
    pub fn with_policy(policy: AssistPolicy) -> Self {
        match policy {
            AssistPolicy::Legacy => Self::Legacy(Interlock::default()),
            AssistPolicy::Fixed | AssistPolicy::Rolling => {
                Self::Restart(RestartOutput::new(policy))
            }
        }
    }
    pub fn confirm_zero(&mut self, m: &mut impl DualMotor, mut clock: impl FnMut() -> u64) {
        if let Self::Restart(r) = self {
            m.disarm();
            let now = clock();
            r.zero.acknowledge(0, now);
            r.assist.acknowledge(now, 0.0, false);
        }
    }
    pub fn drive(
        &mut self,
        m: &mut impl DualMotor,
        r: &mut Response,
        o: Option<Output>,
        observation: Observation,
        mut clock: impl FnMut() -> u64,
    ) -> Result<(), Disarmed> {
        match self {
            Self::Legacy(g) => g.drive(m, r, o, settings::MOTOR_DUTY_CEILING, clock),
            Self::Restart(g) => g.drive(m, r, o, observation, &mut clock),
        }
    }
    pub fn service(
        &mut self,
        m: &mut impl DualMotor,
        r: &mut Response,
        mut clock: impl FnMut() -> u64,
    ) -> Result<(), Disarmed> {
        if let Self::Restart(g) = self
            && let Some((o, obs)) = g.last_sample
        {
            return g.drive(m, r, Some(o), obs, &mut clock);
        }
        Ok(())
    }
    /// DRY legacy preview stays mathematical. Restart budgets are not consumed in DRY.
    pub fn preview(&mut self, o: Output, now: u64) {
        if let Self::Legacy(g) = self {
            g.preview(o, now);
        }
    }
    /// Caller has already shut down actual outputs; acknowledge that completion.
    pub fn reset(&mut self, now: u64, response: &Response) {
        match self {
            Self::Legacy(g) => g.reset(),
            Self::Restart(g) => {
                if response.finished || g.started {
                    g.assist.stop(now, reason(response));
                }
                g.zero.acknowledge(0, now);
                g.zero.clear_pending();
                let mut t = g.assist.acknowledge(now, 0.0, false);
                t.approved_pwm = Some(0.0);
                g.record(t);
                g.applied = (0, 0);
                g.approved = Some(0.0);
                g.gate_blocked = false;
            }
        }
    }
    pub fn applied(&self) -> (i16, i16) {
        match self {
            Self::Legacy(g) => g.applied,
            Self::Restart(g) => g.applied,
        }
    }
    pub fn telemetry(&self, now: u64) -> Telemetry {
        match self {
            Self::Legacy(g) => g.telemetry(now),
            Self::Restart(g) => {
                let t = g.assist.status(now);
                g.zero.telemetry(now, g.assist.pulses_applied(), t.applied)
            }
        }
    }
    pub fn kicks(&self, now: u64) -> u8 {
        self.telemetry(now).kicks
    }
    pub fn trace(&self, now: u64) -> Option<Trace> {
        if let Self::Restart(g) = self {
            let mut t = g.assist.status(now);
            t.approved_pwm = g.approved;
            if g.gate_blocked && t.state != State::Disarmed {
                t.suppressed = Some(Reason::ReversalDeadTime);
            }
            Some(t)
        } else {
            None
        }
    }
    pub fn recorder(&self) -> Option<&Recorder<64>> {
        if let Self::Restart(g) = self {
            Some(&g.log)
        } else {
            None
        }
    }
}
pub struct RestartOutput {
    assist: Assist,
    zero: ZeroInterlock,
    log: Recorder<64>,
    last_record: Option<Trace>,
    last_sample: Option<(Output, Observation)>,
    started: bool,
    applied: (i16, i16),
    approved: Option<f32>,
    gate_blocked: bool,
}
impl RestartOutput {
    fn new(policy: AssistPolicy) -> Self {
        let assist = if policy == AssistPolicy::Rolling {
            Assist::new_rolling(Config {
                enabled: true,
                ..Config::rolling_defaults()
            })
        } else {
            Assist::new(Config {
                enabled: true,
                ..Default::default()
            })
        };
        Self {
            assist,
            zero: Default::default(),
            log: Recorder::new(),
            last_record: None,
            last_sample: None,
            started: false,
            applied: (0, 0),
            approved: None,
            gate_blocked: false,
        }
    }
    fn record(&mut self, t: Trace) {
        let changed = self.last_record.is_none_or(|p| {
            p.state != t.state
                || p.requested != t.requested
                || p.eligible != t.eligible
                || p.applied != t.applied
                || p.suppressed != t.suppressed
                || p.requested_pwm != t.requested_pwm
                || p.proposed_pwm != t.proposed_pwm
                || (t.approved_pwm.is_some() && p.approved_pwm != t.approved_pwm)
                || p.commanded_pwm != t.commanded_pwm
                || p.pulses_remaining != t.pulses_remaining
                || p.fresh_request_required != t.fresh_request_required
        });
        if changed {
            self.log.record(t);
            self.last_record = Some(t);
        }
    }
    fn stop(&mut self, m: &mut impl DualMotor, clock: &mut impl FnMut() -> u64, fault: Fault) {
        m.disarm();
        let now = clock();
        self.assist.stop(now, fault);
        self.zero.acknowledge(0, now);
        self.zero.clear_pending();
        let mut t = self.assist.acknowledge(now, 0.0, false);
        t.approved_pwm = Some(0.0);
        self.record(t);
        self.applied = (0, 0);
        self.approved = Some(0.0);
        self.gate_blocked = false;
    }
    fn fail(
        &mut self,
        m: &mut impl DualMotor,
        r: &mut Response,
        clock: &mut impl FnMut() -> u64,
        fault: Fault,
    ) {
        // Physical shutdown precedes bookkeeping; preserve an existing policy reason.
        m.disarm();
        if !r.finished {
            r.abort(match fault {
                Fault::Deadline | Fault::Clock => balance::Fault::Timing,
                Fault::Tilt => balance::Fault::Tilt,
                Fault::Sensor => balance::Fault::Invalid,
                _ => balance::Fault::Motor,
            });
        }
        self.stop(m, clock, fault);
    }
    fn drive(
        &mut self,
        m: &mut impl DualMotor,
        r: &mut Response,
        o: Option<Output>,
        obs: Observation,
        clock: &mut impl FnMut() -> u64,
    ) -> Result<(), Disarmed> {
        let now = clock();
        r.tick(now);
        if r.state() != ResponseState::Active || r.finished {
            m.disarm();
            if r.finished || self.started {
                self.stop(m, clock, reason(r));
            }
            return Ok(());
        }
        self.started = true;
        let o = o.unwrap_or(Output {
            limited_pd: r.limited_pd.unwrap_or(0),
            preview: Preview {
                mapped_pwm: 0.0,
                final_pwm: 0.0,
                kick_active: false,
                kick_elapsed_ms: 0,
            },
        });
        // Integrated mode has no mapper Kick generator and never accepts its pulses.
        if o.preview.kick_active || o.preview.final_pwm != o.preview.mapped_pwm {
            self.fail(m, r, clock, Fault::InvalidInput);
            return Err(Disarmed);
        }
        let fresh_sample = self
            .last_sample
            .is_none_or(|(_, previous)| previous.sample_us != obs.sample_us);
        self.last_sample = Some((o, obs));
        let t = self.assist.update(Input {
            now_us: now,
            sample_us: obs.sample_us,
            active: true,
            fault: None,
            pitch: obs.pitch,
            rate: obs.rate,
            limited_pd: o.limited_pd,
            mapped_pwm: o.preview.mapped_pwm,
        });
        self.record(t);
        if t.state == State::Disarmed {
            self.fail(
                m,
                r,
                clock,
                match t.suppressed {
                    Some(Reason::Fault(f)) => f,
                    _ => Fault::InvalidInput,
                },
            );
            return Err(Disarmed);
        }
        let direction = if o.preview.mapped_pwm == 0.0 {
            0
        } else {
            o.limited_pd.signum()
        };
        let allowed = self.zero.approve(direction, now);
        let approved = if allowed { t.proposed_pwm } else { 0.0 };
        if !approved.is_finite()
            || approved.abs() > f32::from(settings::MOTOR_DUTY_CEILING)
            || (approved != 0.0 && approved.signum() != f32::from(direction))
        {
            self.fail(m, r, clock, Fault::InvalidInput);
            return Err(Disarmed);
        }
        self.approved = Some(approved);
        self.gate_blocked = !allowed;
        let mut approval = t;
        approval.approved_pwm = Some(approved);
        if !allowed {
            approval.suppressed = Some(Reason::ReversalDeadTime);
        }
        self.record(approval);
        let assisted = allowed && self.assist.proposed_assist() && approved != 0.0;
        let command = balance::wheel_commands(approved as i16);
        // Keep the existing driver-write cadence for fresh samples. Empty polls
        // service pulse timing without adding unnecessary GPIO/PWM reattachments.
        let needs_write = fresh_sample
            || command != self.applied
            || assisted != t.applied
            || self.zero.needs_zero_ack();
        if needs_write {
            let mut approved_output = o;
            approved_output.preview.final_pwm = approved;
            approved_output.preview.kick_active = false;
            let gate_time = clock();
            if gate_time < now || gate_time.saturating_sub(now) > settings::MAX_LOOP_GAP_US {
                self.fail(m, r, clock, Fault::Deadline);
                return Err(Disarmed);
            }
            if policy::drive(
                m,
                r,
                Some(approved_output),
                settings::MOTOR_DUTY_CEILING,
                gate_time,
            )
            .is_err()
            {
                self.fail(m, r, clock, Fault::Driver);
                return Err(Disarmed);
            }
            if r.finished {
                self.stop(m, clock, Fault::Disarmed);
                return Ok(());
            }
            let done = clock();
            self.zero.acknowledge(approved as i16, done);
            let mut ack = self
                .assist
                .acknowledge(done, f32::from(command.1), assisted);
            ack.approved_pwm = Some(approved);
            if !allowed {
                ack.suppressed = Some(Reason::ReversalDeadTime);
            }
            self.applied = command;
            self.record(ack);
            if ack.state == State::Disarmed {
                self.fail(
                    m,
                    r,
                    clock,
                    match ack.suppressed {
                        Some(Reason::Fault(f)) => f,
                        _ => Fault::Acknowledgement,
                    },
                );
                return Err(Disarmed);
            }
        }
        let done = clock();
        if done < now || done.saturating_sub(now) > settings::MAX_LOOP_GAP_US {
            self.fail(m, r, clock, Fault::Deadline);
            return Err(Disarmed);
        }
        r.tick(done);
        if r.finished {
            self.stop(m, clock, reason(r));
        }
        Ok(())
    }
}

fn reason(r: &Response) -> Fault {
    match r.disarm_reason {
        Some(policy::DisarmReason::ActiveTimeout) => Fault::Timeout,
        Some(policy::DisarmReason::Fault(balance::Fault::Tilt)) => Fault::Tilt,
        Some(policy::DisarmReason::Fault(balance::Fault::Timing)) => Fault::Deadline,
        Some(policy::DisarmReason::Fault(balance::Fault::Motor)) => Fault::Driver,
        Some(policy::DisarmReason::Fault(balance::Fault::Invalid | balance::Fault::Stale)) => {
            Fault::Sensor
        }
        _ => Fault::Disarmed,
    }
}
