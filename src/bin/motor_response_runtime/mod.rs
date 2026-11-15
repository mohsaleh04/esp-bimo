use embassy_time::Timer;
use esp_bimo::{
    balance,
    board::Board,
    motors::{DualMotor, DutyLimit, l298n::L298n},
    orientation::{AccelRejection, ComplementaryFilter},
    pwm_mapping,
};
use esp_hal::{
    clock::CpuClock,
    time::{Duration, Instant},
    timer::timg::{MwdtStage, TimerGroup},
};
use log::{info, warn};
#[path = "../balance_test/calibration.rs"]
mod calibration;
#[path = "../balance_motor_response_test/policy.rs"]
mod policy;
use balance::{Fault, Settings, wheel_commands};
use policy::{
    DisarmReason, LoopTiming, Output, Response, ResponseState, SensorFault, drive, input_fault,
};
mod bidirectional;
mod events;
pub mod ground;
mod ground_timing;
use events::{EventLog, Snapshot};
mod capture;
mod control_profile;
pub mod output;
mod restart_assist;
mod rolling_budget;
mod settings;
use settings::{
    ARMING_TIMEOUT_US, KICK, MAPPING, MAX_LOOP_GAP_US, MOTOR_DUTY_CEILING, PD_OUTPUT_LIMIT,
};

pub struct RunConfig {
    pub label: &'static str,
    pub live: bool,
    pub window_us: u64,
    pub ground: bool,
    pub preparation_secs: u64,
}

#[derive(Clone, Copy)]
struct Record {
    us: u64,
    pitch: f32,
    rate: f32,
    output: Option<Output>,
    ready: bool,
    state: ResponseState,
    limited_pd: Option<i16>,
    neutral: bool,
    age_us: u64,
    dt_us: u64,
    loop_us: u64,
    max_loop_us: u64,
    work_us: u64,
    missed: u32,
    poll_wait: bool,
    accel_used: bool,
    accel_age_us: Option<u64>,
    accel_rejection: Option<AccelRejection>,
}
fn log_record(r: Record, tilt_limit: f32) {
    let ready = r.ready;
    let pd = r.limited_pd;
    let state = r.state;
    let neutral = r.neutral;
    let mapped = r.output.map(|o| o.preview.mapped_pwm);
    let kick = r.output.is_some_and(|o| o.preview.kick_active);
    let elapsed = r.output.map(|o| o.preview.kick_elapsed_ms);
    let commands = r
        .output
        .map_or((0, 0), |o| wheel_commands(o.preview.final_pwm as i16));
    info!(
        "state={state:?} neutral={neutral} accel_used={} correction_age_us={:?} accel_rejection={:?} FRESH age={}us dt={}us poll_wait={} loop={}us max_loop={}us work={}us missed={} pitch={:.2} rate={:.2} ready={ready} live_tilt_ok={} tilt_cutoff_deg={tilt_limit} PD={pd:?} PWM={mapped:?} kick={kick}/{elapsed:?}ms calc_A/LEFT={} B/RIGHT={}; motors OFF",
        r.accel_used,
        r.accel_age_us,
        r.accel_rejection,
        r.age_us,
        r.dt_us,
        r.poll_wait,
        r.loop_us,
        r.max_loop_us,
        r.work_us,
        r.missed,
        r.pitch,
        r.rate,
        r.pitch.abs() < tilt_limit,
        commands.0,
        commands.1
    );
}

// The production/suspended path keeps its current behavior. Ground faults stop
// hardware at the detection checkpoint, before any later telemetry bookkeeping.
fn deadline_fault(
    ground_mode: bool,
    motors: &mut impl DualMotor,
    response: &mut Response,
    late: bool,
) -> bool {
    if ground_mode {
        ground::deadline(motors, response, late)
    } else if late && response.supervising() {
        response.fail(Fault::Timing);
        true
    } else {
        false
    }
}

fn log_ground_record(r: Record, log_max: u64) {
    let rejection = match r.accel_rejection {
        None => "ok",
        Some(AccelRejection::NonFinite) => "nonfinite",
        Some(AccelRejection::Magnitude { .. }) => "magnitude",
        Some(AccelRejection::Sideways { .. }) => "sideways",
        Some(AccelRejection::PitchPlane { .. }) => "plane",
    };
    info!(
        "GROUND {:?} p={:.1} r={:.1} PD={:?} neutral={} ready={} age={}us corr={:?} rej={} kick={} loop={}us max={}us miss={} log_max={}us; OFF",
        r.state,
        r.pitch,
        r.rate,
        r.limited_pd,
        r.neutral,
        r.ready,
        r.age_us,
        r.accel_age_us,
        rejection,
        r.output.is_some_and(|o| o.preview.kick_active),
        r.loop_us,
        r.max_loop_us,
        r.missed,
        log_max
    );
}

#[allow(dead_code)]
pub async fn run(config: RunConfig) -> ! {
    run_profile(config, false, false, output::AssistPolicy::Legacy).await
}
#[allow(dead_code)]
pub async fn run_bidirectional(config: RunConfig) -> ! {
    run_profile(config, true, false, output::AssistPolicy::Legacy).await
}
#[allow(dead_code)]
pub async fn run_balance_control(config: RunConfig) -> ! {
    run_profile(config, true, true, output::AssistPolicy::Legacy).await
}
#[allow(dead_code)]
pub async fn run_balance_control_assist(config: RunConfig, restart: bool) -> ! {
    run_profile(
        config,
        true,
        true,
        if restart {
            output::AssistPolicy::Fixed
        } else {
            output::AssistPolicy::Legacy
        },
    )
    .await
}
#[allow(dead_code)] // Shared runtime also compiles in the unchanged response binaries.
pub async fn run_balance_control_policy(config: RunConfig, policy: output::AssistPolicy) -> ! {
    run_profile(config, true, true, policy).await
}
async fn run_profile(
    config: RunConfig,
    bidirectional: bool,
    continuous: bool,
    policy: output::AssistPolicy,
) -> ! {
    let restart = policy != output::AssistPolicy::Legacy;
    let live = config.live;
    let window_us = config.window_us;
    let preparation_secs = config.preparation_secs;
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    let mut motors = L298n::new(
        board.motor_pwm,
        board.inactive_outputs,
        DutyLimit::new(MOTOR_DUTY_CEILING),
    );
    motors.disarm();
    motors.set_command_logging(false);
    let _buzzer = board.buzzer;
    let _int1 = board.bmi160_int1;
    let _qmc = board.qmc5883l_drdy;
    let mut bus = board.i2c;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);
    let mut watchdog = TimerGroup::new(board.balance_watchdog).wdt;
    watchdog.set_timeout(MwdtStage::Stage0, Duration::from_millis(100));
    watchdog.disable();
    info!(
        "{}: live={live}, ceiling={MOTOR_DUTY_CEILING}%, window={window_us}us. A=LEFT B=RIGHT; forward=(-u,+u). No serial input.",
        config.label
    );
    if continuous {
        info!(
            "OUTPUT AUTHORITY: {}; mapper Kick {}; no independent assist budget coexists",
            policy.label(),
            if restart {
                "disabled"
            } else {
                "legacy behavior"
            }
        );
    }
    if !live {
        warn!("Movement disabled by diagnostic-specific feature/source gates; DRY only.");
    }
    let valid_window = if continuous {
        window_us == control_profile::ACTIVE_US
    } else {
        ground::window_valid(window_us)
    };
    if config.ground && !valid_window {
        warn!("GROUND BLOCKED: invalid configured ACTIVE window ({window_us}us); motors disarmed.");
        loop {
            Timer::after_secs(1).await;
        }
    }
    let Some(mut response) = (if continuous {
        if restart {
            control_profile::response_with_restart(live, true)
        } else {
            control_profile::response(live)
        }
    } else {
        Response::new(
            Settings {
                output_limit: PD_OUTPUT_LIMIT,
                ..Settings::default()
            },
            MAPPING,
            if bidirectional {
                bidirectional::kick_config(KICK)
            } else {
                KICK
            },
            MOTOR_DUTY_CEILING,
            live,
            window_us,
            ARMING_TIMEOUT_US,
        )
    }) else {
        warn!("Invalid duty/timing configuration; permanently disarmed.");
        loop {
            Timer::after_secs(1).await;
        }
    };
    if continuous {
        info!(
            "CONTROL tilt cutoff=+/-{}deg; recovery projected limit={}deg",
            response.control.tilt_limit(),
            balance::CONTROL_FALL_ANGLE_DEG - restart_assist::recovery::CUTOFF_MARGIN
        );
        info!(
            "CONTROL STATE {}",
            control_profile::Stage::Calibrating.label()
        );
    }
    let setup = match calibration::initialize(&mut bus).await {
        Ok(imu) => calibration::learn_frame(&mut bus, &imu)
            .await
            .map(|(frame, bias)| (imu, frame, bias)),
        Err(e) => Err(e),
    };
    let (imu, frame, bias) = match setup {
        Ok(v) => v,
        Err(e) => {
            motors.disarm();
            if continuous {
                info!("CONTROL STATE {}", control_profile::Stage::Disarmed.label());
            }
            warn!("Calibration failed: {e}; reset required.");
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    if continuous {
        info!(
            "CONTROL STATE {}",
            control_profile::Stage::Preparing.label()
        );
    }
    info!(
        "Calibration valid; motors OFF. Secure wheels; preparation {preparation_secs}s. LIVE waits for 0.5s within +/-5deg and +/-5deg/s, then a NEW neutral sample (limited PD within +/-2). Motors stay disarmed in both waits (10s combined timeout). Only neutral acceptance starts the configured ACTIVE window; then tilt gently. Active telemetry prints AFTER disarm."
    );
    Timer::after_secs(preparation_secs).await;
    let mut filter = ComplementaryFilter::new(0.5);
    let epoch = Instant::now();
    response.begin(0);
    info!(
        "Response state={:?}; arming_timeout={ARMING_TIMEOUT_US}us begins now; ACTIVE window={window_us}us starts only on fresh post-ready neutral acceptance.",
        response.state()
    );
    if continuous {
        info!(
            "CONTROL STATE {}",
            control_profile::Stage::from_response(response.state()).label()
        );
    }
    if continuous {
        info!(
            "CONTROL CAPTURE: acquisition has no serial writes; DRY stops after {}us; LIVE window remains {}us. Dump only after DISARMED; reset required.",
            capture::DRY_CAPTURE_US,
            control_profile::ACTIVE_US
        );
    }
    let mut capture = capture::Capture::<Snapshot, 64>::new();
    let mut behavior = control_profile::Behavior::default();
    let mut logged_transitions = 0;
    let mut previous_sample = None;
    let mut timing = LoopTiming::default();
    let mut ground_timing = ground_timing::GroundTiming::default();
    let mut reported_ground_misses = 0;
    let mut latest = None;
    let mut issue = "waiting for first valid estimate";
    let mut last_accel_rejection = None;
    let mut last_bus_error = None;
    let mut previous_accel_used = None;
    let mut events = EventLog::<32>::new();
    let mut reversal = output::MotorOutput::with_policy(if continuous {
        policy
    } else {
        output::AssistPolicy::Legacy
    });
    reversal.confirm_zero(&mut motors, || epoch.elapsed().as_micros());
    let mut ground_guard = ground::GroundGuard::default();
    let mut ground_stop = None;
    let mut observed_sample_us = None;
    let mut observed_pitch = None;
    let mut observed_rate = None;
    let mut observed_accel = None;
    let mut observed_preview = None;
    let mut last_requested = (0, 0);
    let mut last_commanded = (0, 0);
    let mut last_energized = (0, 0, 0);
    let mut dropped_records = 0u32;
    let mut empty_polls = 0u32;
    let mut sample_count = 0u32;
    let mut sample_dt_us = 0u64;
    let mut last_log = 0;
    let mut motor_armed = false;
    let mut records: [Option<Record>; 64] = [None; 64];
    let mut record_count = 0;
    let mut next_record = 0;
    loop {
        let now = epoch.elapsed().as_micros();
        let mut control_pitch = None;
        response.tick(now);
        let previous_work_us = timing.work_us;
        let late = timing.begin(now, MAX_LOOP_GAP_US);
        if config.ground {
            ground_timing.begin(timing.interval_us, previous_work_us);
        }
        if deadline_fault(config.ground, &mut motors, &mut response, late) {
            issue = "loop interval exceeded 30ms deadline";
        }
        if continuous {
            control_profile::enforce(&mut response, &mut motors, late);
        }
        if continuous && response.state() == ResponseState::DryRun && now >= capture::DRY_CAPTURE_US
        {
            response.stop();
        }
        if response.finished {
            let _ = drive(
                &mut motors,
                &mut response,
                None,
                MOTOR_DUTY_CEILING,
                epoch.elapsed().as_micros(),
            );
            watchdog.disable();
            last_commanded = (0, 0);
            if bidirectional {
                reversal.reset(epoch.elapsed().as_micros(), &response);
            }
            if continuous {
                behavior.update(epoch.elapsed().as_micros(), response.state(), None, (0, 0));
                info!(
                    "CONTROL STATE {}; behavior={behavior:?}; reason={:?}",
                    control_profile::Stage::Disarmed.label(),
                    response.disarm_reason
                );
            }
            events.observe(Snapshot {
                us: now,
                sample_us: observed_sample_us,
                state: response.state(),
                pitch: observed_pitch,
                rate: observed_rate,
                pd: response.limited_pd,
                mapped: observed_preview.map(|p: pwm_mapping::Preview| p.mapped_pwm),
                kick: false,
                kick_ms: observed_preview.map(|p| p.kick_elapsed_ms),
                requested: last_requested,
                commanded: last_commanded,
                assist: reversal.trace(epoch.elapsed().as_micros()),
                reversal: bidirectional.then(|| reversal.telemetry(now)),
                last_energized,
                accel_used: observed_accel,
                rejection: last_accel_rejection,
                correction_age_us: response.accel_correction_age_us,
                sensor_fault: response.sensor_fault,
                disarm: response.disarm_reason,
                loop_us: timing.interval_us,
                work_us: timing.work_us,
                missed: timing.missed_deadlines,
            });
            if bidirectional {
                info!(
                    "BIDIR accepted_kicks={} first_applied={:?}",
                    reversal.kicks(epoch.elapsed().as_micros()),
                    events.first_applied
                );
                info!(
                    "BIDIR acknowledged_zero={:?}; release={:?}",
                    events.last_interlock_zero, events.last_interlock_release
                );
                info!(
                    "BIDIR applied_kick_on={:?}; cancelled_or_expired={:?}",
                    events.last_applied_kick_on, events.last_applied_kick_off
                );
            }
            if let Some(log) = reversal.recorder() {
                info!(
                    "RESTART ARBITER: overwritten={} first={:?} last={:?} terminal={:?}",
                    log.overwritten, log.first_applied, log.last_applied, log.terminal
                );
                for t in log.recent() {
                    info!("RESTART ARBITER {t:?}");
                }
            }
            if config.ground {
                info!(
                    "GROUND TIMING: worst={:?} poll_max={}us drive_max={}us summary_max={}us",
                    ground_timing.worst,
                    ground_timing.max_poll_us,
                    ground_timing.max_drive_us,
                    ground_timing.max_summary_us
                );
            }
            info!(
                "FLIGHT RECORDER: kick_starts={} kick_ends={} requested_reversals={} command_changes={} zero_transitions={} event_overwrites={} ground_stop={ground_stop:?}",
                events.kick_starts,
                events.kick_ends,
                events.reversals,
                events.command_changes,
                events.zero_transitions,
                events.overwritten
            );
            info!(
                "PINNED rejection_onset={:?}; last_rejection={:?}",
                events.rejection_onset, events.last_rejection
            );
            info!(
                "PINNED kick_on={:?}; kick_off={:?}",
                events.last_kick_on, events.last_kick_off
            );
            info!(
                "PINNED before_stop={:?}; TERMINAL={:?}",
                events.before_terminal, events.terminal
            );
            if continuous {
                info!(
                    "CONTROL CAPTURE: retained={} overwritten={} (periodic samples); missed_deadlines={} max_loop_us={}; terminal sample is pinned separately",
                    capture.recent().count(),
                    capture.overwritten,
                    timing.missed_deadlines,
                    timing.max_interval_us
                );
                for sample in capture.recent() {
                    info!("CONTROL SAMPLE {sample:?}");
                }
            }
            for event in events.recent() {
                info!("EVENT flags={} {:?}", event.flags, event.sample);
            }
            let age = latest.map(|r: Record| now.saturating_sub(r.us));
            for event in response.transitions()[logged_transitions..]
                .iter()
                .flatten()
            {
                if continuous {
                    info!(
                        "CONTROL STATE {} -> {} at {}us: {}",
                        control_profile::Stage::from_response(event.from).label(),
                        control_profile::Stage::from_response(event.to).label(),
                        event.us,
                        event.reason
                    );
                }
                info!(
                    "STATE {:?} -> {:?} at {}us: {}; disarm_reason={:?}",
                    event.from,
                    event.to,
                    event.us,
                    event.reason,
                    if event.to == ResponseState::Disarmed {
                        response.disarm_reason
                    } else {
                        None
                    }
                );
            }
            let detail = match response.disarm_reason {
                Some(DisarmReason::ArmingTimeout) => {
                    "readiness/neutral arming deadline expired; ACTIVE never started"
                }
                Some(DisarmReason::ActiveTimeout) => {
                    "configured ACTIVE motor-response window expired"
                }
                Some(DisarmReason::Stop) if continuous => {
                    "10-second motor-disabled DRY capture complete; reset required"
                }
                Some(DisarmReason::Fault(Fault::Stale)) => {
                    "last valid estimate exceeded 40ms freshness limit"
                }
                Some(DisarmReason::Fault(Fault::Tilt)) => {
                    "pitch reached/exceeded configured tilt cutoff"
                }
                _ => issue,
            };
            warn!(
                "DISARM: response_state={:?} reason={:?} controller={:?}; sensor_fault={:?} correction_age_us={:?} accel_rejection={last_accel_rejection:?} bus_error={last_bus_error:?} dropped_records={dropped_records}; detail={detail}; tilt_cutoff_deg={}; sample_age_us={age:?}; loop_us={} work_us={} max_loop_us={} max_work_us={} missed_deadlines={}; final A/LEFT=0 B/RIGHT=0; reset required; buffered telemetry follows.",
                response.state(),
                response.disarm_reason,
                response.control.state(),
                response.sensor_fault,
                response.accel_correction_age_us,
                response.control.tilt_limit(),
                timing.interval_us,
                timing.work_us,
                timing.max_interval_us,
                timing.max_work_us,
                timing.missed_deadlines
            );
            for r in records[..record_count].iter().flatten() {
                log_record(*r, response.control.tilt_limit());
            }
            loop {
                Timer::after_secs(1).await;
            }
        }
        let mut record = None;
        let poll_started = config.ground.then(Instant::now);
        let acquisition = imu.read(&mut bus);
        if let Some(start) = poll_started {
            ground_timing.poll(start.elapsed().as_micros());
        }
        match acquisition {
            Ok(Some(sample)) => {
                let now = epoch.elapsed().as_micros();
                sample_count = sample_count.saturating_add(1);
                observed_sample_us = Some(now);
                let (a, g) = calibration::observation(sample);
                let corrected_g = g - bias;
                last_accel_rejection = frame.accel_rejection(a);
                last_bus_error = None;
                if let Some(before) = previous_sample.replace(now) {
                    sample_dt_us = now.saturating_sub(before);
                    let dt = sample_dt_us as f32 * 1e-6;
                    let initialized = filter.pitch_deg().is_some();
                    if let Some(fault) = input_fault(a.0, corrected_g.0, sample.gyro_raw) {
                        issue = "invalid raw/corrected sensor input; no propagation allowed";
                        observed_accel = None;
                        response.sensor_fail(fault);
                        filter.reset();
                        latest = None;
                    } else if let Some(est) = filter.update(&frame, a, corrected_g, dt) {
                        observed_pitch = Some(est.pitch_deg);
                        control_pitch = Some(est.pitch_deg);
                        observed_rate = Some(est.rate_dps);
                        observed_accel = Some(est.accel_used);
                        issue = "valid estimator result";
                        let output = if config.ground {
                            ground::sample(
                                &mut motors,
                                &mut response,
                                est.pitch_deg,
                                est.rate_dps,
                                est.accel_used,
                                dt,
                                now,
                            )
                        } else {
                            response.sample(est.pitch_deg, est.rate_dps, est.accel_used, dt, now)
                        };
                        if response.disarm_reason == Some(DisarmReason::Fault(Fault::Invalid)) {
                            issue = "sensor validity policy rejected estimate; see sensor_fault";
                        } else if response.disarm_reason == Some(DisarmReason::Fault(Fault::Timing))
                        {
                            issue = "invalid/nonmonotonic sample timing or dt >40ms";
                        }
                        if bidirectional && response.state() == ResponseState::DryRun {
                            if let Some(o) = output {
                                reversal.preview(o, now);
                            } else {
                                reversal.reset(epoch.elapsed().as_micros(), &response);
                            }
                        }
                        observed_preview = output.map(|o| o.preview);
                        last_requested =
                            output.map_or((0, 0), |o| wheel_commands(o.preview.final_pwm as i16));
                        // Check real elapsed time after I2C/filter work, before issuing commands.
                        response.tick(epoch.elapsed().as_micros());
                        if deadline_fault(
                            config.ground,
                            &mut motors,
                            &mut response,
                            timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US),
                        ) {
                            issue = "acquisition/filter work exceeded 30ms deadline";
                        }
                        if continuous {
                            control_profile::enforce(
                                &mut response,
                                &mut motors,
                                timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US),
                            );
                        }
                        if response.state() == ResponseState::Active && !response.finished {
                            if !motor_armed {
                                watchdog.enable();
                                watchdog.feed();
                                motors.arm();
                                motor_armed = true;
                            }
                            let drive_started = config.ground.then(Instant::now);
                            let result = if bidirectional {
                                reversal.drive(
                                    &mut motors,
                                    &mut response,
                                    output,
                                    output::Observation {
                                        sample_us: now,
                                        pitch: est.pitch_deg,
                                        rate: est.rate_dps,
                                    },
                                    || epoch.elapsed().as_micros(),
                                )
                            } else if config.ground {
                                ground::drive(
                                    &mut motors,
                                    &mut response,
                                    &mut ground_guard,
                                    output,
                                    MOTOR_DUTY_CEILING,
                                    MAPPING.deadband,
                                    epoch.elapsed().as_micros(),
                                )
                            } else {
                                drive(
                                    &mut motors,
                                    &mut response,
                                    output,
                                    MOTOR_DUTY_CEILING,
                                    epoch.elapsed().as_micros(),
                                )
                            };
                            if let Some(start) = drive_started {
                                ground_timing.drive(start.elapsed().as_micros());
                            }
                            ground_stop = ground_guard.stop_reason();
                            if result.is_err() {
                                issue = if ground_stop.is_some() {
                                    "ground guard veto: reversal/additional kick"
                                } else {
                                    "motor output rejected"
                                };
                                response.fail(Fault::Motor);
                            } else if response.state() == ResponseState::Active
                                && !response.finished
                            {
                                if restart {
                                    last_requested = reversal
                                        .trace(epoch.elapsed().as_micros())
                                        .map_or((0, 0), |t| wheel_commands(t.requested_pwm as i16));
                                }
                                last_commanded = if bidirectional {
                                    reversal.applied()
                                } else {
                                    last_requested
                                };
                                if last_commanded != (0, 0) {
                                    last_energized = (
                                        epoch.elapsed().as_micros(),
                                        last_commanded.0,
                                        last_commanded.1,
                                    );
                                }
                            } else {
                                last_commanded = (0, 0);
                            }
                            response.tick(epoch.elapsed().as_micros());
                            if deadline_fault(
                                config.ground,
                                &mut motors,
                                &mut response,
                                timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US),
                            ) {
                                issue = "motor update exceeded 30ms loop deadline";
                            }
                            if !response.finished {
                                watchdog.feed();
                            }
                        }
                        if response.control.sample_fresh(now) {
                            record = Some(Record {
                                us: now,
                                pitch: est.pitch_deg,
                                rate: est.rate_dps,
                                output,
                                ready: response.control.ready(now),
                                state: response.state(),
                                limited_pd: response.limited_pd.or(output.map(|o| o.limited_pd)),
                                neutral: response.neutral_satisfied,
                                age_us: epoch.elapsed().as_micros().saturating_sub(now),
                                dt_us: sample_dt_us,
                                loop_us: timing.interval_us,
                                max_loop_us: timing.max_interval_us,
                                work_us: timing.work_us,
                                missed: timing.missed_deadlines,
                                poll_wait: false,
                                accel_used: est.accel_used,
                                accel_age_us: response.accel_correction_age_us,
                                accel_rejection: last_accel_rejection,
                            });
                        } else {
                            latest = None;
                        }
                    } else {
                        observed_accel = None;
                        issue = "estimator returned no orientation; see sensor_fault and accel_rejection";
                        latest = None;
                        if !dt.is_finite() || dt <= 0.0 || dt > 0.1 {
                            response.sensor_fault = Some(SensorFault::SampleTiming);
                            response.fail(Fault::Timing);
                        } else {
                            response.sensor_fail(if !initialized {
                                SensorFault::NoGravityReference
                            } else if last_accel_rejection.is_some() {
                                SensorFault::FilterGyroPropagationExpired
                            } else {
                                SensorFault::FilterInvalid
                            });
                        }
                    }
                }
            }
            Ok(None) => {
                empty_polls = empty_polls.saturating_add(1);
            }
            Err(error) => {
                issue = "BMI160 I2C read error";
                last_bus_error = Some(error);
                latest = None;
                observed_accel = None;
                response.sensor_fail(SensorFault::I2cRead);
                filter.reset();
                previous_sample = None;
            }
        }
        if deadline_fault(
            config.ground,
            &mut motors,
            &mut response,
            timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US),
        ) {
            issue = "sensor poll/loop work exceeded 30ms deadline";
        }
        if continuous
            && restart
            && record.is_none()
            && response.state() == ResponseState::Active
            && !response.finished
        {
            if reversal
                .service(&mut motors, &mut response, || epoch.elapsed().as_micros())
                .is_err()
            {
                response.fail(Fault::Motor);
            }
            last_commanded = reversal.applied();
            last_requested = reversal
                .trace(epoch.elapsed().as_micros())
                .map_or((0, 0), |t| wheel_commands(t.requested_pwm as i16));
        }
        response.tick(epoch.elapsed().as_micros());
        if continuous {
            control_profile::enforce(
                &mut response,
                &mut motors,
                timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US),
            );
        }
        if response.state() != ResponseState::Active || response.finished {
            let _ = drive(
                &mut motors,
                &mut response,
                None,
                MOTOR_DUTY_CEILING,
                epoch.elapsed().as_micros(),
            );
            last_commanded = (0, 0);
            if motor_armed {
                watchdog.disable();
            }
        }
        if bidirectional
            && (response.finished || !response.control.sample_fresh(epoch.elapsed().as_micros()))
        {
            reversal.reset(epoch.elapsed().as_micros(), &response);
        }
        if continuous {
            behavior.update(
                epoch.elapsed().as_micros(),
                response.state(),
                control_pitch,
                last_commanded,
            );
        }
        let snapshot = Snapshot {
            us: epoch.elapsed().as_micros(),
            sample_us: observed_sample_us,
            state: response.state(),
            pitch: observed_pitch,
            rate: observed_rate,
            pd: response.limited_pd,
            mapped: observed_preview.map(|p| p.mapped_pwm),
            kick: !response.finished
                && if restart {
                    reversal.telemetry(epoch.elapsed().as_micros()).kick_applied
                } else {
                    observed_preview.is_some_and(|p| p.kick_active)
                },
            kick_ms: observed_preview.map(|p| p.kick_elapsed_ms),
            requested: last_requested,
            commanded: last_commanded,
            assist: reversal.trace(epoch.elapsed().as_micros()),
            reversal: bidirectional.then(|| reversal.telemetry(epoch.elapsed().as_micros())),
            last_energized,
            accel_used: observed_accel,
            rejection: last_accel_rejection,
            correction_age_us: response.accel_correction_age_us,
            sensor_fault: response.sensor_fault,
            disarm: response.disarm_reason,
            loop_us: timing.interval_us,
            work_us: timing.work_us,
            missed: timing.missed_deadlines,
        };
        events.observe(snapshot);
        if continuous {
            capture.record(snapshot.us, snapshot);
        }
        if !motor_armed && !continuous {
            for event in response.transitions()[logged_transitions..]
                .iter()
                .flatten()
            {
                info!(
                    "STATE {:?} -> {:?} at {}us: {}; disarm_reason={:?}",
                    event.from,
                    event.to,
                    event.us,
                    event.reason,
                    if event.to == ResponseState::Disarmed {
                        response.disarm_reason
                    } else {
                        None
                    }
                );
            }
            logged_transitions = response.transitions().len();
        }
        // Keep a valid estimate across empty polls; never advance filter/kick time
        // or rewrite motor commands simply because data-ready was not asserted.
        if let Some(r) = record {
            latest = Some(r);
            if motor_armed && (r.us >= next_record || previous_accel_used != Some(r.accel_used)) {
                if record_count < records.len() {
                    records[record_count] = Some(r);
                    record_count += 1;
                } else {
                    dropped_records = dropped_records.saturating_add(1);
                }
                next_record = r.us + 100_000;
            }
            previous_accel_used = Some(r.accel_used);
        }
        if deadline_fault(
            config.ground,
            &mut motors,
            &mut response,
            timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US),
        ) {
            issue = "telemetry/event capture exceeded loop deadline";
            let _ = drive(
                &mut motors,
                &mut response,
                None,
                MOTOR_DUTY_CEILING,
                epoch.elapsed().as_micros(),
            );
            last_commanded = (0, 0);
            if motor_armed {
                watchdog.disable();
            }
        }
        if !continuous
            && !motor_armed
            && epoch.elapsed().as_micros().saturating_sub(last_log) >= 200_000
        {
            let report_now = epoch.elapsed().as_micros();
            let age = latest.map(|r| report_now.saturating_sub(r.us));
            let fresh = response.control.sample_fresh(report_now);
            let status = if fresh {
                "FRESH (empty polls are normal)"
            } else if age.is_some_and(|a| a > balance::MAX_SAMPLE_GAP_US) {
                "STALE (>40ms)"
            } else {
                issue
            };
            let summary_started = config.ground.then(Instant::now);
            if bidirectional {
                info!(
                    "BIDIR PREVIEW requested={last_requested:?} simulated={:?} actual=(0,0) interlock={:?}",
                    reversal.applied(),
                    reversal.telemetry(report_now)
                );
            }
            if config.ground && timing.missed_deadlines != reported_ground_misses {
                let gap = ground_timing.worst;
                info!(
                    "GROUND GAP interval={}us prior_work={}us summary={}us poll={}us drive={}us outside={}us miss={}; OFF",
                    gap.interval_us,
                    gap.previous_work_us,
                    gap.summary_log_us,
                    gap.sensor_poll_us,
                    gap.motor_drive_us,
                    gap.outside_work_and_log_us,
                    timing.missed_deadlines
                );
                reported_ground_misses = timing.missed_deadlines;
            } else if fresh && let Some(mut r) = latest {
                r.ready = response.control.ready(report_now);
                r.state = response.state();
                r.limited_pd = response.limited_pd;
                r.neutral = response.neutral_satisfied;
                r.accel_age_us = response.accel_correction_age_us;
                r.age_us = report_now.saturating_sub(r.us);
                r.loop_us = timing.interval_us;
                r.max_loop_us = timing.max_interval_us;
                r.work_us = timing.work_us;
                r.missed = timing.missed_deadlines;
                r.poll_wait = record.is_none();
                if config.ground {
                    log_ground_record(r, ground_timing.max_summary_us);
                } else {
                    log_record(r, response.control.tilt_limit());
                }
            } else if config.ground {
                info!(
                    "GROUND {:?} estimate={:?} rej={:?} age={age:?} corr={:?} loop={}us max={}us miss={} log_max={}us; OFF",
                    response.state(),
                    response.sensor_fault,
                    last_accel_rejection,
                    response.accel_correction_age_us,
                    timing.interval_us,
                    timing.max_interval_us,
                    timing.missed_deadlines,
                    ground_timing.max_summary_us
                );
            } else {
                info!(
                    "State={:?}; controller={:?}; limited_pd={:?}; neutral={}; sensor_fault={:?} accel_rejection={last_accel_rejection:?} correction_age_us={:?} bus_error={last_bus_error:?}; estimate={status}; age_us={age:?}; ready={}; sample_dt_us={sample_dt_us}; samples={sample_count} empty_polls={empty_polls}; loop_us={} work_us={} max_loop_us={} max_work_us={} missed_deadlines={}; motors OFF",
                    response.state(),
                    response.control.state(),
                    response.limited_pd,
                    response.neutral_satisfied,
                    response.sensor_fault,
                    response.accel_correction_age_us,
                    response.control.ready(report_now),
                    timing.interval_us,
                    timing.work_us,
                    timing.max_interval_us,
                    timing.max_work_us,
                    timing.missed_deadlines
                );
            }
            if let Some(start) = summary_started {
                ground_timing.summary(start.elapsed().as_micros());
            }
            // Timestamp AFTER blocking writes, so log duration cannot itself trigger
            // the old false fallback message. No logging takes place in ACTIVE.
            last_log = epoch.elapsed().as_micros();
        }
        Timer::after_millis(1).await;
    }
}
