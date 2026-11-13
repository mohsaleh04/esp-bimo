#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{
    balance,
    board::Board,
    motors::{DualMotor, DutyLimit, l298n::L298n},
    orientation::ComplementaryFilter,
    pwm_mapping,
};
use esp_hal::{
    clock::CpuClock,
    time::{Duration, Instant},
    timer::timg::{MwdtStage, TimerGroup},
};
use log::{info, warn};
#[path = "balance_test/calibration.rs"]
mod calibration;
#[path = "balance_motor_response_test/policy.rs"]
mod policy;
use balance::{Fault, Settings, State, wheel_commands};
use esp_bimo::motors;
use policy::{LoopTiming, Output, Response, drive};
use pwm_mapping::{Config, KickConfig};
esp_bootloader_esp_idf::esp_app_desc!();

// Only for physically supported, suspended wheels. This is NOT floor balancing.
// Review the supply/current and ceiling, then explicitly edit this flag for LIVE.
const SUPPLY_AND_CEILING_VERIFIED: bool = false;
const MOTOR_DUTY_CEILING: u8 = 70;
const WINDOW_US: u64 = 2_000_000;
const PREPARATION_SECS: u64 = 15;
const MAX_LOOP_GAP_US: u64 = 30_000;
const PD_OUTPUT_LIMIT: i16 = 15; // Independent of ordinary PWM maximum.
const MAPPING: Config = Config {
    enabled: true,
    deadband: 2.0,
    pd_output_limit: PD_OUTPUT_LIMIT as f32,
    minimum_duty: 30.0,
    maximum_duty: 50.0,
    slew_per_second: None,
};
const KICK: KickConfig = KickConfig {
    enabled: true,
    kick_duty: 70.0,
    kick_duration_ms: 150,
    trigger_threshold: 4.0,
    rearm_delay_ms: 300,
    reversal_pause_ms: 100,
};
const LIVE: bool = cfg!(feature = "balance-motor-response-live") && SUPPLY_AND_CEILING_VERIFIED;

#[derive(Clone, Copy)]
struct Record {
    us: u64,
    pitch: f32,
    rate: f32,
    output: Option<Output>,
    ready: bool,
    age_us: u64,
    dt_us: u64,
    loop_us: u64,
    max_loop_us: u64,
    work_us: u64,
    missed: u32,
    poll_wait: bool,
}
fn log_record(r: Record) {
    let ready = r.ready;
    let pd = r.output.map(|o| o.limited_pd);
    let mapped = r.output.map(|o| o.preview.mapped_pwm);
    let kick = r.output.is_some_and(|o| o.preview.kick_active);
    let elapsed = r.output.map(|o| o.preview.kick_elapsed_ms);
    let commands = r
        .output
        .map_or((0, 0), |o| wheel_commands(o.preview.final_pwm as i16));
    info!(
        "FRESH age={}us dt={}us poll_wait={} loop={}us max_loop={}us work={}us missed={} pitch={:.2} rate={:.2} ready={ready} live_tilt_ok={} PD={pd:?} PWM={mapped:?} kick={kick}/{elapsed:?}ms calc_A/LEFT={} B/RIGHT={}; motors OFF",
        r.age_us,
        r.dt_us,
        r.poll_wait,
        r.loop_us,
        r.max_loop_us,
        r.work_us,
        r.missed,
        r.pitch,
        r.rate,
        r.pitch.abs() < balance::FALL_ANGLE_DEG,
        commands.0,
        commands.1
    );
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
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
        "SUSPENDED WHEELS ONLY: live={LIVE}, ceiling={MOTOR_DUTY_CEILING}%, window={WINDOW_US}us. A=LEFT B=RIGHT; forward=(-u,+u). No serial input."
    );
    if cfg!(feature = "balance-motor-response-live") && !SUPPLY_AND_CEILING_VERIFIED {
        warn!("LIVE BLOCKED by source supply/ceiling flag; running DRY only.");
    }
    let Some(mut response) = Response::new(
        Settings {
            output_limit: PD_OUTPUT_LIMIT,
            ..Settings::default()
        },
        MAPPING,
        KICK,
        MOTOR_DUTY_CEILING,
        LIVE,
        WINDOW_US,
    ) else {
        warn!("Invalid duty/timing configuration; permanently disarmed.");
        loop {
            Timer::after_secs(1).await;
        }
    };
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
            warn!("Calibration failed: {e}; reset required.");
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    info!(
        "Calibration valid; motors OFF. Secure wheels; preparation {PREPARATION_SECS}s. LIVE then arms ONCE after 0.5s within +/-5deg and +/-5deg/s. After arming, first hold neutral (PD within +/-2), then tilt gently. Active telemetry prints AFTER disarm."
    );
    Timer::after_secs(PREPARATION_SECS).await;
    let mut filter = ComplementaryFilter::new(0.5);
    let epoch = Instant::now();
    let mut previous_sample = None;
    let mut timing = LoopTiming::default();
    let mut latest = None;
    let mut issue = "waiting for first valid estimate";
    let mut empty_polls = 0u32;
    let mut sample_count = 0u32;
    let mut sample_dt_us = 0u64;
    let mut last_log = 0;
    let mut motor_armed = false;
    let mut records: [Option<Record>; 32] = [None; 32];
    let mut record_count = 0;
    let mut next_record = 0;
    loop {
        let now = epoch.elapsed().as_micros();
        response.tick(now);
        let late = timing.begin(now, MAX_LOOP_GAP_US);
        if motor_armed && late {
            issue = "loop interval exceeded 30ms deadline";
            response.fail(Fault::Timing);
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
            let age = latest.map(|r: Record| now.saturating_sub(r.us));
            let detail = match response.control.state() {
                State::Fault(Fault::Stale) => "last valid estimate exceeded 40ms freshness limit",
                State::Fault(Fault::Tilt) => "pitch reached/exceeded 15deg cutoff",
                State::Fault(Fault::TrialExpired) => "one-shot response window expired",
                _ => issue,
            };
            warn!(
                "DISARM: {:?}; detail={detail}; sample_age_us={age:?}; loop_us={} work_us={} max_loop_us={} max_work_us={} missed_deadlines={}; final A/LEFT=0 B/RIGHT=0; reset required; buffered telemetry follows.",
                response.control.state(),
                timing.interval_us,
                timing.work_us,
                timing.max_interval_us,
                timing.max_work_us,
                timing.missed_deadlines
            );
            for r in records[..record_count].iter().flatten() {
                log_record(*r);
            }
            loop {
                Timer::after_secs(1).await;
            }
        }
        let mut record = None;
        match imu.read(&mut bus) {
            Ok(Some(sample)) => {
                let now = epoch.elapsed().as_micros();
                sample_count = sample_count.saturating_add(1);
                let (a, g) = calibration::observation(sample);
                if let Some(before) = previous_sample.replace(now) {
                    sample_dt_us = now.saturating_sub(before);
                    let dt = sample_dt_us as f32 * 1e-6;
                    if let Some(est) = filter.update(&frame, a, g - bias, dt) {
                        issue = "valid estimator result";
                        let output =
                            response.sample(est.pitch_deg, est.rate_dps, est.accel_used, dt, now);
                        if response.control.state() == State::Fault(Fault::Invalid) {
                            issue = "invalid/gyro-only estimate rejected by LIVE policy";
                        } else if response.control.state() == State::Fault(Fault::Timing) {
                            issue = "invalid/nonmonotonic sample timing or dt >40ms";
                        }
                        // Check real elapsed time after I2C/filter work, before issuing commands.
                        response.tick(epoch.elapsed().as_micros());
                        if timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US)
                            && motor_armed
                        {
                            issue = "acquisition/filter work exceeded 30ms deadline";
                            response.fail(Fault::Timing);
                        }
                        if response.control.state() == State::Active && !response.finished {
                            if !motor_armed {
                                watchdog.enable();
                                watchdog.feed();
                                motors.arm();
                                motor_armed = true;
                            }
                            if drive(
                                &mut motors,
                                &mut response,
                                output,
                                MOTOR_DUTY_CEILING,
                                epoch.elapsed().as_micros(),
                            )
                            .is_err()
                            {
                                issue = "motor output rejected";
                                response.fail(Fault::Motor);
                            }
                            response.tick(epoch.elapsed().as_micros());
                            if timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US) {
                                issue = "motor update exceeded 30ms loop deadline";
                                response.fail(Fault::Timing);
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
                                age_us: epoch.elapsed().as_micros().saturating_sub(now),
                                dt_us: sample_dt_us,
                                loop_us: timing.interval_us,
                                max_loop_us: timing.max_interval_us,
                                work_us: timing.work_us,
                                missed: timing.missed_deadlines,
                                poll_wait: false,
                            });
                        } else {
                            latest = None;
                        }
                    } else {
                        issue = "estimator rejected dt/input/gravity reference";
                        latest = None;
                        response.fail(Fault::Invalid);
                    }
                }
            }
            Ok(None) => {
                empty_polls = empty_polls.saturating_add(1);
            }
            Err(_) => {
                issue = "BMI160 I2C read error";
                latest = None;
                response.fail(Fault::Invalid);
                filter.reset();
                previous_sample = None;
            }
        }
        if timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US) && motor_armed {
            issue = "sensor poll/loop work exceeded 30ms deadline";
            response.fail(Fault::Timing);
        }
        response.tick(epoch.elapsed().as_micros());
        if response.control.state() != State::Active || response.finished {
            let _ = drive(
                &mut motors,
                &mut response,
                None,
                MOTOR_DUTY_CEILING,
                epoch.elapsed().as_micros(),
            );
            if motor_armed {
                watchdog.disable();
            }
        }
        // Keep a valid estimate across empty polls; never advance filter/kick time
        // or rewrite motor commands simply because data-ready was not asserted.
        if let Some(r) = record {
            latest = Some(r);
            if motor_armed && r.us >= next_record && record_count < records.len() {
                records[record_count] = Some(r);
                record_count += 1;
                next_record = r.us + 100_000;
            }
        }
        timing.check_work(epoch.elapsed().as_micros(), MAX_LOOP_GAP_US);
        if !motor_armed && epoch.elapsed().as_micros().saturating_sub(last_log) >= 200_000 {
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
            if fresh && let Some(mut r) = latest {
                r.ready = response.control.ready(report_now);
                r.age_us = report_now.saturating_sub(r.us);
                r.loop_us = timing.interval_us;
                r.max_loop_us = timing.max_interval_us;
                r.work_us = timing.work_us;
                r.missed = timing.missed_deadlines;
                r.poll_wait = record.is_none();
                log_record(r);
            } else {
                info!(
                    "State={:?}; estimate={status}; age_us={age:?}; ready={}; sample_dt_us={sample_dt_us}; samples={sample_count} empty_polls={empty_polls}; loop_us={} work_us={} max_loop_us={} max_work_us={} missed_deadlines={}; motors OFF",
                    response.control.state(),
                    response.control.ready(report_now),
                    timing.interval_us,
                    timing.work_us,
                    timing.max_interval_us,
                    timing.max_work_us,
                    timing.missed_deadlines
                );
            }
            // Timestamp AFTER blocking writes, so log duration cannot itself trigger
            // the old false fallback message. No logging takes place in ACTIVE.
            last_log = epoch.elapsed().as_micros();
        }
        Timer::after_millis(1).await;
    }
}
