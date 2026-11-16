#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{
    balance::{Controller, Fault, Request, Settings, State, wheel_commands},
    board::Board,
    motors::{DualMotor, DutyLimit, l298n::L298n},
    orientation::ComplementaryFilter,
    pwm_mapping::{Config as PwmConfig, KickConfig, KickState},
};
use esp_hal::{
    clock::CpuClock,
    peripherals::TIMG1,
    time::{Duration, Instant},
    timer::timg::{MwdtStage, TimerGroup, Wdt},
};
use log::{info, warn};
#[path = "balance_test/calibration.rs"]
mod calibration;

esp_bootloader_esp_idf::esp_app_desc!();

// Mathematical DRY preview only; these are not validated live settings.
// This never changes driver limits or grants permission to energize motors.
const DRY_PWM: PwmConfig = PwmConfig {
    enabled: true,
    deadband: 2.0,
    pd_output_limit: 15.0,
    minimum_duty: 25.0,
    maximum_duty: 50.0,
    slew_per_second: None,
};

// Simulation only: both enabled flags must be true to preview a kick.
const DRY_KICK: KickConfig = KickConfig {
    enabled: true,
    kick_duty: 50.0,
    kick_duration_ms: 50,
    trigger_threshold: 4.0,
    rearm_delay_ms: 300,
    reversal_pause_ms: 100,
};

fn disable(motors: &mut L298n, watchdog: &mut Wdt<TIMG1<'static>>, running: &mut bool) {
    // Disable outputs before watchdog changes or potentially blocking logging.
    motors.disarm();
    if *running {
        watchdog.disable();
        *running = false;
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    let settings = Settings::default(); // Edit documented gains/limit here via Settings::default.
    assert!(settings.valid());
    assert!(DRY_PWM.valid());
    assert_eq!(DRY_PWM.pd_output_limit, f32::from(settings.output_limit));
    assert!(DRY_KICK.valid(DRY_PWM));
    info!("DRY-only kick: {DRY_KICK:?}; no motor activation");
    info!("DRY-only PWM mapping: {DRY_PWM:?}; motors remain OFF");
    let mut motors = L298n::new(
        board.motor_pwm,
        board.motor_outputs,
        DutyLimit::new(settings.output_limit as u8),
    );
    motors.set_command_logging(false);
    motors.disarm();
    let _buzzer = board.buzzer; // HIGH/inactive; no sound/vibration during calibration.
    let _int1 = board.bmi160_int1;
    let _qmc = board.qmc5883l_drdy;
    let mut bus = board.i2c;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);
    let timg1 = TimerGroup::new(board.balance_watchdog);
    let mut watchdog = timg1.wdt;
    // HAL enable() selects Stage0 ResetSystem, with later stages disabled.
    watchdog.set_timeout(MwdtStage::Stage0, Duration::from_millis(100));
    let mut watchdog_running = false;
    let _console = board.balance_console; // Logs only; no serial input is processed.

    info!("BALANCE EXPERIMENT: automatic DRY after calibration; motors always disabled.");
    info!(
        "Gains/limit: {settings:?}, Ki=0; live feature={} (ignored: DRY-only binary)",
        cfg!(feature = "balance-live")
    );
    let setup = match calibration::initialize(&mut bus).await {
        Ok(imu) => match calibration::learn_frame(&mut bus, &imu).await {
            Ok((frame, bias)) => Ok((imu, frame, bias)),
            Err(error) => Err(error),
        },
        Err(error) => Err(error),
    };
    let (imu, frame, bias) = match setup {
        Ok(setup) => setup,
        Err(error) => {
            motors.disarm();
            warn!("CALIBRATION FAILED: {error}; motors disabled, reset required.");
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    // This diagnostic never grants live permission, even with balance-live enabled.
    let mut control = Controller::new(settings, true, false).expect("balance settings");
    control
        .request(Request::Dry, 0)
        .expect("calibrated dry run");
    let mut filter = ComplementaryFilter::new(0.5);
    let epoch = Instant::now();
    let mut previous = None;
    let mut latest = None;
    let mut mapped: Option<esp_bimo::pwm_mapping::Preview> = None;
    let mut kick_state = KickState::default();
    let mut last_log = Instant::now();
    let mut announced = State::DryRun;
    let mut invalid_reason = "waiting for two fresh samples";
    info!(
        "Calibration valid. Automatic DryRun; no serial commands needed; motors permanently OFF."
    );

    loop {
        control.tick(epoch.elapsed().as_micros());
        if control.state() != State::DryRun || !control.sample_fresh(epoch.elapsed().as_micros()) {
            mapped = None;
            kick_state.reset();
        }
        if control.state() != State::Active {
            disable(&mut motors, &mut watchdog, &mut watchdog_running);
        }
        match imu.read(&mut bus) {
            Ok(Some(sample)) => {
                let now = epoch.elapsed().as_micros();
                let (a, g) = calibration::observation(sample);
                if let Some(before) = previous.replace(now) {
                    let dt = now.saturating_sub(before) as f32 * 1e-6;
                    if let Some(estimate) = filter.update(&frame, a, g - bias, dt) {
                        if estimate.accel_used || control.state() != State::Active {
                            control.sample(estimate.pitch_deg, estimate.rate_dps, now);
                            if !estimate.accel_used {
                                control.block_arming();
                            }
                            if control.state() == State::DryRun {
                                let was_kicking = mapped.is_some_and(|v| v.kick_active);
                                mapped = control
                                    .output(estimate.pitch_deg, estimate.rate_dps)
                                    .and_then(|raw| {
                                        kick_state.update(
                                            DRY_PWM,
                                            DRY_KICK,
                                            f32::from(raw),
                                            dt,
                                            now / 1_000,
                                        )
                                    });
                                // A 50ms preview may fall between the 10Hz summaries.
                                // Log transitions too, while the real motors remain disarmed.
                                if let Some(preview) = mapped
                                    && preview.kick_active != was_kicking
                                {
                                    info!(
                                        "DRY kick transition: raw_pd={:.2} mapped_pwm={} kick_active={} kick_elapsed_ms={} final_(A,B)={:?}; actual motors OFF",
                                        settings.kp * estimate.pitch_deg
                                            + settings.kd * estimate.rate_dps,
                                        preview.mapped_pwm,
                                        preview.kick_active,
                                        preview.kick_elapsed_ms,
                                        wheel_commands(preview.final_pwm as i16)
                                    );
                                }
                            }
                            latest = Some(estimate);
                        } else {
                            control.fault(Fault::Invalid);
                            latest = None;
                            mapped = None;
                            kick_state.reset();
                        }
                    } else {
                        invalid_reason = if dt <= 0.0 || dt > 0.1 {
                            "filter timing gap: dt must be >0 and <=100 ms"
                        } else if !(g - bias).finite() {
                            "nonfinite gyro input"
                        } else {
                            "gravity reference unavailable or gyro-only estimate exceeded 500 ms"
                        };
                        control.fault(Fault::Invalid);
                        latest = None;
                        mapped = None;
                        kick_state.reset();
                    }
                }
                if control.state() == State::Active {
                    if let Some(estimate) = latest {
                        if let Some(u) = control.output(estimate.pitch_deg, estimate.rate_dps) {
                            let (a, b) = wheel_commands(u);
                            if motors.command(a, b).is_err() {
                                control.fault(Fault::Motor);
                            }
                            // Feed ONLY after a fresh, valid sample and successful bounded command.
                            control.tick(epoch.elapsed().as_micros());
                            if control.state() == State::Active {
                                watchdog.feed();
                            }
                        } else {
                            control.fault(Fault::Invalid);
                        }
                    } else {
                        control.fault(Fault::Invalid);
                    }
                }
            }
            Ok(None) => {}
            Err(_) => {
                invalid_reason = "BMI160 read error; waiting for fresh filter samples";
                filter.reset();
                previous = None;
                latest = None;
                mapped = None;
                kick_state.reset();
                control.fault(Fault::Invalid);
            }
        }
        control.tick(epoch.elapsed().as_micros());
        if control.state() != State::DryRun || !control.sample_fresh(epoch.elapsed().as_micros()) {
            mapped = None;
            kick_state.reset();
        }
        if control.state() != State::Active {
            disable(&mut motors, &mut watchdog, &mut watchdog_running);
            if control.state() != announced {
                info!(
                    "State {:?}; ENA/ENB LOW; deliberate request required to restart",
                    control.state()
                );
                announced = control.state();
            }
            if last_log.elapsed() >= Duration::from_millis(100) {
                if let Some(estimate) =
                    latest.filter(|_| control.sample_fresh(epoch.elapsed().as_micros()))
                {
                    let raw_pd = settings.kp * estimate.pitch_deg + settings.kd * estimate.rate_dps;
                    let u = control.output(estimate.pitch_deg, estimate.rate_dps);
                    let mapped_pwm = mapped.map(|v| v.mapped_pwm);
                    let kick_active = mapped.is_some_and(|v| v.kick_active);
                    let kick_elapsed_ms = mapped.map(|v| v.kick_elapsed_ms);
                    let final_pwm = mapped.map(|v| v.final_pwm);
                    let mapped_u = final_pwm.map(|v| v as i16); // Driver resolution is 1%; truncate toward zero.
                    let wheels = mapped_u.map(wheel_commands);
                    info!(
                        "{:?}: estimate valid; pitch={:.2}deg rate={:.2}deg/s accel_used={} live_ready={} raw_pd={raw_pd:.2} limited_pd={u:?} mapped_pwm={mapped_pwm:?} kick_active={kick_active} kick_elapsed_ms={kick_elapsed_ms:?} final_pwm={final_pwm:?} final_u={mapped_u:?} dry_(A,B)={wheels:?}; actual motors OFF",
                        control.state(),
                        estimate.pitch_deg,
                        estimate.rate_dps,
                        estimate.accel_used,
                        control.ready(epoch.elapsed().as_micros())
                    );
                } else {
                    mapped = None;
                    kick_state.reset();
                    let reason = if latest.is_some() {
                        "no valid fresh sample within 40 ms (stale or timing fault)"
                    } else {
                        invalid_reason
                    };
                    info!(
                        "{:?}: estimate INVALID: {reason}; dry_u=None; motors OFF",
                        control.state()
                    );
                }
                last_log = Instant::now();
            }
        }
        Timer::after_millis(1).await;
    }
}
