//! BMI160 -> pitch estimate -> PD -> PWM preview. No motor driver is constructed.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{
    balance,
    board::Board,
    motors::control as motor_control,
    orientation::{ComplementaryFilter, PitchFrame, Vector},
    pwm_mapping,
    sensors::bmi160::Bmi160,
};
use esp_hal::{
    Blocking, clock::CpuClock, i2c::master::I2c, time::Instant, timer::timg::TimerGroup,
};
use log::{info, warn};

#[path = "../balance_v2/calibration.rs"]
mod calibration;
#[path = "../balance_v2/controller.rs"]
mod controller;
use controller::{Controller, Fault};

esp_bootloader_esp_idf::esp_app_desc!();

const MAX_LOOP_US: u64 = 30_000;
const LOG_INTERVAL_US: u64 = 100_000;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(peripherals);
    // Hold the GPIOs in Board's inactive state. MCPWM is never configured.
    let _inactive_motor_outputs = board.motor_outputs;
    let mut bus = board.i2c;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timers = TimerGroup::new(board.timer_group);
    esp_rtos::start(timers.timer0, board.rtos_interrupt);

    info!("BALANCE V2 DRY: TB6612 STBY LOW; hypothetical output only.");
    let fault = match calibration::initialize(&mut bus).await {
        Ok(imu) => match calibration::learn_frame(&mut bus, &imu).await {
            Ok((frame, bias)) => {
                info!("Support the body upright again. DRY capture starts in 5 seconds.");
                Timer::after_secs(5).await;
                run(&mut bus, &imu, frame, bias).await
            }
            Err(reason) => reason,
        },
        Err(reason) => reason,
    };
    warn!(
        "BALANCE V2 FAULT: {fault}; hypothetical A/LEFT=0 B/RIGHT=0; motors OFF; reset required."
    );
    loop {
        Timer::after_secs(1).await;
    }
}

async fn run(
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
    frame: PitchFrame,
    bias: Vector,
) -> &'static str {
    let fault = match preview_loop(bus, imu, frame, bias).await {
        Err(fault) => fault,
        Ok(never) => match never {},
    };
    match fault {
        Fault::SensorRead => "BMI160 read failed",
        Fault::InvalidSensor => "invalid sensor input or clipped gyro",
        Fault::InvalidEstimate => "pitch estimate unavailable or invalid",
        Fault::Timing => "control loop or sample timing invalid",
        Fault::StaleSample => "no fresh BMI160 sample within 40 ms",
        Fault::CorrectionExpired => "no accelerometer correction within 50 ms",
        Fault::Tilt => "pitch reached the tilt cutoff",
        Fault::InvalidOutput => "PD or PWM mapping failed",
    }
}

async fn preview_loop(
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
    frame: PitchFrame,
    bias: Vector,
) -> Result<core::convert::Infallible, Fault> {
    let epoch = Instant::now();
    let mut controller = Controller::new(0);
    let mut filter = ComplementaryFilter::new(0.5);
    let mut previous_loop: Option<u64> = None;
    let mut last_log = 0;
    loop {
        let started = epoch.elapsed().as_micros();
        if previous_loop.replace(started).is_some_and(|previous| {
            started
                .checked_sub(previous)
                .is_none_or(|gap| gap > MAX_LOOP_US)
        }) {
            return Err(controller.fail(Fault::Timing));
        }
        controller.check_freshness(started)?;
        controller.check_correction(started)?;

        let sample = imu
            .read(bus)
            .map_err(|_| controller.fail(Fault::SensorRead))?;
        let now = epoch.elapsed().as_micros();
        if now.saturating_sub(started) > MAX_LOOP_US {
            return Err(controller.fail(Fault::Timing));
        }
        controller.check_freshness(now)?;
        controller.check_correction(now)?;
        if let Some(sample) = sample {
            let (accel, gyro) = calibration::observation(sample);
            let gyro = gyro - bias;
            if !accel.finite()
                || !gyro.finite()
                || sample
                    .gyro_raw
                    .iter()
                    .any(|&value| value == i16::MIN || value == i16::MAX)
            {
                return Err(controller.fail(Fault::InvalidSensor));
            }
            if let Some(dt) = controller.fresh_sample(now)? {
                let estimate = filter
                    .update(&frame, accel, gyro, dt)
                    .ok_or_else(|| controller.fail(Fault::InvalidEstimate))?;
                let output = controller.preview(
                    estimate.pitch_deg,
                    estimate.rate_dps,
                    estimate.accel_used,
                    now,
                )?;
                if epoch.elapsed().as_micros().saturating_sub(started) > MAX_LOOP_US {
                    return Err(controller.fail(Fault::Timing));
                }
                if now.saturating_sub(last_log) >= LOG_INTERVAL_US {
                    last_log = now;
                    info!(
                        "DRY pitch={:.2}deg rate={:.2}deg/s dt={:.3}s accel={} pd={} hypothetical A/LEFT={}% B/RIGHT={}% direction_inputs={:?}; motors OFF",
                        estimate.pitch_deg,
                        estimate.rate_dps,
                        dt,
                        estimate.accel_used,
                        output.pd,
                        output.wheels[0],
                        output.wheels[1],
                        output.direction_inputs,
                    );
                }
            }
        }
        Timer::after_millis(1).await;
    }
}
