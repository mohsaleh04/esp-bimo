//! Independently gated, one-shot V2 feedback experiment.
#![no_std]
#![no_main]

use core::convert::Infallible;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{
    balance,
    board::Board,
    motors::{DualMotor, DutyLimit, control as motor_control, tb6612::Tb6612},
    orientation::{ComplementaryFilter, PitchFrame, Vector},
    pwm_mapping,
    sensors::bmi160::Bmi160,
};
use esp_hal::{
    Blocking,
    clock::CpuClock,
    i2c::master::I2c,
    peripherals::TIMG1,
    time::{Duration, Instant},
    timer::timg::{MwdtStage, TimerGroup, Wdt},
};
use log::{info, warn};

#[path = "balance_test/calibration.rs"]
mod calibration;
#[path = "../balance_v2/controller.rs"]
mod controller;
#[path = "balance_v2_live/session.rs"]
mod session;
use controller::{Controller, Fault};
use session::{MOTOR_CEILING, Session, State, Stop};

esp_bootloader_esp_idf::esp_app_desc!();

// Review the supply, restraint, cutoff and intended test stage before changing this.
const V2_LIVE_AUTHORIZED: bool = true;
const MAX_LOOP_US: u64 = 30_000;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let board = Board::init(esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(CpuClock::max()),
    ));
    esp_println::logger::init_logger_from_env();
    if !V2_LIVE_AUTHORIZED {
        warn!("Balance V2 LIVE rejected: source authorization is false; motor GPIOs remain LOW.");
        loop {
            core::hint::spin_loop();
        }
    }

    let mut bus = board.i2c;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timers = TimerGroup::new(board.timer_group);
    esp_rtos::start(timers.timer0, board.rtos_interrupt);
    match calibration::initialize(&mut bus).await {
        Ok(imu) => match calibration::learn_frame(&mut bus, &imu).await {
            Ok((frame, bias)) => {
                info!(
                    "Return upright with the body supported. Motors disarmed; 5 seconds to settle."
                );
                Timer::after_secs(5).await;
                let mut motors = Tb6612::new(
                    board.motor_pwm,
                    board.motor_outputs,
                    DutyLimit::new(MOTOR_CEILING),
                );
                motors.disarm();
                motors.set_command_logging(false);
                let mut watchdog = TimerGroup::new(board.balance_watchdog).wdt;
                watchdog.set_timeout(MwdtStage::Stage0, Duration::from_millis(100));
                watchdog.disable();

                let mut trial = Session::new(&mut motors, 0);
                let stop =
                    match feedback(&mut trial, &mut watchdog, &mut bus, &imu, frame, bias).await {
                        Err(reason) => trial.finish(reason),
                        Ok(never) => match never {},
                    };
                watchdog.disable();
                warn!("Balance V2 finished: {stop:?}; motors disarmed.");
            }
            Err(error) => warn!("Calibration failed: {error}; motors disarmed."),
        },
        Err(error) => warn!("BMI160 setup failed: {error}; motors disarmed."),
    }
    loop {
        Timer::after_secs(1).await;
    }
}

async fn feedback(
    trial: &mut Session<'_, Tb6612>,
    watchdog: &mut Wdt<TIMG1<'static>>,
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
    frame: PitchFrame,
    bias: Vector,
) -> Result<Infallible, Stop> {
    let epoch = Instant::now();
    let now = || epoch.elapsed().as_micros();
    let mut controller = Controller::new(0);
    let mut filter = ComplementaryFilter::new(0.5);
    let mut previous_loop = 0;

    loop {
        let started = now();
        check_timing(previous_loop, started)?;
        previous_loop = started;
        trial.tick(started)?;
        check_sensor_age(&mut controller, started)?;

        let sample = imu.read(bus).map_err(|_| Stop::Sensor(Fault::SensorRead))?;
        let received = now();
        check_timing(started, received)?;
        trial.tick(received)?;
        check_sensor_age(&mut controller, received)?;

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
                return Err(Stop::Sensor(Fault::InvalidSensor));
            }
            if let Some(dt) = controller.fresh_sample(received).map_err(Stop::Sensor)? {
                let estimate = filter
                    .update(&frame, accel, gyro, dt)
                    .ok_or(Stop::Sensor(Fault::InvalidEstimate))?;
                let preview = controller
                    .preview(
                        estimate.pitch_deg,
                        estimate.rate_dps,
                        estimate.accel_used,
                        received,
                    )
                    .map_err(Stop::Sensor)?;

                let before_command = now();
                check_timing(started, before_command)?;
                check_sensor_age(&mut controller, before_command)?;
                trial.observe(
                    estimate.pitch_deg,
                    estimate.rate_dps,
                    estimate.accel_used,
                    preview.wheels,
                    now,
                    || {
                        watchdog.enable();
                        watchdog.feed();
                    },
                )?;
                let completed = now();
                check_timing(started, completed)?;
                check_sensor_age(&mut controller, completed)?;
                if trial.state() == State::Active {
                    watchdog.feed();
                }
            }
        }
        Timer::after_millis(1).await;
    }
}

fn check_sensor_age(controller: &mut Controller, now: u64) -> Result<(), Stop> {
    controller.check_freshness(now).map_err(Stop::Sensor)?;
    controller.check_correction(now).map_err(Stop::Sensor)
}

fn check_timing(start: u64, end: u64) -> Result<(), Stop> {
    if end
        .checked_sub(start)
        .is_none_or(|elapsed| elapsed > MAX_LOOP_US)
    {
        Err(Stop::Sensor(Fault::Timing))
    } else {
        Ok(())
    }
}
