#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{
    board::Board,
    orientation::{self, CalibrationResult, ComplementaryFilter, PitchFrame, Vector},
    sensors::{bmi160::Bmi160, sample::ImuSample},
};
use esp_hal::{
    Blocking,
    clock::CpuClock,
    gpio::Input,
    i2c::master::I2c,
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};
use log::{info, warn};

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "orientation_diagnostic/calibration_trace.rs"]
mod calibration_trace;
use calibration_trace::{Rejection, TracedCalibration};

// Only this diagnostic receives detailed read errors; estimator API is unchanged.
enum SampleError {
    Read {
        error: esp_bimo::sensors::bmi160::Error,
        read_us: u64,
    },
    Timeout {
        waited_ms: u64,
    },
}

fn log_read_error(error: &SampleError) {
    match error {
        SampleError::Read { error, read_us } => warn!(
            "CAL read_error: {error:?}, read_us={read_us}; required=successful fresh read, configured per-transaction I2C timeout=20000us"
        ),
        SampleError::Timeout { waited_ms } => {
            warn!("CAL read_error/no_fresh_sample: waited_ms={waited_ms}, freshness timeout=100ms")
        }
    }
}

struct CalibrationLog {
    last_report: Instant,
    fresh: u32,
    fresh_without_int1: u32,
    read_us_last: Option<u64>,
    dt_count: u32,
    dt_sum: f32,
    dt_min: f32,
    dt_max: f32,
    dt_last: Option<f32>,
    latest: Option<(ImuSample, Instant)>,
    resets: [u32; 8],
    totals: [u32; 8],
    reasons: [Option<Rejection>; 8],
    read_error: Option<SampleError>,
}

impl CalibrationLog {
    fn new() -> Self {
        Self {
            last_report: Instant::now(),
            fresh: 0,
            fresh_without_int1: 0,
            read_us_last: None,
            dt_count: 0,
            dt_sum: 0.0,
            dt_min: f32::INFINITY,
            dt_max: 0.0,
            dt_last: None,
            latest: None,
            resets: [0; 8],
            totals: [0; 8],
            reasons: [None; 8],
            read_error: None,
        }
    }
    fn sample(
        &mut self,
        sample: ImuSample,
        now: Instant,
        dt: Option<f32>,
        int1_high: bool,
        read_us: u64,
    ) {
        self.fresh += 1;
        if !int1_high {
            self.fresh_without_int1 += 1;
        }
        self.read_us_last = Some(read_us);
        self.latest = Some((sample, now));
        self.dt_last = dt;
        if let Some(dt) = dt {
            self.dt_count += 1;
            self.dt_sum += dt;
            self.dt_min = self.dt_min.min(dt);
            self.dt_max = self.dt_max.max(dt);
        }
    }
    fn rejection(&mut self, reason: Rejection) {
        let i = reason.index();
        self.resets[i] += 1;
        self.totals[i] += 1;
        self.reasons[i] = Some(reason);
    }
    fn report(&mut self, calibration: &TracedCalibration, force: bool) {
        let now = Instant::now();
        let elapsed = now - self.last_report;
        if !force && elapsed < Duration::from_secs(1) {
            return;
        }
        let seconds = elapsed.as_micros() as f32 * 1e-6;
        let rate = if seconds > 0.0 {
            self.fresh as f32 / seconds
        } else {
            0.0
        };
        let dt_stats = if self.dt_count > 0 {
            Some((self.dt_min, self.dt_sum / self.dt_count as f32, self.dt_max))
        } else {
            None
        };
        let (accepted, accepted_s) = calibration.progress();
        info!(
            "CAL fresh_rate={rate:.1}Hz fresh={} fresh_with_INT1_low={} read_us_last={:?} dt_last_s={:?} dt_min_mean_max_s={:?}; accepted={accepted}/{} window={accepted_s:.3}/3.000s",
            self.fresh,
            self.fresh_without_int1,
            self.read_us_last,
            self.dt_last,
            dt_stats,
            calibration_trace::MIN_SAMPLES
        );
        if let Some((sample, time)) = self.latest {
            let (a, g) = observation(sample);
            info!(
                "CAL raw_accel_xyz={:?} raw_gyro_xyz={:?}; accel_m_s2={:?} gyro_uncal_dps={:?} sample_age_ms={}",
                sample.accel_raw,
                sample.gyro_raw,
                a.0,
                g.0,
                (now - time).as_millis()
            );
        } else {
            info!("CAL raw XYZ unavailable: no successful fresh read yet");
        }
        for i in 0..8 {
            if self.resets[i] == 0 {
                continue;
            }
            let count = self.resets[i];
            let total = self.totals[i];
            match self.reasons[i].expect("counted rejection has details") {
                Rejection::Gravity { magnitude, finite } => warn!(
                    "CAL reset gravity: interval={count} total={total}, magnitude={magnitude:.4}m/s2 finite={finite}; required finite and [{:.4},{:.4}]m/s2",
                    calibration_trace::MIN_G * orientation::GRAVITY,
                    calibration_trace::MAX_G * orientation::GRAVITY
                ),
                Rejection::Gyro { magnitude, finite } => warn!(
                    "CAL reset gross_gyro: interval={count} total={total}, raw magnitude={magnitude:.4}deg/s finite={finite}; required finite and <= {:.3}deg/s (gross-motion/fault guard, not bias correction)",
                    calibration_trace::MAX_GYRO_DPS
                ),
                Rejection::Drift { distance } => warn!(
                    "CAL reset accel_drift: interval={count} total={total}, distance_from_window_first={distance:.4}m/s2 > {:.4}; current sample becomes new first",
                    calibration_trace::MAX_DRIFT
                ),
                Rejection::Variance { accel, gyro } => warn!(
                    "CAL reset variance: interval={count} total={total}, accel_var_xyz={accel:?} threshold_each<={:.6}(m/s2)^2, gyro_var_xyz={gyro:?} threshold_each<={:.6}(deg/s)^2",
                    calibration_trace::MAX_ACCEL_VARIANCE,
                    calibration_trace::MAX_GYRO_VARIANCE
                ),
                Rejection::Timing { dt } => warn!(
                    "CAL reset timing: interval={count} total={total}, dt={dt:.6}s; required finite and 0 < dt <= {:.3}s",
                    calibration_trace::MAX_DT_S
                ),
                Rejection::Motion {
                    accel_shift,
                    gyro_shift,
                } => warn!(
                    "CAL reset motion_trend: interval={count} total={total}, early/late accel_mean_shift={accel_shift:.5}m/s2 <= {:.5}, gyro_mean_shift={gyro_shift:.5}deg/s <= {:.3} required",
                    calibration_trace::MAX_ACCEL_MEAN_SHIFT,
                    calibration_trace::MAX_GYRO_MEAN_SHIFT
                ),
                Rejection::Bias { magnitude } => warn!(
                    "CAL reset bias_plausibility: interval={count} total={total}, mean_gyro_magnitude={magnitude:.4}deg/s > {:.3}; excessive bias or constant rotation; cannot distinguish from IMU alone",
                    calibration_trace::MAX_BIAS_DPS
                ),
                Rejection::Read => {
                    warn!("CAL reset read_error: interval={count} total={total}");
                    if let Some(error) = &self.read_error {
                        log_read_error(error);
                    }
                }
            }
        }
        // Keep latest raw sample and cumulative counts, clear interval statistics.
        self.fresh = 0;
        self.fresh_without_int1 = 0;
        self.dt_count = 0;
        self.dt_sum = 0.0;
        self.dt_min = f32::INFINITY;
        self.dt_max = 0.0;
        self.resets = [0; 8];
        self.last_report = now;
    }
}

fn observation(sample: ImuSample) -> (Vector, Vector) {
    (
        Vector(sample.accel_m_s2()),
        Vector(sample.gyro_rad_s().map(f32::to_degrees)),
    )
}

/// Poll the actual fresh-data status every 1 ms rather than gating reads on a
/// sampled INT1 level/20 ms fallback. INT1 is recorded as evidence, not a gate.
async fn next_sample(
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
    int1: &Input<'_>,
) -> Result<(ImuSample, Instant, bool, u64), SampleError> {
    let start = Instant::now();
    loop {
        let int1_high = int1.is_high();
        let read_start = Instant::now();
        match imu.read(bus) {
            Ok(Some(sample)) => {
                return Ok((
                    sample,
                    Instant::now(),
                    int1_high,
                    read_start.elapsed().as_micros(),
                ));
            }
            Ok(None) => {}
            Err(error) => {
                return Err(SampleError::Read {
                    error,
                    read_us: read_start.elapsed().as_micros(),
                });
            }
        }
        if start.elapsed() >= Duration::from_millis(100) {
            return Err(SampleError::Timeout {
                waited_ms: start.elapsed().as_millis(),
            });
        }
        Timer::after_millis(1).await;
    }
}

async fn stationary_window(
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
    int1: &Input<'_>,
) -> Result<CalibrationResult, &'static str> {
    let start = Instant::now();
    let mut calibration = TracedCalibration::default();
    let mut diagnostics = CalibrationLog::new();
    let mut previous = None;
    while start.elapsed() < Duration::from_secs(60) {
        match next_sample(bus, imu, int1).await {
            Ok((sample, now, int1_high, read_us)) => {
                let (a, g) = observation(sample);
                let dt = previous
                    .replace(now)
                    .map(|before| (now - before).as_micros() as f32 * 1e-6);
                diagnostics.sample(sample, now, dt, int1_high, read_us);
                // Use measured sample timestamps, not count/100 Hz or an
                // accumulated nominal interval. Each reset starts a new span.
                if let Some(reason) = calibration.push_at(a, g, (now - start).as_micros()) {
                    diagnostics.rejection(reason);
                }
                if let Some(result) = calibration.result() {
                    diagnostics.report(&calibration, true);
                    return Ok(result);
                }
            }
            Err(error) => {
                diagnostics.rejection(calibration.read_error());
                diagnostics.read_error = Some(error);
                previous = None;
            }
        }
        diagnostics.report(&calibration, false);
    }
    diagnostics.report(&calibration, true);
    Err("stationary calibration timed out; inspect CAL reset reasons and thresholds above")
}

async fn idle(reason: &str) -> ! {
    warn!("ORIENTATION INVALID: {reason}; motors remain disabled. Reset to retry.");
    loop {
        Timer::after_secs(1).await;
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    // Retain all six motor GPIOs LOW. Do not initialize a motor driver or PWM,
    // expose an arm path, or run either motor-test binary from this diagnostic.
    let _inactive_outputs = board.motor_outputs;
    let _buzzer = board.buzzer; // active-low input stays HIGH; calibration is silent
    let _qmc = board.qmc5883l_drdy;
    let int1 = board.bmi160_int1;
    let mut bus = board.i2c;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);

    info!("ORIENTATION: motors disabled; disconnect motor battery. Pitch axes UNCONFIRMED.");
    let mut sensor = None;
    for address in [0x68, 0x69] {
        match Bmi160::init(&mut bus, address).await {
            Ok(imu) => {
                sensor = Some((imu, address));
                break;
            }
            Err(error) => warn!("BMI160 init 0x{address:02X}: {error:?}"),
        }
    }
    let Some((imu, address)) = sensor else {
        idle("BMI160 initialization failed").await;
    };
    let mut config = [0; 4];
    if bus.write_read(address, &[0x40], &mut config).is_err() {
        idle("cannot read BMI160 rate/range configuration").await;
    }
    info!("BMI160 ACC_CONF/RANGE/GYR_CONF/RANGE={config:02X?}; expected 28/03/28/00 (100 Hz each)");
    if config != [0x28, 0x03, 0x28, 0x00] {
        idle("unsupported rate/range configuration").await;
    }

    info!(
        "POSE 1: hold body UPRIGHT at the intended balance reference; keep it supported and still."
    );
    info!("5 seconds to position and settle before calibration starts.");
    Timer::after_secs(5).await;
    info!(
        "Collecting >={} measured seconds AND >={} fresh samples (independent requirements, 60 s timeout).",
        calibration_trace::MIN_DURATION_US as f32 * 1e-6,
        calibration_trace::MIN_SAMPLES
    );
    info!(
        "Status polled every 1 ms; configured ODR 100 Hz is not an assumed dt. Inspect CAL fresh_rate and fresh_with_INT1_low."
    );
    let upright = match stationary_window(&mut bus, &imu, &int1).await {
        Ok(result) => result,
        Err(error) => idle(error).await,
    };
    info!(
        "Upright accel mean m/s2={:?}; gyro bias dps={:?}",
        upright.accel_mean.0, upright.gyro_bias_dps.0
    );
    info!(
        "POSE 2: lean FORWARD 15-30 degrees about the wheel axle, without rolling/yawing; hold still."
    );
    info!("6 seconds to reposition, then another stationary window. No motor will move.");
    Timer::after_secs(6).await;
    let forward = match stationary_window(&mut bus, &imu, &int1).await {
        Ok(result) => result,
        Err(error) => idle(error).await,
    };
    if (forward.gyro_bias_dps - upright.gyro_bias_dps).norm() > 0.5 {
        idle("gyro changed between stationary poses; repeat calibration").await;
    }
    let Some(frame) = PitchFrame::from_poses(upright.accel_mean, forward.accel_mean) else {
        idle("poses must differ by 10-45 degrees with normal gravity").await;
    };
    info!(
        "Learned sensor vectors: UP={:?}, tilt projection={:?}, positive pitch GYRO AXIS={:?}",
        frame.up.0, frame.tilt.0, frame.axis.0
    );
    info!("Sign uses your forward pose: forward-positive. Mapping must be checked experimentally.");
    info!(
        "Return upright, then slowly lean forward/back: angle and rate should agree in sign while moving."
    );

    let mut filter = ComplementaryFilter::new(0.5);
    let mut previous = None;
    let mut last_log = Instant::now();
    let mut last_read_error_log: Option<Instant> = None;
    loop {
        match next_sample(&mut bus, &imu, &int1).await {
            Ok((sample, now, _, _)) => {
                let (a, g) = observation(sample);
                if let Some(before) = previous.replace(now) {
                    let dt = (now - before).as_micros() as f32 * 1e-6;
                    let estimate = filter.update(&frame, a, g - upright.gyro_bias_dps, dt);
                    if last_log.elapsed() >= Duration::from_millis(100) {
                        match estimate {
                            Some(out) => info!(
                                "PITCH={:.2} deg RATE={:.2} deg/s dt={:.4}s accel_used={} accel_xyz={:?} gyro_xyz_dps={:?}",
                                out.pitch_deg, out.rate_dps, dt, out.accel_used, a.0, g.0
                            ),
                            None => {
                                warn!("PITCH INVALID: sample/gap/gravity rejected, dt={dt:.4}s")
                            }
                        }
                        last_log = Instant::now();
                    }
                }
            }
            Err(error) => {
                filter.reset();
                previous = None;
                if last_read_error_log.is_none_or(|last| last.elapsed() >= Duration::from_secs(1)) {
                    warn!("PITCH INVALID: read failure; filter reset, motors disabled");
                    log_read_error(&error);
                    last_read_error_log = Some(Instant::now());
                }
            }
        }
    }
}
