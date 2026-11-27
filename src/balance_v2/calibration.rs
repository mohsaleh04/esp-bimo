//! Startup orchestration only; all calibration/axis math stays in orientation.rs.
use embassy_time::Timer;
use esp_bimo::{
    orientation::{Calibration, CalibrationResult, PitchFrame, Vector},
    sensors::{bmi160::Bmi160, sample::ImuSample},
};
use esp_hal::{
    Blocking,
    i2c::master::I2c,
    time::{Duration, Instant},
};
use log::{info, warn};

pub fn observation(sample: ImuSample) -> (Vector, Vector) {
    (
        Vector(sample.accel_m_s2()),
        Vector(sample.gyro_rad_s().map(f32::to_degrees)),
    )
}
pub async fn initialize(bus: &mut I2c<'_, Blocking>) -> Result<Bmi160, &'static str> {
    for address in [0x68, 0x69] {
        let started = Instant::now();
        let imu = match Bmi160::init(bus, address).await {
            Ok(imu) => imu,
            Err(error) => {
                warn!(
                    "BMI160 init address=0x{address:02X} failed after {}us: {error:?}; motors remain disarmed",
                    started.elapsed().as_micros()
                );
                continue;
            }
        };
        {
            let mut config = [0; 4];
            bus.write_read(address, &[0x40], &mut config)
                .map_err(|error| {
                    warn!("BMI160 init address=0x{address:02X} stage=rate/range readback READ registers=0x40..0x43 I2C={error:?} timeout={}", matches!(error, esp_hal::i2c::master::Error::Timeout));
                    "BMI160 configuration read failed"
                })?;
            if config != [0x28, 0x03, 0x28, 0x00] {
                warn!(
                    "BMI160 init address=0x{address:02X} rate/range mismatch: actual={config:02X?} expected=[28, 03, 28, 00]"
                );
                return Err("unexpected BMI160 rate/range configuration");
            }
            info!(
                "BMI160 init address=0x{address:02X} OK after {}us; rate/range={config:02X?}; motors remain disarmed",
                started.elapsed().as_micros()
            );
            return Ok(imu);
        }
    }
    warn!(
        "BMI160 init failed at both 0x68 and 0x69; see per-address stage/error above; no automatic retry"
    );
    Err("BMI160 initialization failed")
}
async fn stationary(
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
) -> Result<CalibrationResult, &'static str> {
    let start = Instant::now();
    let mut last_log = start;
    let mut c = Calibration::default();
    while start.elapsed() < Duration::from_secs(60) {
        if let Some(sample) = imu
            .read(bus)
            .map_err(|_| "sensor error during calibration; reset to retry")?
        {
            let (a, g) = observation(sample);
            let rejection = c.push_at(a, g, start.elapsed().as_micros());
            if let Some(result) = c.result() {
                return Ok(result);
            }
            if last_log.elapsed() >= Duration::from_secs(1) {
                info!(
                    "CAL accepted/time={:?}, latest rejection={rejection:?}; motors disabled",
                    c.progress()
                );
                last_log = Instant::now();
            }
        }
        Timer::after_millis(1).await;
    }
    Err("calibration timed out; keep robot supported/stationary and reset")
}

pub async fn learn_frame(
    bus: &mut I2c<'_, Blocking>,
    imu: &Bmi160,
) -> Result<(PitchFrame, Vector), &'static str> {
    info!("POSE 1: support body UPRIGHT and still. 5 seconds to settle; motors disabled.");
    Timer::after_secs(5).await;
    let upright = stationary(bus, imu).await?;
    info!("Gyro bias deg/s={:?}", upright.gyro_bias_dps.0);
    info!(
        "POSE 2: manually lean FORWARD 15-30 degrees, no roll/yaw, support still. 6 seconds to reposition."
    );
    Timer::after_secs(6).await;
    let forward = stationary(bus, imu).await?;
    if (forward.gyro_bias_dps - upright.gyro_bias_dps).norm() > 0.5 {
        return Err("stationary gyro means disagree; reset/recalibrate");
    }
    let frame = PitchFrame::from_poses(upright.accel_mean, forward.accel_mean)
        .ok_or("invalid upright/forward poses; reset/recalibrate")?;
    warn!(
        "Frame learned from your forward pose: axis={:?}, up={:?}; verify sign in DRY mode",
        frame.axis.0, frame.up.0
    );
    Ok((frame, upright.gyro_bias_dps))
}
