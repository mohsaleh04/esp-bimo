# Phase 2: orientation and axis-identification diagnostic

This is a sensor-only diagnostic. **There is no PD/PID, balance control, arm
call, motor driver or PWM initialization.** It retains the six inactive motor
GPIO owners for the whole task; ENA/ENB and IN1–IN4 stay LOW. Disconnect motor
battery power during this test. The buzzer input stays HIGH/inactive so sound
and vibration do not disturb calibration. Normal firmware, buzzer implementation,
BMI160 driver, and both motor-test binaries are unchanged.

## Existing sensor setup and what is unknown

The current BMI160 driver uses I2C0 at 100 kHz on SDA14/SCL13, and active-high
INT1 data-ready on GPIO41. The 12-byte burst is gyro XYZ followed by accel XYZ,
all signed little-endian. Scaling remains 16.4 LSB/(degree/s), 16384 LSB/g,
and g=9.806 m/s². The working reset/startup/interrupt sequence is reused.

ACC_CONF and GYR_CONF reset to 0x28 (100 Hz, normal bandwidth), with ranges
0x03 (±2 g) / 0x00 (±2000°/s). The diagnostic reads 0x40–0x43 after init and
requires `[0x28, 0x03, 0x28, 0x00]`, logging and refusing estimation if they
mismatch. It does not change the working configuration. These values are
specified by the [Bosch BMI160 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf).

The repository does **not** establish the sensor's physical mounting axes.
An old static gravity vector alone cannot identify robot-forward or the wheel
axle. No X/Y/Z axis is hard-coded as pitch. Historical L298N A=RIGHT and B=LEFT wheel observations
do not determine the IMU axes.

## Guided experiment: learn the physical pitch axis

The binary learns a continuous sensor-frame basis, so it also supports a sensor
mounted at an angle rather than exactly along principal robot axes. Calibration
is temporary and repeated every boot; no mapping is persisted.

1. Keep motor power disconnected. Support the robot upright at the intended
   zero-pitch body reference, without obstructing a slow manual forward lean.
   Use stable support rather than trying to hold it perfectly still in midair.
2. Run the diagnostic manually using the command below. At the POSE 1 prompt,
   it gives 5 seconds to position/settle, then requires a valid stationary
   window of at least 3 measured seconds and 120 fresh samples (independent requirements). It averages all three
   gyro axes for bias and all three accelerometer axes for the upright vector.
3. At POSE 2, lean the body **forward 15–30° about its wheel axle**, with minimal
   roll/yaw. There are 6 seconds to reposition, then another stationary window.
   Keep it supported and still until the mapping prints. It accepts separation
   10–45°. Too little lean or leaning too far is rejected; reset to retry.
4. Record `UP`, `tilt projection`, and `positive pitch GYRO AXIS` vectors printed
   in sensor XYZ coordinates. These are the experimentally selected mapping.
   For an aligned mounting the gyro vector will be close to one signed sensor
   axis; for an oblique mounting it may have several substantial weights.
5. Return upright: pitch should be near 0°. Lean forward slowly: angle should
   be positive and rate positive while increasing forward lean. Lean backward:
   angle should be negative, and the rate negative while moving backward.
   At either held pose, rate should return near zero. Compare ±15–30° readings
   with an independent angle reference if available.
6. Rock sideways and check that this produces little pitch response. If it does
   not, repeat the experiment with a purer forward-only POSE 2 and inspect the
   printed XYZ acceleration/gyro data. A mistaken roll/backward calibration
   pose cannot be identified as such from IMU data alone.

**The actual physical mapping remains unconfirmed until this experiment is
performed.** The positive sign is defined by the user's forward pose; the
software cannot know whether that pose was really forward. Upright is a chosen
body reference, not an automatically determined center-of-mass balance angle.

## Mapping and filter equations

Let `u` be normalized mean upright acceleration, and `v` normalized mean forward
acceleration (both in the sensor frame). Compute:

```text
pitch_axis k = normalize(v × u)
tilt projection f = normalize(u × k)
accel_angle = atan2(accel · f, accel · u)       [degrees]
pitch_rate = (gyro_dps - gyro_bias_dps) · k    [degrees/second]
```

`f` is the direction in which the measured acceleration changes during forward
lean. It is not necessarily the physical forward-pointing unit vector. The
cross-product order gives gyro and acceleration the same sign, as tested for
axis permutations, inversions, and oblique mounting rotations.

For every fresh sample, `dt` is measured in seconds between successful burst
read completion timestamps from the ESP32 monotonic clock. No nominal 10 ms
interval is substituted. With tau=0.5 seconds:

```text
alpha = tau / (tau + dt)
predicted = wrap180(previous_pitch + pitch_rate * dt)
pitch = wrap180(predicted + (1 - alpha) * wrap180(accel_angle - predicted))
```

The first valid estimate is initialized from acceleration, not from zero.
Wrapped innovation avoids a false correction through zero at ±180°. This is
a one-axis complementary filter, not full 3D attitude or yaw estimation.

## Calibration and validity rules

Calibration duration is now the **actual first-to-last fresh-sample timestamp
span**, at least 3,000,000 µs. A separate minimum of 120 samples supplies a
reasonable amount of data; sample count never substitutes for elapsed time.
At 60 Hz a clean window completes after approximately 3 seconds and 181 samples,
not after the old 250 samples (about 4.17 seconds). At lower rates, the independent
sample minimum may require longer. Nonpositive/nonmonotonic time and gaps over
50 ms still reset the window. The legacy measured-dt `push` API remains available;
the diagnostic uses the absolute-timestamp `push_at` API.

The revised noise and motion policy is centralized in `calibration_limits`:

| Gate | Limit / interpretation |
| --- | --- |
| Gravity magnitude | Unchanged 0.85–1.15 g |
| Raw acceleration drift from first sample | Unchanged maximum 0.05 g |
| Per-axis acceleration variance | Unchanged maximum (0.02 g)² |
| Per-axis centered gyro variance | Maximum 1.2 (deg/s)², about 50% above the observed 0.78 upper envelope |
| Raw gyro magnitude | Maximum 10 deg/s, a gross-motion/fault guard rather than a low-bias stationary gate |
| Estimated bias magnitude | Maximum 5 deg/s, an explicit conservative plausibility prior |
| Early/late mean acceleration change | Maximum 0.015 g |
| Early/late mean gyro change | Maximum 0.75 deg/s |

The old 0.09 gyro-variance threshold was below the measured stationary variance
0.21–0.78. The old raw 3 deg/s gate also rejected 3–4 deg/s readings before bias
could be estimated. The new centered variance is measured around the window's
mean, so a constant gyro offset does not inflate it. The accepted mean is the
gyro bias; it is not automatically subtracted before stationarity is assessed.
The 10 deg/s gross guard allows the observed 4 deg/s peaks plus roughly five
standard deviations at the 1.2 variance ceiling. The 5 deg/s mean-bias prior
allows the observed range with margin but fails closed for an implausibly large
mean: that event is labeled bias plausibility, not established motion.

Early means use the first second; late means use samples at/after 2 seconds.
Comparing those averages detects slow pose drift or gyro changes that could be
hidden by the broader gyro-noise bound. The 0.015 g acceleration change is about
0.86° of gravity-vector change and is more noise-tolerant than comparing single
samples. The 0.75 deg/s mean change is approximately a 3–4 standard-error margin
under the 1.2 variance bound with around 40–60 samples per one-second group.
Correlated noise can reduce that statistical margin; these are practical policy
limits, not guaranteed confidence bounds. A moving robot is not allowed to
teach the software a progressively wider noise threshold.

Raw validity, gross motion, pose drift, excess variance, mean trends, read errors
and timing gaps restart the window. Each pose retains its 60-second timeout.
The two pose gyro means must still agree within 0.5 deg/s. Gyro bias is fixed
after startup and is never learned during ordinary motion.

**An accelerometer and gyroscope cannot prove complete stationarity.** A constant
rotation about gravity and a constant gyro bias can produce identical readings.
Slow yaw within the bias prior, sufficiently tiny movements, and constant
translation can evade these checks. Keep the robot mechanically supported and
still; do not claim that IMU-only checks prevent every possible moving
calibration. Larger bias or drift requires investigation rather than automatic
threshold relaxation. Temperature drift, accel offset/scale, and mounting
movement remain limitations.

The diagnostic reports the real engine's reset decisions; the former duplicate
observer/state machine was removed, so diagnostic thresholds cannot drift from
the calibration implementation. Rate-limited raw/timing logs and rejection
measurements remain available, including the new trend and bias reasons.

During filtering, accel correction is rejected outside 0.85–1.15 g, if the
pitch-plane projection is degenerate, or if lateral acceleration along the
pitch axis exceeds 0.35 g. Short invalid-accel intervals use gyro integration
and print `accel_used=false`; after 0.5 seconds without valid correction the
estimate becomes invalid. Correction gates cannot reject every translation:
linear acceleration with a plausible magnitude can still distort apparent tilt.
This estimator assumes predominantly forward/backward motion and small roll.

Invalid/nonfinite gyro or dt, nonpositive dt, and gaps above 100 ms reset the
filter. A fresh valid gravity sample is required to reinitialize. No fresh data
for 100 ms or a bus error prints `PITCH INVALID` and clears the previous sample
time. The diagnostic now polls fresh-data status every 1 ms regardless of INT1 level,
using the existing driver's fresh-data result. The former INT1-level/20 ms
fallback gate could miss interrupt activity and needlessly delay reads.
`fresh_with_INT1_low` counts actual fresh reads made while the INT1 probe was LOW;
`read_us_last`, observed rate and measured dt distinguish I²C/read latency and
missed hints. No sensor ODR, I²C frequency or normal-firmware interrupt behavior
was changed. Bosch specifies that data-ready interrupts clear on data read or
when the next update starts, so software polling of their level is not a FIFO.
The [BMI160 datasheet, §2.6.10](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf)
supports treating the fresh-data register as the read decision here.

This removes a software throttling path; it does **not** prove that the next
hardware run will achieve 100 Hz. Logging/scheduling and I²C still add latency.
The configured 100 Hz is checked by register readback, while the consumed sample
rate continues to be measured and reported separately.

Sensor reads/filter updates aim to consume the 100 Hz output. Logs are limited
to approximately 10 Hz and include pitch degrees, bias-corrected rate deg/s,
measured dt, correction validity and sensor-native XYZ vectors. Actual cadence
is subject to I²C, logging and scheduling. Host read time is not hardware sensor
time; there is no FIFO and lost samples cannot be reconstructed. Sampling gaps
and the chosen dt are visible in logs. No real-time balancing performance is
claimed from these software checks.

## Exact commands

Build only (no flash):

```sh
cd /home/saleh/RustRoverProjects/esp-bimo
cargo build --release --locked --features orientation-diagnostic --bin orientation-diagnostic
```

When you explicitly choose to flash and run the axis-identification experiment:

```sh
cargo run --release --locked --features orientation-diagnostic --bin orientation-diagnostic
```

This is a different binary from either motor-test program. Do not select a
motor-test binary for this experiment. The agent has not flashed anything.

## Software checks and files

`src/orientation.rs` contains HAL-independent mapping, calibration and filtering.
`src/bin/orientation_diagnostic.rs` integrates it with the existing board/BMI160.
`tests/orientation.rs` tests the production estimator source via the small host
harness in `tests/orientation/`. Cargo.toml/Cargo.lock register the gated binary
and add exactly libm 0.2.16 for no_std atan2/square root/modulo; no existing crate
version changes were made. `src/lib.rs` exports the module.

```sh
cargo fmt --check
cargo check --locked --features orientation-diagnostic --bin orientation-diagnostic
cargo build --release --locked --all-features --bins
cargo clippy --locked --all-features --bins --lib -- -D warnings
CARGO_ENCODED_RUSTFLAGS='' cargo +stable test --locked --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu
```

The empty encoded rustflags are necessary to bypass the MCU project's global
`-nostartfiles` for the host test executable. Add `--offline` when using cached
dependencies. Tests cover mapping/sign, oblique mounting, angle initialization,
variable dt, wrap boundaries, drift correction, acceleration rejection,
nonfinite/gap handling, bias estimation, noisy 60 Hz calibration, independent timestamp/count gates,
read/gap resets, slow tilt, rocking, changing yaw and excessive-noise rejection. They do not prove
the real mounting, sign, sensor noise, mechanical stiffness or angle accuracy.
Those require the guided hardware experiment above.

External motor enable pull-downs/cutoff remain necessary for reset-time safety:
software cannot guarantee GPIO states before initialization or during a reset.
