# Bimo — Milestone 2

Phase 2 sensor-only work: [orientation and axis-identification diagnostic](docs/orientation.md).

The normal firmware now owns a disarmed L298N dual-motor driver; it never
commands movement. See [motor API, safety checks and opt-in test procedure](docs/motors.md).
The separate `motor-test` feature/binary automatically pulses channel A once
after a 3-second boot wait. No board was flashed or motor test run by the agent.

The following Milestone 1 notes describe the original bring-up; current
Milestone 2 behavior and safety details are in the motor document above.

## Milestone 1 bring-up notes

Independent `no_std` / `no_main` Rust firmware for ESP32-S3 N16R8.
This milestone exercises the shared I²C bus and BMI160 only. No PID, motor
activation, servo PWM, or other sensor sampling is implemented.

## Wiring

| Function | GPIO | Milestone 1 |
| --- | --- | --- |
| I²C SDA / SCL | 14 / 13 | I2C0, 100 kHz |
| QMC5883L DRDY | 12 | Input; not consumed |
| BMI160 INT1 | 41 | Input; sensor data-ready routed here, polling used |
| Tilt | 2 | Untouched |
| Vibration | 4 | Untouched |
| IR analog / digital | 1 / 39 | Untouched |
| L298N IN1 / IN2 / IN3 / IN4 | 5 / 6 / 7 / 15 | Output LOW |
| L298N ENA / ENB | 16 / 17 | Output LOW, initialized first |
| Passive buzzer | 21 | Repeating 12-bar A minor melody |
| Servo | 18 | Reserved, untouched |

Hall, OLED, ultrasonic, and touch hardware are not used. I²C requires
pull-ups to 3.3 V and a common ground; do not pull ESP32 GPIOs up to 5 V.

## Motor safety

Keep motor power disconnected for this milestone's first tests. Remove any
L298N ENA/ENB jumpers that tie the enables high before connecting ESP32 GPIOs.
Use external pull-downs on both enable inputs, and hardware gating of motor
power/enables if reset safety is required. Verify your actual module wiring.

After HAL initialization, firmware sets ENA/ENB low, then all four direction
pins low, before initializing I²C, the heap, or the RTOS. It retains the GPIO
drivers throughout normal operation and sensor failure/retry paths. There is
no API here that drives a motor pin high. **Software does not guarantee GPIO
states during power-up, reset, bootloader operation, flashing, or a crash.**

## Layout and reuse

- `src/buzzer.rs`: repeating melody in an Embassy task using LEDC PWM.
- `src/board.rs`: GPIO ownership, inactive motor/buzzer outputs, I²C configuration.
- `src/sensors/bmi160.rs`: identity check, initialization, readiness and burst reads.
- `src/sensors/sample.rs`: signed sample decoding and physical-unit conversion.
- `src/bin/main.rs`: generated startup, scan, periodic logging and retries.

BMI160 initialization and the 12-byte gyro-then-accel little-endian layout were
copied/adapted from `/home/saleh/RustRoverProjects/esp-s3-lab/src/bin/main.rs`.
The lab's compass module and AHT20 initialization/read code were also inspected.
The lab project is read-only reference material; no runtime or build dependency
on it exists.

Preserved lab settings: INT_OUT_CTRL=0x0A, INT_MAP_1=0x80, INT_EN_1=0x10,
accelerometer normal command 0x11 with 10 ms wait, gyro normal command 0x15
with 100 ms wait. Additions: chip ID 0xD1 verification, soft reset with 10 ms
wait for MCU-only restarts, explicit ±2 g / ±2000 degrees/s range writes,
range/PMU readback, status checks, error logging and retries instead of unwraps.
Other configuration remains at reset defaults, as in the lab. Scaling uses
16384 LSB/g, 16.4 LSB/(degree/s), gravity 9.806 m/s², and PI/180.
See the [Bosch BMI160 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf).

The main Embassy task owns the bus. Short blocking register operations retain
the lab's HAL API, with a configured 20 ms software timeout per transaction;
startup delays, retry waits and sampling intervals use Embassy timers. Samples
are logged approximately every 200 ms (5 Hz), not at a balancing-control rate.
The driver tries 0x68 then 0x69. Five consecutive errors/not-ready samples trigger
reinitialization; failed initialization is retried after two seconds.

Scanning probes 0x08–0x77 using address-only writes, supported by the installed
esp-hal 1.2.2 implementation. ACKs identify responding addresses, not device
models. QMC5883L (typically 0x0D) and AHT20 (0x38) may appear in the scan but are
not initialized. Future drivers can borrow the same bus sequentially; no mutex,
extra tasks, or placeholder drivers are needed yet.

## Toolchain and validation

Dependencies, versions, Cargo.lock, heap size (73,744 bytes), and target settings
are unchanged. Toolchain is `esp`, target `xtensa-esp32s3-none-elf`; the inspected
compiler reports 1.97.0-nightly. esp-hal 1.2.2 and esp-rtos 0.4.0 provide the
existing Embassy integration. There were no `generator parameters:` comments
in the starting main.rs. PSRAM is not enabled or required by this milestone;
the N16R8 external memory has not been validated.

From this directory:

```sh
cargo fmt --check
cargo check --locked
cargo build --locked
cargo build --release --locked
cargo clippy --locked --bin esp-bimo --lib -- -D warnings
rustc +stable --edition=2024 --test tests/imu_sample.rs -o /tmp/esp-bimo-imu-tests
/tmp/esp-bimo-imu-tests
```

The host tests exercise the production conversion code: signed extremes,
little-endian byte/axis order, gravity scaling, radian conversion and zero data.
They do not validate electrical behavior or register transactions on real hardware.
Use the direct rustc command because Cargo's default target is the bare-metal
Xtensa MCU. Source `~/export-esp.sh` if the Xtensa linker is not in PATH.

Validation in this workspace: formatting, locked check, debug/release builds,
strict Clippy and both host tests passed. Both linked builds emitted a warning
that the ELF has a LOAD segment with RWX permissions; builds still succeeded.
No board was flashed and no hardware behavior is claimed from these checks.

## Manual flash and test

No board has been flashed automatically. Once motor power is disconnected,
the enable jumpers/pull-downs are checked, and the sensor wiring is verified:

```sh
cd /home/saleh/RustRoverProjects/esp-bimo
cargo run --release --locked
```

The existing runner invokes `espflash flash --monitor --chip esp32s3` over the
serial/USB bootloader connection. If selecting a port explicitly is necessary:

```sh
espflash flash --monitor --chip esp32s3 --port /dev/ttyACM0 target/xtensa-esp32s3-none-elf/release/esp-bimo
```

Replace the port with your board's actual port. If connection fails, check
permissions and USB cable, then enter bootloader mode using BOOT/RESET.
Logging defaults to `ESP_LOG=info` via `.cargo/esp-config.toml`.

Expected observations:

1. Boot banner, GPIO/I²C description, responding addresses and scan count.
2. BMI160 ready at 0x68 (or 0x69), followed by accel/gyro lines at about 5 Hz.
3. At rest, total acceleration magnitude around 9.806 m/s² and angular rates
   near zero (bias/noise expected). Tilt slowly to observe gravity moving
   between sensor axes; rotate to observe gyro changes. Axes are sensor-native,
   without mounting correction, bias calibration or sensor fusion.
4. With sensor absent at boot, errors and periodic init retries without panic.
   Reset and power-cycle with motor power still disconnected; check enable and
   direction levels using measurement equipment before any future powered tests.

Real-board scanning, timing, interrupts, sensor accuracy, and reset-time motor
safety remain hardware validation tasks. This firmware is not a balance controller.

The 12-bar melody uses A4, C5, E5 and G5 from A natural minor, with opening,
contrasting middle and resolving phrases. It repeats about every 17.3 seconds in a separate Embassy task while scanning
and sensor sampling continue. It uses LEDC
low-speed channel 0 and timers 0–3; no dependency was added. Motor outputs
remain low during playback. Sound has not been tested on the board.

Buzzer input polarity is configured as active-low for testing: GPIO21 starts
HIGH, and GPIO-matrix inversion makes zero PWM duty physically HIGH during
rests. The 50% note waveform and melody are unchanged. Actual module polarity
and whether it supports pitched tones still need hardware confirmation.

## First experimental PD controller — balance-test

`balance-test` is a separate experiment. The normal application and the working
orientation/motor diagnostic binaries are unchanged. No firmware was flashed
and no motor was activated by the agent. This is **not tuned for unattended
balancing**. The robot's 950 g mass alone is insufficient to choose stable gains;
motor torque, wheel geometry, center of mass and delay also matter.

The control policy is in `src/balance.rs`, hardware orchestration in
`src/bin/balance_test.rs`, and startup calibration orchestration in
`src/bin/balance_test/calibration.rs`. The latter calls the existing BMI160,
stationary calibration, learned pitch-frame and complementary-filter functions;
it does not implement a second orientation algorithm.

The body reference is the upright pose used during calibration. With forward
pitch and pitch rate positive, and a zero-degree target:

```text
u = clamp(Kp * pitch_deg + Kd * pitch_rate_dps, -limit, +limit)
RIGHT/A electrical command = -u
LEFT/B electrical command  = +u
Ki = 0; no integral state
```

A positive forward lean should command the wheels physically FORWARD to move
under the body. This sign must be verified on the real robot; it is not a
claim of closed-loop stability. PD uses measured gyro rate, not an angle
finite-difference with an assumed fixed time step. The filter receives actual
elapsed time between fresh successful reads.

Initial editable parameters (intentionally weak, unvalidated):

| Parameter | Initial value | Meaning |
| --- | --- | --- |
| `Settings::default().kp` | 1.0 | duty-percent per degree |
| `Settings::default().kd` | 0.08 | duty-percent per degree/second |
| `Settings::default().output_limit` | 15% | symmetric ceiling; settings reject limits above 30% |
| `ARM_ANGLE_DEG` | ±5° | must be approximately upright to request ARM; DRY allows any tilt |
| `ARM_RATE_DPS` | ±5°/s | low angular rate required for arming |
| `UPRIGHT_DWELL_US` | 500 ms | continuously eligible fresh samples before a request |
| `FALL_ANGLE_DEG` | 15° | reaching either tilt limit in Active faults/disarms |
| `MAX_SAMPLE_GAP_US` | 40 ms | stale/timing cutoff checked before and after reads/commands |
| `MAX_ACTIVE_US` | 2 seconds | maximum initial live trial, then fault/disarm |
| TIMG1 watchdog | 100 ms | hardware reset fallback if successful live updates cease |

Output is truncated toward zero to the existing driver's 1% resolution.
There is **no minimum-duty jump, full-duty startup kick, automatic compensation,
self-righting or gain adaptation**. If low duty only buzzes or cannot overcome
friction, stop and diagnose; the controller will not increase its limit itself.
The existing L298N driver also inserts approximately 1.2 ms of disabled time on
every command, including same-direction updates. This reduces effective drive;
no compensation is applied. A future driver can retain the physical command
mapping while implementing the `DualMotor` interface differently.

States and console commands:

| State | Behavior |
| --- | --- |
| Calibration | Both motors disabled; two supported stationary poses as in orientation diagnostic |
| Observe (default) | Logs angle/rate/readiness; no motor commands |
| DryRun | `DRY` plus LF: calculates/logs u and A/B commands, motors remain disabled |
| Active | `ARM` plus LF, only in a `balance-live` build and after all arming checks |
| Fault | Motors disabled; STOP or RESET explicitly clears the latch, then fresh upright dwell and ARM are required |

Send `STOP` plus LF, `!`, or a space to stop. `!`/space do not need Enter.
Invalid/overlong command lines are logged as rejected and stop control, but do not clear a fault latch. CRLF works; CR alone does
not submit a command. Commands queued before calibration completes are
discarded. An additional mode request during Active stops that trial and
requires a new dwell; repeatedly sending ARM cannot extend it.

Calibration failure leaves the program disabled until reset. Invalid/nonfinite
estimates, rejected acceleration correction, sensor-read errors, stale data,
timing faults, reaching ±15°, and trial expiry disarm. Faults never automatically
resume control when the robot is raised upright. In Active, the application and
motor-command telemetry are silent so USB/logging cannot block an energized
control update. State/fault messages print only after outputs are disabled.
There are no live-tuning console commands.

### Exact test procedure

1. Keep motor power disconnected. Retain an accessible independent power cutoff.
   Check common ground, valid L298N logic power and enable pull-downs using
   accessible points as described in `docs/motors.md`. No chassis dismantling
   or wiring modification is performed by this firmware.
2. Build the observation/dry-run version (build commands do not flash):

   ```sh
   cargo build --release --locked --features balance-test --bin balance-test
   ```

   Only when you explicitly choose to flash:

   ```sh
   cargo run --release --locked --features balance-test --bin balance-test
   ```

3. Support upright for the first stationary calibration, then follow the prompt
   for the forward 15–30° stationary pose. Return upright. All motor outputs stay
   LOW. Calibration/frame are relearned on every reset. The buzzer is silent.
4. Use the native USB Serial/JTAG console (GPIO19/20) with LF line endings.
   Send `DRY` after calibration, even while still in the forward pose.
   Slowly lean forward/backward; no upright dwell is needed for DRY. Verify:
   forward-positive angle/rate -> positive u -> A negative, B positive;
   backward-negative angle/rate -> negative u -> A positive, B negative.
   A held lean should have rate near zero. Dry-run lines explicitly say actual
   motors OFF. `ARM` is rejected in this build even after successful calibration.
5. Before any live test, suspend/restrain both wheels clear of the ground, with
   no fingers, clothes or wires near them. Confirm the sensor sign and the
   previously observed physical mapping. Do not proceed directly to floor
   balancing based only on software logs.
6. If explicitly ready for a brief live trial, build/flash the live-capable
   version with motor power disconnected:

   ```sh
   cargo build --release --locked --features balance-live --bin balance-test
   # Only when the owner deliberately chooses to flash:
   cargo run --release --locked --features balance-live --bin balance-test
   ```

   This still boots into Observe, never into Active. Recalibrate and repeat DRY.
   STOP, hold upright, and wait for readiness again. Connect motor power only
   when the suspended setup, independent cutoff and supply checks are ready.
   Hold near upright at low rate and send exactly `ARM` plus LF. The first live
   trial is limited to two seconds, 15% duty and the tilt/timing conditions.
7. **Verify feedback sign with wheels suspended:** a small forward lean must
   produce physical forward wheel motion, and a small backward lean the reverse.
   Use the existing safe short manual channel tests first if needed to reconfirm
   wiring/polarity. If the wheels accelerate the wrong way, stop/cut power and
   correct the sign before any further live trial. If the cap is too weak for
   visible motion, the physical sign check is inconclusive: do not enable an
   automatic minimum-duty jump or proceed to unsupported floor balancing.
8. `!`/space stops at the next USB poll; a complete STOP line also works.
   After faults, keep the robot supported, inspect the cause, and use a new
   STOP or RESET, then a fresh upright dwell and a deliberate ARM request. Do not leave it alone.
   USB disconnection or exiting the monitor is **not** an immediate stop command.
   The normal two-second trial cap still applies while the CPU runs.

### Watchdog and electrical limits

The binary reserves TIMG1's hardware MWDT, independently of the RTOS TIMG0 timer.
Installed esp-hal 1.2.2 `Wdt::enable()` configures Stage0 `ResetSystem` and disables
later stages. It is enabled before arming, fed only after a fresh valid sample
and a successful command, and disabled only after motor disarm. A main-loop hang
or panic should therefore trigger a reset rather than rely solely on Rust Drop.
The watchdog path is source-inspected and compiled, **not hardware-validated**.
See [Espressif watchdog documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html).

This is a reset fallback, **not a guaranteed motor-power cutoff**. Drop does not
run on panic-abort, and GPIO behavior through watchdog reset/bootloader/debugging
cannot be guaranteed by this software. External enable pull-downs or independent
hardware gating/cutoff must be verified before powered trials. A debugger can
also affect watchdog behavior. Do not infer an unconditional 100 ms physical
shutdown guarantee from the configured timeout.

The software 40 ms deadline is checked around operations; blocking I²C reads
can delay when a fault is noticed. Existing I²C transactions have a 20 ms timeout
and a BMI160 read can contain more than one transaction. The watchdog is the
separate reset fallback; it cannot cure invalid L298N logic power or wiring.

**A 9 V adapter with 6 V-rated motors remains an electrical risk.** Low average
PWM duty does not turn the supply into a guaranteed 6 V source, limit stall
current, or guarantee safe heating. L298N voltage drop varies with load. Motor
current, motor-terminal voltage, adapter behavior and thermal performance remain
unverified. The duty ceiling is not electrical protection. Keep initial tests
brief and suspended, and resolve supply/current safety before floor balancing.

Validation commands (software only):

```sh
CARGO_ENCODED_RUSTFLAGS='' cargo +stable test --locked --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu
cargo build --release --locked --features balance-test --bin balance-test
cargo build --release --locked --features balance-live --bin balance-test
cargo clippy --locked --all-features --bins --lib -- -D warnings
```

OBSERVE does not latch tilt faults after calibration: the forward calibration pose
may remain tilted. Send `DRY` plus LF from OBSERVE at any tilt; it never energizes
the motors. DRY reports PD output for fresh valid estimates (including the filter's
bounded gyro-only estimates, marked `accel_used=false`). Invalid/stale estimates
report `dry_u=None`; DRY resumes calculations when valid samples return.
`live_ready` applies only to ARM. Gyro-only samples cannot establish live readiness.
Live ACTIVE retains all tilt, freshness, timing, and estimate validity protections.
Returning upright or sending DRY/ARM does not clear a live fault: send `STOP` or
`RESET` explicitly, then complete a new upright dwell before ARM.

### Separate PWM startup diagnostic

`motor-pwm-diagnostic` is an output-only, one-shot test using the existing L298N
MCPWM driver and wiring. It does not measure a startup threshold automatically:
without encoders, the operator must record whether each wheel starts turning.
A negative result means only that the tested pulse/duty did not start it.

Before manually running, suspend and secure both wheels, keep a motor-power
cutoff within reach, and verify common ground and valid L298N logic supply.
The current 9 V supply exceeds the motors' 6 V rating; a PWM ceiling does not
limit instantaneous voltage or guarantee safe current/temperature. Avoid stalls.
Software cannot guarantee reset-time GPIO levels or stop a pulse during a CPU
hang; use external enable pull-downs/gating for that guarantee.

Configuration is at the top of `src/bin/motor_pwm_diagnostic.rs`: a 15-second
preparation delay and a 30% conservative ceiling. Set `RUNNING_START_A_PERCENT`
and `RUNNING_START_B_PERCENT` to individually verified safe spinning duties;
the 30% defaults are placeholders, not experimentally verified thresholds.
Do not raise the ceiling on the current supply without verifying motor safety.

For A/right first, the running test holds its configured starting duty for one
second, then reduces it by 5 percentage points every second down to 5%.
It does not deliberately stop between running steps, though the reused driver
briefly detaches enables during command updates (about 1.2 ms). Logging and
scheduling add small timing overhead. Observe when the wheel stops; if it never
started, this is not a valid running-threshold measurement.

Both motors are then disarmed for a 2-second coast pause. Starting attempts run
from 5% through the configured ceiling in 5-point increments, each with a 250 ms
pulse followed by stop/disarm and a 2-second pause. Increase the pause if a wheel
has not fully stopped; no encoder can confirm standstill. A failed attempt means
only that the tested duty/pulse did not start that wheel under those conditions.
There is no boost or automatic detection, and later attempts still run after
visible motion begins. After A finishes, both motors remain disabled for an
additional 3 seconds before the same sequence runs for B/left. They are never
commanded together. Forward polarity remains A negative and B positive.

The starting duties, lower bound, step size, pulse duration, and pauses are named
constants. Bounds/alignment are checked at compile time; 100% is rejected.
All tests run once, followed by unconditional stop/disarm and permanent idle.
Reset repeats the sequence. The running test includes logging while energized;
a blocked log, CPU hang, or executor stall can delay shutdown. Keep the physical
motor-power cutoff accessible throughout.

Build without flashing:

```sh
cargo build --release --locked --features motor-pwm-diagnostic --bin motor-pwm-diagnostic
```

Only when ready for the physical test, manually flash/run:

```sh
cargo run --release --locked --features motor-pwm-diagnostic --bin motor-pwm-diagnostic
```

No serial commands are needed. Logs identify phase, signed commands, effective
duty, stopping, and completion. No movement is claimed by the firmware.

### DRY-only PWM compensation preview

`balance-test` still boots with motors disarmed, calibrates, then automatically
enters DRY. No serial input is needed. It never grants live permission, including
when `balance-live` is selected. PWM mapping only calculates telemetry; it is not
connected to motor commands. Calibration, faults, the estimator, and driver limits
are unchanged. Any future live experiment must be a separately selected firmware
configuration with its own deliberate post-calibration activation gate; none is
enabled here. Selecting a mapping must never grant motor permission.

Edit `DRY_PWM` in `src/bin/balance_test.rs`:

- `enabled`: currently `true` for mathematical DRY preview only.
- `deadband`: defaults to 2 raw PD percentage points; magnitude <= this maps to zero.
- `minimum_duty`: 25%, the lower endpoint of the proportional range immediately
  above the deadband. It is not a temporary starting kick.
- `maximum_duty`: currently 50%; clips both signs. DRY-only values such as
  a 40% floor and 45% ceiling can preview the measured suspended-wheel startup
  threshold. They do not establish a safe live configuration.
- `slew_per_second`: `None` by default; e.g. `Some(50.0)` limits changes to
  50 percentage points per second, using fresh-sample measured dt. Ramps can
  pass below the configured duty floor. Exact zero bypasses the ramp and stops;
  reversals discard the previous sign, then ramp in the new direction.

Disabled mapping is identity except for the configured maximum. Enabled mapping:
`u=0` inside the deadband, otherwise
`u=sign(PD) * (minimum_duty + clamp((abs(PD)-deadband)/(pd_output_limit-deadband), 0, 1) * (maximum_duty-minimum_duty))`.
Input is the existing bounded integer PD command (`Settings::output_limit`, 15%
by default); raw floating-point PD and that limited value are both logged.
This proportional mapping still jumps from zero to the lower PWM endpoint just
above the deadband; review that discontinuity before any future live test.
There is no motion detection, start/run hysteresis, 100% kick, or high-duty pulse.

At about 10 Hz, logs show pitch, pitch rate, raw PD, limited PD, mapped signed
output, and electrical A/B commands `(-u,+u)`. Floating mapping state is retained
for slew precision; final commands truncate toward zero to the driver's 1% units.
Invalid/stale data clears the mapping state and logs `dry_u=None`.

Tune deadband first from upright noise, then preview the floor/ceiling and optional
slew. The measured 40% starting and 25% running values came from suspended wheels;
load, friction and supply changes can change them. The measured 9.72 V full-duty
motor voltage on a 12 V supply exceeds the motors' 6 V rating. PWM duty does not
bound instantaneous voltage/current or establish thermal safety. Do not infer
live safety from DRY results or enable full-duty operation.

Build only:

```sh
cargo build --release --locked --features balance-test --bin balance-test
```

### Optional startup-kick simulation (DRY only)

`DRY_KICK` in `balance_test.rs` is currently enabled for mathematical DRY preview. Both `DRY_PWM.enabled`
and `DRY_KICK.enabled` must be true to preview it. This never enables motors,
even with `balance-live`; PD gains, calibration and live control are unchanged.
There are no blocking waits in the mapper. It uses measured sample dt and
elapsed control-loop milliseconds, and returns one signed physical-forward
command for both wheels: A/right = negative, B/left = positive.

Defaults to review:

| Constant | Value | Meaning |
| --- | --- | --- |
| `kick_duty` | 50% | Temporary DRY duty; separate from ordinary PWM ceiling |
| `kick_duration_ms` | 50 ms | End the kick at the first update at/after this time |
| `trigger_threshold` | 4 PD points | Must exceed the 2-point deadband |
| `rearm_delay_ms` | 300 ms | Continuous deadband/zero dwell to rearm; also minimum time between kick starts |
| `reversal_pause_ms` | 100 ms | Output zero until the requested sign stays stable for this pause |

The first meaningful request can start one kick. Holding that request never
restarts it. After expiration, output returns to ordinary mapped PWM, including
its floor/ceiling and optional slew. The kick is a separate temporary override
of those ordinary mapping limits; it is not separately capped by the ordinary PWM ceiling, and bypasses slew limiting. It is still strictly below 100% by validation.
No 50% permanent floor is added. Exact zero/deadband cancels a kick immediately.

Small fluctuations below the trigger cannot kick. Once consumed, a same-direction
request needs the full continuous stopped-command dwell to rearm. A reversal
cancels the current kick and commands zero, restarting its pause if the requested
sign changes again. After the pause a meaningful request can kick only if the
cooldown has elapsed. If cooldown rejects that transition, holding it does not
produce a delayed kick: it uses ordinary mapping instead.

`KickState::reset()` clears command history on invalid/stale orientation or when
leaving DRY (stop/fault/disarm of the simulated controller). Physical motor outputs
are permanently disarmed in this binary; that ongoing safety action is separate
from simulated command state. On fresh valid data after reset, a new meaningful
request is treated as a new startup. Update errors also reset history.

Summary logs distinguish raw/limited PD, ordinary `mapped_pwm`, `kick_active`,
`kick_elapsed_ms`, final command, and A/B polarity. Kick transitions also log,
because a 50 ms kick can occur between 100 ms summary lines. Invalid samples
still report `dry_u=None`; mapped state is discarded.

This is an open-loop static-friction hypothesis, not measured speed or movement.
The suspended-wheel 40% starting/25% running observations do not establish
loaded behavior or safe motor voltage/current. Do not use this preview as a live
configuration; future live testing still requires a separately selected firmware
and explicit safety review of supply, timing and post-calibration activation.


Current proportional preview uses `pd_output_limit=15`, deadband=2, PWM range
25–50%, mapping and kick enabled, and slew disabled. Kick settings remain 50%
for 50 ms, trigger=4 limited PD points, rearm=300 ms, reversal pause=100 ms.
Examples: PD ±2 -> 0%, ±8.5 -> ±37.5%, ±15 -> ±50%; larger magnitudes saturate.
Ordinary mapping and kick triggers both use LIMITED PD. Raw PD is diagnostic
only. The binary asserts that mapping `pd_output_limit` matches the controller's
output limit; public `Config` initializers now require that additional field.
Function signatures, kick state transitions, motor API and physical mapping are
unchanged. Validation rejects nonfinite/reversed/equal PWM endpoints and
`pd_output_limit <= deadband`. These settings never activate motors in DRY,
including with `balance-live`, and are not validated live motor settings.

### Explicitly selected motor kick/hold hardware diagnostic

`motor-kick-diagnostic` is separate from balancing and PWM mapping. It reads no
serial input. It starts disarmed, waits 15 seconds, then (only with valid settings)
tests A/right alone followed by B/left alone. One trial per motor is configured:
50% kick for 50 ms, immediately followed by 30% hold for 1000 ms, then stop/disarm.
There is a 2000 ms disabled pause after each trial and an additional 3000 ms
between motors. It ends unconditionally disarmed and idles until reset. Reset
restarts the sequence. Electrical forward remains A negative, B positive.

These are candidates, not verified safe/working values. `TEST.ceiling` defaults
to **30%**, so the 50% candidate kick is **blocked**, with an explanatory log.
The program rejects invalid settings rather than silently clamping or bypassing
its ceiling. Edit the constants in `src/bin/motor_kick_diagnostic.rs` only after
verifying the motor supply, valid L298N logic supply/common ground, and a safe
suspended-wheel setup. Either lower candidate duties or explicitly configure a
verified ceiling. Both kick and holding duty must fit it; 100% is rejected.

Compile-time opt-in is independent of `balance-live`:

- `motor-kick-diagnostic`: output-only preview, never arms motors. With default
  invalid candidate/ceiling combination, it logs BLOCKED and stays disabled.
- `motor-kick-live`: includes the diagnostic feature and explicitly permits real
  output, but still refuses settings that violate the ceiling. `balance-live`
  alone cannot permit movement in this diagnostic.

Build without flashing:

```sh
cargo build --release --locked --features motor-kick-diagnostic --bin motor-kick-diagnostic
cargo build --release --locked --features motor-kick-live --bin motor-kick-diagnostic
```

To manually run output-only preview:

```sh
cargo run --release --locked --features motor-kick-diagnostic --bin motor-kick-diagnostic
```

**Only after supply/ceiling verification, source configuration, and securing both
wheels**, the explicit real-movement command is:

```sh
cargo run --release --locked --features motor-kick-live --bin motor-kick-diagnostic
```

Motor ratings are 6 V; the previous 12 V supply/full-duty terminal measurement
of about 9.72 V does not establish that any proposed PWM setting is safe. Keep a
physical motor-power cutoff accessible. No wheel movement is automatically
measured. Deadlines are nonblocking Embassy timers, not hardware pulse cutoffs:
blocking logs, driver updates, executor stalls or CPU hangs can delay shutdown.
The reused L298N driver briefly detaches enables during kick-to-hold updates;
there is no intentional stop/pause between those phases. Software cannot
establish reset-time GPIO safety; external enable gating/pull-downs are needed
for that guarantee. A panic may bypass normal cleanup.

Host checks for the sequence, bounds, polarity and stop paths:

```sh
rustc +stable --test tests/motor_kick.rs -o /tmp/bimo-kick-tests
/tmp/bimo-kick-tests
```

### Suspended-wheel orientation-to-motor response (NOT floor balancing)

New binary/feature: `balance-motor-response-test`.
Motor permission feature: **`balance-motor-response-live`**, independent of
`balance-live`. The test feature reuses `balance-test` board-resource selection,
but does not change or run that DRY binary. Calibration helper code is included
by path; estimator, PD, proportional mapper, kick state machine, readiness
policy, motor polarity function and L298N driver are reused without changes.

Current physical labels for this diagnostic: **A = LEFT, B = RIGHT**. Historical
labels elsewhere predate this correction. Forward electrical commands remain
`(-u,+u)` and must be observed again with wheels suspended.

Constants at the top of `src/bin/balance_motor_response_test.rs`:

| Setting | Value |
| --- | --- |
| `SUPPLY_AND_CEILING_VERIFIED` | **false**, blocks real movement even with live feature |
| `MOTOR_DUTY_CEILING` | 70%; both ordinary mapping and kick validated against this |
| `WINDOW_US` | 5,000,000 us maximum from post-ready neutral acceptance into ACTIVE |
| `PREPARATION_SECS` | 15 seconds after BOTH calibration poses complete |
| `ARMING_TIMEOUT_US` | 10,000,000 us after preparation, covering readiness + neutral wait |
| `MAX_LOOP_GAP_US` | 30,000 us; active loop gap/work over this faults |
| Ordinary PD limit | ±15, asserted to match mapping input limit |
| Proportional mapping | deadband=2, minimum=30%, maximum=50%, slew disabled |
| Kick candidate | 70% for 150 ms, trigger=4 LIMITED PD points |
| Kick rearm/reversal | 300 ms zero dwell/cooldown; 100 ms reversal pause |
| Readiness | fresh, gravity-valid estimate; within ±5° and ±5°/s continuously for 500 ms |
| Tilt cutoff | magnitude **>=15°**, inherited from existing controller |
| Sample freshness | >40 ms without valid fresh data disarms |
| Hardware watchdog | 100 ms system-reset fallback, fed after fresh validated output only |

Configuration rejects a kick or ordinary maximum above the duty ceiling, rather
than silently bypassing or hiding a clamp. The 70% ceiling is a candidate,
**not a certified safe value**. The reported 9 V/1 A adapter result (70%/150 ms,
then 30%/1 s) was with suspended wheels only. It does not validate loaded use,
50% sustained commands, motor terminal voltage, stall current, or temperature.
Motors are rated 6 V; PWM percent is not a voltage/current guarantee. Verify the
actual supply, common ground and L298N logic supply before any live build.

Exact activation sequence:

1. HAL/board initialization drives enables/direction LOW; driver is disarmed.
2. Two-pose BMI160 calibration completes with motors disabled. Failure idles
   permanently until reset. Keep the chassis supported throughout both poses.
3. The 15-second preparation interval completes, still disabled.
4. Without BOTH the diagnostic-specific live feature and source safety flag,
   remain DRY permanently. Neither `balance-live` nor a predicted kick can arm.
5. With both gates explicitly enabled, fresh valid estimates establish the
   existing 500 ms upright/low-rate readiness dwell. This enters
   `ArmedWaitingForNeutral`; motors remain physically disarmed. Readiness and
   neutral waiting share a 10-second deadline starting after preparation.
6. A NEW post-readiness sample must have LIMITED PD within +/-2 AND current
   readiness still valid. Only then enter `Active`, start the five-second timer,
   and arm the driver at zero. The acceptance frame cannot kick. A subsequent
   meaningful LIMITED PD request can trigger the existing kick. Holding a
   non-neutral pose does not consume ACTIVE time; it can hit `ArmingTimeout`.
7. Zero cancels the kick; reversals follow the existing zero-pause/cooldown rules.
   Cutoff, invalid data (including gyro-only estimates outside the bounded ACTIVE policy), read errors,
   timing/stale faults, motor errors or window expiration cancel mapping/kick,
   disarm and permanently end the trial. Raising the robot upright cannot rearm.
   Reset is required for another run, including calibration and preparation.

Build only (no flash):

```sh
cargo build --release --locked --features balance-motor-response-test --bin balance-motor-response-test
cargo build --release --locked --features balance-motor-response-live --bin balance-motor-response-test
```

Manual DRY flash/run (wheels supported; motors remain disabled):

```sh
cargo run --release --locked --features balance-motor-response-test --bin balance-motor-response-test
```

For the first physical response trial: first review DRY signs and configuration.
Secure the chassis with BOTH wheels entirely off the ground, clear of fingers,
clothing and wires. Keep a physical motor-power cutoff immediately accessible.
Review the current supply and ceiling, then explicitly edit
`SUPPLY_AND_CEILING_VERIFIED=true` only in this diagnostic. The deliberate live
flash/run command is:

```sh
cargo run --release --locked --features balance-motor-response-live --bin balance-motor-response-test
```

Follow calibration prompts, secure support during the preparation interval, then
hold upright/low-rate. Readiness enters a bounded neutral wait, not ACTIVE.
Hold neutral (limited PD within +/-2) to start the short window, then lean gently within about ±10° and visually verify that
positive pitch/request produces the physical forward wheel directions via
A/LEFT negative and B/RIGHT positive. Cut motor power if signs are wrong. Do not
attempt ground-contact or free-standing balancing with this firmware.

Waiting/DRY telemetry is immediate. ACTIVE has **no blocking log calls**: a fixed
buffer captures calculated pitch/rate/limited-PD/mapping/kick/A-B snapshots at
about 10 Hz, printed only after disarm, with the final fault/timeout reason.
Short transitions can fall between snapshots; telemetry represents calculated
commands, not measured motor movement. Watch the wheels directly during the run.

The loop polls with a 1 ms asynchronous yield and uses measured sensor dt; it
introduces no kick/reversal sleep. This is not a guaranteed 100 Hz real-time loop:
existing I2C is blocking (20 ms transaction timeout), and the reused motor driver
briefly detaches enables and blocks about 1.2 ms per nonzero command. The new
30 ms deadline detects excessive delay when execution resumes; no logging occurs
while outputs are active. The watchdog resets after about 100 ms if progress
stops, but reset is NOT a guaranteed hardware motor cutoff. Reset/preboot GPIO
states, brownouts and panics still require external enable pull-downs/gating for
reliable de-energization. Reboots repeat calibration, not automatic live output.

### Response diagnostic freshness/timing logging correction

`BMI160::read` polls data-ready and returns `Ok(None)` until BOTH fresh gyro and
accel are available. The loop yields 1 ms, so most polls between roughly 100 Hz
samples normally have no new sample. Such a poll does not invalidate the cached
estimate, reset readiness/filter/kick, disarm, or reissue a motor command.

The old logger tested a per-iteration `record` variable. At a reporting deadline
an empty poll incorrectly printed `ready=false; no fresh valid estimate`, even
if the previous estimate was only a few milliseconds old. Its fallback also
checked a timestamp set before blocking log writes, so a slow write could itself
cause that misleading fallback. These were logging errors, not evidence that
all those sensor reads were stale. Actual gaps or readiness failures cannot be
excluded from the historical line count alone.

The corrected DRY summary uses the cached, accepted valid estimate. `FRESH`
means age <=40 ms; `poll_wait=true` simply means the current poll did not produce
a sample. `STALE (>40ms)` is separate from an initial wait, I2C error, or rejected
estimate. `ready` is the real readiness-policy result at report time, not a
fabricated value on an empty poll. Gyro-only estimates cannot establish readiness.

Timing fields are microseconds: `age` is time since the accepted estimate was
read, `dt` is measured fresh-sample interval, `loop` is start-to-start loop time,
`max_loop` is its high-water mark, and `work` is time spent in the current loop.
`missed` counts loops exceeding the unchanged 30 ms deadline, once per loop even
if multiple checks detect lateness. Misses are counted in DRY too, but fault LIVE.
A prolonged operation can count as a late work loop and a late following interval.
Invalid/waiting summaries also show acquisition and empty-poll counts; disarm logs
include explicit fault/detail and maximum loop/work times. This does not claim a
guaranteed 100 Hz acquisition rate; use measured `dt` to assess actual cadence.

The final motor-output gate now checks current elapsed time/freshness itself.
Actual commands are issued only for a new accepted estimate, with a fresh
orientation and ACTIVE state, while valid previous output can persist between
normal empty polls. The no-new-sample path checks work deadlines too. Exceeding
40 ms, missed active deadlines, sensor read failure, invalid LIVE estimates or correction-age expiry, timeout, and >=15° tilt retain immediate software disarm/lockout.
“Immediate” means when execution detects the event; a CPU stall still requires
hardware protection. No freshness/deadline thresholds were relaxed.

DRY intentionally computes mathematical PD/PWM beyond 15° and now explicitly
logs `live_tilt_ok=false`, calculated commands and `motors OFF`. That is not a
live command or permission to bypass cutoff. ACTIVE cannot command at >=15°
in either direction. Blocking logs remain absent from ACTIVE; buffered entries
print after outputs are disabled. DRY logs are reduced to one line per interval,
with the reporting timer updated after writes, reducing polling interference.
Slow output backends can still delay DRY acquisition; maximum loop time and
misses expose this. Existing blocking I2C/driver operations and watchdog/reset
limitations still apply.

Before considering LIVE, rerun DRY with the robot supported, inspect measured
sample/loop timing and any explicit errors, and confirm stable readiness while
upright plus correct calculated signs during gentle tilt. Resolve real timing
misses first. `SUPPLY_AND_CEILING_VERIFIED` remains **false**, and all source/
feature/post-calibration/neutral/one-shot interlocks remain in force. This change
does not recommend enabling LIVE based solely on software tests.


### Readiness, neutral and ACTIVE timer separation

The first LIVE run held pitch near +3.88 degrees: with Kp=1 and low pitch rate,
raw PD is about +3.88, truncated to LIMITED PD +3. Readiness allows +/-5 degrees,
but neutral requires LIMITED PD magnitude <=2. These are different gates.
The original policy armed the shared controller at readiness, starting both its
2-second cap and the wrapper timer, while returning no output until a later
neutral sample. That explains `ready=true`, PD/PWM `None`, and `TrialExpired`
without movement. It did not indicate a sensor or motor-driver failure.

Current response states are explicit in summaries and transition logs:

- `WaitingForReady`: build/source opt-ins passed, waiting for 500 ms stable
  upright with fresh gravity-valid data; motor driver stays disarmed.
- `ArmedWaitingForNeutral`: readiness passed once; still physically disarmed.
  Logs show actual LIMITED PD and `neutral` even though PWM is absent. A NEW
  sample must be neutral with current readiness still true to proceed.
- `Active`: post-ready neutral accepted. Only here is the shared controller
  armed; the response policy’s five-second timer starts. The shared controller is
  used in DRY mode for sample/readiness tracking, preserving its separate
  production two-second limit. Only response `Active` can permit motor output.
  The acceptance sample outputs zero; later movement requests drive the mapper.
- `Disarmed`: finished due to `ArmingTimeout`, `ActiveTimeout`, safety fault or
  stop. Motor output and kick state are disabled permanently until reset.
- `DryRun`: the feature/source interlocks do not permit physical movement.

`ARMING_TIMEOUT_US=10_000_000` begins after calibration and the 15-second
preparation interval. It covers BOTH readiness and neutral waiting, does not
restart when readiness fluctuates, and ends even if no sample arrives.
`WINDOW_US=5_000_000` begins only on the fresh post-ready neutral acceptance
sample, when motor-command eligibility starts. It includes subsequent zero
commands: it does not wait indefinitely for the operator to request movement.
Both timeout names and phase are logged distinctly. Neutral is still required;
no PD gains, deadband, mapping, kick, ceiling or polarity changed.

All transitions have a timestamp and reason. Waiting transitions print while
outputs are disabled. ACTIVE and terminal transitions are buffered and printed
after disarm, alongside the response-phase state and disarm reason. The separate
shared controller may still be `Observe` during the logical neutral-wait stage;
that is intentional, so its ACTIVE timer has not started prematurely.

Existing >=15-degree cutoff, 40 ms freshness, invalid gyro and bounded correction-age protection, timing
and one-shot disarm rules apply during the supervised neutral wait and ACTIVE.
The neutral gate additionally rechecks current readiness, rather than accepting
neutral PD caused by an unsafe angle/rate cancellation. Source verification and
diagnostic-specific live feature remain required and retain their current user
configuration. No firmware is flashed as part of software validation.

### Accelerometer correction rejection vs invalid orientation

An accelerometer is used as a gravity-direction observation only when its input
is finite, magnitude is 0.85–1.15 g, sideways component is <=0.35 g, and the
pitch-plane projection is >=0.5 g. `PitchFrame::accel_rejection()` now reports
which EXISTING gate failed, with measured values and thresholds (m/s²). It is
also used by `accel_angle`; no filter equations, gates, or calibration math were
relaxed. Normal hand translation, sideways tilt, or motor vibration can cross a
gate for a single sample; pure forward/backward tilt without translation need
not. The previous log did not record enough information to identify the actual
physical source or failing gate.

`accel_used=false` means a previously initialized filter rejected the current
accelerometer correction and instead propagated pitch with the fresh corrected
gyroscope. It does NOT by itself mean that the returned orientation is invalid.
The existing estimator returns no estimate for bad dt/gyro, missing initial
gravity reference, or over 500 ms accumulated gyro-only propagation. Previously,
the LIVE response policy additionally rejected EVERY false flag, including a
single valid gyro-propagated sample, producing the ambiguous `Fault(Invalid)`.

New response policy: **ACTIVE only**, permit gyro-propagated estimates for at
most `MAX_GYRO_ONLY_US=50_000` since the last accepted accelerometer correction.
This is an absolute elapsed-time limit, checked on every poll and in the final
output gate, not a count of samples or a fresh 50 ms allowance on each rejection.
An accepted correction at/before the deadline resets the age to zero; a correction
arriving after it cannot revive the trial. Zero/stop/fault/disarm behavior and
kick cancellation remain unchanged; faults clear tracking and permanently end
the trial. The last age/fault snapshot is retained only for diagnosis. New runs
start with no correction history.

The 50 ms choice is a conservative software candidate (~5 samples at 100 Hz,
~3 at 60 Hz), ten times shorter than the estimator's existing 500 ms fallback.
With the existing 0.5 s filter time constant, normal 10 ms updates already weight
integrated gyro by about 98%. A short skipped correction preserves continuity
without treating dynamic acceleration as gravity. Constant residual gyro bias b
would add about b*0.05 degrees over this interval; unknown vibration, clipping,
misalignment and gyro corruption can produce larger errors. This bound is NOT a
hardware-validated guarantee or permission for loaded/free-standing balancing.

Before ACTIVE, corrected acceleration is still mandatory for readiness and the
post-ready neutral gate. A rejection resets the initial readiness dwell, and a
rejection in supervised `ArmedWaitingForNeutral` disarms as before. No gyro-only
sample can enter ACTIVE. In ACTIVE all of these remain independent hard stops:

- >40 ms without a fresh valid gyro/estimate, even within the 50 ms correction bound;
- nonfinite acceleration, nonfinite raw/corrected gyro, exact gyro ADC rail
  values (possible saturation), nonfinite pitch/rate, or no filter estimate;
- I2C read failure, invalid sample timing, 30 ms loop deadline violation, >=15°
  estimated tilt, ACTIVE timeout, or other existing faults.

Nonfinite raw/corrected inputs and gyro clipping are checked BEFORE filter
propagation; no grace period applies to these cases. Failed sensor reads are not
relabelled as benign accelerometer rejection. `sensor_fault` identifies I2C,
gyro/accel validity, missing reference, correction-age expiry, filter failure or
sample freshness/timing separately from the response state and disarm reason.

Telemetry adds `accel_used`, correction age, and measured `accel_rejection`.
ACTIVE correction-loss/recovery transitions are captured in the same fixed
telemetry buffer, not synchronously logged while motors are energized. If the
buffer fills, `dropped_records` is reported. Every terminal disarm always logs its
specific sensor fault, last measured gate failure, correction age and any I2C
error. Threshold violations are not inferred from `accel_used` alone.

State-machine timers (10 s readiness/neutral, 5 s ACTIVE), PD gains, mapping,
kick, duty ceiling, polarity, supply flag and feature interlocks remain unchanged.
Review new DRY diagnostics before another suspended-wheel run. Short gyro-only
validity cannot detect all finite-but-wrong gyro data, nor translation that
happens to pass the acceleration gates. Freshness is measured from MCU read/
estimate timestamps, not hardware-timestamped acquisition; CPU stalls, I2C
latency, watchdog/reset and external motor-enable safety limitations still apply.


The response diagnostic now permits a **5-second ACTIVE window**; the separate
**10-second readiness/neutral timeout is unchanged**. The timer still starts on
fresh post-ready neutral acceptance, not at boot, calibration, preparation, or
initial readiness. All terminal conditions permanently disarm until reset.
The response policy owns this timer and enforces all sensor, >=15° tilt, timing,
duty and kick safeguards. Its shared controller stays in `DryRun` during the
response phase, so the production controller's unchanged `MAX_ACTIVE_US=2s`
cannot prematurely terminate this separate experiment. The response phase,
not that internal tracking state, gates every physical motor command. Local
configuration validation rejects response windows longer than 5 seconds.
