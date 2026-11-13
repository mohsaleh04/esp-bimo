# Bimo — Milestone 2

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
