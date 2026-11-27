# Bimo

Rust firmware for the ESP32-S3 N16R8 self-balancing robot. The live Balance V2
firmware demonstrated physical self-balancing with the independent 2S motor
supply. See [live balance](docs/balance-v2-live.md) for the exact command,
operating limits, and hardware checks.

## Development

Use the `esp` toolchain installed by espup and source `~/export-esp.sh`
before building and running on Unix.

```sh
cargo build --release --locked --bin esp-bimo
cargo run --release --locked
```

`cargo run` selects `esp-bimo` and invokes espflash with the serial monitor.
The default firmware keeps the TB6612 disarmed, plays the buzzer melody,
scans I²C, and logs BMI160 samples. Select `balance-v2-test` explicitly
for the dry pitch and motor-command preview. The powered balance session
requires explicit selection of both the live binary and its Cargo feature.

## Wiring and diagnostics

| Function | GPIO |
| --- | --- |
| I²C SDA / SCL (I²C0, 100 kHz) | 14 / 13 |
| QMC5883L DRDY / BMI160 INT1 | 12 / 41 |
| Tilt / vibration inputs | 2 / 4 |
| IR analog / digital | 1 / 39 |
| TB6612 AIN1 / AIN2 / PWMA (left) | 15 / 16 / 17 |
| TB6612 BIN1 / BIN2 / PWMB (right) | 7 / 6 / 5 |
| TB6612 STBY | 40 |
| Passive buzzer / head servo PWM | 21 / 18 |
| HC-SR04 TRIG / ECHO (level-shifted) | 38 / 47 |

See the [TB6612 driver and electrical precautions](docs/motors.md) and the
following diagnostic procedures.

- [Hardware bring-up, touch, IR, eyes, buzzer, and HC-SR04](docs/hardware_diagnostics.md)
- [Head servo](docs/servo_diagnostics.md)
- [BMI160 orientation and axis identification](docs/orientation.md)
- [TB6612 logic and disconnected-motor output voltage](docs/tb6612-diagnostic.md)
- [TB6612 dry direction preview](docs/tb6612-direction-dry.md) and [suspended-wheel direction](docs/tb6612_diagnostics.md)
- [Fixed-duty motor characterization](docs/motor_duty_characterization.md) and [motor continuation](docs/motor_continuation_characterization.md)
- Motor kick/hold diagnostic. Select `motor-kick-diagnostic` with its matching
  feature for the dry sequence. The separate `motor-kick-live` feature enables hardware actuation
  when the diagnostic settings pass validation.

## Software checks

```sh
cargo fmt --check
cargo check --locked --bins
CARGO_ENCODED_RUSTFLAGS='' cargo +stable test --locked \
  --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu
```

Build the mutually exclusive TB6612 LOADED and NO-LOAD configurations in
separate invocations. The [TB6612 diagnostic guide](docs/tb6612-diagnostic.md)
lists the commands. [Motor driver checks](docs/motors.md) include the
standalone tests.
