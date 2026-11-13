# MCPWM diagnostic: A / RIGHT wheel

Both static enable-HIGH tests rotated their respective motors: A (GPIO16) is
RIGHT, B (GPIO17) is LEFT. Physical forward/reverse is still uncalibrated.
These results establish that both bridge channels can drive motors; they do
not establish PWM frequency, duty, or starting torque under PWM.

## Finding and correction

The earlier PWM test had `DutyLimit::new(40)` while requesting `command(75, 0)`.
The driver correctly clamps signed-percent magnitude: that request delivered
at most **40%, not 75%**. Its printed requested duty did not disclose the clamp.
The separate diagnostic now has both a **50% limit and a +50% request**.
The driver logs requested/effective duty and period/compare register readback
before attaching the enables, so serial logging cannot hold a running motor on.

This corrects a demonstrated test-configuration/reporting defect. It is **not
proof of the cause of motor buzzing**, and does not claim a hardware fix.
Insufficient starting torque at the earlier effective duty is a possible
explanation, not an established diagnosis. Measure ENA relative to common ground
at an accessible point if the new diagnostic still buzzes: expected active-high
10 kHz, approximately 50 µs HIGH / 50 µs LOW during its pulse. Do not increase
supply voltage or alter wiring based on the software inspection alone.

## Installed esp-hal 1.2.2 source audit

Source root: `~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/esp-hal-1.2.2/src/`.

- `mcpwm/operator.rs`: `Operator::with_pins` consumes the operator in the
  standard HAL path. `PwmPin::new` configures generator actions/update mode,
  connects the selected output, and enables the pad. `NoPin` deliberately skips
  pad work, while the driver's actual enable `Output` owners were already
  initialized LOW/output-enabled by board setup.
- `gpio/interconnect.rs` / `gpio/mod.rs`: calling `connect_to(&Output)` forwards
  through Flex/AnyPin to the same output-matrix connection path used by the
  normal HAL PWM pin. GPIO16/17 have no native MCPWM alternate function, so
  these signals route through the GPIO matrix. The selected MCPWM0 operator0
  outputs are `PWM0_0A` / `PWM0_0B`, non-inverted. NoPin is not evidence that
  the real enables cannot be manually connected.
- `mcpwm/mod.rs` / `mcpwm/timer.rs`: retained timer0 holds a function-clock
  guard. Dropping unused operators/timers does not drop the last clock request.
  A 40 MHz peripheral clock, divide-by-4 timer clock, and 1000-count up-counter
  produce 10 kHz. Period=999; compare=500 corresponds to nominal 50%.
- `PwmActions::UP_ACTIVE_HIGH` raises at timer zero and lowers at compare.
  `PwmUpdateMethod::SYNC_IMMEDIATLY` means immediate update in this HAL and the
  installed esp32s3 PAC register documentation; zero does not mean frozen.
- Stop first switches enables back to their LOW GPIO latches. Neither compare
  zero edge precedence nor stopping the timer while HIGH determines shutdown.

No concrete defect in this NoPin/routing/timer/polarity implementation was
established. Accordingly those mechanisms, 10 kHz configuration and the
`DualMotor` interface remain unchanged. Rewriting them would introduce another
unverified variable. Register readback is not proof of the physical waveform.
See also [Espressif MCPWM guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/mcpwm.html).

## Separate one-shot test

`motor-pwm-test` is a new binary gated by the existing `motor-test` feature.
The existing static `motor-test` binary is retained unchanged. The normal
firmware, its disarmed motor behavior, sensor code, I²C and LEDC buzzer are
unchanged. No dependency or version change is required.

Build only:

```sh
cargo build --release --locked --features motor-test --bin motor-pwm-test
```

After supply/ground, free-wheel and accessible power-cutoff checks, this is the
manual flash command **only when the user decides to run the physical test**:

```sh
cargo run --release --locked --features motor-test --bin motor-pwm-test
```

This binary automatically waits 3 seconds, arms, commands A=+50% and B=0,
holds for 250 ms by a software deadline, stops/disarms both, drops the driver,
then idles until reset. A reset causes a new one-shot sequence. B remains
GPIO-disabled with both direction inputs LOW throughout. All logs occur while
the enables are disabled, before or after the energized interval. The log
should show effective A=50%, B=0%, period=999, compare A=500, B=0.

The timer deadline and Drop are not a hardware cutoff against CPU failure,
interrupt stalls, reset, or invalid L298 logic supply. Existing logic-rail and
boot/reset safety limitations in [motor safety notes](motors.md) still apply.
No flashing or physical motor test was performed during this change.
