# Milestone 2: L298N motor driver

Normal firmware initializes the driver **disarmed**, owns it for the lifetime
of the main task, and never calls `arm()` or sends a movement command. No motor
test, balancing, PID, servo operation, or autonomous motion occurs at startup.
The existing BMI160 INT1 check on GPIO41, sensor driver, SDA14/SCL13 setup and
active-low buzzer are preserved.

## Ownership and PWM

| Electrical channel | Enable PWM | Input pair | Positive command | Negative command |
| --- | --- | --- | --- | --- |
| A | GPIO16 / ENA | GPIO5 / IN1, GPIO6 / IN2 | HIGH, LOW | LOW, HIGH |
| B | GPIO17 / ENB | GPIO7 / IN3, GPIO15 / IN4 | HIGH, LOW | LOW, HIGH |

A/B do not mean left/right. Positive/negative do not yet mean robot
forward/reverse. Those relationships must be observed with wheels off the ground.
Zero command means enable LOW and both corresponding inputs LOW (coast).
Stopping is not a mechanical brake and does not disconnect the battery.

`src/motors/control.rs` contains the hardware-independent `DualMotor` trait,
command gate and signed-percent mapping. A future TB6612FNG implementation can
implement that trait without changing callers. `src/motors/l298n.rs` implements
this interface for the wired L298N.

`InactiveOutputs` moves intact from board setup into `L298n::new`; all six
original GPIO drivers are retained. There is no pin stealing, cloning, unsafe
code, or drop-and-recreate handoff. The enable output latches remain LOW.
MCPWM0 operator0 A/B generate independent duty cycles at 10 kHz using timer0
(period 999, 1000 counts). The buzzer keeps its existing LEDC peripheral,
channel and all four LEDC timers; its code is unchanged.

The MCPWM HAL channel objects use `NoPin` while the driver owns the real GPIO
outputs. It explicitly routes the PWM signals to enables only for armed,
nonzero commands. A zero-duty channel stays physically disconnected from PWM,
so stopping does not depend on zero-compare event ordering. At 100%, timestamp
1000 exceeds period 999 and the up-counter output stays HIGH for the cycle.

Every command first stops both channels, waits 1 ms with enables LOW before
changing direction, updates compare values, waits two PWM periods (200 µs),
then connects nonzero enables to PWM. The delay is break-before-make logic,
not a claim that a spinning motor has stopped. Commands block for about 1.2 ms
and briefly interrupt both channels; this milestone is not a control-rate API.
Do not reverse a loaded spinning motor without a suitable application-level
stop/wait/current-management policy.

## Public API

```rust,ignore
use esp_bimo::motors::{DualMotor, DutyLimit, l298n::L298n};

let mut motors = L298n::new(
    board.motor_pwm,
    board.inactive_outputs,
    DutyLimit::new(30).expect("0..=100 percent"),
);
// Construction is stopped and disarmed; nothing moves here.
// The following is an API illustration, NOT a normal-firmware startup sequence:
motors.arm();
motors.command(15, -20)?; // A positive 15%, B negative 20%
motors.stop();           // both disabled/coasting; remains armed
motors.disarm();         // stop plus reject further nonzero commands
```

`DutyLimit::new(0..=100)` returns `Some`; higher values return `None`.
Commands are `i16` signed percent. Magnitude is clamped to the configured
ceiling, including values outside ±100 and `i16::MIN`, without overflow.
For example, with limit 30, +90 maps to +30%, not 27%. Limit zero maps all
commands to stopped. Both channels map independently.

Calling `command` while disarmed first stops both channels and returns
`Err(Disarmed)` for a nonzero request. `(0, 0)` remains allowed. `arm()` also
stops before opening the command gate; it never starts motion on its own.
The normal main has a 30% ceiling but leaves the gate closed permanently.
There is no automatic expiry of general API commands: an armed application
must explicitly stop/disarm. Only the separate physical-test program implements
the short pulse deadline described below.

`stop()` preloads both enable GPIO latches LOW, disconnects PWM through the
GPIO matrix, then clears the direction inputs and PWM compare values. It has
no fallible HAL operations and needs no Embassy scheduling. `disarm()` adds a
closed command gate. `Drop` disarms before stopping the timer and releasing
peripheral guards. This avoids stopping a timer with an enable latched HIGH.

**Drop runs on normal Rust destruction, not on reset, power loss, a panic-abort,
CPU lockup, or all forms of task cancellation.** Software cannot guarantee GPIO
levels during boot/reset/bootloader/flashing or guarantee shutdown after a CPU
failure. External enable pull-downs and independent hardware cutoff/gating are
needed for such guarantees. The driver neither verifies those protections nor
measures voltage/current. A duty ceiling is not a current limit or an RPM limit.

## Verify the hardware before any powered test

1. Keep motor battery power disconnected while flashing and checking idle
   outputs. Verify ENA/ENB and IN1–IN4 are LOW at accessible measurement points.
   ENA/ENB jumpers must remain removed. The separate **5V-EN regulator jumper**
   is not an enable-control jumper.
2. Confirm battery negative, L298N ground and ESP32 ground are common. Use
   accessible connectors or measurement points first; no chassis dismantling
   or lower-chassis rewiring is required or proposed by this implementation.
3. Verify the actual L298N logic rail. The ST chip specifies VSS=4.5–7 V
   (nominally 5 V). Four AA cells around 6 V do **not** establish that the
   module's onboard regulator has enough input headroom, especially under
   load. Measure at an accessible labeled logic-rail terminal/test point
   relative to common ground and identify the module/regulator if uncertain.
   If no safe measurement point is accessible, leave powered motor tests
   deferred until a measurement approach is agreed.
4. Do not feed the module's logic rail from ESP32 3.3 V or blindly connect
   USB 5 V to the module's 5 V terminal. Its input/output behavior depends on
   the actual regulator/jumper circuit. First identify that circuit and verify
   measured voltage; alternatives to rewiring include accessible measurement
   and module identification. No regulator/jumper modification is prescribed.
5. With a valid logic supply the ST chip's HIGH input minimum is 2.3 V, so
   nominal 3.3 V GPIO signaling is compatible. Confirm actual module wiring,
   rail validity, and absence of backfeed; this has not been measured here.
6. Raise and restrain the robot so both wheels can turn freely. Keep fingers,
   wires and clothing clear, and make battery disconnection immediately
   accessible. Know how to disconnect motor power independently of software.
7. TT-motor stall current, battery chemistry/current capability, module diode
   protection, thermal performance and supply sag have not been established.
   The L298 loses voltage across its bridge; a 6 V battery does not imply 6 V
   at the motors. A 20% pulse may fail to start a motor. Do not increase duty
   or hold a stalled motor on to compensate before verifying supplies/current.

References: [ST L298 datasheet](https://www.st.com/resource/en/datasheet/l298.pdf),
[installed esp-hal MCPWM API](https://docs.espressif.com/projects/rust/esp-hal/1.2.2/esp32s3/esp_hal/mcpwm/index.html).
Generic chip documentation cannot establish the regulator circuit on this module.

## Optional physical test — automatic one-shot in a separate binary

**The agent has not flashed or run this test.** Normal firmware is unchanged;
only the explicit test binary automatically moves a motor after boot.
The test still requires both `--features motor-test` and `--bin motor-test`.
It has no serial/USB command input, parser, or receive driver. Stage logs use
the existing logger. It does not play the buzzer or initialize the IMU.

Sequence on every boot/reset of the test binary:

1. Log **waiting**, with both motors disarmed; wait 3 seconds using Embassy.
2. Log **starting**, arm and command A=+20%, B=0. The existing driver performs
   its disabled direction/PWM preparation before routing ENA to PWM.
3. Hold that command for 250 ms using an elapsed-time software deadline,
   with no logging, USB operations, or Embassy awaits while energized.
4. Immediately stop/disarm both motors, then log **stopping**. Logging is
   deliberately after disabling the outputs so a blocked logger cannot
   prolong motor power.
5. Drop the driver, log **finished**, and idle forever. No repetition occurs
   until the ESP32 is reset. B stays disabled through the entire sequence.

Build only (no flashing or motor activity):

```sh
cargo build --release --locked --features motor-test --bin motor-test
```

When explicitly ready to flash, disconnect battery power first:

```sh
cargo run --release --locked --features motor-test --bin motor-test
```

**Flashing normally reboots the ESP32. This test starts automatically after
its 3-second wait, with no further confirmation or serial command.** Complete
the supply/ground and raised-wheel checks above before a powered test. Keep
motor power disconnected until the robot and cutoff are ready; a reset with
motor power connected will trigger the sequence again.

Disconnect motor power after the test, record which wheel/direction channel
A positive actually corresponds to, and restore normal firmware:

```sh
cargo run --release --locked --bin esp-bimo
```

**The 250 ms timing is a software bound under normal CPU/timer execution,
not an independent hardware cutoff.** Polling granularity and interrupts can
add timing overhead. CPU failure, reset, or invalid logic supply cannot be
made safe by this deadline or Drop. There is no longer a serial abort path;
use the accessible battery cutoff if anything unexpected occurs.

## Software validation

```sh
cargo fmt --check
cargo check --locked
cargo build --locked --bins --features motor-test
cargo build --release --locked --bins --features motor-test
cargo clippy --locked --all-features --bins --lib -- -D warnings
rustc +stable --edition=2024 --test tests/motor_control.rs -o /tmp/esp-bimo-motor-tests
/tmp/esp-bimo-motor-tests
rustc +stable --edition=2024 --test tests/imu_sample.rs -o /tmp/esp-bimo-imu-tests
/tmp/esp-bimo-imu-tests
```

Host tests exercise production mapping, direction-input truth table, duty
validation/clamping, signed extremes, and gate behavior. They do not validate GPIO timing, PWM routing/frequency,
stopping under electrical faults, or the actual motors.
Those remain explicit, pending real-hardware checks. Target builds and HAL
source inspection support the ownership/routing design, not hardware proof.
