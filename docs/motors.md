# TB6612 motor driver

Normal firmware initializes the driver **disarmed**, owns it for the lifetime
of the main task, and never calls `arm()` or sends a movement command. No motor
test, balancing, PID, servo operation, or autonomous motion occurs at startup.
The existing BMI160 INT1 check on GPIO41, sensor driver, SDA14/SCL13 setup and
active-low buzzer are preserved.

## Ownership and PWM

| Electrical channel | Enable PWM | Input pair | Positive command | Negative command |
| --- | --- | --- | --- | --- |
| A | GPIO17 / PWMA | GPIO15 / AIN1, GPIO16 / AIN2 | LOW, HIGH | HIGH, LOW |
| B | GPIO5 / PWMB | GPIO7 / BIN1, GPIO6 / BIN2 | HIGH, LOW | LOW, HIGH |

A is the intended left wheel and B the right wheel. Verify physical direction
with wheels clear of the ground. GPIO40 drives STBY, and a zero command
deasserts STBY so both bridges coast.

`src/motors/control.rs` contains the hardware-independent `DualMotor` trait,
command gate and signed-percent mapping. `src/motors/tb6612.rs` implements
that interface. `MotorOutputs` moves all seven GPIOs into `Tb6612::new`. There is no pin stealing, cloning, unsafe
code, or drop-and-recreate handoff. The enable output latches remain LOW.
MCPWM0 operator0 A/B generate independent duty cycles at a requested 35 kHz using timer0
(period 999, 1000 counts). The buzzer keeps its existing LEDC peripheral,
channel and all four LEDC timers; its code is unchanged.

The MCPWM HAL channel objects use `NoPin` while the driver owns the real GPIO
outputs. It explicitly routes the PWM signals to enables only for armed,
nonzero commands. A zero-duty channel stays physically disconnected from PWM,
so stopping does not depend on zero-compare event ordering. At 100%, timestamp
1000 exceeds period 999 and the up-counter output stays HIGH for the cycle.

The driver caches the last accepted pair after duty clamping. Repeated
nonzero commands with the same effective pair leave the bridge alone.
Changes to duty with the same directions and active channels update only
the changed MCPWM compare registers. Both use `SYNC_ON_ZERO`, so each
compare becomes active at the next timer zero. The two writes can straddle
that boundary.

Start, reversal, adding or stopping a channel, and changes involving
100% use the gated path. It lowers STBY, waits 1 ms before changing
direction, writes and checks the compares, and waits 200 µs plus an
observed timer wrap before routing PWM and raising STBY. The wrap wait
has a 200 µs timeout. STBY is a GPIO, so the first visible PWM pulse may
be a fraction of a correctly configured cycle. A compare readback mismatch
or a missing wrap on this path disarms the driver. Stop and disarm lower
STBY immediately, without waiting for the compare latch.

## Public API

```rust,ignore
use esp_bimo::motors::{DualMotor, DutyLimit, tb6612::Tb6612};

let mut motors = Tb6612::new(
    board.motor_pwm,
    board.motor_outputs,
    DutyLimit::new(30),
);
// Construction is stopped and disarmed; nothing moves here.
// The following is an API illustration, NOT a normal-firmware startup sequence:
motors.arm();
motors.command(15, -20)?; // A positive 15%, B negative 20%
motors.stop();           // both disabled/coasting; remains armed
motors.disarm();         // stop plus reject further nonzero commands
```

`DutyLimit::new` caps its argument at 100%.
Commands are `i16` signed percent. Magnitude is clamped to the configured
ceiling, including values outside ±100 and `i16::MIN`, without overflow.
For example, with limit 30, +90 maps to +30%, not 27%. Limit zero maps all
commands to stopped. Both channels map independently.

Calling `command` while disarmed first stops both channels and returns
`Err(Disarmed)` for a nonzero request. `(0, 0)` remains allowed. `arm()` also
stops before opening the command gate; it never starts motion on its own.
The normal main has a 30% ceiling but leaves the gate closed permanently.
There is no automatic expiry of general API commands: an armed application
must explicitly stop/disarm. Balance and the powered diagnostics enforce their own session or pulse deadlines.

`stop()` lowers STBY, preloads both enable GPIO latches LOW, disconnects PWM through the
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

## Hardware and diagnostics

See [live balance](balance-v2-live.md) for arming and shutdown behavior.
Use the [logic and no-load diagnostic](tb6612-diagnostic.md) for wiring
and electrical measurements, and the [suspended-wheel diagnostic](tb6612_diagnostics.md)
for physical direction checks.

## Software validation

```sh
cargo fmt --check
cargo check --locked --bins
for test in motor_control imu_sample motor_kick tb6612_direction_authorization tb6612_no_load; do
    rustc +stable --edition=2024 --test "tests/${test}.rs" -o "/tmp/esp-bimo-${test}" &&
        "/tmp/esp-bimo-${test}"
done
```

Host tests exercise production mapping, direction-input truth table, duty
validation/clamping, signed extremes, and gate behavior. They do not validate GPIO timing, PWM routing/frequency,
stopping under electrical faults, or the actual motors.
Those remain explicit, pending real-hardware checks. Target builds and HAL
source inspection support the ownership/routing design, not hardware proof.
