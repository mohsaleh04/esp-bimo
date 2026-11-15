# Fixed-duty motor characterization (default DRY)

This independent binary uses the existing L298N/MCPWM driver and Board pins. It
runs no estimator, PD, balance controller, Rolling Assist or startup Kick. Forward
uses the confirmed robot-motion mapping (-u,+u): A/LEFT negative, B/RIGHT positive.
Both motors run together from the shared supply in each step. Backward uses
(+u,-u). One paired driver command sets A/LEFT and B/RIGHT; this does not imply
atomic hardware edges or equal physical wheel response.

Sequence, once per reset:

1. Both motors disarmed during 15-second preparation.
2. BOTH motors FORWARD: STEP 1/6=40%, 2/6=50%, 3/6=60%.
3. BOTH motors BACKWARD: STEP 4/6=40%, 5/6=50%, 6/6=60%.
4. Final stop/disarm and permanent idle until reset.

Each duty is applied for at most 1 second in the normal scheduled execution path,
with a 5-second stop/disarm pause after every step. Ceiling is 60%; there is no
boost or retry. Total scheduled duration is 51 seconds plus startup/log overhead;
a 100-second timeout from sequence initialization prevents new commands and clips
waits/pulses. Driver rejection aborts the sequence. Actual stop/disarm occurs before
STOP logs and every pause; no serial output occurs while a pulse is energized.
Timers are cooperative software bounds, not guarantees during a CPU/scheduler hang.
Existing driver Drop shutdown is retained. Software does not guarantee boot/reset
GPIO levels; independent motor-power cutoff remains mandatory.

Before LIVE: physically restrain the robot and secure both wheels. Verify motor
supply/current limits under load for the 6V-rated motors. Attach insulated probes
across the selected motor with motor power OFF; never hold loose probes near moving
parts. Use meter capture/min-max or external recording if one second is too short
to settle; do not extend a stalled pulse merely to read the meter. Ordinary meter
PWM voltage readings do not establish peak voltage, safe current or movement. Cut
motor power immediately for stalled buzzing, unexpected direction, heating or unsafe
motion; the program cannot detect stall or confirm rotation. Observe and record
both motors, direction, duty, actual movement and instrument reading during each labelled
step. STEP labels show direction, duty and requested signed A/B commands. STOP
labels report the driver-commanded pulse duties, then actual software stop/disarm
(A=0 B=0) and the neutral pause. FINISHED reports completed stages out of six.
In DRY, requested duties are previews and driver-commanded duties remain zero.
These are software acknowledgements, not electrical or wheel-motion measurements.

Default DRY flash-and-monitor command, for the operator only:

```sh
cargo run --release --bin motor-duty-characterization --features motor-duty-characterization
```

For a deliberately authorized LIVE test, change **only this diagnostic's**
`MOTOR_POWER_AND_RESTRAINT_VERIFIED` to true after those checks, then use the existing
explicit motor LIVE feature:

```sh
cargo run --release --bin motor-duty-characterization --features motor-duty-characterization,motor-kick-live
```

`motor-kick-live` alone cannot bypass the false source flag. Balance LIVE features
cannot authorize this binary. These commands flash and start the sequence; they
were not executed during implementation. Use `cargo build` instead of `cargo run`
to compile without accessing the board. Do not select the separate motor-kick binary.
