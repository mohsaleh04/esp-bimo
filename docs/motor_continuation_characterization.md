# Motor continuation characterization — default DRY

Separate firmware, no PD/controller/estimator/Rolling Assist. Existing L298N,
MCPWM settings and pin ownership are reused. A=LEFT, B=RIGHT; robot-forward
uses (-u,+u), backward (+u,-u). Both motors are commanded together.

| Stage | Direction | Startup (up to 100 ms) | Hold (up to 1 second) |
|---|---|---:|---:|
| 1 | Forward | 70% | 50% |
| 2 | Forward | 70% | 40% |
| 3 | Forward | 70% | 30% |
| 4 | Backward | 65% | 50% |
| 5 | Backward | 65% | 40% |
| 6 | Backward | 65% | 30% |

Begin disarmed, wait 15 seconds. Each stage begins at zero, applies startup,
then changes directly to same-direction hold without an intervening delay/rearm.
Stop/disarm both motors and pause 5 seconds after each stage. No retry or repeat
until reset. Nominal duration including preparation and pauses is 51.6 seconds;
the overall timeout remains 100 seconds, measured before preparation. Phase waits
are clipped to that deadline; expired commands or driver rejection abort the test.
Final stop/disarm and driver Drop shutdown are unconditional.

`PREPARATION_MS`, `STARTUP_MS`, `HOLD_MS`, `PAUSE_MS`, `TOTAL_MS` and `CEILING`
are local constants in the new sequence module. These are cooperative software
timing bounds, not hard guarantees during a CPU/scheduler hang. Boot/reset GPIO
levels are not guaranteed by firmware. Keep an independent motor-power cutoff.

Serial STAGE headers identify direction, requested startup/hold duties and durations.
No serial printing occurs while motors are energized. After shutdown, STARTUP and
HOLD reports include acknowledged driver-commanded signed A/B duty and elapsed time;
STOP reports A=0 B=0/disarmed and pause. FINISHED gives completed stages out of six.
DRY executes the timing sequence with commanded outputs always zero. Command
acknowledgements are not measurements of voltage, speed, current or physical motion.

Before LIVE, verify the shared 9V supply/current limits for the 6V-rated motors
under actual load; neither duty nor a multimeter average proves safe peak voltage
or current. Restrain the chassis independently, keep people/probes clear of both
wheels and have a reachable independent cutoff. Preattach insulated meter probes
with motor power OFF. Observe whether BOTH wheels start and continue during hold;
record direction, duty, movement and measured voltage. Cut motor power for stalled
buzzing, heating or unsafe movement; there is no automatic stall detection. This
experiment tests continuation only, not autonomous balance.

Default DRY (operator command; flashes and runs):

```sh
cargo run --release --bin motor-continuation-characterization --features motor-continuation-characterization
```

LIVE requires changing **this new binary's** `MOTOR_POWER_AND_RESTRAINT_VERIFIED`
from false to true after the checks above, independently of the existing explicit
`motor-kick-live` feature:

```sh
cargo run --release --bin motor-continuation-characterization --features motor-continuation-characterization,motor-kick-live
```

Balance LIVE features cannot authorize this binary. Enabling the LIVE feature alone
leaves this diagnostic DRY while its source gate is false. Other binaries' source
flags do not grant permission here. Use `cargo build` instead of `cargo run` to
compile without accessing hardware. No commands above were executed on hardware.
