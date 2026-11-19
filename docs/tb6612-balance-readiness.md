# TB6612 balance readiness

Board creates STBY LOW before PWM/direction GPIO initialization. MotorOutputs
owns GPIO40 and moves into the bridge driver. No second owner is created in
the balance runtime. Other diagnostic binaries own their GPIO independently.

`motors::tb6612::Tb6612` implements TB6612 standby gating. arm() stops first
and grants command permission with STBY LOW. Repeating an effective command
leaves the outputs alone. Same-direction duty changes with the same active
channels use compare-only writes latched at timer zero. Starts, direction or
active-channel changes, and duty changes involving 100% use the existing
break-before-make path, with a timer-wrap check before routing and raising
STBY. Rejected or zero commands, stop(), disarm() and ordinary Drop lower
STBY first. A compare readback mismatch or reconfiguration latch timeout
disarms. See [the motor driver notes](motors.md) for timing details.

STBY LOW makes the TB6612 outputs high impedance (coast). This is not short
braking. PWM LOW while STBY HIGH can brake the selected bridge. Requested
frequency remains 35 kHz, period 999, unchanged from the existing code. The old
10 kHz comments/logs were incorrect. No frequency measurement was performed.

Logical forward now maps to (u,u), A LEFT and B RIGHT. The shared driver
inverts only A. The previous (-u,u) encoded the old wiring convention and would
request opposite logical wheel directions. PD gains and pitch computation are
unchanged. The sign relating positive pitch to physical forward lean still
requires physical verification. The control-test restraint gate was reset to
false because its previous confirmation predates this contract change.

## Shared balance LIVE authorization

All shared balance profiles check the source-level
`TB6612_BALANCE_LIVE_VERIFIED` gate before calibration. The gate defaults to
false. An explicit LIVE request fails closed and idles disarmed until reset.
DRY runs use the existing profile features without their LIVE feature.

## Verification and remaining work

The release builds and direct motor-control tests pass. The full host replay
suite is currently failing, including additional failures from stale expected
wheel-sign fixtures. It must be updated and reviewed before live balance.
Full output is /tmp/bimo-stby-suite. Unit tests proving GPIO transition order
and a dedicated physical-sign calibration binary are still required.

The shared driver has no panic shutdown hook yet. Drop does not run on abort.
The diagnostic-specific panic handler does not protect balance binaries.
A stalled CPU can leave STBY HIGH. The external 10 kohm pull-down helps reset
and floating states but cannot override a driven HIGH pin. Physical power
cutoff remains essential. This firmware is not cleared for live balance.

## Dry procedure

1. Keep VM disconnected and use USB logic power.
2. Build with:
   cargo build --release --locked --bin balance-control-test --features balance-control-test
3. In a separately authorized flash session run the dry firmware, inspect
   calibration, pitch sign, commanded A/B and fault/disarm logs.
4. Tilt forward and backward and compare the sign of reported pitch and logical
   wheel commands against the expected corrective motion.
5. Before a first powered test, resolve the replay failures, implement/test panic
   shutdown and prepare a separately gated bounded calibration pulse.
6. With wheels suspended, suitable current-limited VM and accessible cutoff,
   verify each channel's logical forward and the pitch-to-correction sign.
   Keep live balance disabled until these checks have passed.

No flash or live test was performed during this update.
