# Experimental bounded continuous bidirectional balance-control profile

`balance-control-test` is a NEW, separate firmware profile. It continuously
recalculates corrections on fresh BMI160 estimates during a **maximum 30-second
ACTIVE experiment**. It does not provide unlimited running or autonomous,
unrestrained balancing. Independent mechanical restraint and a motor-power
cutoff remain mandatory. No gains have been tuned by this software change.

## Architecture and states

The profile reuses the existing shared runtime, two-pose calibration, learned
pitch frame, complementary filter, PD function, PWM mapper, L298N driver,
acknowledged-zero interlock, watchdog and fixed-memory flight recorder.
`control_profile.rs` adds only configuration selection, strict fault enforcement
and behavior statistics. Shared PWM/settings constants now live in `settings.rs`
so host replay uses the same values. Existing diagnostic control paths are
preserved; the previously enabled bidirectional wrapper gates are reset to false.

States logged with `CONTROL STATE`:

1. `CALIBRATING`: existing upright/forward two-pose procedure; motors disarmed.
2. `PREPARING`: 30 seconds, motors disarmed; secure independent support.
3. `WAITING_FOR_READY`: corrected, fresh upright estimates; existing 500ms dwell
   within ±5° and ±5°/s. No output or implicit startup pulse.
4. `ARMED_WAITING_FOR_NEUTRAL`: still physically disarmed; a NEW fresh neutral
   sample (|limited PD|≤2) AND current readiness must be accepted.
5. `ACTIVE`: only now can the final motor gate pass commands. The configured 30-second
   timer starts on neutral acceptance; that sample still commands zero. Later
   meaningful requests can receive the initial Kick.
6. `DISARMED`: actual driver shutdown and terminal latch; reset required.

Default DRY selection uses `DRY_RUN` after preparation, bypassing LIVE arming.
It evaluates math with actual motor outputs always zero, even above the live
cutoff. Acquisition is serial-silent for a fixed 10-second DRY capture, then
permanently stops with reason Stop and prints the capture. Reset is required
to capture again. It never transitions into ACTIVE. Lifecycle labels describe the existing
Response state, not a second arming authority. ACTIVE transitions and statistics
are printed AFTER motor shutdown, never as blocking logs in the powered loop.

## Unchanged control and safety

- PD = 1.0 × pitch(deg) + 0.08 × rate(deg/s), Ki=0. Limited to ±15 and truncated
  toward zero to integer PD units. Forward=(-u,+u), A LEFT, B RIGHT.
- |limited PD|≤2 is zero. Ordinary PWM is proportional 30–50%; the first nonzero
  integer request, |PD|=3, maps to 31.54%, truncated to 31% at the driver.
- Initial Kick eligibility |limited PD|≥3; 70%, at most 150ms, cancelled by zero
  or unsafe data/state. One accepted Kick per trial. No second Kick on reversal.
- Reversal first sends actual driver commands (0,0), then waits ≥100ms from the
  driver's successful completion timestamp. Sensor/control polling continues;
  rapid direction chatter restarts/cancels the wait conservatively. This is NOT
  measured wheel stop, braking, or protection against all reversal current.
- |pitch|≥21° cuts output, sample/gyro freshness ≤40ms, accepted accelerometer
  correction age ≤50ms during ACTIVE, control-loop deadline ≤30ms. Invalid,
  clipped/nonfinite gyro, invalid estimates, I2C and driver errors disarm.
- New-profile sensor/timing faults also permanently latch during readiness.
  Calibration failures remain disarmed and require reset. The existing ten-second
  combined readiness/neutral timeout and developer-selected 30-second ACTIVE timeout remain.
- No automatic rearming; no balancing without restraint; no minimum-duty boost
  beyond the existing explicitly configured mapper/Kick behavior.

BMI160 is configured at 100Hz. The runtime polls nonblocking data readiness at
about 1ms intervals and updates the estimator/PD/mapping only for a fresh sample,
approximately 100Hz. Actual sample timestamp differences determine filter dt and
Kick timing; elapsed wall time supervises deadlines and timeouts. Empty polls do
not repeat motor commands or fabricate samples. There is no fixed-dt assumption.
The existing L298N driver has short synchronous IO delays; no new blocking delay
was added. The 100ms watchdog is backup only. Software cannot guarantee GPIO
levels through reset/crash or all electrical faults; external cutoff remains
necessary.

## Telemetry and metrics

Existing flight-recorder entries retain pitch, rate, limited PD before mapping,
proportional mapped PWM, requested vs accepted motor commands, Kick/interlock
state, correction age, freshness, deadline misses and terminal reason. Requested
In default legacy mode, Kick previews can differ from accepted output because
of the one-Kick gate. The optional integrated Restart Assist mode separately
records proposed, approved and acknowledged duty; see [output arbitration](output_arbitration.md).
Pinned first applied command, applied Kick edges and reversal entry/release
survive ring-buffer overwrites. In this profile ALL serial writes during
acquisition/readiness/ACTIVE are deferred until actual driver disarm, including
state transitions. A separate 64-slot, 10Hz periodic snapshot ring retains recent
pitch/rate, requested/applied commands, Kick/interlock, sensor and timing fields.
Old periodic samples are overwritten with an explicit counter; the existing
event recorder and its pinned fault/terminal slots remain independent. Buffer
fullness cannot trigger a print, allocation, wait, or a safety-policy change. Deferred logs may be lost on reset/power failure.

`behavior` summarizes fresh estimate count, maximum absolute pitch (including a
finite terminal tilt estimate), angle zero crossings, total ACTIVE elapsed time,
zero-output time and longest contiguous zero interval. Time is integrated from
measured loop timestamps using the PREVIOUS held actual command. Resolution is
loop/command-observation time, not waveform measurement. DRY has real zero output
throughout, so its zero time is deliberately not simulated wheel behavior.
Zero-crossing counts ignore exact-zero samples but do not debounce noise; they
are observations, not evidence of damping or autonomous stability.

## Selection and manual procedure

Default/DRY feature and binary: `balance-control-test`.
LIVE feature: `balance-control-live` only. `balance-live` and other diagnostic
features cannot grant motor permission to this wrapper.

The four LIVE source flags in `src/bin/balance_control_test.rs` have been
restored to FALSE. The separate `RESTART_ASSIST_REVIEWED` flag is also FALSE.
LIVE requires the original four flags and this profile's LIVE feature; selecting
Restart Assist additionally requires its own source review gate and
`balance-control-restart-assist`. Default LIVE behavior remains legacy one-Kick;
no independent initial pulse coexists with the opt-in shared assist budget.
See [the complete arbitration and gate description](output_arbitration.md).

Build without flashing:

```sh
cargo build --release --locked --features balance-control-test --bin balance-control-test
cargo build --release --locked --features balance-control-live --bin balance-control-test
```

Manual board DRY command (flashes when YOU execute it; motors remain disabled):

```sh
cargo run --release --locked --features balance-control-test --bin balance-control-test
```

Before any later LIVE session: inspect restraint in both directions, body support,
wheel/cable clearance and an independent, reachable motor-power cutoff. Verify
loaded motor terminal voltage/current, inrush, adapter limiting, L298N logic
supply/grounds and heating for the 6V motors. Duty percentage is not a voltage or
current guarantee. Complete DRY and inspect sign, sensor acceptance and deadlines.
Use the preparation interval to secure the fixture. Hold upright/neutral; once
ACTIVE begins, only permit small supported displacements. A first powered trial
is bounded by the developer-selected 30-second timeout. Accept only correct polarity, the selected mode's pulse budget (one legacy Kick or two total integrated assists), ≥100ms
acknowledged-zero intervals before opposite output, zero missed deadlines and
terminal physical zero/disarm. Cut power for wrong direction, violent/chattering
motion, restraint loading, stall/heating/current excursion or absent shutdown.

Remaining blockers: this profile has not been tested on hardware; loaded reversal
current and wheel deceleration are unmeasured, no encoders exist, 100ms zero-command
pauses and startup friction can impair stabilization, and restraint alters the
plant dynamics. Existing successful response tests do not tune these gains or
validate free-standing balance. Do not remove restraint or extend the window.

## Host-only analysis (no board, no motor peripherals)

Run the replay from the project root:

```sh
CARGO_ENCODED_RUSTFLAGS='' cargo +stable run --offline --locked \
  --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu \
  --example balance-control-replay
```

This host target bypasses the ESP flashing runner. It uses an in-memory driver,
real PD/mapper/interlock/policy/settings and synthetic estimated pitch/rate input.
Scenarios cover oscillation, excessive overshoot, and rapid alternating requests.
It prints requested/applied commands, state, Kick/reversal data and final metrics.
Virtual LIVE policy exercises interlocks without accessing hardware. This is
signal replay, not a dynamics simulation: no inertia, torque, friction, sensor
noise, supply or tether model is inferred. The gyro/accelerometer calibration
and estimator remain real-hardware tasks and are not validated by synthetic poses.


## Serial-overhead fix

The original DRY loop printed a complete Behavior Debug dump, BIDIR PREVIEW and
GROUND summary/GAP every 200ms. esp-println's log backend formats and writes
synchronously, including FIFO waiting; there is no asynchronous log queue.
The measured 34,891us summary interval covered those combined writes, versus
about 2,105us prior sensor/control work. These recurring prints explain the
38,246us loop gaps. Per-line transport/formatting costs were not measured
separately. Updating last_log after printing only limited repetition frequency;
it did not remove the blocking cost or forgive the next loop's deadline.

The balance-control profile now bypasses the entire periodic-output block and
pre-ACTIVE transition printing. It records bounded samples/events only. DRY must
therefore end before logs can be viewed: 10 seconds after preparation, then a
permanent Stop, actual driver disarm and watchdog disable precede formatting or
serial writes. LIVE's configured ACTIVE timer, arming timeout, PD/PWM/Kick/interlock and
all deadlines are unchanged. Real misses still increment missed_deadlines and
max_loop_us, and LIVE still latches timing shutdown. DRY remains observation-only
and retains real timing violations in its final report; they are never reset or
excluded as "logging time". Other response profiles keep their existing logging.

Repeat hardware DRY to measure the actual result before considering LIVE. Host
checks verify bounds and fault policy but cannot predict USB/serial timing.

## Restart Assist integration (off by default)

See [integrated output arbitration](output_arbitration.md) and
[Restart Assist policy limits](restart_assist.md). The host replay can exercise
the same authoritative arbiter without hardware. The firmware integration is
behind its own feature/source gate; all five source gates are false. The
30-second ACTIVE window is unchanged. The default mode remains legacy one-Kick.

For the separately gated, default-disabled recurring budget experiment, see
[Rolling Restart Assist](rolling_restart_assist.md). It preserves the 30-second
ACTIVE window and all existing output/sensor interlocks.

The balance-control cutoff is explicitly 21 degrees (inclusive). Older standalone,
suspended-wheel and ground-response profiles retain their 15-degree default. The
recovery projected limit is derived as 21 minus the existing 3-degree margin,
yielding 18 degrees; its minimum-angle eligibility and all other parameters remain
unchanged. The linear projection is not a guarantee of future physical tilt.
