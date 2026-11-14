# Integrated Restart Assist output arbitration

Only `balance-control-test` can select the new mode. It is DISABLED by default.
The 30,000,000us ACTIVE window, gains, ordinary mapping/deadband, motor polarity,
70% ceiling, ±15° cutoff, 30ms deadline and sensor validity policy are unchanged.
No hardware was flashed or operated during implementation.

## One selected authority

`output::MotorOutput` is an enum: Legacy OR Restart, never both.

- **Legacy (default):** existing one-Kick behavior; validated response profiles
  always select this mode. Their mapper/Kick policy remains unchanged.
- **Restart (opt-in):** mapper Kick generation is disabled in the Response
  configuration. It supplies ordinary mapped PWM only. Restart Assist is the
  sole pulse authority for BOTH initial startup and later restarts.

The new mode's path is:

1. Response readiness, new neutral acceptance and ACTIVE authorization.
2. Fresh valid orientation -> existing PD and ordinary proportional PWM mapping.
3. Restart Assist proposes ordinary, zero, or assisted output under one budget.
4. Shared `ZeroInterlock` approves or replaces the proposal with zero. This is
   the same extracted command guard used by the legacy bidirectional driver.
5. Current deadline, polarity, bounds and existing `policy::drive` safety gate.
6. Actual `DualMotor` driver command; only successful completion is acknowledged.
7. Pulse count/time accounting and telemetry update from that acknowledgement.

No assist is added after step 4 or after the final output gate. No legacy initial
Kick is exempted from accounting. An accidental mapper Kick in Restart mode is
rejected as an invalid input and disarms rather than entering a second budget.

The initial known zero state is acknowledged after an actual driver disarm,
before readiness waiting. It can qualify the initial zero interval while motors
stay disabled. The neutral acceptance sample remains zero; a later meaningful,
fresh stable demand is still required. Driver ownership of all GPIOs is retained.

## Limits and reversal

The reviewed candidates remain 70% maximum duty, 150ms maximum individual pulse,
1s cooldown from acknowledged pulse end, 2 accepted pulses total, 300ms cumulative
assist-on time, ≥100ms acknowledged zero and ≥20ms fresh candidate dwell.
The initial pulse consumes pulse one; one eligible restart can consume pulse
two. Budget exhaustion permits bounded ordinary PWM but never another assist.

The legacy reversal guard was factored into `ZeroInterlock`, independent of any
pulse budget. Both modes call it. A reversal request requires an actual zero
command and ≥100ms from its completion. If already holding zero, the guard still
forces a confirming zero write when a new pending reversal has no acknowledgement.
Candidate/cooldown checks can extend the zero interval; they cannot shorten it.
Zero command and dead-time are NOT wheel-speed measurements or mechanical brakes.

Control polls with no new sensor sample still service assist expiration using
cached data only while the existing sample/correction freshness limits permit.
They do not renew sample timestamps or create fresh startup requests. A fresh
sensor sample keeps the existing driver-write cadence; unchanged empty polls
avoid extra driver IO. This preserves existing PWM setup and avoids resetting a
reversal timer with fabricated acknowledgements. Faults/timeout/disarm stop the
actual driver before bookkeeping, cancel assists, and permanently latch shutdown.

The existing 60ms scheduling/IO reserve remains. At ideal 100Hz zero-latency
replay, nominal 150ms-max pulses end at about 90ms. Measured command completion
is charged to budgets; these are not voltage/current/physical pulse measurements.
A hang cannot be made safe by software alone; watchdog/reset behavior is backup,
and independent motor-power cutoff remains mandatory.

## Gates — all source gates false

In `src/bin/balance_control_test.rs`:

- `PHYSICAL_RESTRAINT_VERIFIED = false`
- `INDEPENDENT_POWER_CUTOFF_VERIFIED = false`
- `SUPPLY_LIMITS_VERIFIED = false`
- `SENSOR_FAULT_REVIEWED = false`
- **`RESTART_ASSIST_REVIEWED = false`** (new separate policy review gate)

Ordinary LIVE requires its original four flags AND `balance-control-live`.
Restart selection additionally requires BOTH `balance-control-restart-assist`
and `RESTART_ASSIST_REVIEWED`. Neither `balance-live` nor other response features
can select this policy. Feature selection alone never changes the false source
gates. Default mode remains legacy. The pre-calibration OUTPUT AUTHORITY log
explicitly identifies the selected mode and whether mapper Kick is enabled.

Firmware DRY stays completely motor-disabled and does not spend an assist budget
or simulate driver acknowledgements. With Restart selected its ordinary mapper
preview has no independent Kick. Use the host integrated replay to review assist
arbitration. No source gates need enabling to build or run that host replay.

Build checks, without flashing (source gates remain false):

```sh
cargo build --release --locked --features balance-control-test,balance-control-restart-assist --bin balance-control-test
cargo build --release --locked --features balance-control-live,balance-control-restart-assist --bin balance-control-test
```

Host-only integrated replay (bypasses ESP runner; no peripheral access):

```sh
CARGO_ENCODED_RUSTFLAGS='' cargo +stable run --offline --locked \
  --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu \
  --example balance-control-replay -- --integrated-assist
```

## Accurate, fixed-memory telemetry

`requested_pwm` is the assist/ordinary demand; `proposed_pwm` is the policy's
proposal. `approved_pwm` is None before final arbitration or Some(value) after
approval, including Some(0) when the shared reversal gate denies a pulse.
`commanded_pwm` and `applied` change only with actual driver acknowledgements.
Approved floats may be quantized to integer percent by the unchanged driver API.
A proposal/approval can therefore exist with commanded zero, e.g. driver rejection.
Approval is time-stamped permission, not application: the final deadline and
freshness rechecks can revoke it before IO, and driver rejection still disarms.
The standard Snapshot also records requested/commanded A/B, PD and mapped PWM.

In integrated mode, Snapshot Kick edges and accepted_kicks count accepted assists;
there are no mapper Kick requests. Legacy logs retain their old request-vs-command
semantics. The full assist Trace is included in periodic snapshots, and a bounded
64-slot change recorder retains proposal/approval/ack transitions. Stable holds
are coalesced; first/last applied pulse and terminal context are pinned. Overwrites
are explicit. Recording is fixed-memory and does no formatting, waiting or IO.
All serial formatting/printing remains after permanent physical disarm.

## Remaining review and hardware work

The same production arbiter runs in offline integration tests with an in-memory
DualMotor. Tests cover shared initial/restart count, third-pulse suppression,
post-IO reversal timing, empty polls, cutoff, correction age, fault/driver errors,
late approvals, 30-second expiry and no rearming. Legacy regression tests retain
their validated behavior. Offline success is not physical balance validation.

Before a separately authorized restrained run, review actual command/ack timing,
loaded startup torque, reversal current, supply/thermal limits and a reachable
independent cutoff. The earlier 150ms candidate may start differently when the
conservative reserve ends it near 90ms. No wheel-speed/current feedback exists.
Ordinary PWM may still stall after assist budget exhaustion; the assist budget
is not a limit on all motor energy over 30 seconds. Keep physical restraint in
both directions; do not infer autonomous stability or remove any safety gate.

The selectable [rolling budget](rolling_restart_assist.md) uses this same arbiter
and driver acknowledgement path. It has a separate default-off LIVE review gate;
fixed lifetime budgeting remains available.
