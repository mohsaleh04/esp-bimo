# Proposed tethered ground-contact test — NOT free-standing balancing

Status: software prepared; **ground movement disabled**. Do not run ground LIVE
until the correction-loss cause, supply and restraint are reviewed. Nothing in
this preparation flashes or operates the robot.

## Evidence and unresolved fault

The supplied new run ends with `Fault(Tilt)`, not `CorrectionExpired`: it never
left `ArmedWaitingForNeutral`. PD remained >=4, so the neutral criterion
`abs(limited PD)<=2` did not pass. Pitch rose from about 4.5° to the 15° cutoff.
All displayed corrections were accepted and deadlines were met. This is the
expected stop path and is not evidence of accelerometer correction loss.

For the earlier `CorrectionExpired` at about 50.6ms, the policy's order of checks
shows that valid fresh gyro/estimator results continued but no accelerometer
correction was accepted within 50ms. No missed loop deadlines were reported.
We cannot recover the failing acceleration gate from the provided summary.
Possible gates remain unchanged: 0.85–1.15g magnitude, sideways <=0.35g and
pitch-plane >=0.5g, plus finite inputs. Motor/fixture vibration, translation or
sideways tilt are hypotheses, not established causes. Do not increase tolerance.

The actual current suspended profile is 30s, not the older documented 5s.
A 64-record, first-fill buffer fills after ~6.4s even at 10Hz; correction edge
records can fill it sooner. Thus 128 drops alone do not prove kick chatter.

## Overflow-resistant record and kick review

The main telemetry buffer remains bounded. A separate 32-event rolling recorder
retains simultaneous state/sensor/kick/command changes. Dedicated slots retain:
last correction-loss onset, last measured rejection with threshold, last kick-on
and kick-off, pre-stop context and the first terminal fault. A recent-event ring
overflow cannot erase these slots; after terminal disarm they freeze. Events
include sample/read time, estimated pitch/rate, correction age, PD/mapped output,
requested commands, accepted driver commands, last energized-command timestamp,
sensor/disarm reason and timing. Commands are not measured wheel movement.

Kick-start/end, requested reversals (including through zero), command changes,
zero transitions and event-overwrite counters are printed AFTER disarming. Active
logs never block the control loop. Power loss/reset can still lose this RAM-only
record. Not every old event is retained; overwrites are counted honestly.

Existing kick behavior is unchanged: trigger at limited-PD magnitude >=4,
deadband <=2, 300ms continuous zero dwell and global kick-start cooldown, 100ms
stable reversal pause, exact zero cancels a pulse. Holding a request does not
repeat kicks. A meaningful stable reversal can permit another kick after the
pause/cooldown. PD quantization plus the existing proportional floor can toggle
ordinary output between zero and about 31.5% for PD 2/3 without any kick. We did
not change gains, mapper, slew, kick or calibration to hide that behavior.

## Separate profile and explicit opt-ins

Binary: `balance-ground-contact-test`.
Preview feature: `balance-ground-contact-test`.
Motor permission feature: `balance-ground-contact-live`.

All four source flags in `src/bin/balance_ground_contact_test.rs` are FALSE:

- `PHYSICAL_RESTRAINT_VERIFIED`
- `INDEPENDENT_POWER_CUTOFF_VERIFIED`
- `SUPPLY_LIMITS_VERIFIED`
- `SENSOR_FAULT_REVIEWED`

All flags AND the ground-specific live feature are required. `balance-live` and
`balance-motor-response-live` cannot enable this profile. These flags are operator
commitments; there is no sensor that proves a tether/cutoff is present. Leave
them false if any prerequisite is unmet. Preview can log indefinitely with
outputs disarmed; it cannot energize motors.

The new profile reuses the SAME calibration/estimator/response policy and runtime:

| Parameter | Ground profile |
|---|---|
| Preparation after both calibration poses | 30s, motors disarmed |
| Readiness + fresh post-ready neutral wait | 10s bounded |
| ACTIVE motor-command window | **1s maximum**, then permanent disarm |
| Readiness | +/-5°, +/-5°/s for 500ms, then NEW neutral PD within +/-2 |
| Ordinary mapping / gains | unchanged, 30–50% and PD limit +/-15 |
| Kick | unchanged 70% / 150ms; maximum ONE kick per ground trial |
| Direction | A/LEFT negative, B/RIGHT positive for forward |
| Ground extra stops | any further kick onset or meaningful reversal |
| Sensor/loop protections | unchanged 50ms correction age, 40ms sample freshness, 30ms loop deadline |
| Tilt cutoff | unchanged magnitude >=15° |
| Watchdog | existing 100ms reset fallback, not a hardware motor cutoff |

There is no commanded translation target, self-righting, autonomous motion or
indefinite balancing mode. The short feedback response requires the body to
remain externally supported. The ground guard aborts instead of tuning away
oscillations. Its stop appears as `Fault(Motor)` with a separate explicit
`ground_stop=AdditionalKick/ReversalRequested`; it is a command veto, not a claim
of a broken motor driver. Every fault/timeout requires reset/recalibration.

The suspended profile remains 30s/15s preparation with its present live opt-in;
the added ground guard never changes that profile. Shared runtime extraction
avoids copying acquisition/calibration/safety code into another firmware.

## Proposed manual procedure, before any ground activation

1. **Diagnose suspended first.** Build the instrumented suspended test and inspect
   pinned rejection/onset, kick counters and terminal context. Reproduce the
   correction loss only with wheels suspended. Determine the measured gate and
   its correlation with motor commands, and inspect external sensor/fixture
   mounting if indicated. Resolve it without raising tolerance, or stop here.
   The new Tilt-only log does not satisfy this prerequisite.
2. **Verify supply limits.** A 9V/1A label is neither current-limit validation nor
   evidence that 70% pulses are safe for 6V motors under load. Verify actual
   voltage/current limits, motor/driver loading and pulse behavior with suitable
   instrumentation. Do not infer voltage from PWM duty. If unresolved, keep
   `SUPPLY_LIMITS_VERIFIED=false`; no ground LIVE.
3. **Restrain mechanically and arrange independent cutoff.** Use secure body
   support plus fore/aft restraint anchored to a stable fixture, limiting travel
   and preventing a fall/runaway in either direction. Restraint attaches to a
   structural part, not wiring. Keep wheels/cables/hands clear. The chassis must
   not depend on wheel control to stay upright. Arrange an immediately accessible
   upstream motor-power switch/cutoff; disconnecting USB or waiting for firmware
   is not an independent motor cutoff. Prefer accessible supply-side control;
   this plan does not require opening or rewiring the lower chassis. Rehearse
   with motor power disconnected; use a second operator if needed.
4. **Rehearse DRY.** With all source flags false, use the preview binary below.
   Follow the same two-pose calibration while motor power is cut and the body is
   supported. Use the 30s preparation interval to place both wheels on the test
   surface and secure the final fixture. The fixture/support must allow the
   calibration poses while disabled, then controlled small test tilts. If that
   cannot be done safely/reliably, do not proceed. Confirm upright neutral PD and
   physical forward/backward signs; do not bypass the neutral gate. Do not rush
   a countdown—leave power cut and reset if not prepared.
5. **Review before enabling.** Only after steps 1–4 pass, explicitly set the four
   flags true and select ONLY the ground live binary/feature. This change leaves
   all four false. Have the physical cutoff ready before connecting motor power;
   keep body support and restraint throughout startup/calibration/preparation.
   Complete fixture setup with motor power off, and only supply motor power once
   it is safe to start after the countdown. Never use the longer suspended
   firmware on the ground.
6. **One short supported response.** At the end of preparation hold the calibrated
   upright/low-rate pose until readiness and a NEW neutral sample pass. Neutral
   acceptance starts the 1s window at zero output. Gently request one direction
   with a small controlled tilt (roughly 3–5°, stay well below 15°); observe wheel
   traction/fixture motion. An early/late gesture may produce no movement: do
   not bypass safety gates or add a startup kick to force it. Do not reverse
   during that trial. Cut motor power immediately for unexpected movement,
   restraint shift, stall/buzzing, sagging supply, or abnormal heating.
7. **End and inspect.** Wait for permanent disarm, cut motor power, then inspect
   pinned fault/command context and the event ring. A stall/movement threshold
   is not automatically detected. Do not progress to free-standing balancing or
   increase duty/window based on this one trial. Any next direction or retry
   requires a new reviewed, supported reset/calibration sequence.

Build only (safe, no flashing):

```sh
cargo build --release --locked --features balance-ground-contact-test --bin balance-ground-contact-test
cargo build --release --locked --features balance-ground-contact-live --bin balance-ground-contact-test
```

Manual preview (flashes, so operator runs it only when ready; motors stay disabled):

```sh
cargo run --release --locked --features balance-ground-contact-test --bin balance-ground-contact-test
```

Future LIVE command, **NOT authorized for execution in this task** and blocked
until source flags are explicitly configured after review:

```sh
cargo run --release --locked --features balance-ground-contact-live --bin balance-ground-contact-test
```

Software cannot guarantee reset-time GPIO levels or bound a CPU hang. Existing
blocking I2C (20ms timeout) and short driver delays remain. External enable
pull-downs/gating and the independent supply cutoff are still required for
hardware-level safety. Finite-but-wrong sensor data, tether dynamics, loaded
motor current and mechanical slip remain unvalidated risks.

## Ground safety audit and 30.222ms investigation (2026-11-14)

Evidence supplied from ground DRY:

- `live=false`, `state=DryRun`, and motor outputs disabled throughout.
- Sample dt normally 8.9–10.3ms, work maximum 3018us, loop maximum 30222us.
- The maximum increased when the verbose invalid-estimate lines were printing.
- Sideways acceleration exceeded 3.4320998m/s² (e.g. 5.489919); the filter spent
  over 500ms without accelerometer correction in DRY, reset, then correctly
  reported `NoGravityReference` until that gate passed again.
- Large intentional tilts, including >60°, were mathematical DRY previews.

Source timing confirms the ordinary summary write occurred AFTER the final
`work_us` checkpoint. Therefore the roughly 27ms difference between maximum
body work and the worst interval falls outside measured acquisition/processing
work. Blocking logging is the leading hypothesis, supported by the longer
error lines; sleep/wake scheduling, interrupts and transport backpressure cannot
be separated from this old log. It does NOT prove a 30ms I2C operation or a LIVE
control-loop failure. No threshold was increased.

Ground-only changes from this audit:

1. Ground vetoes are now latched inside `GroundGuard`. `ground::drive()` is the
   final tested output gate: it checks policy/freshness, applies the guard, and
   both latches errors and calls the physical driver's `disarm()`. Previously
   the correct runtime handled these steps separately, so safety depended on
   the caller remembering to latch a returned rejection. No demonstrated LIVE
   cutoff bypass was found; this closes that composition weakness.
2. Every ground deadline checkpoint disarms the driver at detection, before
   subsequent diagnostic bookkeeping. The strict boundary is unchanged:
   30000us is allowed, 30001us fails. Start-to-start gaps AND work overruns count.
3. Ground summaries are shorter; verbose suspended-wheel output is unchanged.
   Ground polling, motor-driver calls, and summary writes have individual
   wall-clock measurements. `GROUND GAP` pins the largest interval together
   with that SAME preceding loop's work, summary, poll and drive times, plus
   the unaccounted remainder (yield/scheduling/unmeasured overhead). Disarm
   prints maxima too. These measure wall time, not isolated CPU execution or
   proof of a hardware operation's cause. The summary write remains outside
   ACTIVE, and all ACTIVE capture is fixed-memory with no formatting/allocation.

The shared state machine and suspended profile are unchanged. For LIVE:

- Before readiness, missing gravity/invalid samples prevent arming; fresh valid
  data must establish the full upright dwell. The overall 10s wait remains
  bounded. Calibration and preparation have no motor-command path.
- `ArmedWaitingForNeutral` still requires a NEW corrected neutral sample with
  current readiness. Sensor faults after entering this supervised state latch.
- `Active` begins only at neutral acceptance, initially outputting zero, with
  exactly a one-second maximum response window for ground.
- >=15° magnitude, nonfinite estimates, sensor/I2C failures and no gravity
  reference permanently end the supervised trial. Fresh finite gyro-only
  estimates are allowed only in ACTIVE up to the existing absolute 50ms age;
  50001us fails, and >40000us since a fresh estimate fails independently.
- A second kick or meaningful direction reversal permanently ends ground
  output; zero requests and an old preview cannot recover a disarmed trial.
- DRY intentionally never enters the LIVE readiness/ACTIVE path. It may compute
  PWM above 15° and observe the estimator's 500ms fallback, while the final gate
  always keeps physical outputs disabled. `ready=true` in DRY is informative,
  not permission to activate.

Automated coverage now drives an instrumented fake `DualMotor` through the
SAME ground gate and asserts disarm calls, zero outputs and no further command
calls after faults, including an attempted external re-arm. It covers every
sensor-fault category, malformed estimates/dt, both tilt signs, exact correction/
freshness/deadline boundaries, clock rollback, arming/ACTIVE timeouts, raw driver
errors, output bounds, reversal, second kick and default opt-ins. A synthetic
30222us test validates attribution arithmetic; it is not a new hardware timing
measurement. Inspection of `L298n::disarm` confirms it detaches PWM, zeros compare
values and drives all six GPIOs LOW. Host tests do not measure actual GPIO
voltages, physical wheel stopping distance or power cutoff behavior.

Readiness verdict: software fault paths are audited/tested, but there is NO
unconditional hardware GO. All four flags remain false. Repeat the hardened
DRY test upright with modest single-plane tilts and inspect timing attribution;
resolve any unexplained gaps and review the earlier powered correction-loss
fault. Complete independent fixture, cutoff and supply/current verification.
Only then deliberately opt into the existing one-second procedure. No claim
of free-standing stability, loaded motor safety, or immunity to CPU hangs is
made, and no LIVE hardware run is part of this audit.

Final strict-ground fix: an unsafe pitch while `WaitingForReady` previously only
inhibited arming; it did not permanently terminate the run. Ground LIVE now
uses `ground::sample()` to latch >=15° or nonfinite estimates immediately after
preparation, including before readiness, and disarm the driver at that entry
point. DRY remains a mathematical preview, and suspended startup is unchanged.
The shared response policy only gains an explicit `abort()` entry point for
this stricter ground guard; its existing suspended behavior is unchanged.
