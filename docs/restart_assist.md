# Restart Assist policy — integrated mode disabled by default

The `motor_response_runtime/restart_assist.rs` state machine remains pure and
no_std-compatible. Config defaults disabled. It now has an opt-in firmware
integration through a single output arbiter, disabled by separate feature and
source review gates. See [integrated arbitration](output_arbitration.md).
The existing one-Kick authority is the default; integrated mode replaces that
pulse authority with the shared two-pulse budget and reuses its electrical
reversal guard. It does not add pulses downstream of a guard.

`--restart-assist` remains the isolated host policy replay. Prefer the existing
host example's `--integrated-assist` option to exercise the actual firmware
arbiter with an in-memory driver. Neither option accesses ESP peripherals.

## Proposed bounds (not measured safe settings)

| Parameter | Default candidate |
|---|---:|
| Enabled | false |
| Reference duty | 70%, from existing Kick configuration; never above driver ceiling |
| Maximum acknowledged assist duration | 150ms |
| Minimum cooldown | 1000ms after acknowledged assist end |
| Pulse budget | 2 accepted pulses TOTAL per ACTIVE session |
| Cumulative acknowledged assist-on budget | 300ms |
| Required acknowledged zero interval | 100ms |
| Fresh, stable nonzero candidate dwell | 20ms |

Two pulses permit investigating one restart after one initial assist, while
limiting repetition substantially. The 1s cooldown limits rapid retriggering and
allows observations between attempts. These bounds do not establish safe motor
voltage/current/temperature or torque. The assist budget does not limit ordinary
PWM stall current or total motor energy across the 30-second session. The model's two-pulse budget counts its
initial pulse: it does NOT mean an existing initial Kick PLUS two more assists.

To enforce duration bounds with variable control timing, the proposal ends early:
it reserves two 30ms intervals, one for the next decision and one for acknowledged
command completion. At a 100Hz, zero-latency host cadence a 150ms maximum pulse
therefore ends around 90ms. A valid near-deadline/IO replay ends at 148ms. The
30ms safety deadline itself is unchanged. Remaining cumulative time at or below
this 60ms headroom cannot start another pulse; BudgetExhausted can therefore
report unused but unspendable time. Time is charged from acknowledged assist
start until acknowledged replacement by zero or ordinary PWM, including IO time.
Actual overruns are not clipped away from telemetry. Missed deadlines and
untimely acknowledgements fault and permanently disarm the proposal policy.

This is a software scheduling bound under timely updates/acknowledgements, not a
physical cutoff guarantee. A hung processor/driver can retain an old command;
independent motor-power cutoff remains necessary. Shorter effective pulses may
fail to start the real motors. Do not extend them or raise duty based on replay.

## State flow

- **Idle:** ordinary mapping only; no wheel-speed inference.
- **StartupCandidate:** after at least 100ms of acknowledged zero output, a fresh
  meaningful demand (|limited PD|>=3, deadband<=2) must remain in one direction
  for 20ms. Proposed output remains zero during this confirmation.
- **AssistActive:** only an accepted, nonzero assisted-command acknowledgement
  increments the pulse count and starts pulse timing. A proposal alone is not
  counted as application. Zero demand, direction conflict, fault or time budget
  immediately cancels the assist proposal.
- **Cooldown:** no new assist for 1s after acknowledged end. Ordinary PWM may be
  proposed, subject to reversal interlock. A denied startup which already moved
  to ordinary PWM cannot turn into a delayed Kick when cooldown expires. Another
  assist requires another acknowledged-zero interval and fresh candidate.
- **BudgetExhausted:** no more assists in this session; ordinary commands still
  pass the reversal and safety checks. The budget never rearms on neutral.
- **Disarmed:** fault/stop/timeout is terminal; proposed PWM is zero forever.
  Recovery needs a deliberately created new session, not a new sample or a flag.

Opposite polarity is never proposed until at least 100ms after a successful
zero-output acknowledgement. Direction conflict during a pulse first proposes
zero and cancels that pulse. Candidate sign chatter restarts candidate dwell.
The interlock uses the existing DEAD_TIME_US constant; neither wait proves zero
physical wheel speed. No speed/current feedback or motor-movement detection is
implemented. No automatic high-duty response is inferred from buzzing or stall.

Callers must acknowledge only successful driver completion, with completion
clock, integer signed duty and assist classification. Ordinary PWM quantization
matches the existing signed-integer driver API. Bad/mismatched acknowledgements,
nonfinite input, nonmonotonic time, stale samples, control gaps >30ms, sensor
fault, |pitch|>=21deg, timeout and disarm latch zero proposals. The host adapter
actually stops its in-memory driver before recording fault completion. A future
hardware adapter would have to provide the same physical-stop ordering.

## Telemetry

A fixed-capacity recorder stores raw Trace values with no formatting, allocation,
waiting or IO. It reports request, eligibility and accepted application separately;
suppression reason; active/last pulse duration; cooldown remaining; pulse/time
budget remaining; total acknowledged on-time; requested assist PWM, proposed PWM
and acknowledged commanded PWM. First/last applied context and terminal context
are pinned independently of ring saturation; overwritten records are counted.

A pre-ack trace can correctly show a cancellation/zero proposal while the last
acknowledged command is still assisted. Only the next successful replacement
acknowledgement ends that applied interval. These are software command facts,
not measured wheel movement, actual current, or confirmed electrical waveforms.
The reason field can describe neutral, cooldown, candidate confirmation or pulse
completion as well as suppressed requests and faults.

## Run offline

From the project root (explicit host target bypasses espflash):

```sh
CARGO_ENCODED_RUSTFLAGS='' cargo +stable run --offline --locked \
  --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu \
  --example balance-control-replay -- --restart-assist
```

The replay explicitly enables the alternative in memory, drives repeated positive
and negative stop/start demands, exhausts the pulse budget, then permanently
stops. Formatting happens AFTER the replay. It is signal/policy replay, not a
motor/pendulum/thermal model. Tests also cover chatter, true acknowledgement
ordering, stale/fault data, cooldown, cumulative budget, tilt, deadline shutdown,
the current 30-second timeout and inability to rearm automatically.

## Interaction and remaining review blockers

Do NOT insert assist proposals after the output arbiter or enable legacy mapper
Kick generation in integrated mode. That would create a second authority.
The opt-in integration now counts the initial pulse in the same budget and
reuses the shared acknowledged-zero guard; all source gates remain false.
See [selection and integration constraints](output_arbitration.md).

Before any separate restrained hardware trial: review budget/acknowledgement
semantics, actual GPIO timing, watchdog behavior, loaded breakaway response,
reversal current, supply limits, motor heating and physical restraint in both
directions. Thirty seconds is a session timeout, not evidence that repeated
assists are safe. No free-standing balancing, self-righting or automatic stall
recovery is authorized.

An isolated optional [rolling-window policy](rolling_restart_assist.md) now shares
the output arbiter. The fixed lifetime two-pulse option is retained unchanged.
