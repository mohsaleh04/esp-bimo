# Existing balance-control trajectory correction

The existing `balance-control-recovery-assist` feature now qualifies correction
by body trajectory. There is no new binary, mode or LIVE authorization. Without
that feature, legacy/fixed/ordinary Rolling Assist behavior is unchanged. Existing
LIVE/source verification gates remain in force. No source authorization is enabled
by this change. This is not stall detection: wheel speed/current are not measured.

## Qualification on fresh samples

After an initial acknowledged assist, sustained same-direction ordinary correction
can qualify through either path:

| Path | Absolute pitch | Outward rate | Fresh evidence | Net worsening |
|---|---:|---:|---:|---:|
| Fast | >=6 degrees | >=3 deg/s | >=100 ms | >=0.5 degrees |
| Slow | >=8 degrees | nonnegative | >=500 ms | >=0.2 degrees |

Pitch and limited-PD direction must agree and demand must remain outside +/-2.
Evidence must be no more than 20 ms old/apart; cached polls cannot build a trend.
Existing cooldown and shared rolling credit must be available. Losing resources
resets qualification; credit expiry alone cannot request a pulse. These are
experimental thresholds, not tuned values or proof of an ineffective motor.

First try a 200 ms ordinary-drive probe: base mapped magnitude +5 percentage
points, capped at the unchanged 50% ordinary maximum. The actual increase is compared at the driver's 1% resolution after capping:
45→50 adds 5 points, 48→50 adds 2, and 50→50 adds zero. If fewer than one integer
point can be added, skip the probe immediately (also if the cap is reached during
an active probe) and enter the existing acknowledged-zero/qualification sequence.
Its 100 ms zero wait and 20 ms dwell remain mandatory. ProbeCapped identifies the
skip in deferred telemetry. No ledger, cooldown or eligibility check is bypassed. This is ordinary drive, not an assist pulse, so it does
not consume assist-ledger time; its magnitude and duration are separately bounded.
It may help a moving motor, but no moving/stopped state is inferred. Improvement
cancels the probe rather than unconditionally escalating to startup assistance.

If worsening persists, command zero through the same arbiter, wait 100 ms after
acknowledged zero, then the original 20 ms stable-demand dwell before a Kick.
There is no fabricated neutral demand, alternate output path, or arming bypass.

The projected-pitch limit remains **21 - 3 = 18 degrees**. The linear screen
reserves remaining probe time + remaining zero wait + dwell + configured 150 ms
maximum pulse + the original conservative 60 ms projection reserve. Before a useful probe
this horizon is 530 ms; when skipped, its 200 ms is excluded and the horizon is
330 ms. Once zero-wait begins, a later duty decrease does not restart the probe. During a pulse it uses the 60 ms projection reserve.
Margin loss cancels the episode; ordinary control continues unless a real fault
requires disarm. Constant-rate projection cannot bound future acceleration.

## Pulse duration and early termination

The old `<3 deg/s` test labelled slowing outward motion as improvement and could
cancel a recovery pulse even though tilt was still worsening. A positive outward
rate below 3 now remains eligible through the slow path and does not itself cancel
an applied pulse. A claimed 28 ms event cannot be attributed exactly without its
transition records: neutral demand, reversal, faults and margin loss can also end
pulses early and remain valid cancellation causes.

For the recovery-enabled control path, both initial and subsequent assists target
**100 ms from accepted pulse start**, preserving the **150 ms maximum**. Timing
reserve is now 30 ms for the next control decision plus a **checked 20 ms update/
driver acknowledgement limit**. Late command completion reports CommandDeadline and permanently disarms; the
30 ms loop deadline is unchanged. At an ideal 10 ms cadence, the target ends at
100 ms. A deterministic near-boundary replay with 30 ms intervals and 20 ms IO
ends at 149 ms, below the 150 ms ceiling. CPU hangs/violated timing bounds still
require independent hardware power cutoff; software cannot guarantee boot/reset
GPIO safety. The normal Rolling/fixed paths retain their previous 60 ms reserve.

Neutral demand, direction conflict, invalid/stale data, margin loss and faults can
terminate sooner. Actual inward angular velocity cancels rather than being forced
to complete a minimum pulse. To tolerate small angle noise, measured improvement
of >=0.1 degree from the episode's peak also cancels. Thus 100 ms is a target,
not a minimum on-time or guaranteed physical startup. It is not a 100% duty boost.
Forward remains 70%, backward 65%; shared limits remain two pulses/300 ms per
1.5-second window and 300 ms cooldown. No budget is replenished by this policy.

## Reducing drive and requalifying

Immediately after a pulse, return to the ordinary mapped command. While the PD
direction still points into the lean but angular velocity points toward upright,
scale ordinary PWM by `clamp(1 + outward_rate / 10, 0, 1)`. For example, -5 deg/s
halves duty; -10 deg/s commands zero. Neutral and reversal/fault guards still take
priority. Base PD gains/mapping/minimum are unchanged; final applied duty may fall
below 30% during inward motion to avoid overshoot. This taper may also stop motors;
its physical effect needs validation. No scale reduction is applied to an actual
opposite-direction correction after the reversal interlock.

After completion/cancellation, enter AwaitImprovement. Held worsening, neutral/sign
chatter, cooldown expiry and budget recovery do not rearm it. Requalification needs
fresh continuous observations for >=200 ms, showing either >=0.5 degree reduction
while moving toward upright, or a stable upright pose within the existing +/-5deg
and +/-5deg/s readiness bounds. Then return to Watching; a NEW fast/slow worsening
trend must qualify again. Cooldown and ledger entries are retained. This observes
body improvement, not proof that a motor caused it; restraint/operator forces can
also create that trajectory. Faults instead enter permanent Spent/disarmed state.

## Telemetry and validation

Existing fixed-memory flight/ARBITER records show limited PD, mapped/requested/approved/
commanded duty, Kick state, ledger and recovery phase/reason. DriveProbe identifies
the bounded ordinary increase, AwaitImprovement the rearm wait, Rearmed observed
improvement, TrajectoryImproving taper/early termination, and PulseTarget the normal
100 ms end. No serial output, allocation or blocking delay is added to powered
control. Logs print after permanent shutdown, with existing overflow accounting.

Tests cover slow/fast deterioration in both signs, ordinary probe bounds, inward
motion taper, a rate drop below 3 without premature cancellation, improvement then
new deterioration, no periodic retries, acknowledgement reserve, pulse jitter,
shared ledger and the unchanged cutoff/fault/timeout/zero-output paths. These are
synthetic software tests, not a physical dynamics prediction.

The +/-21deg permanent fault, 18deg recovery projection screen, 30-second ACTIVE
window, 100 ms reversal neutral time, sensor validity/correction age, watchdog,
motor polarity, hardware and all LIVE gates remain unchanged. Before any separately
authorized physically restrained trial, review deferred logs and verify motor supply/
current limits and independent cutoff. Probes and pulses may accelerate moving
wheels or remove needed torque during zero wait. No autonomous balancing is proven.

## Optional same-direction zero-wait optimization

`balance-control-short-recovery-zero` is a separate **default-off** opt-in that
implies the existing recovery feature. It grants no LIVE authorization. Without
it, the 100 ms recovery zero interval described above is unchanged.

The historical generic startup prerequisite reused `DEAD_TIME_US=100000` for a
stopped-command interval before all assists. It separated startups and required
fresh demand, but never established wheel stoppage. The independent ZeroInterlock
uses the same numeric duration for a different purpose: zero acknowledgement before
opposite electrical polarity. That reversal guard remains untouched.

Qualified recovery already requires sustained valid trajectory evidence, available
300 ms cooldown and shared 2-pulse/300 ms per 1.5-second budget, an improvement-gated
new episode, and a fresh 20 ms stable-demand candidate. For SAME DIRECTION relative
to the most recent acknowledged nonzero robot-motion command only, the opt-in
removes the additional 100 ms coast. A zero command must STILL be acknowledged,
then a fresh candidate must STILL persist for 20 ms. At a 10 ms sample cadence the
observed zero-to-pulse interval is 30 ms instead of 120 ms because candidate timing
starts on the first fresh post-acknowledgement sample. It is never a zero-free
transition or a shortcut to initial arming/startup.

Changing direction cancels recovery and the original reversal wait remains >=100
ms after acknowledgement, followed by the normal demand qualification as applicable.
Rapid changes cannot qualify a pulse every update. Neutral, improvement, stale or
invalid evidence, budget/cooldown loss and faults retain existing cancellation and
permanent-disarm behavior. Budgets, dwell, the 100 ms target/150 ms maximum pulse,
70/65% duties, 21-degree fault, 18-degree margin, and 30-second session remain intact.

Projection uses the actual recovery zero-wait requirement: the optional same-
direction path subtracts 100 ms from the former horizon (530→430 ms with a useful
probe, 330→230 ms without one); all other reserves remain. Deferred recovery
telemetry includes `zero_wait_us` (0 or 100000), describing this additional recovery
wait, NOT the actual observed zero duration or generic initial-start requirement.

The three existing protection mechanisms are not interchangeable: cooldown spaces
pulses, the ledger caps count/time, and the reversal guard protects polarity changes.
They make the generic *same-polarity recovery* coast unnecessary as an additional
software eligibility barrier when all the other qualification rules remain. This
is not a physical safety proof: rotor speed, current and externally forced reverse
rotation remain unknown. Independent restraint and power cutoff remain mandatory;
the new opt-in needs separate hardware validation before routine use.

Build only, without authorizing any outputs:

```sh
cargo build --release --locked --bin balance-control-test --features balance-control-short-recovery-zero
```

LIVE-capable compilation only (does not flash/run):

```sh
cargo build --release --locked --bin balance-control-test --features balance-control-live,balance-control-short-recovery-zero
```
