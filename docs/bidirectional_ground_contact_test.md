# Restrained bidirectional ground-contact response diagnostic

This is a one-second motor-response experiment, not autonomous balancing. Keep
independent mechanical support throughout. The tether must withstand motion in
BOTH directions without allowing a fall, tipping the table, trapping cables, or
pulling the robot into a person. Zero PWM is a coast command, not a brake or a
measurement of stopped wheels.

## Selection and gates

Binary/DRY feature: `balance-bidirectional-ground-test`.
LIVE feature: `balance-bidirectional-ground-live`.
All four source gates in `src/bin/balance_bidirectional_ground_test.rs` are false:
`PHYSICAL_RESTRAINT_VERIFIED`, `INDEPENDENT_POWER_CUTOFF_VERIFIED`,
`SUPPLY_LIMITS_VERIFIED`, `SENSOR_FAULT_REVIEWED`.
Only their conjunction AND this diagnostic's LIVE feature permits movement.
Neither `balance-live` nor either older response feature grants permission.
Existing ground gates have also been returned to false for this preparation.

Manual DRY command (flashes when YOU run it; not executed during development):

```sh
cargo run --release --locked --features balance-bidirectional-ground-test --bin balance-bidirectional-ground-test
```

Build-only LIVE verification, with all source gates still false:

```sh
cargo build --release --locked --features balance-bidirectional-ground-live --bin balance-bidirectional-ground-test
```

## Unchanged control and safety

- Learned two-pose BMI160 calibration, complementary filter, Kp=1, Kd=0.08.
- Limited PD ±15, deadband ≤2, ordinary proportional PWM 30–50%, no slew.
- Initial Kick trigger: |limited PD| >=3, also reused by the separate bounded
  balance-control profile. Other response profiles retain 4.
- Initial Kick candidate 70%, at most 150ms; zero PD/deadband can cancel sooner.
  This is NOT a forced pulse on arming. It requires a meaningful post-arm request.
- Duty ceiling 70% applies to both ordinary and Kick requests; no bypass.
- Forward=(-u,+u), A=LEFT, B=RIGHT. Direction names refer to requested motion.
- Preparation **30s**, disarmed. Calibration/stabilization durations unchanged.
- Fresh upright pose ±5° and ±5°/s for 500ms, then a NEW neutral sample (PD≤2).
  Motors remain disarmed in both waits. Combined arming timeout 10s.
- ACTIVE timer begins only on fresh neutral acceptance: **1 second maximum**.
- |pitch|≥15° permanently disarms. Fresh gyro/sample limit 40ms, accepted
  accelerometer correction limit 50ms during ACTIVE, loop deadline 30ms.
  NoGravityReference, invalid/nonfinite data, I2C/driver errors, stale data and
  missed deadlines retain existing fail-closed behavior. No automatic rearm.
- Fixed-memory flight recorder; ACTIVE event capture does not print or allocate.
  The 100ms watchdog is a backup, not an independent motor cutoff. Software and
  reset/watchdog behavior cannot guarantee inactive GPIOs before boot or during
  every hardware failure; external enable pull-downs and power cutoff matter.

## Reversal interlock

The mapper is unchanged, including its own 100ms reversal pause. A NEW final
command gate independently enforces **100,000µs** of acknowledged zero outputs:

1. A meaningful request for the opposite sign cancels any applied Kick and sends
   both motor commands to zero through the actual driver.
2. The dead-time timestamp is captured AFTER successful driver completion.
   Opposite polarity cannot pass until at least 100ms has elapsed. A nonblocking
   sensor/control loop continues all validity/deadline/timeout checks meanwhile.
3. A return to the original direction cancels the pending reversal. Another
   opposite request starts a new zero/dead-time sequence. Deadband requests stay
   exactly zero. Rapid alternating requests cannot shorten the wait.
4. After the initial Kick ends or is cancelled, later Kick previews are suppressed
   at this gate. Reversed motion uses ordinary mapped PWM (30–50%) only. It may
   fail to start under load. There is no automatic additional boost.
5. Any fault or ACTIVE timeout disarms the actual driver, clears the pending
   interlock, cancels Kick output and permanently ends the trial.

An accepted driver call is software evidence of the commanded output, not proof
of the electrical waveform, motor current, wheel movement, or zero speed. No
encoders are installed. Coast time does not guarantee safe mechanical reversal.

## Logs and acceptance

The shared recorder preserves requested vs `commanded` outputs, pitch/rate,
limited PD, mapped PWM, sensor/correction age, loop times and fault context.
`reversal=Telemetry { pending_direction, zero_elapsed_us, kicks, kick_applied }`
is the additional interlock telemetry. Pending=0 means no reversal wait; ±1
identifies the requested physical direction. During wait `commanded=(0,0)`.
Requested `kick` fields describe the unchanged mapper; the telemetry's kick_applied
and commanded outputs describe what passed the NEW gate. Suppressed Kick requests
may therefore still appear in the mapper trace. The BIDIR accepted_kicks and
pinned applied_kick_on/cancelled_or_expired records report accepted output.
First applied output and latest interlock entry/release are pinned separately. `last_energized` retains the last
nonzero command at shutdown. Kick count is bounded at one for this profile.

DRY summaries additionally show `BIDIR PREVIEW`, simulated output and explicit
`actual=(0,0)`. DRY intentionally bypasses LIVE readiness/ACTIVE and may preview
unsafe tilts; it cannot energize the motors. Inspect pending/dead-time and zero
intervals in DRY first. LIVE recorder events print AFTER permanent disarm: the
first ACTIVE event with nonzero `commanded` is the first recorded accepted motor
command; TERMINAL must show Disarmed and commanded=(0,0), with ActiveTimeout or
an explicit fault reason. Event buffer overwrites are counted; pinned terminal,
last sensor rejection and Kick transitions survive. There is no full waveform
capture and no wheel-speed feedback.

## Manual physical procedure (no LIVE authorization in this change)

1. With motor power disconnected, inspect bidirectional restraint, body support,
   clear wheel path, cable slack, and access to a separate motor-power switch.
   Keep a second person ready to cut power if possible. Do not rely on USB reset.
2. Verify actual supply/current limits under load, motor terminal voltage/current,
   L298N logic supply/common ground and thermal behavior. A PWM percentage does
   not establish safe voltage/current for the 6V motors. Check inrush/stall and
   adapter sag/current limiting; the earlier suspended test is insufficient.
3. Run DRY first; complete both poses and use the 30s preparation interval to
   secure the ground-contact fixture. Verify signs, sensor acceptance and zero
   missed deadlines. Inspect both sign changes and interlock zero intervals.
4. Only in a later explicitly reviewed session, verify each source commitment
   and select this diagnostic's own LIVE feature. Do not combine experiments.
   Hold upright and still for readiness, then neutral acceptance; only afterward
   allow a very small, supported fore/aft displacement. Never push toward cutoff.
5. Accept only a bounded trial with at most one applied Kick, correct wheel
   polarity, zero outputs before every polarity reversal and ≥100ms acknowledged
   dead-time. Require zero missed deadlines and a terminal actual zero/disarm
   within the one-second window (or earlier protective fault). Record complete
   logs and externally observed wheel/tether motion, voltage/current and timing.
6. Cut motor power immediately for wrong direction, restraint loading, unexpected
   acceleration, chatter, stalling, heating/current excursions, reset, or absent
   shutdown. A protective sensor/tilt fault is a safe abort, not tuning success.
   Do not extend the window, raise duties or release restraint based on this test.
