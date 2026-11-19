# Isolated TB6612 diagnostic: logic-only and explicitly gated NO-LOAD

This document applies only to **`tb6612-diagnostic`** (hyphen, singular).
The separate [`tb6612_diagnostics` direction tool](tb6612_diagnostics.md)
requires a compile-time stage choice and fresh console authorization for a pulse.
Do not use those paths for this procedure. No existing balancing code is changed.

## Source inspection

The normal Board owns STBY and the six TB6612 PWM/direction GPIOs.
`src/motors/tb6612.rs` routes PWM using MCPWM0. This standalone
diagnostic owns the GPIOs independently. Both paths use `src/motor_gpio.rs`.
The intended wheel mapping is A=LEFT and B=RIGHT. GPIO15/16 still need
continuity checks and physical direction remains unverified.

| Driver input | ESP32-S3 GPIO | Commanded state |
| --- | --- | --- |
| STBY | 40 | LOW, initialized first |
| PWMA (A = LEFT) | 17 | LOW |
| PWMB (B = RIGHT) | 5 | LOW |
| AIN1 / AIN2 | 15 / 16 | LOW / LOW |
| BIN1 / BIN2 | 7 / 6 | LOW / LOW |

These are source-level wiring expectations, not proof of physical wiring.
`src/bin/main.rs` contains no `generator parameters:` comment in this checkout.
Cargo selects esp-hal ~1.2.2 with esp32s3/unstable/log-04, Xtensa ESP32-S3,
and the `esp` toolchain. The default features are empty. The generated config
includes `esp-config.toml`, which is absent here; no config defaults were changed.
Existing ADC use in `hardware_diagnostics.rs` is GPIO2 microphone / GPIO1 IR,
not motor supply or current sensing. No encoder/rail sensing is used here.

## Connected-motor request: `tb6612-diagnostic-loaded` (BLOCKED)

Current hardware reported by the operator: ESP32-S3-N16R8, VM 5 V / 2 A
adapter, VCC 3.3 V, common GND verified, STBY GPIO40 with an external 10 kohm
pull-down, both 6 V TT motors connected (~3.5 ohm each), wheels suspended.
These are operator-provided facts, not firmware measurements. Earlier 9 V / 1 A
notes below are historical and do not describe the current loaded setup.

**No motor-connected activation has been implemented or authorized as safe.**
Requirement 10 permits withholding activation if it cannot be reasonably justified.
This feature boots LOW, reports the current-limit concern, waits 5 seconds with
STBY LOW, marks A forward/reverse and B forward/reverse SKIPPED at commanded
PWM=0%, reasserts LOW and idles permanently until reset. It does not determine
whether either channel can drive a motor; that objective remains unverified.
No arbitrary duty/pulse threshold is offered as a substitute for current limiting.

Build only:

```sh
source ~/export-esp.sh
cargo build --release --locked --bin tb6612-diagnostic --features tb6612-diagnostic-loaded
```

Optional explicit flash/monitor command, **not run by the agent** (this builds
and flashes the BLOCKED, non-driving mode):

```sh
source ~/export-esp.sh
cargo run --release --locked --bin tb6612-diagnostic --features tb6612-diagnostic-loaded
```

Do not select the legacy `tb6612_diagnostics` binary with connected motors.
LOADED + NO-LOAD together intentionally fail compilation. Loaded mode also
suppresses the old `tb6612-diagnostic-motor-test` branch; that feature or legacy
live/both flags cannot unlock this loaded mode. Existing NO-LOAD operation is
unchanged when loaded is absent and still requires both motors disconnected.

### Why movement is blocked

The simple zero-back-EMF estimate is 5 / 3.5 = approximately 1.43 A per motor.
This ignores inductance, driver/wiring voltage drop, winding temperature,
measurement uncertainty and supply transients; it is neither a measured current
nor a guaranteed worst-case bound. Free suspension reduces mechanical load but
does not remove startup/stall or electrical fault risk. A 2 A adapter rating
alone says nothing about current-limit setpoint, response time or output-capacitor
energy, and cannot be treated as per-channel motor protection.

The Toshiba datasheet linked below specifies a 1.2 A per-channel average absolute
maximum, and peak ratings under particular pulse conditions (2 A, 20 ms pulses,
duty <=20%; 3.2 A, 10 ms single pulse). Those are not unconditional safe operating
settings. The 1.43 A estimate exceeds the average absolute maximum; it does not
by itself rule out every possible brief pulse, but available data is insufficient
to justify one that reliably diagnoses these motors. PWM off-time can circulate
winding current; duty is not dependable peak-current limiting. Thermal shutdown
and undervoltage protection are not a specified programmable current limiter.

Before implementing movement, establish a suitable current-limit setpoint and
transient behavior against the motor and actual breakout's electrical/thermal
limits, including fault/stall handling and safe reversal/coast time. A documented,
verified current-limited supply or hardware limiter is needed; this can be on VM
without changing motor GPIO wiring. A physical cutoff remains necessary. No
specific current setting is invented here. Supply overshoot/regeneration,
decoupling, motor noise/brownout and damaged/shorted hardware remain unverified.
The reported STBY pull-down helps floating/reset states but does not override a
GPIO driven HIGH or guarantee shutdown of a stalled CPU. Panic handling remains
best effort, with the limitations described below.

### Pin ownership and PWM review

The existing binary constructs GPIO40, 16, 17, 5, 6, 7 and 15 once and retains
all seven in static storage. It never calls Board::init or the shared Tb6612 driver,
so their same-numbered pins are not claimed concurrently. Other diagnostics are
separate binaries. No sensor, buzzer, servo, PWM or balancing task is started in
the loaded build. No dependency, target, PID or wiring setting was changed.

Loaded mode leaves GPIO17/5 LOW and configures neither LEDC nor MCPWM. The
existing no-load implementation uses static HIGH as 100% PWM; it is excluded from
loaded-only compilation. The shared Tb6612 driver configures MCPWM with a 40 MHz
clock, period 999 and requested 35 kHz while its comments/logs say 10 kHz. This
existing mismatch was observed, not changed or reused as a basis for safe PWM.

## Build stages (no automatic flashing)

```sh
source ~/export-esp.sh
cargo build --release --locked --bin tb6612-diagnostic --features tb6612-diagnostic
```

This selects LOGIC-ONLY. It never configures LEDC/MCPWM, drives an input HIGH,
starts the RTOS, allocates a heap, or invokes a motor/balance controller.
All seven pins are initialized immediately after HAL init, before logging.
HAL's Output constructor sets the initial latch before enabling output.

### NO-LOAD output-voltage sequence

**Before powering the board, physically disconnect BOTH motors from AO1/AO2
and BO1/BO2** (often printed A01/A02 and B01/B02). Wheels lifted off the ground
is not sufficient. Firmware cannot detect or enforce physical disconnection.
The feature below is the operator's explicit acknowledgement of this condition.
Use only an external high-impedance DC voltmeter on these outputs; never current,
resistance or continuity mode on powered terminals. Do not short adjacent pins.

```sh
source ~/export-esp.sh
cargo build --release --locked --bin tb6612-diagnostic --features tb6612-diagnostic-no-load
```

This is build-only; no flash command is executed. Do not use the similarly named
legacy binary. Ordinary `tb6612-diagnostic` remains logic-only. Legacy live/both
features do not enable this sequence. `tb6612-diagnostic-motor-test` remains a
blocked loaded-test request; it does not unlock loaded motor operation. If combined
with no-load, only the no-load sequence below runs; motors must stay disconnected.

At boot all seven inputs are LOW, before logging. After the boot checklist,
a 5-second grace period holds them LOW. Four phases then run once:

| Phase | Active channel | Direction inputs | PWM | External meter expectation |
| --- | --- | --- | --- | --- |
| A forward, 5 s | A only | AIN1 LOW, AIN2 HIGH | PWMA HIGH (100%) | red AO1, black AO2: approximately -VM |
| OFF, 3 s | neither | all LOW | both LOW | outputs high impedance, floating reading possible |
| A reverse, 5 s | A only | AIN1 HIGH, AIN2 LOW | PWMA HIGH (100%) | red AO1, black AO2: approximately +VM |
| OFF, 3 s | neither | all LOW | both LOW | high impedance |
| B forward, 5 s | B only | BIN1 HIGH, BIN2 LOW | PWMB HIGH (100%) | red BO1, black BO2: approximately +VM |
| OFF, 3 s | neither | all LOW | both LOW | high impedance |
| B reverse, 5 s | B only | BIN1 LOW, BIN2 HIGH | PWMB HIGH (100%) | red BO1, black BO2: approximately -VM |
| OFF, 3 s, then permanent idle | neither | all LOW | both LOW | high impedance |

STBY is HIGH only during active phases, LOW during every OFF interval and at
completion. The inactive channel always has PWM and both direction inputs LOW.
Forward/reverse are logical wheel commands. The shared driver mapping inverts A only. For 100% duty the PWM input is
constant HIGH; no PWM timer is required. Logs announce the next active phase
while STBY is LOW; activation follows the log immediately. Shutdown precedes OFF
logging, so console backpressure cannot prolong an active phase. Timing is software
commanded (5 s / 3 s), not an external measurement or independent safety deadline.
The nominal sequence after the checklist is 37 s plus logging/software overhead.
Reset or power cycling re-arms the boot sequence; idle does not repeat it.

No ADC voltage/current/RPM measurement is made. The expected +/-VM assumes correct
wiring, powered healthy driver and compatible logic levels; meter results are for
the operator to record. A floating disabled output need not measure zero.

## External electrical checks

Start with VM and motors disconnected. Confirm actual wiring and common ground.
Keep a physical motor-power cutoff accessible. Power logic only, then follow
the numbered boot messages using a meter in DC-voltage mode:

1. Black probe on driver GND, red on VCC: expect the intended 3.3 V logic supply.
   Never put the 9 V motor adapter on VCC or ESP32 GPIO.
2. Black on driver GND, red on STBY: expect near 0 V. Check PWM and direction
   inputs likewise. These are expected observations, not firmware measurements.
3. Motors still disconnected: verify VM-GND polarity and actual supply voltage
   externally. For logic-only disconnect VM after checking; for NO-LOAD keep the
   verified VM supply powered, with both motors disconnected throughout. Before
   any separately authorized loaded testing use a suitable supply
   with verified current limiting, preferably <=6 V. Record its limit setting and
   behavior; no universal safe current setting is established by this code.
4. Verify reset/unpowered-MCU states and external pull-downs or hardware gating,
   especially on STBY. Check the actual breakout for pull-ups or STBY jumpers.

The [Toshiba datasheet](https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660)
specifies internal 200 kohm pull-downs on the control inputs, and high-impedance
outputs in standby. These weak pulls do not establish board-level reset safety.
Use verified external pull-downs/gating for reset, bootloader and power sequencing;
software starts only after HAL init. Confirm logic compatibility: the datasheet
requires HIGH >=0.7*VCC, so 3.3 V GPIO is not guaranteed HIGH with 5 V VCC.

With the reported ~3.5 ohm windings, idealized zero-back-EMF V/R is ~2.57 A
per motor at 9 V or ~1.71 A at 6 V, before driver/wiring losses and inductance.
These are estimates, not measured currents. A 9 V 1 A label does not specify a
safe current-limit transient response. PWM duty does not reliably limit peak
current. Even <=6 V alone does not prove safety: review driver current/pulse
ratings, breakout cooling, motor stall limits, wiring, decoupling and supply
regeneration behavior before confirming hardware readiness.

## Shutdown and loaded-test restriction

The diagnostic owns its inputs in static storage. A binary-local panic handler
attempts to pull STBY LOW first, then PWM/direction LOW, before halting; it does
not rely on Drop/unwinding. A recursive-borrow fault skips the attempt. Pin transitions occur only in short
critical sections without logging/delays; GPIO writes are infallible. Pre-publication
initialization only writes LOW.
The blocked request and normal completion both reassert stop. GPIO writes are
infallible HAL operations; there are no fallible motor/PWM setup paths here.

Software shutdown cannot replace a physical cutoff or protect against all CPU
faults, debug halts, reset or wiring errors. Standby does not stop inertia.

Loaded MOTOR_TEST implementation remains blocked and awaits explicit hardware confirmation, including
external VCC/VM/STBY readings and a suitable current-limited supply. Once approved,
it must have a boot grace period, brief bounded single-channel pulses, the other
channel held inactive, stopped intervals, and final STBY LOW. No simultaneous
channels, 70% kick, balancing, recovery, repeats or retries are authorized.
No hardware movement, voltage/current readings or RPM validation is claimed by
software tests/builds.

## Previous logic-only validation (before NO-LOAD update)

Release builds succeeded for logic-only, blocked MOTOR_TEST combined with legacy
live features, and the default `esp-bimo` binary. Linker warns of a LOAD segment
with RWX permissions for both new and existing binaries. Formatting and diff
whitespace checks passed. Direct host motor-control tests: 6 passed.

The existing host suite (`tests/orientation/Cargo.toml`, `--no-fail-fast`)
reported 208 passed, 22 failed across 6 targets. Its tested source files were
not modified by this change. Failures are listed below; this is not a green
project-wide suite. Full output for this run is `/tmp/esp-bimo-diagnostic-suite.log`.

- `control_profile_cutoff_is_21_inclusive_and_latched_without_changing_default`
- `arbiter_tests::faults_zero_actual_interface_and_no_rearm_even_if_external_caller_arms_driver`
- `arbiter_tests::ordinary_commands_stay_bounded_and_budget_not_reset_by_neutral`
- `recovery_tests::capped_probe_projection_excludes_200ms_but_useful_probe_keeps_it`
- `recovery_tests::control_21_degree_boundary_stops_actual_outputs_for_all_authorities`
- `recovery_tests::faults_in_pending_recovery_permanently_zero_proposals`
- `recovery_tests::capped_probe_integrated_zero_wait_and_fault_stop_actual_motor_interface`
- `recovery_tests::recovery_margin_and_evidence_boundaries_are_conservative`
- `recovery_tests::recovery_now_accepts_12_to_18_margin_range_and_rejects_18`
- `recovery_tests::probe_cap_reached_mid_probe_skips_remaining_wait_and_can_cancel`
- `recovery_tests::integrated_faults_during_zero_or_pulse_permanently_stop_real_interface`
- `recovery_tests::short_recovery_zero_cancellation_and_faults_never_complete_a_pending_kick`
- `restart_tests::tilt_cutoff_and_current_90_second_timeout_still_disarm`
- `tests::pre_ready_deadline_fault_latches_and_both_tilt_signs_stop`
- `tests::tilt_nan_deadline_and_driver_failures_never_rearm`
- `rolling_tests::rolling_faults_tilt_and_deadline_remain_permanent_after_credit_recovers`
- `timing_and_independent_default_off_source_authorization`
- `default_source_gate_is_off_and_balance_features_cannot_authorize_motion`
- `arming_does_not_force_a_kick_requires_post_ready_neutral_then_request`
- `physical_output_gate_disarms_on_fault_timeout_and_dry_even_with_old_kick`
- `ready_but_not_neutral_outlasts_active_window_then_hits_arming_timeout`
- `ground_preparation_is_scoped_and_all_new_gates_disabled`

## NO-LOAD validation commands

```sh
cargo fmt --all -- --check
rustc +stable --edition=2024 --test tests/tb6612_no_load.rs -o /tmp/bimo-no-load-tests
/tmp/bimo-no-load-tests
cargo build --release --locked --bin tb6612-diagnostic --features tb6612-diagnostic
cargo build --release --locked --bin tb6612-diagnostic --features tb6612-diagnostic-no-load
```

The host sequence test checks the single-pass order, initial grace, per-channel
phases, all stopped intervals and final OFF. It cannot prove physical GPIO levels,
disconnected motors or voltages. A physical power cutoff and verified external
pull-downs/gating remain necessary; panic shutdown is only a best-effort attempt.

Verified after the NO-LOAD update: formatting and whitespace checks passed;
logic-only, NO-LOAD and default firmware release builds succeeded, with the same
RWX linker warning. The sequence test and six motor-control tests passed. A fresh
full host-suite run again reported 208 passed / 22 failed; names match the list
above. Its current full log is `/tmp/esp-bimo-no-load-suite.log`. No flash or
physical voltage measurement was performed.

## Loaded fail-closed validation

Formatting and whitespace checks passed. Release builds passed for loaded,
logic-only, prior NO-LOAD, loaded combined with legacy live/motor-test features,
and default `esp-bimo`. They retain the linker LOAD-segment RWX warning. The
loaded + NO-LOAD build was explicitly tested and rejected with the intended
mutually-exclusive-modes compile error. The prior sequence test (1) and
motor-control tests (6) passed. The full existing host suite again reported
208 passed / 22 failed across 6 targets; the failure names are listed above.
Current full output: `/tmp/esp-bimo-loaded-suite.log`. No balancing or test
source was changed in this update. No flash or physical validation was performed.
