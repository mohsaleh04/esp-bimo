# Live balance

The current Balance V2 firmware demonstrated self-balancing on Bimo with
its independent 2S motor supply. Build the live image without flashing with
`cargo build --release --locked --bin balance-v2-live --features balance-v2-live`.
The separate dry image is available through
`cargo run --release --locked --bin balance-v2-test`.

To flash the live image explicitly:

```sh
cargo run --release --locked --target xtensa-esp32s3-none-elf --bin balance-v2-live --features balance-v2-live
```

This invokes the project's `espflash flash --monitor --chip esp32s3` runner.
It runs BMI160 calibration and the five-second upright preparation
automatically. Motors remain disarmed until the arming conditions pass.

**A reset or reboot, including a watchdog reset, can restart the whole sequence
while the LIVE build remains installed.** Completion latches only for the
current boot. Keep an independent physical motor-power disconnect within reach
and disconnect VM before resetting or reflashing.

The binary shares the V2 PD and PWM preview code: Kp 1.2, Kd 0.08, PD
clamped to ±10, deadband ±1, and normal duty mapped to 10–45%. It uses the
production TB6612 driver, whose hardware ceiling is set to 45% here.
There is no Kick or restart/recovery assist.

The arming deadline is 10 seconds. Arming requires 500 ms of consecutive fresh gravity-corrected measurements
within ±5° and ±5°/s, followed by a new upright estimate with zero mapped PWM.
The neutral sample sends zero. The active window lasts up to 180 seconds,
then disarms and latches completion.

Each fresh valid estimate yields a new PD command. Empty polls do not resend
a command or feed the watchdog. The controller checks 40 ms sample freshness,
50 ms gravity-correction age, inclusive 25° tilt cutoff, and 30 ms loop gap
and work limits. Sensor, timing and motor-command faults and both deadlines
disarm and latch. A reversal waits for at least 100 ms of electrical coast
measured from a completed zero command. A 100 ms watchdog is a reset fallback
during the active window. A zero command allows the wheels to coast.
A watchdog reset cannot replace the independent power disconnect.

## Hardware revalidation

1. **Electrical validation.** Keep VM disconnected for USB/DRY setup. Use a
   verified regulated motor supply at or below the nominal 6 V motor rating
   and a verified hardware current limit appropriate to the actual TT motors
   and TB6612 module. Verify VM, VCC, common ground, continuity, the STBY
   pull-down at reset and startup current. Provide an independent, accessible
   motor-power cutoff. Low PWM duty does not cap on-pulse voltage or peak current.
   Verify the actual VM under load with the battery setup being used.
2. **Suspended-wheel polarity.** After electrical validation, use the separate
   one-wheel [direction diagnostic](tb6612_diagnostics.md) with both wheels
   securely suspended. Check A=left, B=right and that positive commands
   physically propel both wheels forward. With the body supported, use
   `balance-v2-test` to confirm a manual forward/backward lean produces the
   intended pitch and corrective command signs. A logic-only DRY run verifies
   only the computed signs. Stop on a mismatch before proceeding.
3. **Supported ground contact.** With the electrical and polarity checks
   complete, restrain and support the body so it cannot fall or drive away.
   Keep the motor cutoff within reach. Start upright and apply a small
   supported perturbation while watching that wheels move in
   the corrective direction. Cut motor power for wrong direction, unexpected
   movement, or loss of control/telemetry. Reflash the disarmed default
   firmware with VM disconnected to end live testing.

Host tests simulate the motor interface. They cannot establish actual
electrical limits, pulse timing, mechanical polarity, or fault shutdown
during a CPU halt.
