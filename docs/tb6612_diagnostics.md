# TB6612 suspended-wheel direction diagnostic

This diagnostic is independent of the balance runtime. The checked-in source
has `MANUALLY_AUTHORIZED_STAGE = None`. A pulse requires the `tb6612-live`
feature, one explicitly selected stage in source, and an exact `RUN` line
from the USB Serial/JTAG console after the prompt. `tb6612-both` blocks
activation. Free text, a missing confirmation, or a timeout leaves the outputs
off until reset.

## Build and optional manual flash

Use native USB Serial/JTAG. Disconnect motor power at VM during flashing and
the first DRY check. Substitute the actual serial device if it differs from
`/dev/ttyACM0`.

```sh
cargo build --release --locked --bin tb6612_diagnostics
espflash flash --chip esp32s3 --port /dev/ttyACM0 --monitor target/xtensa-esp32s3-none-elf/release/tb6612_diagnostics
```

For a separately authorized one-wheel trial, change only
`MANUALLY_AUTHORIZED_STAGE` in `src/bin/tb6612_diagnostics.rs` to one of
`Some(Stage::LeftForward)`, `Some(Stage::LeftReverse)`,
`Some(Stage::RightForward)`, or `Some(Stage::RightReverse)`. Rebuild and
manually flash with VM disconnected:

```sh
cargo build --release --locked --bin tb6612_diagnostics --features tb6612-live
espflash flash --chip esp32s3 --port /dev/ttyACM0 --monitor target/xtensa-esp32s3-none-elf/release/tb6612_diagnostics
```

The stage and intended GPIO values are printed before a five-second pause.
When the confirmation prompt appears, verify both wheels are suspended and
that the independent motor-power disconnect is accessible. If proceeding
after the electrical review, supply VM and type uppercase `RUN` then Enter
within 30 seconds. Keep hands clear of the wheels. A missing or invalid line
does not run a pulse. After the pulse, disconnect VM and record the observed
wheel and direction. Restore the source selection to `None` after testing.
A new boot or a watchdog reset needs another manual confirmation.

| Source stage | Selected wheel | Direction input levels | Duty | Software target |
|---|---|---|---|---|
| `LeftForward` | A / left | AIN1 LOW, AIN2 HIGH | 15% | 250 ms |
| `LeftReverse` | A / left | AIN1 HIGH, AIN2 LOW | 15% | 250 ms |
| `RightForward` | B / right | BIN1 HIGH, BIN2 LOW | 15% | 250 ms |
| `RightReverse` | B / right | BIN1 LOW, BIN2 HIGH | 15% | 250 ms |

The other channel gets PWM zero and both direction inputs LOW. Observe which
wheel responds and whether its direction agrees with logical forward or
reverse. A forward wheel is one that would propel the chassis forward if on
the ground. The code inverts channel A's direction inputs using the shared
polarity definition. Neither polarity nor a pulse proves physical travel.
If the wrong wheel responds, motion is reversed, or a wheel does not start,
disconnect VM and record the outcome without increasing duty or repeating
automatically.

## Shutdown and electrical limitations

AIN1/2=15/16, BIN1/2=7/6, PWMA/B=17/5 and STBY=40. STBY is initialized LOW
before the other motor GPIOs. DRY leaves all motor pins LOW and skips LEDC
setup. For the one pulse, the driver configures 20 kHz, 10-bit LEDC, prepares
direction and duty with STBY LOW, and raises STBY with a 250 ms software target.
There is no logging, USB input, ramp, Kick or retry during the pulse. Normal
shutdown first lowers STBY and then clears directions and PWM. A 300 ms
watchdog is a reset fallback, not a hardware disconnect or an observed bound
on output under all faults. A CPU/debug halt or reset may require the
independent cutoff and an external STBY pull-down. With STBY LOW both bridges
are disabled, and the wheels can still coast.

The reported approximately 9 V adapter and nominal 6 V TT motors are
electrically unverified. PWM on-time applies pulses near VM. Duty, duration
and suspended wheels do not establish a safe voltage or peak-current limit.
Before a defensible powered trial, verify a regulated motor supply at or
below the motor rating, a hardware current limit appropriate to the exact
motor and module, logic supply/common ground, wiring continuity and STBY
pull-down during reset. Check actual VM and startup current with suitable
equipment. A successful build or log cannot verify any of those conditions.
