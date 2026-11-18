# TB6612 first motor test

This is a separate binary, not a migration of L298N or the balance controller.
Board::init creates I2C/buzzer/sensor resources and lacks STBY, so this binary
owns its pins directly. Old wheel labels differ: this test strictly uses A=LEFT,
B=RIGHT. The old PWM mapper, kick, recovery, PD and assists are not invoked.
Some old motor tests have duration comments inconsistent with their actual code;
none of those routines are reused. Existing files remain unchanged.

## Build and flash

Default DRY (no STBY HIGH or nonzero PWM):

```sh
cargo build --release --bin tb6612_diagnostics
cargo run --release --bin tb6612_diagnostics
```

Explicit LIVE, separate motors only (flashing starts the sequence after 5s):

```sh
cargo run --release --bin tb6612_diagnostics --features tb6612-live
```

Only after physically checking the independent left/right tests, enable the
additional BOTH stages. This still runs separate motors first:

```sh
cargo run --release --bin tb6612_diagnostics --features tb6612-both
```

| Stage | Motor | Direction inputs | Duty | Pulse |
|---|---|---|---|---|
| 1 | A LEFT | AIN1 LOW / AIN2 HIGH | 15% | 250ms |
| 2 | A LEFT | AIN1 HIGH / AIN2 LOW | 15% | 250ms |
| 3 | B RIGHT | BIN1 HIGH / BIN2 LOW | 15% | 250ms |
| 4 | B RIGHT | BIN1 LOW / BIN2 HIGH | 15% | 250ms |
| 5, optional | Both | AIN2 and BIN1 HIGH | 15% each | 250ms |
| 6, optional | Both | AIN1 and BIN2 HIGH | 15% each | 250ms |

Each stage has a 2s stopped pause. '+'/'-' request logical forward/reverse. The bridge mapping inverts A only. Observe and record physical wheel direction yourself.
There is no encoder/current feedback or inference that the wheels moved.

## Wiring and bounds

AIN1/2=15/16, BIN1/2=7/6, PWMA/B=17/5, STBY=40. LEDC timer0 channels0/1
provide 20kHz PWM with 10-bit resolution. HAL 1.2.2 ESP32-S3 APIs are used.
Logging uses existing USB Serial/JTAG; OLED, servo and sensors are untouched.
STBY is the first GPIO output initialized LOW. PWM starts at zero and remains
zero in DRY. During LIVE, direction/duty are prepared while STBY is LOW, then
STBY is HIGH only for the short pulse. Shutdown clears STBY before logging,
zeroes PWM and directions, and never repeats until reset. Stops use standby
(disabled/high-impedance outputs), not a guarantee of immediate mechanical stop.

TEST_DUTY and AUTHORIZED_MAX_DUTY both remain 15. No automatic 20/25% escalation
exists. Raising either requires renewed authorization after physical inspection.
RUN_MS=250 leaves margin below a 300ms software deadline; this is NOT a hardware
cutoff. A CPU fault, interrupt delay, panic or debug halt can leave LEDC running
and STBY HIGH. Drop only helps normal unwinding, not abort/panic/reset or stalls.
USB loss does not stop the motor. Use an independent motor-power cutoff and a
hardware STBY pull-down; pre-firmware/reset safety depends on hardware. Support
wheels clear of the surface and keep access to the power cutoff during testing.

Assuming 9V VM does not establish motor voltage/current suitability. Confirm
motor ratings, driver current/thermal limits, logic supply and common ground.
15% PWM is not current limiting and can still produce high stall-current pulses.
No high-duty or long-duration automatic test is enabled. No motor movement or
hardware validation is claimed by a successful build.
