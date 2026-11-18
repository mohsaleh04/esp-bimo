# Head servo diagnostic

Build default limited test:

```sh
cargo build --release --bin servo_diagnostics
cargo run --release --bin servo_diagnostics
```

The run command flashes the board and starts motion after a 3s startup delay.
USB Serial/JTAG logging uses GPIO19/20, with the existing configuration unchanged.
This binary owns only LEDC timer0/channel0, GPIO18, and disabled wheel outputs.
No OLED, sensors, balance code or other binary is initialized.

## Hardware prerequisites and unverified assumptions

The installed model cannot be identified by firmware: verify its label/datasheet
and confirm it is position-controlled, NOT continuous rotation, before running.
A continuous-rotation servo interprets these pulses as speed, so this sequence
cannot position it. No physical verification was performed.

GPIO18 is signal; external regulated 5V (7805) supplies the servo, with shared
ESP ground. Confirm the regulator's current/thermal capacity meets servo startup
and stall demand. Do not power the servo from an ESP GPIO. STBY GPIO40 and wheel
pins5/6/7/15/16/17 are held LOW throughout. Keep a physical STBY pull-down for
reset/pre-boot. GPIO48 remains unused.

Logical center is 90 degrees / 1500us, not a measured current angle. Initial
centering can move abruptly because no position feedback exists. Subsequent
commands ramp one logical degree every 20ms. Default limits are 1000..2000us
mapped to logical 0..180; these are conservative guesses, NOT calibrated limits
or a guarantee of 180 degrees of physical travel. Actual endpoints and horn
clearance must be verified. Even the default near-center motion requires that
verification; first commissioning with unloaded linkage is preferable.

Default sequence: 90 -> 80 -> 90 -> 100 -> 90, with 2s dwell at each target.
Change LEFT_SIGN to +1 if installation reverses physical left/right. Constants
MIN_PULSE_US/MAX_PULSE_US and angle offsets are in the binary. PWM is 50Hz,
14-bit hardware LEDC. No RTOS or allocation is needed.

## Explicit full sweep

Only after confirming servo type, safe pulse endpoints and mechanical clearance:

```sh
cargo run --release --bin servo_diagnostics --features servo-full-sweep
```

This opts into logical 90 -> 0 -> 90 -> 180 -> 90 (or reversed via LEFT_SIGN),
with 2s dwell at each target. Without that feature no endpoint is commanded.
Logical +/-90 does not establish physical +/-90 until calibrated.
After final centering and settling, duty becomes zero (signal LOW); no further
servo pulses or sequence repetitions occur until reset. Servo power remains
connected and holding torque is not guaranteed after pulses stop.
