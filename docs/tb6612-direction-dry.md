# TB6612 direction preview with VM disconnected

`tb6612-direction-dry` runs once per reset and prints four hypothetical
commands. It retains the motor GPIOs initialized by `Board::init` at LOW,
starting with STBY. It has no motor driver, PWM routing, or activation option.
The DRY build cannot check continuity, voltage, motor current, or wheel travel.

## Build and optional manual flash

Disconnect the motor adapter from VM before connecting the ESP32-S3 via its
native USB Serial/JTAG port. Leave VM disconnected through flashing and reset.
Use the actual ESP32-S3 device path if it differs from `/dev/ttyACM0`.

```sh
cargo build --release --locked --target xtensa-esp32s3-none-elf --bin tb6612-direction-dry --features tb6612-direction-dry
espflash flash --chip esp32s3 --port /dev/ttyACM0 --monitor target/xtensa-esp32s3-none-elf/release/tb6612-direction-dry
```

## Interpret the log

- Confirm the banner says `[DRY ONLY]`. The intended mapping is A=LEFT
  (AIN1=15, AIN2=16, PWMA=17), B=RIGHT (BIN1=7, BIN2=6, PWMB=5), STBY=40.
- Expect A forward, A reverse, B forward, B reverse. In the hypothetical
  column the selected wheel shows 15% after clamping the illustrative ±20%
  request; the other wheel shows 0%. These are example numbers for the log.
- In hypothetical direction tuples, `true` means HIGH. A forward is
  `(false,true)`; B forward is `(true,false)`. Reverse swaps each pair.
- Every row must say commanded STBY LOW, both PWMs 0%, and both direction
  pairs LOW/LOW. Expect STOP before, between and after the previews, then DONE
  with no repeat until reset. These are software-commanded states.

## Before a separate powered direction test

The reported approximately 9 V adapter and nominal 6 V TT motors do not have a
verified safe operating envelope. A low duty setting still applies pulses near
VM during on-time; it does not establish a current limit or a 6 V voltage limit.
Keep this diagnostic unpowered at VM regardless of the adapter available.

Use a verified regulated motor supply at the motor's rated voltage or lower
with an adjustable, verified hardware current limit. Establish an allowable
startup/stall-current envelope from the exact motor and TB6612 module ratings,
then check VM and VCC, common ground, wiring continuity, STBY pull-down during
reset, and startup current with suitable instrumentation or a bench supply
with trustworthy current readback and a verified limit. Use independent
motor-power cutoff and support both wheels off the ground before reviewing a
separately authorized powered test. Unloaded wheels alone do not bound startup
current.
