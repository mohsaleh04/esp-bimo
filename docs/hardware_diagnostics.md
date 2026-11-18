# Hardware diagnostics

Build: `cargo build --release --bin hardware_diagnostics`

Flash and monitor: `cargo run --release --bin hardware_diagnostics`

No serial input required. Use the native ESP32-S3 USB Serial/JTAG connector
(GPIO19/20). esp-println previously selected `auto`, which can fall back to UART0
TX43/RX44; Cargo now explicitly selects jtag-serial for every binary so touch
GPIO43/44 cannot conflict with runtime logging. UART-only adapter logging no
longer works. ROM boot messages can still appear on GPIO43 before firmware runs;
verify the touch module tolerates that or disable ROM UART output separately.
Do not connect an external UART adapter to touch pins.

This standalone binary does not call Board::init: the existing board and motor
code use L298N and have no TB6612 STBY ownership. GPIO40 is its first output,
held LOW; all six wheel control outputs also stay LOW. GPIO18 and GPIO48 are
untouched. A physical STBY pull-down is required during reset/pre-boot.

OLED assumes SSD1306 128x64, four-wire SPI, internal charge pump and 3.3V logic.
The local driver uses bounded blocking software SPI mode 0 at <=500kHz;
SSD1306 SPI has no acknowledgement, so successful boot cannot prove display
presence. Reset is driven and a static pair of rectangular eyes rendered.

Touch and tilt are raw digital levels, no assumed polarity or internal pull.
Inputs must be driven by 3.3V-compatible modules; transitions are polled (no
hardware debounce), and short pulses can be missed during ultrasonic polling.
MAX4466 and IR use ADC1 GPIO2/1 with 11dB attenuation, uncalibrated 12-bit raw
counts. The 250ms microphone window reports actual duration, count, mean, min,
max, peak-to-peak and counts within 16 codes of either rail. This flags ADC rail
proximity, not proof of microphone amplifier clipping; no dB or voltage claims.

HC-SR04 uses a 10us trigger and separate 30ms rise/fall deadlines; already-high
ECHO is rejected. Distance assumes 343m/s. The installed ECHO divider must keep
GPIO47 within 3.3V limits. Cycles are at least 250ms apart. BMI160 I2C14/13,
INT41 and QMC DRDY12 are not initialized in this first-stage diagnostic.

No physical test or flashing was performed as part of implementation.

## Touch eye animations

`src/eyes.rs` contains the reusable pure state machine, framebuffer renderer and
30ms debounce. Assumes active-HIGH touch modules. Only debounced rising edges
trigger animation; holding touch does not retrigger. New events replace current
animation; simultaneous events give chest priority. Head shows happy arches,
briefly closes, then opens; chest looks left/right/center. Automatic blinks use
a deterministic varying 4–7s idle interval (restarted after touch animation).
Frames are transferred only when the pose changes. No heap or RTOS is needed.

Ultrasonic acquisition now polls rise/fall as states with 30ms deadlines, while
continuing touch and animation updates. OLED transfers and ADC sampling pause
during acquisition; a pending frame waits at most approximately 60ms for an
entire echo attempt. Software SPI frame transfer itself remains bounded blocking
(~16ms minimum plus GPIO overhead); this is a first implementation, not a hard
real-time guarantee. Microphone sample counts can vary accordingly. Existing
startup reset delays remain; animation uses monotonic deadlines, no long sleeps.

Chest touch also restarts a short (~0.8s) rising/falling giggle pattern on the
existing active-low GPIO21 buzzer. LEDC channel0 and low-speed timers0–3 generate
880/1320/1760Hz tones; monotonic deadlines advance notes without sleeps, including
during echo acquisition. The final rest leaves the buzzer silent (HIGH).
Holding touch does not repeat it. Assumes the existing buzzer hardware supports
pitch modulation; perceived sound requires physical verification. Motors and
servo remain untouched. The microphone can pick up the generated sound.

Chest eye behavior now uses filled five-point stars that wobble horizontally
and vertically every 80ms for 720ms, then reopen. It no longer uses the ordinary
left/right poses. Idle behavior adds occasional 550ms glances with 2–6s pauses;
blink deadlines retain priority and glances do not postpone them. Touch events
interrupt either idle animation. The existing giggle audio is unchanged.

Head touch plays an appreciative two-note chirp (880 then 1320Hz),
then repeats it one octave higher (1760 then 2640Hz), lasting ~0.65s with low 5%/2% duty and a brief gap, alongside the happy eyes. Latest
touch replaces both sound and animation; simultaneous touches favor chest.
No blocking note delays are introduced. Low duty aims for a softer response,
but actual volume/pitch depends on the buzzer hardware and is not verified.

## IR saturation reaction

Two consecutive IR readings equal to 4095 (read every ~250ms) trigger a single
possible-lift reaction: wide hollow eyes wobble left/right for 900ms then reopen,
and an irregular pitch pattern plays. This is distinct from starry tickle eyes.
The detector rearms only after two readings below 3900; sustained saturation
cannot loop the sound. ADC saturation alone does not prove physical lifting:
validate this heuristic against the installed sensor. New touch events may
interrupt the reaction. GPIO18 servo remains uninitialized; STBY stays LOW.

## Chest-triggered head servo (supersedes earlier servo-unused notes)

GPIO18 now drives a position-controlled head servo at boot center (1500us).
Dedicated MCPWM1 timer0/operator0A generates 50Hz with 1us timestamps; buzzer
LEDC resources and wheel MCPWM0 remain untouched. GPIO48 is still unused.
`src/head_servo.rs` shares pulse calibration with `servo_diagnostics` and provides
a nonblocking 90->80->100->90 trajectory, one degree per >=15ms (~0.6s plus
loop/SPI/logging delays). Only debounced chest rising edges trigger motion;
head touch and IR do not. Busy triggers are ignored without queuing. Existing
chest eye/audio effects are unchanged and can restart independently.

PWM stays active at center after completion, providing a continued position
command (not a guarantee of position/torque). The initial physical position is
unknown: first center command may move abruptly; software cannot ramp from an
unmeasured position. Invalid trajectory angles terminate motion and command
center. No position/stall/current/supply feedback is available. An invalid clock
configuration fails before starting movement. External regulated 5V and shared
ground are required; supply health is not measured or guaranteed.

Adjust CENTER_DEG, LEFT_SIGN, OFFSET_DEG, MIN_PULSE_US/MAX_PULSE_US in
`head_servo.rs`; physical angle and endpoints remain uncalibrated. The standalone
servo test retains its one-shot sequence and final PWM shutdown; only its pulse
mapping/constants are imported from the shared module. No physical validation
of this combined firmware has been performed.

### Occasional idle head movement

The combined diagnostic now also commands an idle head glance: first at ~25s
monotonic uptime, then 20–30s after each completed head motion. It alternates
left/right by five logical degrees, steps at 60ms/degree, holds 350ms and returns
to center. Timing varies deterministically. Chest touch interrupts this movement
from the last commanded angle without a jump; repeated tickle events still do
not queue. This supersedes the earlier statement that only chest starts motion.
The standalone servo diagnostic does not instantiate Motion and remains one-shot.

## Standalone HC-SR04 diagnostic

From the repository root, use the ESP32-S3 USB Serial/JTAG connection:

```sh
cargo run --release --bin hc-sr04-diagnostic
```

The configured runner flashes with `espflash flash --monitor --chip esp32s3`.
The diagnostic uses TRIG GPIO38 and ECHO GPIO47. Keep the Echo signal at
3.3 V logic levels at the ESP32-S3 pin. GPIO40 (STBY) and all motor control
outputs are driven LOW. The diagnostic leaves the servo pin untouched.

Each ping reports the initial Echo level, the software-measured Trigger HIGH
duration, the wait from Trigger LOW to Echo HIGH, and the Echo HIGH duration.
Timing values are in microseconds. `None` denotes a phase that did not run.

- `ECHO_STUCK_HIGH`: Echo was already HIGH before triggering, so the ping was skipped.
- `WAIT_RISE_TIMEOUT`: no Echo HIGH observed before the 30 ms rise deadline.
- `WAIT_FALL_TIMEOUT`: no Echo LOW observed before the 30 ms fall deadline.
  `echo_high_us` is the elapsed wait when the deadline was detected.
- `INVALID_PULSE`: both edges were observed, but the pulse width was outside
  117–23,323 µs, corresponding to the nominal 20–4000 mm range at 343 m/s.

Only a valid completed pulse produces `distance_mm`. The timings and outcome
share a ping number and are printed after sampling. There is a 300 ms pause
between attempts.

```sh
CARGO_ENCODED_RUSTFLAGS='' cargo +stable test --locked --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu --test hc-sr04
```
