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
