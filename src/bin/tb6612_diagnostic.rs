#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
};

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(all(
    feature = "tb6612-diagnostic-loaded",
    feature = "tb6612-diagnostic-no-load"
))]
compile_error!(
    "LOADED and NO-LOAD are mutually exclusive: NO-LOAD requires BOTH motors disconnected"
);

#[cfg(feature = "tb6612-diagnostic-no-load")]
#[path = "tb6612_diagnostic/sequence.rs"]
mod sequence;

// Own every driver input for the entire lifetime, including the panic path.
// No PWM peripheral, allocator, RTOS, sensors or balance controller is started.
// The explicitly gated no-load mode uses static HIGH for 100% PWM.
static INPUTS: Mutex<RefCell<Option<InactiveInputs>>> = Mutex::new(RefCell::new(None));

struct InactiveInputs {
    stby: Output<'static>,
    pwma: Output<'static>,
    pwmb: Output<'static>,
    ain1: Output<'static>,
    ain2: Output<'static>,
    bin1: Output<'static>,
    bin2: Output<'static>,
}

impl InactiveInputs {
    fn stop(&mut self) {
        self.stby.set_low();
        self.pwma.set_low();
        self.pwmb.set_low();
        self.ain1.set_low();
        self.ain2.set_low();
        self.bin1.set_low();
        self.bin2.set_low();
    }
}

fn attempt_stop() {
    critical_section::with(|cs| {
        // Do not panic recursively if a fault occurred while borrowing inputs.
        // Initialization before publication here only writes LOW. Active writes
        // are infallible GPIO operations in a short, non-logging critical section.
        if let Ok(mut inputs) = INPUTS.borrow(cs).try_borrow_mut()
            && let Some(inputs) = inputs.as_mut()
        {
            inputs.stop();
        }
    });
}

#[cfg(feature = "tb6612-diagnostic-no-load")]
struct NoLoadIo;

#[cfg(feature = "tb6612-diagnostic-no-load")]
impl sequence::Io for NoLoadIo {
    fn apply(&mut self, phase: sequence::Phase) {
        use esp_bimo::motors::control::{Channel, Direction};
        use sequence::Phase;
        // Stop before logging: console backpressure must never extend a live phase.
        attempt_stop();
        match phase {
            Phase::Off => log::warn!(
                "[OFF] STBY/PWM/directions LOW; both outputs high impedance. Floating meter readings are not guaranteed zero."
            ),
            Phase::AForward => log::warn!(
                "[A FORWARD 5s] A PWM=100%, B disabled; meter red=AO1 black=AO2: expect approximately -VM (external observation only)"
            ),
            Phase::AReverse => log::warn!(
                "[A REVERSE 5s] A PWM=100%, B disabled; meter red=AO1 black=AO2: expect approximately +VM (external observation only)"
            ),
            Phase::BForward => log::warn!(
                "[B FORWARD 5s] B PWM=100%, A disabled; meter red=BO1 black=BO2: expect approximately +VM (external observation only)"
            ),
            Phase::BReverse => log::warn!(
                "[B REVERSE 5s] B PWM=100%, A disabled; meter red=BO1 black=BO2: expect approximately -VM (external observation only)"
            ),
        }
        critical_section::with(|cs| {
            let mut owner = INPUTS.borrow(cs).borrow_mut();
            let inputs = owner.as_mut().expect("motor input owner initialized");
            // Break before make: disables BOTH channels before every change.
            inputs.stop();
            match phase {
                Phase::Off => {}
                Phase::AForward | Phase::AReverse => {
                    let direction = if phase == Phase::AForward {
                        Direction::Positive
                    } else {
                        Direction::Negative
                    };
                    let (first, second) = Channel::A.direction_inputs(direction);
                    inputs.ain1.set_level(first.into());
                    inputs.ain2.set_level(second.into());
                    inputs.pwma.set_high();
                }
                Phase::BForward | Phase::BReverse => {
                    let direction = if phase == Phase::BForward {
                        Direction::Positive
                    } else {
                        Direction::Negative
                    };
                    let (first, second) = Channel::B.direction_inputs(direction);
                    inputs.bin1.set_level(first.into());
                    inputs.bin2.set_level(second.into());
                    inputs.pwmb.set_high();
                }
            }
            // Enable last; the other channel's PWM and directions remain LOW.
            if phase != Phase::Off {
                inputs.stby.set_high();
            }
        });
    }

    fn wait_ms(&mut self, ms: u32) {
        Delay::new().delay_millis(ms);
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    // Abort does not unwind/Drop. Attempt shutdown before any logging or halt.
    // Pre-init faults and CPU/debug stalls still require hardware protection.
    attempt_stop();
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(feature = "tb6612-diagnostic-loaded")]
fn loaded_blocked() {
    attempt_stop();
    log::warn!(
        "[BOOT LOADED] Connected-motor request; activation BLOCKED. No NO-LOAD sequence, PWM timer, balance controller or recovery is started."
    );
    log::info!(
        "[MAP] A=LEFT AIN1=GPIO15 AIN2=GPIO16 PWMA=GPIO17; B=RIGHT BIN1=GPIO7 BIN2=GPIO6 PWMB=GPIO5; STBY=GPIO40"
    );
    log::info!(
        "[STATE] Commanded STBY, both PWM and all direction inputs LOW. PWM=0% on BOTH channels."
    );
    log::warn!(
        "[HARDWARE REPORTED, NOT MEASURED] VM=5V/2A adapter; VCC=3.3V; common GND; STBY external 10kohm pull-down; motors connected, wheels suspended."
    );
    log::warn!(
        "[CURRENT] 5V / ~3.5ohm = ~1.43A theoretical stall current per motor. Adapter 2A rating does NOT prove reliable peak-current limiting. Low PWM duty does NOT guarantee safety."
    );
    log::warn!(
        "[LOADED BLOCKED] Supply current-limit behavior, safe driver/motor current envelope and motor electrical dynamics are unverified. No safe duty or pulse duration is established. Use a suitable verified current limiter before implementing activation."
    );
    log::warn!(
        "[STARTUP 5s] Both channels remain disabled; STBY LOW throughout. Physical cutoff required; firmware cannot detect stalled motors or measure current/voltage/RPM."
    );
    Delay::new().delay_millis(5000);
    for phase in [
        "A LEFT FORWARD",
        "A LEFT REVERSE",
        "B RIGHT FORWARD",
        "B RIGHT REVERSE",
    ] {
        attempt_stop();
        log::warn!(
            "[SKIPPED {phase}] Commanded PWM A=0% B=0%; all direction inputs LOW, STBY LOW. No motion test performed; channel functionality remains unverified."
        );
    }
    attempt_stop();
}

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    // First operations after HAL init. HAL sets the LOW latch before enabling
    // output. Reset, ROM and bootloader time require external hardware gating.
    let inputs = InactiveInputs {
        stby: Output::new(
            esp_bimo::motor_gpio!(p, stby),
            Level::Low,
            OutputConfig::default(),
        ),
        pwma: Output::new(
            esp_bimo::motor_gpio!(p, pwma),
            Level::Low,
            OutputConfig::default(),
        ),
        pwmb: Output::new(
            esp_bimo::motor_gpio!(p, pwmb),
            Level::Low,
            OutputConfig::default(),
        ),
        ain1: Output::new(
            esp_bimo::motor_gpio!(p, ain1),
            Level::Low,
            OutputConfig::default(),
        ),
        ain2: Output::new(
            esp_bimo::motor_gpio!(p, ain2),
            Level::Low,
            OutputConfig::default(),
        ),
        bin1: Output::new(
            esp_bimo::motor_gpio!(p, bin1),
            Level::Low,
            OutputConfig::default(),
        ),
        bin2: Output::new(
            esp_bimo::motor_gpio!(p, bin2),
            Level::Low,
            OutputConfig::default(),
        ),
    };
    critical_section::with(|cs| INPUTS.borrow(cs).replace(Some(inputs)));
    esp_println::logger::init_logger_from_env();
    #[cfg(feature = "tb6612-diagnostic-loaded")]
    loaded_blocked();

    #[cfg(not(feature = "tb6612-diagnostic-loaded"))]
    {
        log::warn!(
            "[BOOT] tb6612-diagnostic; BOTH motors must be physically disconnected from AO1/AO2 and BO1/BO2 for NO-LOAD mode"
        );
        log::info!("[MAP] A=LEFT: AIN1=GPIO15 AIN2=GPIO16 PWMA=GPIO17");
        log::info!("[MAP] B=RIGHT: BIN1=GPIO7 BIN2=GPIO6 PWMB=GPIO5; STBY=GPIO40");
        log::info!(
            "[STATE] Commanded STBY/PWMA/PWMB/AIN1/AIN2/BIN1/BIN2=LOW; PWM peripheral not configured"
        );
        log::warn!(
            "[CHECK 1] Keep VM disconnected and motors disconnected. Verify common GND and physical motor-power cutoff."
        );
        log::warn!(
            "[CHECK 2] External meter: black=driver GND, red=VCC. Expect intended 3.3V logic rail; never connect 9V to VCC/GPIO."
        );
        log::warn!(
            "[CHECK 3] External meter: black=driver GND, red=STBY. Expect near 0V; verify PWM and direction inputs near 0V too."
        );
        log::warn!(
            "[CHECK 4] With BOTH motors disconnected, measure VM-GND externally and verify polarity. NO-LOAD requires verified VM powered; logic-only may keep VM disconnected. Loaded tests remain prohibited."
        );
        log::warn!(
            "[CHECK 5] Verify external pull-downs/gating, especially STBY, during reset/boot and with MCU unpowered. Software cannot protect pre-init or stalled CPU."
        );
        log::warn!(
            "[SUPPLY] Reported 6V TT motors, ~3.5 ohm each; 9V 1A adapter is NOT confirmed suitable. PWM is NOT peak current limiting."
        );
        log::warn!(
            "[FEEDBACK] No rail/current/RPM measurement hardware configured; all observations require external instruments."
        );
        #[cfg(feature = "tb6612-diagnostic-no-load")]
        {
            log::warn!(
                "[NO-LOAD ENABLED] Output terminals become LIVE after 5s. NEVER connect motors. Firmware cannot detect disconnection. Use external DC voltmeter only, not current/ohms mode."
            );
            log::warn!(
                "[STARTUP 5s] All inputs LOW. Then A forward/reverse, B forward/reverse: 5s each, OFF 3s after each. No repeats."
            );
            sequence::run(&mut NoLoadIo);
        }
        #[cfg(not(feature = "tb6612-diagnostic-no-load"))]
        if cfg!(feature = "tb6612-diagnostic-motor-test") {
            attempt_stop();
            log::error!(
                "[MOTOR_TEST BLOCKED] Feature requested, but electrical safety is unconfirmed. No activation implemented; all inputs remain LOW."
            );
        } else {
            log::info!(
                "[MOTOR_TEST DISABLED] Default logic-only stage; no pulses, kick, retries or motion sequence."
            );
        }
    }
    attempt_stop();
    log::warn!(
        "[DONE] Commanded all inputs LOW; idle until reset. Software shutdown is NOT a physical power cutoff."
    );
    let delay = Delay::new();
    loop {
        delay.delay_millis(1000);
    }
}
