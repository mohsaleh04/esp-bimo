#![no_std]
#![no_main]
use esp_backtrace as _;
use esp_bimo::motors::control::{self, Channel, Direction};
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    time::{Duration, Instant, Rate},
    timer::timg::{MwdtStage, TimerGroup},
    usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx},
};
use log::{info, warn};

#[path = "tb6612_diagnostics/sequence.rs"]
mod sequence;
use sequence::{Confirmation, Stage, selected_stage};

esp_bootloader_esp_idf::esp_app_desc!();
// Raising this requires renewed physical-test authorization; no automatic ramp.
const TEST_DUTY: u8 = 15;
const AUTHORIZED_MAX_DUTY: u8 = 15;
const RUN_MS: u64 = 250;
const WATCHDOG_MS: u64 = 300;
const CONFIRMATION_SECS: u64 = 30;

// Deliberately select one stage in source for each authorized build.
const MANUALLY_AUTHORIZED_STAGE: Option<Stage> = None;
struct Gates {
    stby: Output<'static>,
    a1: Output<'static>,
    a2: Output<'static>,
    b1: Output<'static>,
    b2: Output<'static>,
}
impl Gates {
    fn stop(&mut self) {
        self.stby.set_low(); // Disable first, before any logging or direction changes.
        self.a1.set_low();
        self.a2.set_low();
        self.b1.set_low();
        self.b2.set_low();
    }
}
impl Drop for Gates {
    fn drop(&mut self) {
        self.stop();
    }
}
fn log_stage(stage: Stage) -> [(bool, bool); 2] {
    let [a, b] = stage.directions();
    let ain = Channel::A.direction_inputs(a);
    let bin = Channel::B.direction_inputs(b);
    info!(
        "{stage:?}: A/LEFT={a:?}/{}% B/RIGHT={b:?}/{}%; AIN={ain:?} BIN={bin:?}; target={RUN_MS}ms; STBY currently LOW",
        if a == Direction::Stopped {
            0
        } else {
            TEST_DUTY
        },
        if b == Direction::Stopped {
            0
        } else {
            TEST_DUTY
        },
    );
    [ain, bin]
}

fn confirm(serial: &mut UsbSerialJtagRx<'_, esp_hal::Blocking>, delay: &Delay) -> bool {
    // Discard input already buffered before requesting fresh confirmation.
    let mut discard = [0; 64];
    if serial.drain_rx_fifo(&mut discard) == discard.len() {
        return false;
    }
    warn!(
        "ONE PULSE ONLY: after checking restraint and power cutoff, type RUN + Enter within {CONFIRMATION_SECS}s."
    );
    let start = Instant::now();
    let mut confirmation = Confirmation::default();
    while start.elapsed() < Duration::from_secs(CONFIRMATION_SECS) {
        if let Ok(byte) = serial.read_byte() {
            if let Some(accepted) = confirmation.push(byte) {
                return accepted;
            }
        } else {
            delay.delay_millis(1);
        }
    }
    false
}

fn idle(delay: &Delay) -> ! {
    info!("DONE: disarmed; STBY LOW PWM=0; idle until reset");
    loop {
        delay.delay_millis(1000);
    }
}

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let stby = Output::new(
        esp_bimo::motor_gpio!(p, stby),
        Level::Low,
        OutputConfig::default(),
    );
    macro_rules! low {
        ($p:expr) => {
            Output::new($p, Level::Low, OutputConfig::default())
        };
    }
    let pwma = low!(esp_bimo::motor_gpio!(p, pwma));
    let pwmb = low!(esp_bimo::motor_gpio!(p, pwmb));
    let mut gates = Gates {
        stby,
        a1: low!(esp_bimo::motor_gpio!(p, ain1)),
        a2: low!(esp_bimo::motor_gpio!(p, ain2)),
        b1: low!(esp_bimo::motor_gpio!(p, bin1)),
        b2: low!(esp_bimo::motor_gpio!(p, bin2)),
    };
    esp_println::logger::init_logger_from_env();
    let delay = Delay::new();
    assert!(TEST_DUTY > 0 && TEST_DUTY <= AUTHORIZED_MAX_DUTY && AUTHORIZED_MAX_DUTY <= 15);
    assert!(RUN_MS > 0 && RUN_MS < WATCHDOG_MS);
    let stage = selected_stage(
        cfg!(feature = "tb6612-live"),
        cfg!(feature = "tb6612-both"),
        MANUALLY_AUTHORIZED_STAGE,
    );
    let Some(stage) = stage else {
        warn!(
            "DRY: one-wheel LIVE requires tb6612-live, one source-selected stage and no tb6612-both; STBY LOW and PWM GPIOs LOW"
        );
        for stage in Stage::ALL {
            log_stage(stage);
        }
        idle(&delay);
    };

    warn!(
        "One-wheel direction test, A=LEFT B=RIGHT; no current or motion feedback; keep independent motor-power cutoff accessible."
    );
    let [(a1, a2), (b1, b2)] = log_stage(stage);
    let (mut serial, _tx) = UsbSerialJtag::new(p.USB_DEVICE).split();
    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(20),
        })
        .expect("TB6612 PWM timer");
    let mut a = ledc.channel(channel::Number::Channel0, pwma);
    let mut b = ledc.channel(channel::Number::Channel1, pwmb);
    for ch in [&mut a, &mut b] {
        ch.configure(channel::config::Config {
            timer: &timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("zero-duty PWM");
    }
    info!("Startup pause 5s; STBY LOW and both PWM duties zero");
    delay.delay_millis(5000);
    if !confirm(&mut serial, &delay) {
        warn!("Confirmation missing or invalid; no pulse, reset required");
        idle(&delay);
    }

    // Prepare only one channel while standby is still asserted.
    gates.a1.set_level(a1.into());
    gates.a2.set_level(a2.into());
    gates.b1.set_level(b1.into());
    gates.b2.set_level(b2.into());
    if a1 || a2 {
        a.set_duty(TEST_DUTY).expect("A duty");
    } else {
        b.set_duty(TEST_DUTY).expect("B duty");
    }
    delay.delay_micros(100);

    let mut watchdog = TimerGroup::new(p.TIMG1).wdt;
    watchdog.set_timeout(MwdtStage::Stage0, Duration::from_millis(WATCHDOG_MS));
    watchdog.enable();
    watchdog.feed();
    let start = Instant::now();
    gates.stby.set_high();
    while start.elapsed() < Duration::from_millis(RUN_MS) {
        core::hint::spin_loop();
    }
    gates.stop();
    let elapsed = start.elapsed().as_micros();
    a.set_duty(0).expect("final A off");
    b.set_duty(0).expect("final B off");
    watchdog.disable();
    info!(
        "{stage:?} stopped: STBY LOW PWM=0 elapsed_us={elapsed} (software timing, no wheel feedback)"
    );
    idle(&delay);
}
