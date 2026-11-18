#![no_std]
#![no_main]
use esp_backtrace as _;
use esp_bimo::motors::control::{Channel, Direction};
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    time::{Duration, Instant, Rate},
};
use log::{info, warn};
esp_bootloader_esp_idf::esp_app_desc!();
// Raising this requires renewed physical-test authorization; no automatic ramp.
const TEST_DUTY: u8 = 15;
const AUTHORIZED_MAX_DUTY: u8 = 15;
const RUN_MS: u64 = 250; // Margin below the 300ms software limit.
const PAUSE_MS: u32 = 2000;
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
    assert!(RUN_MS > 0 && RUN_MS <= 300);
    let live = cfg!(feature = "tb6612-live");
    warn!(
        "TB6612 mode={} A=LEFT B=RIGHT; STBY=LOW PWM=0; no motion feedback",
        if live { "LIVE" } else { "DRY" }
    );
    warn!(
        "Requires hardware STBY pull-down and external power cutoff; USB disconnect/CPU halt is NOT a stop mechanism"
    );
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
    info!("Startup pause 5s, STBY LOW; support wheels and prepare motor power cutoff");
    delay.delay_millis(5000);
    let stages = [
        ("LEFT +", true, false, true),
        ("LEFT -", true, false, false),
        ("RIGHT +", false, true, true),
        ("RIGHT -", false, true, false),
        ("BOTH +", true, true, true),
        ("BOTH -", true, true, false),
    ];
    for (index, (label, use_a, use_b, forward)) in stages.into_iter().enumerate() {
        if index >= 4 && !cfg!(feature = "tb6612-both") {
            continue;
        }
        gates.stop();
        a.set_duty(0).expect("stop A");
        b.set_duty(0).expect("stop B");
        let direction = if forward {
            Direction::Positive
        } else {
            Direction::Negative
        };
        let a_inputs =
            Channel::A.direction_inputs(if use_a { direction } else { Direction::Stopped });
        let b_inputs =
            Channel::B.direction_inputs(if use_b { direction } else { Direction::Stopped });
        info!(
            "Stage {} {label}: requested A={} B={}%, logical direction={:?}; AIN={a_inputs:?} BIN={b_inputs:?}, duration={RUN_MS}ms; STBY currently LOW, planned={}",
            index + 1,
            if use_a { TEST_DUTY } else { 0 },
            if use_b { TEST_DUTY } else { 0 },
            direction,
            if live {
                "HIGH during pulse"
            } else {
                "LOW throughout DRY"
            }
        );
        if live {
            gates.a1.set_level(a_inputs.0.into());
            gates.a2.set_level(a_inputs.1.into());
            gates.b1.set_level(b_inputs.0.into());
            gates.b2.set_level(b_inputs.1.into());
            a.set_duty(if use_a { TEST_DUTY } else { 0 })
                .expect("A duty");
            b.set_duty(if use_b { TEST_DUTY } else { 0 })
                .expect("B duty");
            delay.delay_micros(100); // PWM settled while STBY LOW.
            let start = Instant::now();
            gates.stby.set_high();
            while start.elapsed() < Duration::from_millis(RUN_MS) {
                core::hint::spin_loop();
            }
            gates.stop();
            a.set_duty(0).expect("stop A");
            b.set_duty(0).expect("stop B");
            info!(
                "Stage {} stopped: STBY LOW PWM=0 elapsed_us={} (software measurement, no wheel feedback)",
                index + 1,
                start.elapsed().as_micros()
            );
        } else {
            info!("DRY: stage skipped; STBY LOW PWM=0");
        }
        delay.delay_millis(PAUSE_MS);
    }
    gates.stop();
    a.set_duty(0).expect("final A off");
    b.set_duty(0).expect("final B off");
    info!("DONE: STBY LOW PWM=0; idle, no repeats until reset");
    loop {
        delay.delay_millis(1000);
    }
}
