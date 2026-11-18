#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self, TimerIFace},
    },
    time::Rate,
};
use log::{info, warn};
esp_bootloader_esp_idf::esp_app_desc!();

use esp_bimo::head_servo::{
    CENTER_DEG, LEFT_SIGN, MAX_PULSE_US, MIN_PULSE_US, PERIOD_US, pulse_us,
};
const LIMITED_OFFSET_DEG: i32 = 10;
const FULL_OFFSET_DEG: i32 = 90;
const STEP_MS: u32 = 20;
const SETTLE_MS: u32 = 2000;

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let _stby = Output::new(
        esp_bimo::motor_gpio!(p, stby),
        Level::Low,
        OutputConfig::default(),
    );
    macro_rules! low {
        ($p:expr) => {
            Output::new($p, Level::Low, OutputConfig::default())
        };
    }
    let _wheel_pins = (
        low!(esp_bimo::motor_gpio!(p, ain1)),
        low!(esp_bimo::motor_gpio!(p, ain2)),
        low!(esp_bimo::motor_gpio!(p, bin1)),
        low!(esp_bimo::motor_gpio!(p, bin2)),
        low!(esp_bimo::motor_gpio!(p, pwma)),
        low!(esp_bimo::motor_gpio!(p, pwmb)),
    );
    esp_println::logger::init_logger_from_env();
    warn!(
        "SERVO ONLY: wheel motors DISARMED; STBY LOW. Assumes position-controlled servo; model/pulse range NOT physically verified."
    );
    assert!(LEFT_SIGN == -1 || LEFT_SIGN == 1);
    assert!(MIN_PULSE_US > 0 && MIN_PULSE_US < MAX_PULSE_US && MAX_PULSE_US < PERIOD_US);
    let full = cfg!(feature = "servo-full-sweep");
    let offset = if full {
        FULL_OFFSET_DEG
    } else {
        LIMITED_OFFSET_DEG
    };
    if full {
        warn!(
            "FULL SWEEP opted in: verify safe mechanical limits! Logical degrees are not calibrated physical degrees."
        );
    } else {
        info!(
            "LIMITED test: center +/-10 logical degrees; requested full 90-degree sweep WILL NOT run."
        );
    }
    let delay = Delay::new();
    // Allow time to observe the startup message before applying the first pulse.
    delay.delay_millis(3000);
    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .expect("servo 50Hz timer");
    let mut servo = ledc.channel(channel::Number::Channel0, low!(p.GPIO18));
    servo
        .configure(channel::config::Config {
            timer: &timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("servo channel");
    let apply = |angle| {
        // Raw 14-bit duty avoids the coarse whole-percent duty API (~200us).
        servo.set_duty_hw((pulse_us(angle) * (1 << 14) + PERIOD_US / 2) / PERIOD_US);
    };
    let mut current = CENTER_DEG;
    info!(
        "Initial center target={current} pulse_us={}; actual starting position unknown",
        pulse_us(current)
    );
    apply(current);
    delay.delay_millis(SETTLE_MS);
    // One pass only. All intermediate returns to center have a 2s dwell.
    for (label, target) in [
        ("left", CENTER_DEG + LEFT_SIGN * offset),
        ("center", CENTER_DEG),
        ("right", CENTER_DEG - LEFT_SIGN * offset),
        ("final center", CENTER_DEG),
    ] {
        info!(
            "Stage {label}: target={target} logical_deg pulse_us={}",
            pulse_us(target)
        );
        while current != target {
            current += if target > current { 1 } else { -1 };
            apply(current);
            delay.delay_millis(STEP_MS);
        }
        info!("Stage {label}: target commanded; holding 2s (no position feedback)");
        delay.delay_millis(SETTLE_MS);
    }
    servo.set_duty(0).expect("disable servo pulses");
    info!(
        "DONE: PWM disabled (signal LOW); idle forever, no repeat; holding torque not guaranteed."
    );
    loop {
        delay.delay_millis(1000);
    }
}
