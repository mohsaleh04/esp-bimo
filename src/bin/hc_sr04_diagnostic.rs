#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_bimo::hc_sr04::{Outcome, TRIGGER_US, measure};
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    time::Instant,
};
use log::{info, warn};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    macro_rules! low {
        ($pin:expr) => {
            Output::new($pin, Level::Low, OutputConfig::default())
        };
    }
    let _stby = low!(esp_bimo::motor_gpio!(p, stby));
    let _motor_pins = (
        low!(esp_bimo::motor_gpio!(p, ain1)),
        low!(esp_bimo::motor_gpio!(p, ain2)),
        low!(esp_bimo::motor_gpio!(p, bin1)),
        low!(esp_bimo::motor_gpio!(p, bin2)),
        low!(esp_bimo::motor_gpio!(p, pwma)),
        low!(esp_bimo::motor_gpio!(p, pwmb)),
    );

    let mut trigger = low!(p.GPIO38);
    let echo = Input::new(p.GPIO47, InputConfig::default());
    let delay = Delay::new();
    esp_println::logger::init_logger_from_env();
    info!(
        "HC-SR04 diagnostic: TRIG GPIO38, ECHO GPIO47, STBY GPIO40 LOW, motors DISARMED, servo PWM disabled"
    );

    delay.delay_millis(300);
    let mut ping = 0u32;
    loop {
        ping = ping.wrapping_add(1);
        let report = measure(
            || (echo.is_high(), now_us()),
            || {
                trigger.set_high();
                let start_us = now_us();
                delay.delay_micros(TRIGGER_US);
                trigger.set_low();
                let end_us = now_us();
                (end_us, end_us - start_us)
            },
        );
        let initial_echo = if report.initial_echo_high {
            "HIGH"
        } else {
            "LOW"
        };
        info!(
            "HC-SR04 ping={ping} initial_echo={initial_echo} trigger_us={:?} wait_rise_us={:?} echo_high_us={:?}",
            report.trigger_us, report.wait_rise_us, report.echo_high_us,
        );
        match report.outcome {
            Outcome::DistanceMm(mm) => info!("HC-SR04 ping={ping} distance_mm={mm}"),
            Outcome::EchoStuckHigh => warn!("HC-SR04 ping={ping} reason=ECHO_STUCK_HIGH"),
            Outcome::WaitRiseTimeout => warn!("HC-SR04 ping={ping} reason=WAIT_RISE_TIMEOUT"),
            Outcome::WaitFallTimeout => warn!("HC-SR04 ping={ping} reason=WAIT_FALL_TIMEOUT"),
            Outcome::InvalidPulse => warn!("HC-SR04 ping={ping} reason=INVALID_PULSE"),
        }
        delay.delay_millis(300);
    }
}

fn now_us() -> u64 {
    Instant::now().duration_since_epoch().as_micros()
}
