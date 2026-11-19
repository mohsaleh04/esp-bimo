//! One-shot calculation of TB6612 direction and duty with motor outputs held LOW.
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_bimo::{
    board::Board,
    motors::control::{Channel, DutyLimit},
};
use esp_hal::delay::Delay;
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

// Illustrative values for the log, independent of any motor test setting.
const PREVIEW_REQUEST: i16 = 20;
const PREVIEW_LIMIT: DutyLimit = DutyLimit::new(15);

fn log_stop() {
    info!(
        "[STOP] hypothetical A=0% B=0%; commanded STBY=LOW PWM_A=0% PWM_B=0% AIN=LOW/LOW BIN=LOW/LOW"
    );
}

#[esp_hal::main]
fn main() -> ! {
    let board = Board::init(esp_hal::init(esp_hal::Config::default()));
    // Board initializes STBY first, then both PWM and all four direction pins LOW.
    // Retain ownership. No PWM peripheral or motor driver is initialized.
    let _inactive_outputs = board.motor_outputs;
    esp_println::logger::init_logger_from_env();

    info!("[DRY ONLY] A=LEFT B=RIGHT; all motor GPIOs commanded LOW; no feedback");
    log_stop();

    for (label, requested) in [
        ("A LEFT FORWARD", [PREVIEW_REQUEST, 0]),
        ("A LEFT REVERSE", [-PREVIEW_REQUEST, 0]),
        ("B RIGHT FORWARD", [0, PREVIEW_REQUEST]),
        ("B RIGHT REVERSE", [0, -PREVIEW_REQUEST]),
    ] {
        let [a, b] = requested.map(|command| PREVIEW_LIMIT.map(command));
        let ain = Channel::A.direction_inputs(a.direction);
        let bin = Channel::B.direction_inputs(b.direction);
        info!(
            "[PREVIEW {label}] request A={} B={}%; hypothetical effective A={:?}/{}% B={:?}/{}% AIN={ain:?} BIN={bin:?}; commanded STBY=LOW PWM_A=0% PWM_B=0% AIN=LOW/LOW BIN=LOW/LOW",
            requested[0], requested[1], a.direction, a.duty_percent, b.direction, b.duty_percent,
        );
        log_stop();
    }

    info!("[DONE] DRY completed; GPIOs remain commanded LOW until reset");
    let delay = Delay::new();
    loop {
        delay.delay_millis(1000);
    }
}
