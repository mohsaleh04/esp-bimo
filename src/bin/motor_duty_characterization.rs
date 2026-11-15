#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_bimo::{
    balance,
    board::Board,
    motors::{Disarmed, DualMotor, DutyLimit, l298n::L298n},
};
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use log::{info, warn};
#[path = "motor_duty_characterization/sequence.rs"]
mod sequence;
use sequence::{CEILING, ON_MS, PAUSE_MS, PREPARATION_MS, TOTAL_MS, apply, shutdown, step};
esp_bootloader_esp_idf::esp_app_desc!();

// Verify restraint, independent motor-power cutoff, and supply/current limits first.
const MOTOR_POWER_AND_RESTRAINT_VERIFIED: bool = true;
const LIVE: bool = cfg!(feature = "motor-kick-live") && MOTOR_POWER_AND_RESTRAINT_VERIFIED;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    let mut motors = L298n::new(
        board.motor_pwm,
        board.inactive_outputs,
        DutyLimit::new(CEILING),
    );
    shutdown(&mut motors);
    motors.set_command_logging(false);
    let _buzzer = board.buzzer;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);
    let start = Instant::now();
    let overall_end = start + Duration::from_millis(TOTAL_MS);
    info!(
        "DUTY CHARACTERIZATION live={LIVE}; A=LEFT B=RIGHT; ceiling={CEILING}%; preparation={PREPARATION_MS}ms; timeout={TOTAL_MS}ms; no Kick/balance/assist."
    );
    info!(
        "Secure robot and BOTH wheels; preattach insulated voltage probes with motor power OFF. Independent power cutoff required. No movement/current feedback."
    );
    Timer::after_millis(PREPARATION_MS).await;
    let mut completed = 0;
    for index in 0..6 {
        if Instant::now() >= overall_end {
            break;
        }
        let (forward, duty) = step(index).unwrap();
        let u = if forward {
            i16::from(duty)
        } else {
            -i16::from(duty)
        };
        let (a, b) = balance::wheel_commands(u);
        info!(
            "STEP {}/6: BOTH MOTORS {} duty={duty}% requested A/LEFT={a}% B/RIGHT={b}% on={ON_MS}ms live={LIVE}",
            index + 1,
            if forward { "FORWARD" } else { "BACKWARD" }
        );
        // Serial work finishes BEFORE power-on. Nothing prints while energized.
        let now = Instant::now();
        if now >= overall_end {
            break;
        }
        let end = (now + Duration::from_millis(ON_MS)).min(overall_end);
        if apply(&mut motors, index, LIVE, (now - start).as_millis()).is_err() {
            shutdown(&mut motors);
            warn!("Command rejected; permanently aborting diagnostic.");
            break;
        }
        Timer::at(end).await;
        shutdown(&mut motors); // Actual zero/disarm BEFORE logging or pauses.
        completed += 1;
        info!(
            "STOP {}/6: driver-commanded pulse A/LEFT={}% B/RIGHT={}%; NOW A=0 B=0 disarmed; pause={PAUSE_MS}ms. Wheel movement NOT measured.",
            index + 1,
            if LIVE { a } else { 0 },
            if LIVE { b } else { 0 }
        );
        Timer::at((Instant::now() + Duration::from_millis(PAUSE_MS)).min(overall_end)).await;
    }
    shutdown(&mut motors);
    drop(motors);
    info!(
        "FINISHED: {completed}/6 stages completed; BOTH outputs LOW/disarmed; no repeat until reset."
    );
    loop {
        Timer::after_secs(1).await;
    }
}
