#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_bimo::{
    board::Board,
    motors::{Disarmed, DualMotor, DutyLimit, tb6612::Tb6612},
};
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use log::{info, warn};
#[path = "motor_kick_diagnostic/sequence.rs"]
mod sequence;
use sequence::{Phase, Settings, apply};

esp_bootloader_esp_idf::esp_app_desc!();

// Candidate values only. Default ceiling deliberately blocks the proposed kick.
// Verify supply and suspended-wheel safety BEFORE explicitly raising the ceiling.
const TEST: Settings = Settings {
    ceiling: 70,
    kick_duty: 70,
    kick_ms: 150,
    hold_duty: 30,
    hold_ms: 1_000,
    repetitions: 1,
};
const PREPARATION_SECS: u64 = 3;
const TRIAL_PAUSE_MS: u64 = 2_000;
const MOTOR_PAUSE_MS: u64 = 3_000;
const LIVE: bool = cfg!(feature = "motor-kick-live"); // balance-live has no effect.

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    let mut motors = Tb6612::new(
        board.motor_pwm,
        board.motor_outputs,
        DutyLimit::new(TEST.ceiling),
    );
    motors.disarm();
    motors.set_command_logging(false);
    let _buzzer = board.buzzer;
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);

    info!(
        "KICK diagnostic: live opt-in={LIVE}; both motors disarmed; preparation={PREPARATION_SECS}s. Secure BOTH wheels; have motor-power cutoff ready."
    );
    info!(
        "Candidate kick={}%/{}ms; hold={}%/{}ms; ceiling={}%; repetitions={}; movement is NOT measured.",
        TEST.kick_duty, TEST.kick_ms, TEST.hold_duty, TEST.hold_ms, TEST.ceiling, TEST.repetitions
    );
    Timer::after_secs(PREPARATION_SECS).await;
    if !TEST.valid() {
        warn!(
            "TEST BLOCKED: invalid settings or duty above ceiling; edit source only after supply/safety verification. No automatic clamping."
        );
    } else {
        'tests: for (right, name) in [(true, "A / LEFT"), (false, "B / RIGHT")] {
            for trial in 1..=TEST.repetitions {
                for (phase, duration_ms) in TEST.phases() {
                    // Log kick BEFORE enabling; switch to hold/stop BEFORE logging.
                    if matches!(phase, Phase::Kick(_)) {
                        info!(
                            "{name}: trial={trial}; transition={phase:?}; duration={duration_ms}ms; live={LIVE}"
                        );
                    }
                    let deadline = Instant::now() + Duration::from_millis(duration_ms);
                    if apply(&mut motors, right, phase, TEST.ceiling, LIVE).is_err() {
                        warn!("Command rejected; aborting sequence.");
                        break 'tests;
                    }
                    if !matches!(phase, Phase::Kick(_)) {
                        info!(
                            "{name}: trial={trial}; transition={phase:?}; duration={duration_ms}ms; live={LIVE}"
                        );
                    }
                    if duration_ms != 0 {
                        Timer::at(deadline).await;
                    }
                }
                info!("{name}: STOPPED/disarmed; trial pause={TRIAL_PAUSE_MS}ms");
                Timer::after_millis(TRIAL_PAUSE_MS).await;
            }
            if right {
                info!("Both motors disabled; pause={MOTOR_PAUSE_MS}ms before B.");
                Timer::after_millis(MOTOR_PAUSE_MS).await;
            }
        }
    }
    let _ = apply(&mut motors, true, Phase::Stop, TEST.ceiling, LIVE);
    drop(motors);
    info!("FINISHED: all motor outputs LOW/disarmed; idle until reset.");
    loop {
        Timer::after_secs(1).await;
    }
}
