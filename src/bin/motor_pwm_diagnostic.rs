#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{
    board::Board,
    motors::{DualMotor, DutyLimit, l298n::L298n},
};
use esp_hal::{
    clock::CpuClock,
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};
use log::{info, warn};

esp_bootloader_esp_idf::esp_app_desc!();

// Set each running-start duty to a value YOU verified spins that suspended wheel.
// 30% is a conservative placeholder, not a verified startup threshold.
const STARTUP_DELAY_SECS: u64 = 15;
const MAX_DUTY_PERCENT: u8 = 100;
const RUNNING_START_A_PERCENT: u8 = 100;
const RUNNING_START_B_PERCENT: u8 = 100;
const MIN_DUTY_PERCENT: u8 = 5;
const DUTY_STEP_PERCENT: u8 = 5;
const RUNNING_STEP_MS: u64 = 1_500;
const STARTING_PULSE_MS: u64 = 1_000;
const STOP_PAUSE_MS: u64 = 2_000;
const BETWEEN_MOTORS_MS: u64 = 3_000;

// The other channel is always commanded zero; confirmed forward polarities.
fn command(motors: &mut L298n, right: bool, duty: u8) -> bool {
    let duty = i16::from(duty);
    let (a, b) = if right { (-duty, 0) } else { (0, duty) };
    motors.command(a, b).is_ok()
}

async fn test_motor(motors: &mut L298n, name: &str, right: bool, start: u8) -> bool {
    info!("{name}: RUNNING test; start={start}%; must already be verified to spin this wheel.");
    motors.arm();
    let mut duty = start;
    loop {
        info!(
            "{name}: RUNNING duty={duty}%; hold={RUNNING_STEP_MS}ms; observe when rotation stops."
        );
        if !command(motors, right, duty) {
            motors.disarm();
            return false;
        }
        Timer::after_millis(RUNNING_STEP_MS).await;
        if duty == MIN_DUTY_PERCENT {
            break;
        }
        duty -= DUTY_STEP_PERCENT;
    }
    motors.disarm();
    info!("{name}: RUNNING complete; both motors OFF; coast pause={STOP_PAUSE_MS}ms.");
    Timer::after_millis(STOP_PAUSE_MS).await;

    // Each starting attempt begins from disabled outputs, without a kick/boost.
    let mut duty = MIN_DUTY_PERCENT;
    loop {
        info!(
            "{name}: STARTING attempt duty={duty}%; pulse={STARTING_PULSE_MS}ms; observe whether rotation begins."
        );
        motors.arm();
        let accepted = command(motors, right, duty);
        if accepted {
            // No logging during the short starting pulse. CPU hangs can delay stop.
            let deadline = Instant::now() + Duration::from_millis(STARTING_PULSE_MS);
            while Instant::now() < deadline {
                core::hint::spin_loop();
            }
        }
        motors.disarm();
        info!("{name}: STOP; both motors LOW/disarmed; coast pause={STOP_PAUSE_MS}ms.");
        if !accepted {
            return false;
        }
        Timer::after_millis(STOP_PAUSE_MS).await;
        if duty == MAX_DUTY_PERCENT {
            break;
        }
        duty += DUTY_STEP_PERCENT;
    }
    true
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p); // Enables and direction outputs LOW first.
    let limit = DutyLimit::new(MAX_DUTY_PERCENT);
    let mut motors = L298n::new(board.motor_pwm, board.inactive_outputs, limit);
    motors.disarm();
    motors.set_command_logging(false); // Use explicit phase/duty logs below; no per-command driver telemetry.
    let _buzzer = board.buzzer; // Keep active-low buzzer inactive.
    const {
        assert!(MIN_DUTY_PERCENT > 0 && MIN_DUTY_PERCENT <= MAX_DUTY_PERCENT);
        assert!(MAX_DUTY_PERCENT <= 100 && DUTY_STEP_PERCENT > 0);
        assert!((MAX_DUTY_PERCENT - MIN_DUTY_PERCENT).is_multiple_of(DUTY_STEP_PERCENT));
        assert!(
            RUNNING_START_A_PERCENT >= MIN_DUTY_PERCENT
                && RUNNING_START_A_PERCENT <= MAX_DUTY_PERCENT
        );
        assert!(
            RUNNING_START_B_PERCENT >= MIN_DUTY_PERCENT
                && RUNNING_START_B_PERCENT <= MAX_DUTY_PERCENT
        );
        assert!((RUNNING_START_A_PERCENT - MIN_DUTY_PERCENT).is_multiple_of(DUTY_STEP_PERCENT));
        assert!((RUNNING_START_B_PERCENT - MIN_DUTY_PERCENT).is_multiple_of(DUTY_STEP_PERCENT));
        assert!(STARTING_PULSE_MS > 0 && STARTING_PULSE_MS <= 2_000);
        assert!(STOP_PAUSE_MS >= 1_000);
    };
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);

    info!(
        "PWM running/starting diagnostic: motors OFF; secure/suspend BOTH wheels; waiting {STARTUP_DELAY_SECS}s."
    );
    info!(
        "No encoder feedback: observe movement yourself. Ceiling={MAX_DUTY_PERCENT}%; no boost; bounded configured sequence only."
    );
    Timer::after_secs(STARTUP_DELAY_SECS).await;

    if test_motor(&mut motors, "A / RIGHT", true, RUNNING_START_A_PERCENT).await {
        motors.disarm();
        info!("A complete; both motors disabled for {BETWEEN_MOTORS_MS}ms before B.");
        Timer::after_millis(BETWEEN_MOTORS_MS).await;
        if !test_motor(&mut motors, "B / LEFT", false, RUNNING_START_B_PERCENT).await {
            warn!("B command rejected; diagnostic aborted.");
        }
    } else {
        warn!("A command rejected; diagnostic aborted; B skipped.");
    }
    // Unconditional final shutdown, also on a rejected command.
    motors.stop();
    motors.disarm();
    drop(motors);
    info!("FINISHED: all outputs LOW/disarmed; no more pulses until reset.");
    loop {
        Timer::after_secs(1).await;
    }
}
