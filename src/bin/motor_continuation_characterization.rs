#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_bimo::{
    balance,
    board::Board,
    motors::{Disarmed, DualMotor, DutyLimit, tb6612::Tb6612},
};
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use log::{info, warn};
#[path = "motor_continuation_characterization/sequence.rs"]
mod sequence;
use sequence::{
    CEILING, HOLD_MS, PAUSE_MS, PREPARATION_MS, Phase, STARTUP_MS, TOTAL_MS, apply, shutdown, step,
};
esp_bootloader_esp_idf::esp_app_desc!();
// Verify shared supply/current limits, physical restraint and independent power cutoff.
const MOTOR_POWER_AND_RESTRAINT_VERIFIED: bool = false;
const LIVE: bool = cfg!(feature = "motor-kick-live") && MOTOR_POWER_AND_RESTRAINT_VERIFIED;
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    let mut motors = Tb6612::new(
        board.motor_pwm,
        board.motor_outputs,
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
        "CONTINUATION live={LIVE}; BOTH motors; A=LEFT B=RIGHT; preparation={PREPARATION_MS}ms; timeout={TOTAL_MS}ms; no balance/PD/Rolling Assist. Independent power cutoff required."
    );
    Timer::after_millis(PREPARATION_MS).await;
    let mut completed = 0;
    for index in 0..6 {
        if Instant::now() >= overall_end {
            break;
        }
        let (forward, startup, hold) = step(index).unwrap();
        let direction = if forward { "FORWARD" } else { "BACKWARD" };
        info!(
            "STAGE {}/6 {direction}: startup={startup}%/{STARTUP_MS}ms then hold={hold}%/{HOLD_MS}ms; live={LIVE}. Phase reports follow STOP.",
            index + 1
        );
        let now = Instant::now();
        if now >= overall_end {
            break;
        }
        let startup_end = (now + Duration::from_millis(STARTUP_MS)).min(overall_end);
        let Ok(start_outputs) = apply(
            &mut motors,
            index,
            Phase::Startup,
            LIVE,
            (now - start).as_millis(),
        ) else {
            shutdown(&mut motors);
            warn!("Startup rejected; permanently aborting.");
            break;
        };
        Timer::at(startup_end).await;
        let hold_start = Instant::now();
        // Timeout aborts before hold; never extend the global test window.
        let hold_outputs = apply(
            &mut motors,
            index,
            Phase::Hold,
            LIVE,
            (hold_start - start).as_millis(),
        );
        let hold_end = (hold_start + Duration::from_millis(HOLD_MS)).min(overall_end);
        if hold_outputs.is_ok() && hold_start < overall_end {
            Timer::at(hold_end).await;
        }
        shutdown(&mut motors); // BEFORE all serial work and pauses.
        let stopped = Instant::now();
        info!(
            "STAGE {}/6 {direction} STARTUP requested={startup}%; driver A/LEFT={} B/RIGHT={}; elapsed={}us",
            index + 1,
            start_outputs.0,
            start_outputs.1,
            (hold_start - now).as_micros()
        );
        match hold_outputs {
            Ok((a, b)) => info!(
                "STAGE {}/6 {direction} HOLD requested={hold}%; driver A/LEFT={a} B/RIGHT={b}; elapsed={}us",
                index + 1,
                (stopped - hold_start).as_micros()
            ),
            Err(_) => warn!("HOLD rejected/timeout; outputs already zero; permanently aborting."),
        }
        info!(
            "STOP {}/6: A=0 B=0 disarmed; pause={PAUSE_MS}ms; movement NOT measured.",
            index + 1
        );
        if hold_outputs.is_err() || stopped >= overall_end {
            break;
        }
        completed += 1;
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
