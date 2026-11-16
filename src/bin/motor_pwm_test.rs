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
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let board = Board::init(p);
    let limit = DutyLimit::new(100);
    let effective = limit.map(100);
    assert_eq!(effective.duty_percent, 100, "100% must not be clamped");
    assert_eq!(effective.direction.inputs(), (true, false)); // IN1 HIGH, IN2 LOW
    let mut motors = L298n::new(board.motor_pwm, board.motor_outputs, limit);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);

    info!("PWM DIAGNOSTIC: waiting 3 seconds; both motors disarmed.");
    Timer::after_secs(3).await;
    // esp-hal 1.2.2 UP_ACTIVE_HIGH sets HIGH at zero and LOW at compare.
    // Period=999 counts 0..=999; set_timestamp(1000) is written without
    // clamping, so the LOW compare event never occurs: constant HIGH after TEZ.
    // command() logs actual period/compare readback BEFORE enabling ENA.
    info!(
        "PWM DIAGNOSTIC: A=RIGHT GPIO16, requested=100%, effective={}%; IN1 HIGH/IN2 LOW; B disabled; 149 ms target (<150 ms).",
        effective.duty_percent,
    );
    motors.arm();
    let started = motors.command(-100, 100).is_ok();
    if started {
        // No logging or blocking peripheral I/O while enabled. Software bound
        // only: a CPU/interrupt stall can delay shutdown beyond the deadline.
        // Leave 1 ms margin below the 150 ms maximum under normal execution.
        let deadline = Instant::now() + Duration::from_millis(60000);
        while Instant::now() < deadline {
            core::hint::spin_loop();
        }
    }
    motors.disarm();
    drop(motors);
    info!("PWM DIAGNOSTIC: stopped/disarmed; all six motor outputs LOW.");
    info!("PWM DIAGNOSTIC: finished (started={started}); locked out until reset.");
    loop {
        Timer::after_secs(1).await;
    }
}
