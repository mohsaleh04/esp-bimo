#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig, interconnect::PeripheralOutput},
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

struct TestOutputs {
    ena: Output<'static>,
    enb: Output<'static>,
    in1: Output<'static>,
    in2: Output<'static>,
    in3: Output<'static>,
    in4: Output<'static>,
}

impl TestOutputs {
    fn stop(&mut self) {
        // Disable bridges before clearing direction inputs.
        self.ena.set_low();
        self.enb.set_low();
        self.in1.set_low();
        self.in2.set_low();
        self.in3.set_low();
        self.in4.set_low();
    }
}

impl Drop for TestOutputs {
    fn drop(&mut self) {
        self.stop();
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    // Own each pin directly from initialization through permanent lockout.
    // No Board/L298n/MCPWM initialization or pin handoff in this diagnostic.
    let mut motors = TestOutputs {
        ena: Output::new(
            esp_bimo::motor_gpio!(p, pwma),
            Level::Low,
            OutputConfig::default(),
        ),
        enb: Output::new(
            esp_bimo::motor_gpio!(p, pwmb),
            Level::Low,
            OutputConfig::default(),
        ),
        in1: Output::new(
            esp_bimo::motor_gpio!(p, ain1),
            Level::Low,
            OutputConfig::default(),
        ),
        in2: Output::new(
            esp_bimo::motor_gpio!(p, ain2),
            Level::Low,
            OutputConfig::default(),
        ),
        in3: Output::new(
            esp_bimo::motor_gpio!(p, bin1),
            Level::Low,
            OutputConfig::default(),
        ),
        in4: Output::new(
            esp_bimo::motor_gpio!(p, bin2),
            Level::Low,
            OutputConfig::default(),
        ),
    };
    motors.ena.disconnect_from_peripheral_output();
    motors.enb.disconnect_from_peripheral_output();
    // Preserve the active-low buzzer's inactive level.
    let _buzzer = Output::new(p.GPIO21, Level::High, OutputConfig::default());
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0, p.FROM_CPU_INTR0);

    if !esp_bimo::motor_gpio::LEGACY_MOTOR_PROBES_LIVE {
        log::warn!("DRY: legacy motor probe locked out during TB6612 wiring verification");
        loop {
            Timer::after_secs(1).await;
        }
    }

    info!("STATIC MOTOR TEST: waiting 3 seconds; all six outputs LOW.");
    Timer::after_secs(3).await;
    info!(
        "STATIC MOTOR TEST: starting B (RIGHT wheel), ENB/IN3 HIGH, IN4 LOW; A disabled; 250 ms target."
    );
    motors.in3.set_high();
    // Software timing, not a hardware cutoff against CPU failure or
    // excessive interrupt latency. Set direction before enabling the bridge.
    let deadline = Instant::now() + Duration::from_millis(1000);
    motors.enb.set_high();
    while Instant::now() < deadline {
        core::hint::spin_loop();
    }
    // No logging or awaits until all six pins are LOW again.
    motors.stop();
    info!("STATIC MOTOR TEST: stopped; all six outputs LOW.");
    info!("STATIC MOTOR TEST: finished; no repeat until reset.");
    loop {
        Timer::after_secs(1).await;
    }
}
