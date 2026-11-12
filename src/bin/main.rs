#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_bimo::{board::Board, sensors::bmi160::Bmi160};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use log::{info, warn};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let board = Board::init(peripherals);
    let _inactive_outputs = board.inactive_outputs;
    let _bmi160_int1 = board.bmi160_int1;
    let _qmc5883l_drdy = board.qmc5883l_drdy;
    let mut bus = board.i2c;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(board.timer_group);
    esp_rtos::start(timg0.timer0, board.rtos_interrupt);

    spawner.spawn(esp_bimo::buzzer::play(board.ledc, board.buzzer).expect("buzzer task"));

    info!("Bimo milestone 1: motor enables/directions LOW; servo unused");
    // --- Founding I2C Devices
    info!("[I2C] I2C0: SDA=14 SCL=13, 100 kHz; Scanning from 0x08 to 0x77");
    Timer::after_millis(100).await;
    let mut found = 0;
    for address in 0x08u8..=0x77 {
        match bus.write(address, &[]) {
            Ok(()) => {
                found += 1;
                info!("I2C ACK at 0x{address:02X}");
            }
            Err(esp_hal::i2c::master::Error::AcknowledgeCheckFailed(_)) => {}
            Err(error) => warn!("I2C probe 0x{address:02X}: {error:?}"),
        }
        Timer::after_millis(1).await;
    }
    info!("[I2C] Scan complete: {found} responding address(es)");
    info!(
        "[I2C] Future sensors: QMC5883L=0x{:02X}, AHT20=0x{:02X} (not initialized)",
        esp_bimo::board::QMC5883L_ADDRESS,
        esp_bimo::board::AHT20_ADDRESS
    );

    // Setup BMI160
    let mut imu = None;
    for address in [0x68, 0x69] {
        match Bmi160::init(&mut bus, address).await {
            Ok(sensor) => {
                info!("[GYRO] ready at 0x{address:02X}: ±2 g, ±2000 deg/s; INT1=41, polling");
                imu = Some(sensor);
                break;
            }
            Err(error) => warn!("[GYRO] init 0x{address:02X}: {error:?}"),
        }
    }

    loop {
        if _bmi160_int1.is_high() && let Some(imu) = imu.as_mut() {
            match imu.read(&mut bus) {
                Ok(Some(sample)) => {
                    let a = sample.accel_m_s2();
                    let g = sample.gyro_rad_s();
                    info!(
                        "[GYRO] ACCEL [{:.3}, {:.3}, {:.3}] m/s²; GYRO [{:.3}, {:.3}, {:.3}] rad/s",
                        a[0], a[1], a[2], g[0], g[1], g[2]
                    );
                }
                Ok(None) => {
                    warn!("[GYRO]: accel/gyro data not ready");
                }
                Err(error) => {
                    warn!("[GYRO] read: {error:?}");
                }
            }
        }
        Timer::after_micros(1).await;
    }
}
