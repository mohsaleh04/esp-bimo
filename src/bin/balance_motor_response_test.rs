#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_bimo::{balance, motors, orientation, pwm_mapping};
#[path = "motor_response_runtime/mod.rs"]
mod runtime;
esp_bootloader_esp_idf::esp_app_desc!();
// Existing suspended-wheel configuration is unchanged.
const SUPPLY_AND_CEILING_VERIFIED: bool = true;
const WINDOW_US: u64 = 30_000_000;
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    runtime::run(runtime::RunConfig {
        label: "SUSPENDED WHEELS ONLY",
        live: cfg!(feature = "balance-motor-response-live") && SUPPLY_AND_CEILING_VERIFIED,
        window_us: WINDOW_US,
        ground: false,
        preparation_secs: 15,
    })
    .await
}
