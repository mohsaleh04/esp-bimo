#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_bimo::{balance, motors, orientation, pwm_mapping};
#[path = "motor_response_runtime/mod.rs"]
mod runtime;
esp_bootloader_esp_idf::esp_app_desc!();

// All remain FALSE until the operator has completed the documented procedure.
const PHYSICAL_RESTRAINT_VERIFIED: bool = false;
const INDEPENDENT_POWER_CUTOFF_VERIFIED: bool = false;
const SUPPLY_LIMITS_VERIFIED: bool = false;
const SENSOR_FAULT_REVIEWED: bool = false;
const WINDOW_US: u64 = 1_000_000;
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let source_verified = runtime::ground::live_allowed(
        true,
        PHYSICAL_RESTRAINT_VERIFIED,
        INDEPENDENT_POWER_CUTOFF_VERIFIED,
        SUPPLY_LIMITS_VERIFIED,
        SENSOR_FAULT_REVIEWED,
    );
    runtime::run(runtime::RunConfig {
        label: "TETHERED GROUND CONTACT ONLY; BODY PHYSICALLY SUPPORTED",
        live_requested: cfg!(feature = "balance-ground-contact-live"),
        live_prerequisites_met: source_verified,
        window_us: WINDOW_US,
        ground: true,
        preparation_secs: 30,
    })
    .await
}
