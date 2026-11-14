#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_bimo::{balance, motors, orientation, pwm_mapping};
#[path = "motor_response_runtime/mod.rs"]
mod runtime;
esp_bootloader_esp_idf::esp_app_desc!();

// Existing operator verification values are retained; the new rolling mode is unreviewed/off.
const PHYSICAL_RESTRAINT_VERIFIED: bool = true;
const INDEPENDENT_POWER_CUTOFF_VERIFIED: bool = true;
const SUPPLY_LIMITS_VERIFIED: bool = true;
const SENSOR_FAULT_REVIEWED: bool = true;
const RESTART_ASSIST_REVIEWED: bool = true;
const ROLLING_ASSIST_REVIEWED: bool = true;
const WINDOW_US: u64 = 30_000_000;
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let selection = runtime::output::select_policy(
        cfg!(feature = "balance-control-restart-assist"),
        RESTART_ASSIST_REVIEWED,
        cfg!(feature = "balance-control-rolling-assist"),
        ROLLING_ASSIST_REVIEWED,
    );
    let live = runtime::ground::live_allowed(
        cfg!(feature = "balance-control-live"),
        PHYSICAL_RESTRAINT_VERIFIED,
        INDEPENDENT_POWER_CUTOFF_VERIFIED,
        SUPPLY_LIMITS_VERIFIED,
        SENSOR_FAULT_REVIEWED,
    );
    runtime::run_balance_control_policy(
        runtime::RunConfig {
            label: if selection.is_some() {
                "BOUNDED BALANCE CONTROL; INDEPENDENT PHYSICAL RESTRAINT REQUIRED"
            } else {
                "POLICY SELECTION BLOCKED/UNREVIEWED; DRY ONLY"
            },
            live: live && selection.is_some(),
            window_us: WINDOW_US,
            ground: true,
            preparation_secs: 30,
        },
        selection.unwrap_or(runtime::output::AssistPolicy::Legacy),
    )
    .await
}
