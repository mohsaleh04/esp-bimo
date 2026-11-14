#[path = "../../balance_control_replay.rs"]
mod replay;
use replay::{balance, motors, pwm_mapping};
fn main() {
    if std::env::args().any(|arg| arg == "--rolling-assist") {
        replay::run_rolling_assist();
    } else if std::env::args().any(|arg| arg == "--integrated-assist") {
        replay::run_integrated_assist();
    } else if std::env::args().any(|arg| arg == "--restart-assist") {
        replay::run_restart_assist();
    } else {
        replay::run();
    }
}
