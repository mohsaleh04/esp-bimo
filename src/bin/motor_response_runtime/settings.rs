//! Shared response settings: firmware and host replay use the same values.
use crate::pwm_mapping::{Config, KickConfig};
pub const MOTOR_DUTY_CEILING: u8 = 70;
pub const ARMING_TIMEOUT_US: u64 = 10_000_000; // After preparation; includes readiness + neutral wait.
pub const MAX_LOOP_GAP_US: u64 = 30_000;
pub const PD_OUTPUT_LIMIT: i16 = 15; // Independent of ordinary PWM maximum.
pub const MAPPING: Config = Config {
    enabled: true,
    deadband: 2.0,
    pd_output_limit: PD_OUTPUT_LIMIT as f32,
    minimum_duty: 30.0,
    maximum_duty: 50.0,
    slew_per_second: None,
};
pub const KICK: KickConfig = KickConfig {
    enabled: true,
    kick_duty: 70.0,
    kick_duration_ms: 150,
    trigger_threshold: 4.0,
    rearm_delay_ms: 300,
    reversal_pause_ms: 100,
};
