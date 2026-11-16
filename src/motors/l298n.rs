//! L298N enable PWM via MCPWM0, independent of the buzzer's LEDC timers.
use esp_hal::{
    delay::Delay,
    gpio::{NoPin, Output, interconnect::PeripheralOutput},
    mcpwm::{
        McPwm, PeripheralClockConfig, PwmPeripheral,
        operator::{PwmPin, PwmPinConfig, PwmUpdateMethod},
        timer::{PwmWorkingMode, Timer},
    },
    peripherals::MCPWM0,
    time::Rate,
};

use super::control::{CommandGate, Direction};
use super::{Disarmed, DualMotor, DutyLimit};
use crate::board::MotorOutputs;

/// Owns all six original GPIO drivers; enable pins are never surrendered to PWM.
/// A/B correspond to ENA/ENB, with no wheel or physical direction assumptions.
pub struct L298n {
    pins: MotorOutputs,
    a: PwmPin<'static, MCPWM0<'static>, 0, true>,
    b: PwmPin<'static, MCPWM0<'static>, 0, false>,
    timer: Timer<0, MCPWM0<'static>>,
    limit: DutyLimit,
    gate: CommandGate,
    command_logging: bool,
}

impl L298n {
    /// Fixed 10 kHz / 1000-count PWM. Starts stopped and disarmed.
    pub fn new(peripheral: MCPWM0<'static>, mut pins: MotorOutputs, limit: DutyLimit) -> Self {
        pins.stop();
        let clock =
            PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).expect("fixed MCPWM clock");
        let timer_config = clock
            .timer_clock_with_frequency(999, PwmWorkingMode::Increase, Rate::from_khz(35))
            .expect("fixed motor PWM frequency");
        let mut pwm = McPwm::new(peripheral, clock);
        pwm.operator0.set_timer(&pwm.timer0);
        // Internal PWM channels target NoPin. Real enables remain owned GPIOs
        // with LOW output latches, routed to PWM only during an armed command.
        let (mut a, mut b) = pwm.operator0.with_pins(
            NoPin,
            PwmPinConfig::UP_ACTIVE_HIGH,
            NoPin,
            PwmPinConfig::UP_ACTIVE_HIGH,
        );
        a.set_update_method(PwmUpdateMethod::SYNC_IMMEDIATLY);
        b.set_update_method(PwmUpdateMethod::SYNC_IMMEDIATLY);
        a.set_timestamp(0);
        b.set_timestamp(0);
        pwm.timer0.start(timer_config);
        Self {
            pins,
            a,
            b,
            timer: pwm.timer0,
            limit,
            gate: CommandGate::default(),
            command_logging: true,
        }
    }

    /// Disable only command telemetry for a time-critical caller. Safety and PWM
    /// behavior stay identical; existing diagnostics default to logging enabled.
    pub fn set_command_logging(&mut self, enabled: bool) {
        self.command_logging = enabled;
    }

    fn direction(first: &mut Output<'_>, second: &mut Output<'_>, direction: Direction) {
        let (a, b) = direction.inputs();
        first.set_level(a.into());
        second.set_level(b.into());
    }
}

impl DualMotor for L298n {
    fn arm(&mut self) {
        self.stop();
        self.gate.arm();
    }

    fn command(&mut self, a: i16, b: i16) -> Result<(), Disarmed> {
        // Every update first detaches both enables, including rejected commands.
        self.stop();
        self.gate.check(a, b)?;
        let requested = (a, b);
        let a = self.limit.map(a);
        let b = self.limit.map(b);
        if a.duty_percent == 0 && b.duty_percent == 0 {
            return Ok(());
        }

        // Break before make on every command, including polarity reversals.
        Delay::new().delay_millis(1);
        Self::direction(&mut self.pins.in1, &mut self.pins.in2, a.direction);
        Self::direction(&mut self.pins.in3, &mut self.pins.in4, b.direction);
        self.a.set_timestamp(a.duty_percent as u16 * 10);
        self.b.set_timestamp(b.duty_percent as u16 * 10);
        // Report effective duty while enables are still detached. A requested
        // percentage above the configured ceiling must not masquerade as output.
        if self.command_logging {
            log::info!(
                "[MOTORS] requested A={} B={}; effective A={}% B={}%; MCPWM0 timer0 period={}, compare A={} B={}, active-high, 10 kHz",
                requested.0,
                requested.1,
                a.duty_percent,
                b.duty_percent,
                self.a.period(),
                self.a.timestamp(),
                self.b.timestamp(),
            );
        }
        // Two full PWM periods while physically detached from both enables.
        Delay::new().delay_micros(200);
        if a.duty_percent != 0 {
            MCPWM0::output_signal::<0, true>().connect_to(&self.pins.ena);
        }
        if b.duty_percent != 0 {
            MCPWM0::output_signal::<0, false>().connect_to(&self.pins.enb);
        }
        Ok(())
    }

    fn stop(&mut self) {
        self.pins.stop();
        self.a.set_timestamp(0);
        self.b.set_timestamp(0);
    }

    fn disarm(&mut self) {
        self.stop();
        self.gate.disarm();
    }
}

impl Drop for L298n {
    fn drop(&mut self) {
        // Detach enables before stopping the counter or releasing PWM guards.
        self.disarm();
        self.timer.stop();
    }
}

impl MotorOutputs {
    pub(crate) fn stop(&mut self) {
        // Write GPIO latches BEFORE switching the matrix away from PWM.
        self.ena.set_low();
        self.enb.set_low();
        self.ena.disconnect_from_peripheral_output();
        self.enb.disconnect_from_peripheral_output();
        self.in1.set_low();
        self.in2.set_low();
        self.in3.set_low();
        self.in4.set_low();
    }
}
