//! TB6612FNG bridge with owned STBY and MCPWM0.
use esp_hal::{
    delay::Delay,
    gpio::{NoPin, Output, interconnect::PeripheralOutput},
    mcpwm::{
        McPwm, PeripheralClockConfig, PwmPeripheral,
        operator::{PwmPin, PwmPinConfig, PwmUpdateMethod},
        timer::{PwmWorkingMode, Timer},
    },
    peripherals::MCPWM0,
    time::{Duration, Instant, Rate},
};

use super::control::{Channel, Direction};
use super::tb6612_transition::{Action, CommandState};
use super::{Disarmed, DualMotor, DutyLimit};
use crate::board::MotorOutputs;

/// Owns STBY, both PWM GPIOs, and all four direction GPIOs.
/// Channel A drives PWMA and channel B drives PWMB.
pub struct Tb6612 {
    pins: MotorOutputs,
    a: PwmPin<'static, MCPWM0<'static>, 0, true>,
    b: PwmPin<'static, MCPWM0<'static>, 0, false>,
    timer: Timer<0, MCPWM0<'static>>,
    limit: DutyLimit,
    command_state: CommandState,
    command_logging: bool,
}

impl Tb6612 {
    /// Fixed requested 35 kHz / 1000-count PWM. Starts stopped and disarmed.
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
        a.set_update_method(PwmUpdateMethod::SYNC_ON_ZERO);
        b.set_update_method(PwmUpdateMethod::SYNC_ON_ZERO);
        a.set_timestamp(0);
        b.set_timestamp(0);
        pwm.timer0.start(timer_config);
        Self {
            pins,
            a,
            b,
            timer: pwm.timer0,
            limit,
            command_state: CommandState::default(),
            command_logging: true,
        }
    }

    /// Disable only command telemetry for a time-critical caller. Safety and PWM
    /// behavior stay identical; existing diagnostics default to logging enabled.
    pub fn set_command_logging(&mut self, enabled: bool) {
        self.command_logging = enabled;
    }

    /// Observe a counter wrap while STBY is low. Compare readback is the
    /// shadow register, so matching it alone does not show that TEZ occurred.
    fn wait_for_timer_zero(&self) -> bool {
        let start = Instant::now();
        let mut previous = self.timer.status().0;
        while start.elapsed() < Duration::from_micros(200) {
            let count = self.timer.status().0;
            if count < previous {
                return true;
            }
            previous = count;
        }
        false
    }

    fn direction(
        first: &mut Output<'_>,
        second: &mut Output<'_>,
        channel: Channel,
        direction: Direction,
    ) {
        let (a, b) = channel.direction_inputs(direction);
        first.set_level(a.into());
        second.set_level(b.into());
    }
}

impl DualMotor for Tb6612 {
    fn arm(&mut self) {
        self.stop();
        self.command_state.arm();
    }

    fn command(&mut self, a: i16, b: i16) -> Result<(), Disarmed> {
        let requested = (a, b);
        let (action, effective) = match self.command_state.plan(self.limit, [a, b]) {
            Ok(plan) => plan,
            Err(error) => {
                self.stop();
                return Err(error);
            }
        };
        let [a, b] = effective;
        let compares = [
            u16::from(a.duty_percent) * 10,
            u16::from(b.duty_percent) * 10,
        ];

        match action {
            Action::Stop => {
                self.stop();
                return Ok(());
            }
            Action::Unchanged => return Ok(()),
            Action::UpdateCompare([update_a, update_b]) => {
                if update_a {
                    self.a.set_timestamp(compares[0]);
                }
                if update_b {
                    self.b.set_timestamp(compares[1]);
                }
                if [self.a.timestamp(), self.b.timestamp()] != compares {
                    self.disarm();
                    return Err(Disarmed);
                }
                self.command_state.record(effective);
                return Ok(());
            }
            Action::Reconfigure => self.stop(),
        }

        // Break before make for start, reversals, active-channel changes and full duty.
        Delay::new().delay_millis(1);
        Self::direction(
            &mut self.pins.ain1,
            &mut self.pins.ain2,
            Channel::A,
            a.direction,
        );
        Self::direction(
            &mut self.pins.bin1,
            &mut self.pins.bin2,
            Channel::B,
            b.direction,
        );
        self.a.set_timestamp(compares[0]);
        self.b.set_timestamp(compares[1]);
        if [self.a.timestamp(), self.b.timestamp()] != compares {
            self.disarm();
            return Err(Disarmed);
        }
        // Report effective duty while enables are still detached. A requested
        // percentage above the configured ceiling must not masquerade as output.
        if self.command_logging {
            log::info!(
                "[MOTORS] requested A={} B={}; effective A={}% B={}%; MCPWM0 timer0 period={}, compare A={} B={}, active-high, requested 35 kHz",
                requested.0,
                requested.1,
                a.duty_percent,
                b.duty_percent,
                self.a.period(),
                self.a.timestamp(),
                self.b.timestamp(),
            );
        }
        // Keep the existing settling delay and check for a subsequent TEZ before
        // routing PWM. A stalled timer must leave the bridge disabled.
        Delay::new().delay_micros(200);
        if !self.wait_for_timer_zero() {
            self.disarm();
            return Err(Disarmed);
        }
        if a.duty_percent != 0 {
            MCPWM0::output_signal::<0, true>().connect_to(&self.pins.pwma);
        }
        if b.duty_percent != 0 {
            MCPWM0::output_signal::<0, false>().connect_to(&self.pins.pwmb);
        }
        // The compare is latched before enable. STBY is a GPIO, so it
        // can expose part of the first correctly configured PWM cycle.
        self.pins.stby.set_high();
        self.command_state.record(effective);
        Ok(())
    }

    fn stop(&mut self) {
        self.pins.stop();
        self.command_state.stop();
        self.a.set_timestamp(0);
        self.b.set_timestamp(0);
    }

    fn disarm(&mut self) {
        self.stop();
        self.command_state.disarm();
    }
}

impl Drop for Tb6612 {
    fn drop(&mut self) {
        // Detach enables before stopping the counter or releasing PWM guards.
        self.disarm();
        self.timer.stop();
    }
}

impl MotorOutputs {
    pub(crate) fn stop(&mut self) {
        self.stby.set_low(); // Hardware disable precedes all PWM/direction changes.
        // Write GPIO latches BEFORE switching the matrix away from PWM.
        self.pwma.set_low();
        self.pwmb.set_low();
        self.pwma.disconnect_from_peripheral_output();
        self.pwmb.disconnect_from_peripheral_output();
        self.ain1.set_low();
        self.ain2.set_low();
        self.bin1.set_low();
        self.bin2.set_low();
    }
}
