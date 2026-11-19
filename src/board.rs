//! Bimo wiring and board-level setup
use esp_hal::{
    Blocking,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::{Config, I2c, SoftwareTimeout},
    peripherals::{FROM_CPU_INTR0, LEDC, MCPWM0, Peripherals, TIMG0},
    time::{Duration, Rate},
};

pub const QMC5883L_ADDRESS: u8 = 0x0D;
pub const AHT20_ADDRESS: u8 = 0x38;

pub struct Board {
    pub timer_group: TIMG0<'static>,
    pub rtos_interrupt: FROM_CPU_INTR0<'static>,
    pub ledc: LEDC<'static>,
    pub motor_pwm: MCPWM0<'static>,
    #[cfg(feature = "balance-test")]
    pub balance_console: esp_hal::peripherals::USB_DEVICE<'static>,
    #[cfg(feature = "balance-test")]
    pub balance_watchdog: esp_hal::peripherals::TIMG1<'static>,
    pub buzzer: Output<'static>,
    pub i2c: I2c<'static, Blocking>,
    pub motor_outputs: MotorOutputs,
    pub bmi160_int1: Input<'static>,
    pub qmc5883l_drdy: Input<'static>,
}

/// Initial inactive state. Move this entire owner into Tb6612; never recreate pins.
pub struct MotorOutputs {
    pub(crate) stby: Output<'static>,
    pub(crate) pwma: Output<'static>,
    pub(crate) pwmb: Output<'static>,
    pub(crate) ain1: Output<'static>,
    pub(crate) ain2: Output<'static>,
    pub(crate) bin1: Output<'static>,
    pub(crate) bin2: Output<'static>,
}

impl Board {
    /// Call immediately after HAL init, before allocation, RTOS or bus setup.
    pub fn init(p: Peripherals) -> Self {
        // Enables first. External pull-downs / hardware gating are required
        // for reset and pre-boot safety; software cannot provide that guarantee.
        let outputs = MotorOutputs {
            stby: Output::new(
                crate::motor_gpio!(p, stby),
                Level::Low,
                OutputConfig::default(),
            ),
            pwma: Output::new(
                crate::motor_gpio!(p, pwma),
                Level::Low,
                OutputConfig::default(),
            ),
            pwmb: Output::new(
                crate::motor_gpio!(p, pwmb),
                Level::Low,
                OutputConfig::default(),
            ),
            ain1: Output::new(
                crate::motor_gpio!(p, ain1),
                Level::Low,
                OutputConfig::default(),
            ),
            ain2: Output::new(
                crate::motor_gpio!(p, ain2),
                Level::Low,
                OutputConfig::default(),
            ),
            bin1: Output::new(
                crate::motor_gpio!(p, bin1),
                Level::Low,
                OutputConfig::default(),
            ),
            bin2: Output::new(
                crate::motor_gpio!(p, bin2),
                Level::Low,
                OutputConfig::default(),
            ),
        };
        let buzzer = Output::new(p.GPIO21, Level::High, OutputConfig::default());
        let i2c = I2c::new(
            p.I2C0,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(20))),
        )
        .expect("I2C configuration")
        .with_sda(p.GPIO14)
        .with_scl(p.GPIO13);

        Self {
            timer_group: p.TIMG0,
            rtos_interrupt: p.FROM_CPU_INTR0,
            ledc: p.LEDC,
            motor_pwm: p.MCPWM0,
            #[cfg(feature = "balance-test")]
            balance_console: p.USB_DEVICE,
            #[cfg(feature = "balance-test")]
            balance_watchdog: p.TIMG1,
            buzzer,
            i2c,
            motor_outputs: outputs,
            bmi160_int1: Input::new(p.GPIO41, InputConfig::default()),
            qmc5883l_drdy: Input::new(p.GPIO12, InputConfig::default()),
        }
    }
}
