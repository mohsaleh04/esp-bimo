//! Register access adapted from esp-s3-lab/src/bin/main.rs.
//! The caller owns the shared bus; this driver only borrows it per operation.
use embassy_time::Timer;
use esp_hal::{
    Blocking,
    i2c::master::{Error as BusError, I2c},
};
use log::warn;

use super::sample::ImuSample;

#[derive(Debug)]
pub enum Error {
    Bus(BusError),
    UnexpectedChipId(u8),
    ConfigurationMismatch,
}

impl From<BusError> for Error {
    fn from(value: BusError) -> Self {
        Self::Bus(value)
    }
}

pub struct Bmi160 {
    address: u8,
}

impl Bmi160 {
    /// Supports either SDO address strap; lab hardware used 0x68.
    pub async fn init(bus: &mut I2c<'_, Blocking>, address: u8) -> Result<Self, Error> {
        let sensor = Self { address };
        let mut id = [0];
        Self::init_read(bus, address, 0x00, &mut id, "chip ID")?;
        if id[0] != 0xD1 {
            warn!(
                "BMI160 init address=0x{address:02X} chip-ID mismatch: actual=0x{:02X} expected=0xD1",
                id[0]
            );
            return Err(Error::UnexpectedChipId(id[0]));
        }

        // Establish known defaults even after an MCU-only restart.
        Self::init_write(bus, address, 0x7E, 0xB6, "soft reset")?;
        Timer::after_millis(10).await;

        // Lab configuration: INT1 active-high, push-pull, data-ready output.
        Self::init_write(bus, address, 0x53, 0x0A, "INT1 output")?;
        Self::init_write(bus, address, 0x56, 0x80, "INT1 mapping")?;
        Self::init_write(bus, address, 0x51, 0x10, "data-ready interrupt")?;
        // Explicitly retain the lab's default ±2 g / ±2000 degrees/s scaling.
        Self::init_write(bus, address, 0x41, 0x03, "accelerometer range")?;
        Self::init_write(bus, address, 0x43, 0x00, "gyroscope range")?;
        Self::init_write(bus, address, 0x7E, 0x11, "accelerometer normal mode")?;
        Timer::after_millis(10).await;
        Self::init_write(bus, address, 0x7E, 0x15, "gyroscope normal mode")?;
        Timer::after_millis(100).await;

        let mut pmu = [0];
        let mut accel_range = [0];
        let mut gyro_range = [0];
        Self::init_read(bus, address, 0x03, &mut pmu, "PMU status readback")?;
        Self::init_read(
            bus,
            address,
            0x41,
            &mut accel_range,
            "accelerometer range readback",
        )?;
        Self::init_read(
            bus,
            address,
            0x43,
            &mut gyro_range,
            "gyroscope range readback",
        )?;
        if pmu[0] & 0x3C != 0x14 || accel_range[0] != 0x03 || gyro_range[0] != 0x00 {
            warn!(
                "BMI160 init address=0x{address:02X} configuration mismatch after existing PMU waits: PMU_STATUS=0x{:02X} masked=0x{:02X} expected=0x14; ACC_RANGE=0x{:02X} expected=0x03; GYR_RANGE=0x{:02X} expected=0x00",
                pmu[0],
                pmu[0] & 0x3C,
                accel_range[0],
                gyro_range[0]
            );
            return Err(Error::ConfigurationMismatch);
        }
        Ok(sensor)
    }

    // Diagnostics only: same transactions, errors and ordering as before.
    // HAL Timeout does not distinguish hardware bus timeout from software timeout.
    fn init_write(
        bus: &mut I2c<'_, Blocking>,
        address: u8,
        register: u8,
        value: u8,
        stage: &str,
    ) -> Result<(), Error> {
        bus.write(address, &[register, value]).map_err(|error| {
            warn!("BMI160 init address=0x{address:02X} stage={stage} WRITE register=0x{register:02X} value=0x{value:02X} I2C={error:?} timeout={}", matches!(error, BusError::Timeout));
            Error::Bus(error)
        })
    }
    fn init_read(
        bus: &mut I2c<'_, Blocking>,
        address: u8,
        register: u8,
        data: &mut [u8],
        stage: &str,
    ) -> Result<(), Error> {
        bus.write_read(address, &[register], data).map_err(|error| {
            warn!("BMI160 init address=0x{address:02X} stage={stage} READ register=0x{register:02X} I2C={error:?} timeout={}", matches!(error, BusError::Timeout));
            Error::Bus(error)
        })
    }

    /// Poll data-ready rather than waiting indefinitely on INT1.
    pub fn read(&self, bus: &mut I2c<'_, Blocking>) -> Result<Option<ImuSample>, Error> {
        let mut status = [0];
        bus.write_read(self.address, &[0x1B], &mut status)?;
        if status[0] & 0xC0 != 0xC0 {
            return Ok(None);
        }
        let mut data = [0; 12];
        bus.write_read(self.address, &[0x0C], &mut data)?;
        Ok(Some(ImuSample::from_bytes(data)))
    }
}
