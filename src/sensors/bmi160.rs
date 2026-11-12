//! Register access adapted from esp-s3-lab/src/bin/main.rs.
//! The caller owns the shared bus; this driver only borrows it per operation.
use embassy_time::Timer;
use esp_hal::{
    Blocking,
    i2c::master::{Error as BusError, I2c},
};

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
        bus.write_read(address, &[0x00], &mut id)?;
        if id[0] != 0xD1 {
            return Err(Error::UnexpectedChipId(id[0]));
        }

        // Establish known defaults even after an MCU-only restart.
        bus.write(address, &[0x7E, 0xB6])?;
        Timer::after_millis(10).await;

        // Lab configuration: INT1 active-high, push-pull, data-ready output.
        bus.write(address, &[0x53, 0x0A])?;
        bus.write(address, &[0x56, 0x80])?;
        bus.write(address, &[0x51, 0x10])?;
        // Explicitly retain the lab's default ±2 g / ±2000 degrees/s scaling.
        bus.write(address, &[0x41, 0x03])?;
        bus.write(address, &[0x43, 0x00])?;
        bus.write(address, &[0x7E, 0x11])?;
        Timer::after_millis(10).await;
        bus.write(address, &[0x7E, 0x15])?;
        Timer::after_millis(100).await;

        let mut pmu = [0];
        let mut accel_range = [0];
        let mut gyro_range = [0];
        bus.write_read(address, &[0x03], &mut pmu)?;
        bus.write_read(address, &[0x41], &mut accel_range)?;
        bus.write_read(address, &[0x43], &mut gyro_range)?;
        if pmu[0] & 0x3C != 0x14 || accel_range[0] != 0x03 || gyro_range[0] != 0x00 {
            return Err(Error::ConfigurationMismatch);
        }
        Ok(sensor)
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
