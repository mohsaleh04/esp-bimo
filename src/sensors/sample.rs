//! BMI160 burst layout and scaling copied/adapted from esp-s3-lab.
//! Sensor axes, uncalibrated; valid for ±2 g and ±2000 degrees/s ranges.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub gyro_raw: [i16; 3],
    pub accel_raw: [i16; 3],
}

impl ImuSample {
    pub fn from_bytes(data: [u8; 12]) -> Self {
        Self {
            gyro_raw: core::array::from_fn(|i| i16::from_le_bytes([data[2 * i], data[2 * i + 1]])),
            accel_raw: core::array::from_fn(|i| {
                i16::from_le_bytes([data[6 + 2 * i], data[7 + 2 * i]])
            }),
        }
    }

    pub fn accel_m_s2(self) -> [f32; 3] {
        self.accel_raw.map(|v| (v as f32 / 16384.0) * 9.806)
    }

    pub fn gyro_rad_s(self) -> [f32; 3] {
        self.gyro_raw
            .map(|v| (v as f32 / 16.4) * (core::f32::consts::PI / 180.0))
    }
}
