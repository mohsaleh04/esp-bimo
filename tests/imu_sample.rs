#[path = "../src/sensors/sample.rs"]
mod sample;
use sample::ImuSample;

#[test]
fn decodes_signed_little_endian_gyro_then_accel() {
    let sample = ImuSample::from_bytes([
        0x00, 0x80, 0xff, 0x7f, 0xff, 0xff, 0x00, 0x40, 0x00, 0xc0, 0x00, 0x00,
    ]);
    assert_eq!(sample.gyro_raw, [i16::MIN, i16::MAX, -1]);
    assert_eq!(sample.accel_raw, [16384, -16384, 0]);
    assert_eq!(sample.accel_m_s2(), [9.806, -9.806, 0.0]);
    let gyro = sample.gyro_rad_s();
    assert!(gyro[0] < -34.0 && gyro[1] > 34.0);
    assert!((gyro[2] + core::f32::consts::PI / 180.0 / 16.4).abs() < 0.000001);
}

#[test]
fn zero_sample_has_zero_physical_values() {
    let sample = ImuSample::from_bytes([0; 12]);
    assert_eq!(sample.accel_m_s2(), [0.0; 3]);
    assert_eq!(sample.gyro_rad_s(), [0.0; 3]);
}
