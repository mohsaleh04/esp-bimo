#[allow(dead_code)]
#[path = "../src/orientation.rs"]
mod orientation;
use orientation::{Calibration, ComplementaryFilter, PitchFrame, Vector};

const G: f32 = 9.806;
fn v(x: f32, y: f32, z: f32) -> Vector {
    Vector([x, y, z])
}
fn gravity(degrees: f32) -> Vector {
    let a = degrees.to_radians();
    v(-libm::sinf(a) * G, 0.0, libm::cosf(a) * G)
}
fn frame() -> PitchFrame {
    PitchFrame::from_poses(gravity(0.0), gravity(25.0)).unwrap()
}
fn near(a: f32, b: f32) {
    assert!((a - b).abs() < 0.02, "{a} != {b}");
}

#[test]
fn two_pose_mapping_gives_positive_forward_angle_and_rate() {
    let f = frame();
    near(f.accel_angle(gravity(0.0)).unwrap(), 0.0);
    near(f.accel_angle(gravity(30.0)).unwrap(), 30.0);
    near(f.accel_angle(gravity(-20.0)).unwrap(), -20.0);
    near(f.rate(v(0.0, 12.0, 0.0)), 12.0);
    near(f.rate(v(12.0, 0.0, 0.0)), 0.0);
}

#[test]
fn mounting_rotation_does_not_assume_a_sensor_axis() {
    // Rotate sensor coordinates 90 deg around X: [x,y,z] -> [x,-z,y].
    fn rotate(a: Vector) -> Vector {
        v(a.0[0], -a.0[2], a.0[1])
    }
    let f = PitchFrame::from_poses(rotate(gravity(0.0)), rotate(gravity(25.0))).unwrap();
    near(f.accel_angle(rotate(gravity(35.0))).unwrap(), 35.0);
    near(f.rate(rotate(v(0.0, 15.0, 0.0))), 15.0);
    // Also exercise non-permutation mounting (37-degree rotation around X).
    fn oblique(a: Vector) -> Vector {
        let c = libm::cosf(37.0_f32.to_radians());
        let s = libm::sinf(37.0_f32.to_radians());
        v(a.0[0], c * a.0[1] - s * a.0[2], s * a.0[1] + c * a.0[2])
    }
    let f = PitchFrame::from_poses(oblique(gravity(0.0)), oblique(gravity(25.0))).unwrap();
    near(f.accel_angle(oblique(gravity(-15.0))).unwrap(), -15.0);
    near(f.rate(oblique(v(0.0, -9.0, 0.0))), -9.0);
}

#[test]
fn degenerate_or_excessive_pose_changes_are_rejected() {
    assert!(PitchFrame::from_poses(gravity(0.0), gravity(2.0)).is_none());
    assert!(PitchFrame::from_poses(gravity(0.0), gravity(70.0)).is_none());
    assert!(PitchFrame::from_poses(v(0.0, 0.0, 0.0), gravity(20.0)).is_none());
    assert!(PitchFrame::from_poses(v(f32::NAN, 0.0, G), gravity(20.0)).is_none());
}

#[test]
fn filter_initializes_at_accel_angle_instead_of_zero() {
    let mut f = ComplementaryFilter::new(0.5);
    let out = f
        .update(&frame(), gravity(20.0), Vector::ZERO, 0.01)
        .unwrap();
    near(out.pitch_deg, 20.0);
    near(out.rate_dps, 0.0);
}

#[test]
fn measured_dt_changes_integration_and_accel_correction_converges() {
    let frame = frame();
    let mut a = ComplementaryFilter::new(0.5);
    let mut b = ComplementaryFilter::new(0.5);
    a.update(&frame, gravity(0.0), Vector::ZERO, 0.01).unwrap();
    b.update(&frame, gravity(0.0), Vector::ZERO, 0.01).unwrap();
    let x = a
        .update(&frame, gravity(0.0), v(0.0, 10.0, 0.0), 0.01)
        .unwrap();
    let y = b
        .update(&frame, gravity(0.0), v(0.0, 10.0, 0.0), 0.02)
        .unwrap();
    near(x.pitch_deg, 0.5 / 0.51 * 0.1);
    near(y.pitch_deg, 0.5 / 0.52 * 0.2);
    for _ in 0..500 {
        a.update(&frame, gravity(0.0), Vector::ZERO, 0.01).unwrap();
    }
    near(a.pitch_deg().unwrap(), 0.0);
}

#[test]
fn wrap_correction_takes_short_path() {
    let frame = frame();
    let mut f = ComplementaryFilter::new(0.5);
    f.update(&frame, gravity(179.0), Vector::ZERO, 0.01)
        .unwrap();
    let out = f
        .update(&frame, gravity(-179.0), Vector::ZERO, 0.01)
        .unwrap();
    assert!(out.pitch_deg.abs() > 178.0);
}

#[test]
fn invalid_dt_and_long_gaps_reset_instead_of_integrating() {
    let frame = frame();
    let mut f = ComplementaryFilter::new(0.5);
    for dt in [0.0, -0.1, f32::NAN, 0.2] {
        assert!(f.update(&frame, gravity(0.0), Vector::ZERO, dt).is_none());
        assert!(f.pitch_deg().is_none());
    }
    let out = f.update(&frame, gravity(15.0), Vector::ZERO, 0.01).unwrap();
    near(out.pitch_deg, 15.0);
}

#[test]
fn dynamic_accel_is_rejected_and_extended_gyro_only_is_invalid() {
    let frame = frame();
    let mut f = ComplementaryFilter::new(0.5);
    f.update(&frame, gravity(0.0), Vector::ZERO, 0.01).unwrap();
    let out = f
        .update(&frame, v(0.0, 0.0, 2.0 * G), v(0.0, 10.0, 0.0), 0.01)
        .unwrap();
    assert!(!out.accel_used);
    near(out.pitch_deg, 0.1);
    let mut last = Some(out);
    for _ in 0..60 {
        last = f.update(&frame, v(0.0, 0.0, 0.0), Vector::ZERO, 0.01);
    }
    assert!(last.is_none());
    assert!(
        f.update(&frame, gravity(0.0), v(f32::INFINITY, 0.0, 0.0), 0.01)
            .is_none()
    );
}

#[test]
fn stationary_window_estimates_bias_and_requires_elapsed_time() {
    let mut c = Calibration::default();
    for _ in 0..300 {
        c.push(gravity(0.0), v(0.2, -0.3, 0.1), 0.001);
    }
    assert!(c.result().is_none()); // count alone cannot complete calibration
    for _ in 0..300 {
        c.push(gravity(0.0), v(0.2, -0.3, 0.1), 0.01);
    }
    let out = c.result().unwrap();
    near(out.gyro_bias_dps.0[0], 0.2);
    near(out.gyro_bias_dps.0[1], -0.3);
    near(out.accel_mean.0[2], G);
    near(frame().rate(v(0.2, 4.7, 0.1) - out.gyro_bias_dps), 5.0);
}

#[test]
fn motion_gaps_and_vibration_prevent_calibration() {
    let mut c = Calibration::default();
    for _ in 0..350 {
        c.push(gravity(0.0), v(0.0, 10.0, 0.0), 0.01);
    }
    assert!(c.result().is_none());
    for _ in 0..200 {
        c.push(gravity(0.0), Vector::ZERO, 0.01);
    }
    c.push(gravity(0.0), Vector::ZERO, 0.2);
    assert!(c.result().is_none());
    for i in 0..400 {
        c.push(
            gravity(0.0),
            v(if i % 2 == 0 { 2.0 } else { -2.0 }, 0.0, 0.0),
            0.01,
        );
    }
    assert!(c.result().is_none());
    let mut c = Calibration::default();
    for i in 0..400 {
        c.push(
            gravity(if i % 2 == 0 { 0.0 } else { 6.0 }),
            Vector::ZERO,
            0.01,
        );
    }
    assert!(c.result().is_none());
}

#[test]
fn reversed_mounting_sign_is_learned_from_the_forward_pose() {
    let f = PitchFrame::from_poses(gravity(0.0), gravity(-25.0)).unwrap();
    near(f.accel_angle(gravity(-30.0)).unwrap(), 30.0);
    near(f.rate(v(0.0, -10.0, 0.0)), 10.0);
    near(f.axis.norm(), 1.0);
    near(f.axis.dot(f.up), 0.0);
}

#[test]
fn tracks_consistent_rotation_with_variable_sample_intervals() {
    let frame = frame();
    let mut f = ComplementaryFilter::new(0.5);
    f.update(&frame, gravity(0.0), Vector::ZERO, 0.01).unwrap();
    let mut angle = 0.0;
    for i in 0..100 {
        let dt = if i % 2 == 0 { 0.007 } else { 0.013 };
        angle += 10.0 * dt;
        let out = f
            .update(&frame, gravity(angle), v(0.0, 10.0, 0.0), dt)
            .unwrap();
        near(out.pitch_deg, angle);
        near(out.rate_dps, 10.0);
    }
}

#[test]
fn rejected_sideways_acceleration_and_nonfinite_calibration_are_invalid() {
    assert!(frame().accel_angle(v(0.0, G, 0.0)).is_none());
    let mut c = Calibration::default();
    for _ in 0..350 {
        c.push(gravity(0.0), v(f32::NAN, 0.0, 0.0), 0.01);
    }
    assert!(c.result().is_none());
    for _ in 0..350 {
        c.push(gravity(0.0), Vector::ZERO, 0.01);
    }
    assert!(c.result().is_some());
    c.push(v(0.0, 0.0, 0.0), Vector::ZERO, 0.01);
    assert!(c.result().is_none());
}

#[test]
fn noisy_stationary_bias_completes_at_60hz_after_three_measured_seconds() {
    let mut c = Calibration::default();
    let mut finished = None;
    for i in 0..200u64 {
        let noise = if i % 2 == 0 {
            libm::sqrtf(0.78)
        } else {
            -libm::sqrtf(0.78)
        };
        c.push_at(
            v(0.002 * G * if i % 2 == 0 { 1.0 } else { -1.0 }, 0.0, G),
            v(0.1, 3.1 + noise, -0.2),
            i * 16_667,
        );
        if let Some(result) = c.result() {
            finished = Some((i, c.progress(), result));
            break;
        }
        if i * 16_667 < 3_000_000 {
            assert!(c.result().is_none());
        }
    }
    let (i, (count, seconds), result) = finished.expect("60 Hz stationary window must finish");
    assert!(i <= 181 && count < 250 && seconds >= 3.0);
    near(result.gyro_bias_dps.0[1], 3.1);
}

#[test]
fn duration_and_sample_count_are_independent_requirements() {
    let mut fast = Calibration::default();
    for i in 0..1000u64 {
        fast.push_at(gravity(0.0), Vector::ZERO, i * 1000);
    }
    assert!(fast.progress().0 >= 120 && fast.result().is_none());
    let mut sparse = Calibration::default();
    for i in 0..70u64 {
        sparse.push_at(gravity(0.0), Vector::ZERO, i * 50_000);
    }
    assert!(sparse.progress().1 >= 3.0 && sparse.result().is_none());
    for i in 70..121u64 {
        sparse.push_at(gravity(0.0), Vector::ZERO, i * 50_000);
    }
    assert!(sparse.result().is_some());
}

#[test]
fn real_tilt_rocking_and_yaw_acceleration_do_not_calibrate() {
    let mut tilt = Calibration::default();
    let mut rock = Calibration::default();
    let mut yaw = Calibration::default();
    for i in 0..400u64 {
        let t = i as f32 / 60.0;
        // A slow 0.5 deg/s tilt passes the old raw gyro gate but is not stationary.
        tilt.push_at(gravity(t * 0.5), v(0.0, 0.5, 0.0), i * 16_667);
        rock.push_at(
            gravity(if i % 20 < 10 { 2.0 } else { -2.0 }),
            v(0.0, if i % 20 < 10 { 4.0 } else { -4.0 }, 0.0),
            i * 16_667,
        );
        // Gravity stays fixed under yaw, so use gyro mean-change/variance checks.
        yaw.push_at(gravity(0.0), v(0.0, 0.0, t), i * 16_667);
        assert!(tilt.result().is_none());
        assert!(rock.result().is_none());
        assert!(yaw.result().is_none());
    }
}

#[test]
fn excess_noise_fast_motion_and_implausible_bias_are_rejected() {
    for mode in 0..3 {
        let mut c = Calibration::default();
        for i in 0..400u64 {
            let g = match mode {
                0 => v(0.0, if i % 2 == 0 { 2.0 } else { -2.0 }, 0.0),
                1 => v(0.0, 20.0, 0.0),
                _ => v(0.0, 8.0, 0.0),
            };
            c.push_at(gravity(0.0), g, i * 16_667);
            assert!(c.result().is_none());
        }
    }
}

#[test]
fn timestamp_gap_and_nonmonotonic_time_reset_the_window() {
    let mut c = Calibration::default();
    for i in 0..150u64 {
        c.push_at(gravity(0.0), Vector::ZERO, i * 16_667);
    }
    assert!(matches!(
        c.push_at(gravity(0.0), Vector::ZERO, 1),
        Some(orientation::CalibrationRejection::Timing { .. })
    ));
    assert_eq!(c.progress().0, 0);
    c.push_at(gravity(0.0), Vector::ZERO, 3_000_000);
    assert!(matches!(
        c.push_at(gravity(0.0), Vector::ZERO, 3_100_000),
        Some(orientation::CalibrationRejection::Timing { .. })
    ));
    assert!(c.result().is_none());
}

#[test]
fn motion_trend_and_bias_guard_report_measured_reason() {
    use orientation::CalibrationRejection;
    let mut trend = Calibration::default();
    let mut got_motion = false;
    let mut bias = Calibration::default();
    let mut got_bias = false;
    for i in 0..200u64 {
        let t = i as f32 / 60.0;
        if let Some(CalibrationRejection::Motion { accel_shift, .. }) =
            trend.push_at(gravity(t * 0.5), v(0.0, 0.5, 0.0), i * 16_667)
        {
            assert!(accel_shift > orientation::calibration_limits::MAX_ACCEL_MEAN_SHIFT);
            got_motion = true;
        }
        if let Some(CalibrationRejection::Bias { magnitude }) =
            bias.push_at(gravity(0.0), v(0.0, 8.0, 0.0), i * 16_667)
        {
            near(magnitude, 8.0);
            got_bias = true;
        }
    }
    assert!(got_motion && got_bias);
}

#[test]
fn sample_read_error_and_bad_gravity_restart_timestamp_window() {
    use orientation::CalibrationRejection;
    let mut c = Calibration::default();
    for i in 0..150u64 {
        c.push_at(gravity(0.0), Vector::ZERO, i * 16_667);
    }
    assert!(matches!(c.read_error(), CalibrationRejection::Read));
    assert_eq!(c.progress(), (0, 0.0));
    assert!(matches!(
        c.push_at(v(0.0, 0.0, 2.0 * G), Vector::ZERO, 4_000_000),
        Some(CalibrationRejection::Gravity { .. })
    ));
    for i in 0..170u64 {
        c.push_at(gravity(0.0), Vector::ZERO, 5_000_000 + i * 16_667);
    }
    assert!(c.result().is_none()); // old elapsed time cannot leak across resets
}

#[test]
fn measured_timestamp_jitter_still_calibrates_without_an_assumed_odr() {
    let mut c = Calibration::default();
    let mut now = 0;
    for i in 0..230 {
        now += if i % 2 == 0 { 13_000 } else { 21_000 };
        c.push_at(gravity(0.0), v(0.2, -0.4, 0.1), now);
        if c.result().is_some() {
            assert!(c.progress().1 >= 3.0 && c.progress().0 < 250);
            return;
        }
    }
    panic!("jittered fresh timestamps should complete");
}
