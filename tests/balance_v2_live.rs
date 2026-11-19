#[allow(dead_code)]
#[path = "../src/balance.rs"]
mod balance;
#[allow(dead_code)]
#[path = "../src/bin/balance_v2_test/controller.rs"]
mod controller;
#[allow(dead_code)]
#[path = "../src/motors/control.rs"]
mod motor_control;
#[allow(dead_code)]
#[path = "../src/pwm_mapping.rs"]
mod pwm_mapping;
#[path = "../src/bin/balance_v2_live/session.rs"]
mod session;

use controller::{Controller, Fault};
use motor_control::{Disarmed, DualMotor};
use session::{Session, State, Stop};

#[derive(Default)]
struct Motor {
    armed: bool,
    command: (i16, i16),
    reject: bool,
}

impl DualMotor for Motor {
    fn arm(&mut self) {
        self.armed = true;
    }
    fn command(&mut self, a: i16, b: i16) -> Result<(), Disarmed> {
        if !self.armed || self.reject {
            return Err(Disarmed);
        }
        self.command = (a, b);
        Ok(())
    }
    fn stop(&mut self) {
        self.command = (0, 0);
    }
    fn disarm(&mut self) {
        self.stop();
        self.armed = false;
    }
}

fn observe(
    session: &mut Session<'_, Motor>,
    at: u64,
    pitch: f32,
    rate: f32,
    corrected: bool,
    wheels: [i16; 2],
) -> Result<(), Stop> {
    session.observe(pitch, rate, corrected, wheels, || at, || {})
}

fn arm(session: &mut Session<'_, Motor>) {
    observe(session, 10_000, 0.0, 0.0, true, [0, 0]).unwrap();
    observe(session, 510_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert_eq!(session.state(), State::WaitingNeutral);
    observe(session, 520_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert_eq!(session.state(), State::Active);
}

#[test]
fn arming_requires_upright_gravity_and_a_later_neutral_output() {
    let mut motor = Motor::default();
    let mut session = Session::new(&mut motor, 0);
    observe(&mut session, 10_000, 0.0, 0.0, true, [0, 0]).unwrap();
    observe(&mut session, 500_000, 0.0, 0.0, false, [0, 0]).unwrap();
    observe(&mut session, 510_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert_eq!(session.state(), State::WaitingUpright);
    observe(&mut session, 1_010_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert_eq!(session.state(), State::WaitingNeutral);
    observe(&mut session, 1_020_000, 4.0, 0.0, true, [33, 33]).unwrap();
    assert_eq!(session.state(), State::WaitingNeutral);
    observe(&mut session, 1_030_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert_eq!(session.state(), State::Active);
    assert!(motor.armed);
    assert_eq!(motor.command, (0, 0));
}

#[test]
fn reversal_holds_zero_from_completed_zero_command() {
    let mut motor = Motor::default();
    let mut session = Session::new(&mut motor, 0);
    arm(&mut session);
    observe(&mut session, 530_000, 5.0, 0.0, true, [34, 34]).unwrap();
    let mut times = [540_000, 542_000].into_iter();
    session
        .observe(-5.0, 0.0, true, [-34, -34], || times.next().unwrap(), || {})
        .unwrap();
    assert_eq!(session.last_command(), 0);
    observe(&mut session, 641_999, -5.0, 0.0, true, [-34, -34]).unwrap();
    assert_eq!(session.last_command(), 0);
    observe(&mut session, 642_000, -5.0, 0.0, true, [-34, -34]).unwrap();
    assert_eq!(session.last_command(), -34);
    observe(&mut session, 650_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert!(motor.armed);
    assert_eq!(motor.command, (0, 0));
}

#[test]
fn arming_and_active_timeouts_latch_and_disarm() {
    let mut motor = Motor::default();
    let mut session = Session::new(&mut motor, 0);
    assert_eq!(session.tick(10_000_000), Err(Stop::ArmingTimeout));
    assert_eq!(
        observe(&mut session, 10_010_000, 0.0, 0.0, true, [0, 0]),
        Err(Stop::ArmingTimeout)
    );
    assert!(!motor.armed);

    let mut session = Session::new(&mut motor, 0);
    arm(&mut session);
    observe(&mut session, 530_000, 5.0, 0.0, true, [34, 34]).unwrap();
    session.tick(180_519_999).unwrap();
    assert_eq!(session.tick(180_520_000), Err(Stop::ActiveTimeout));
    assert_eq!(session.state(), State::Finished(Stop::ActiveTimeout));
    assert!(!motor.armed);
    assert_eq!(motor.command, (0, 0));
}

#[test]
fn estimator_errors_and_invalid_commands_fail_closed() {
    let mut common = Controller::new(0);
    let tilt = common.preview(25.0, 0.0, true, 1).unwrap_err();
    let stale = Controller::new(0).check_freshness(40_001).unwrap_err();
    for reason in [
        tilt,
        stale,
        Fault::SensorRead,
        Fault::InvalidSensor,
        Fault::Timing,
    ] {
        let mut motor = Motor::default();
        let mut session = Session::new(&mut motor, 0);
        arm(&mut session);
        observe(&mut session, 530_000, 5.0, 0.0, true, [34, 34]).unwrap();
        assert_eq!(session.finish(Stop::Sensor(reason)), Stop::Sensor(reason));
        assert_eq!(session.finish(Stop::ActiveTimeout), Stop::Sensor(reason));
        assert!(!motor.armed);
        assert_eq!(motor.command, (0, 0));
    }
    for wheels in [[46, 46], [34, -34]] {
        let mut motor = Motor::default();
        let mut session = Session::new(&mut motor, 0);
        arm(&mut session);
        assert_eq!(
            observe(&mut session, 530_000, 5.0, 0.0, true, wheels),
            Err(Stop::Motor)
        );
        assert!(!motor.armed);
    }
}

#[test]
fn rejected_driver_command_disarms_even_on_neutral_arm() {
    let mut motor = Motor {
        reject: true,
        ..Motor::default()
    };
    let mut session = Session::new(&mut motor, 0);
    observe(&mut session, 10_000, 0.0, 0.0, true, [0, 0]).unwrap();
    observe(&mut session, 510_000, 0.0, 0.0, true, [0, 0]).unwrap();
    assert_eq!(
        observe(&mut session, 520_000, 0.0, 0.0, true, [0, 0]),
        Err(Stop::Motor)
    );
    assert!(!motor.armed);
}

#[test]
fn production_pd_feeds_mock_motor_and_expired_sample_disarms_it() {
    let mut motor = Motor::default();
    let mut trial = Session::new(&mut motor, 0);
    let mut controller = Controller::new(0);
    for now in (10_000..=540_000).step_by(10_000) {
        if controller.fresh_sample(now).unwrap().is_none() {
            continue;
        }
        let pitch = if now >= 540_000 { 4.0 } else { 0.0 };
        let preview = controller.preview(pitch, 0.0, true, now).unwrap();
        observe(&mut trial, now, pitch, 0.0, true, preview.wheels).unwrap();
    }
    assert_eq!(trial.state(), State::Active);
    assert_eq!(trial.last_command(), 21);
    let stale = controller.check_freshness(580_001).unwrap_err();
    assert_eq!(stale, Fault::StaleSample);
    assert_eq!(trial.finish(Stop::Sensor(stale)), Stop::Sensor(stale));
    assert!(!motor.armed);
    assert_eq!(motor.command, (0, 0));
}

#[test]
fn session_needs_only_fresh_upright_neutral_samples_and_cannot_restart_after_completion() {
    let mut motor = Motor::default();
    let mut session = Session::new(&mut motor, 0);
    arm(&mut session);
    assert_eq!(session.last_command(), 0);
    assert_eq!(session.tick(180_520_000), Err(Stop::ActiveTimeout));
    assert_eq!(
        observe(&mut session, 180_530_000, 0.0, 0.0, true, [0, 0]),
        Err(Stop::ActiveTimeout)
    );
    assert!(!motor.armed);
    assert_eq!(motor.command, (0, 0));
}
