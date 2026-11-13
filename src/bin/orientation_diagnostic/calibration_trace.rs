//! Single source of truth: the calibration engine itself reports its decisions.
//! No duplicated gates, thresholds, window statistics or acceptance logic.
pub use crate::orientation::calibration_limits::*;
pub use crate::orientation::{Calibration as TracedCalibration, CalibrationRejection as Rejection};
