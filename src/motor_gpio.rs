//! TB6612FNG wiring. A is LEFT and B is RIGHT, pending physical verification.
//! AIN1 and AIN2 follow the reported wiring and still need continuity checks.

/// Take a typed GPIO from the HAL peripheral set by its motor signal name.
#[macro_export]
macro_rules! motor_gpio {
    ($p:ident, ain1) => {
        $p.GPIO15
    };
    ($p:ident, ain2) => {
        $p.GPIO16
    };
    ($p:ident, pwma) => {
        $p.GPIO17
    };
    ($p:ident, bin1) => {
        $p.GPIO7
    };
    ($p:ident, bin2) => {
        $p.GPIO6
    };
    ($p:ident, pwmb) => {
        $p.GPIO5
    };
    ($p:ident, stby) => {
        $p.GPIO40
    };
}

/// Older motor probes stay dry while the TB6612 wiring is being verified.
/// This is a source-level lockout, independent of Cargo feature selection.
pub const TB6612_LEGACY_PROBES_LIVE: bool = false;
