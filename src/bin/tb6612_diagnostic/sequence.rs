//! Single-pass no-load sequence. Timing is commanded, not externally measured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Off,
    AForward,
    AReverse,
    BForward,
    BReverse,
}

pub trait Io {
    fn apply(&mut self, phase: Phase);
    fn wait_ms(&mut self, ms: u32);
}

pub fn run(io: &mut impl Io) {
    io.apply(Phase::Off);
    io.wait_ms(5000);
    for phase in [
        Phase::AForward,
        Phase::AReverse,
        Phase::BForward,
        Phase::BReverse,
    ] {
        io.apply(phase);
        io.wait_ms(5000);
        io.apply(Phase::Off);
        io.wait_ms(3000);
    }
    io.apply(Phase::Off);
}
