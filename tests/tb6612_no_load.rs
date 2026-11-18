#[path = "../src/bin/tb6612_diagnostic/sequence.rs"]
mod sequence;
use sequence::{Io, Phase, run};

#[derive(Debug, PartialEq)]
enum Event {
    Apply(Phase),
    Wait(u32),
}
#[derive(Default)]
struct Trace(Vec<Event>);
impl Io for Trace {
    fn apply(&mut self, phase: Phase) {
        self.0.push(Event::Apply(phase));
    }
    fn wait_ms(&mut self, ms: u32) {
        self.0.push(Event::Wait(ms));
    }
}
#[test]
fn no_load_is_once_with_grace_independent_channels_and_stopped_gaps() {
    use Event::*;
    use Phase::*;
    let mut trace = Trace::default();
    run(&mut trace);
    assert_eq!(
        trace.0,
        vec![
            Apply(Off),
            Wait(5000),
            Apply(AForward),
            Wait(5000),
            Apply(Off),
            Wait(3000),
            Apply(AReverse),
            Wait(5000),
            Apply(Off),
            Wait(3000),
            Apply(BForward),
            Wait(5000),
            Apply(Off),
            Wait(3000),
            Apply(BReverse),
            Wait(5000),
            Apply(Off),
            Wait(3000),
            Apply(Off),
        ]
    );
}
