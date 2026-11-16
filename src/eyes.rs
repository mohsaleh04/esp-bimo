//! Reusable allocation-free eye animation. Times are monotonic milliseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pose {
    Normal,
    Half,
    Closed,
    Happy,
    Left,
    Right,
    TickleA,
    TickleB,
    LiftLeft,
    LiftRight,
}
pub enum Event {
    Head,
    Chest,
    Lifted,
}
enum State {
    Idle,
    Blink,
    Head,
    Chest,
    Glance,
    Lifted,
}
pub struct Eyes {
    state: State,
    since: u64,
    next: u64,
    sequence: u32,
    glance_due: u64,
    glance_left: bool,
}
impl Eyes {
    pub fn new(now: u64) -> Self {
        Self {
            state: State::Idle,
            since: now,
            next: now + 4000,
            sequence: 1,
            glance_due: now + 2500,
            glance_left: true,
        }
    }
    /// Latest event replaces the current animation; chest wins simultaneous events.
    pub fn trigger(&mut self, event: Event, now: u64) {
        self.state = match event {
            Event::Head => State::Head,
            Event::Chest => State::Chest,
            Event::Lifted => State::Lifted,
        };
        self.since = now;
    }
    pub fn update(&mut self, now: u64) -> Pose {
        if matches!(self.state, State::Idle) && now >= self.next {
            self.state = State::Blink;
            self.since = now;
        } else if matches!(self.state, State::Idle) && now >= self.glance_due {
            self.state = State::Glance;
            self.since = now;
            self.glance_left = !self.glance_left;
        }
        let age = now.saturating_sub(self.since);
        let pose = match self.state {
            State::Idle => return Pose::Normal,
            State::Blink if age < 60 => Pose::Half,
            State::Blink if age < 150 => Pose::Closed,
            State::Blink if age < 210 => Pose::Half,
            State::Head if age < 600 => Pose::Happy,
            State::Head if age < 740 => Pose::Closed,
            State::Head if age < 820 => Pose::Half,
            State::Lifted if age < 900 => {
                if (age / 100) % 2 == 0 {
                    Pose::LiftLeft
                } else {
                    Pose::LiftRight
                }
            }
            State::Lifted if age < 1050 => Pose::Half,
            State::Chest if age < 720 => {
                if (age / 80) % 2 == 0 {
                    Pose::TickleA
                } else {
                    Pose::TickleB
                }
            }
            State::Chest if age < 820 => Pose::Half,
            State::Glance if age < 550 => {
                if self.glance_left {
                    Pose::Left
                } else {
                    Pose::Right
                }
            }
            _ => {
                let was_glance = matches!(self.state, State::Glance);
                self.state = State::Idle;
                self.sequence = self.sequence.wrapping_mul(1664525).wrapping_add(1013904223);
                if !was_glance {
                    self.next = now + 4000 + (self.sequence % 3001) as u64;
                }
                self.glance_due = now + 2000 + ((self.sequence >> 8) % 4001) as u64;
                Pose::Normal
            }
        };
        pose
    }
}
pub struct Debounce {
    stable: bool,
    candidate: bool,
    since: u64,
}
impl Debounce {
    pub fn new(initial: bool) -> Self {
        Self {
            stable: initial,
            candidate: initial,
            since: 0,
        }
    }
    pub fn update(&mut self, raw: bool, now: u64) -> Option<bool> {
        if raw != self.candidate {
            self.candidate = raw;
            self.since = now;
        }
        if raw != self.stable && now.saturating_sub(self.since) >= 30 {
            self.stable = raw;
            Some(raw)
        } else {
            None
        }
    }
}
// Integer ray casting keeps rasterization allocation-free and no_std friendly.
fn star_pixel(x: i32, y: i32) -> bool {
    const V: [(i32, i32); 10] = [
        (0, -10),
        (3, -3),
        (10, -3),
        (4, 2),
        (6, 10),
        (0, 5),
        (-6, 10),
        (-4, 2),
        (-10, -3),
        (-3, -3),
    ];
    let mut inside = false;
    let mut previous = V[9];
    for current in V {
        let (ax, ay) = previous;
        let (bx, by) = current;
        if (ay > y) != (by > y) {
            let cross = (bx - ax) * (y - ay) - (x - ax) * (by - ay);
            if (by > ay && cross > 0) || (by < ay && cross < 0) {
                inside = !inside;
            }
        }
        previous = current;
    }
    inside
}
pub fn render(pose: Pose) -> [u8; 1024] {
    let mut frame = [0; 1024];
    let offset = match pose {
        Pose::Left => -9,
        Pose::Right => 9,
        Pose::TickleA => -2,
        Pose::TickleB => 2,
        Pose::LiftLeft => -5,
        Pose::LiftRight => 5,
        _ => 0,
    };
    for center in [38i32, 90] {
        for y in 18i32..46 {
            for dx in -10i32..10 {
                let lit = match pose {
                    Pose::LiftLeft | Pose::LiftRight => {
                        // Wide hollow eyes distinguish surprise from the starry tickle.
                        let outer = dx * dx * 144 + (y - 32) * (y - 32) * 81 <= 11664;
                        let inner = dx * dx * 100 + (y - 32) * (y - 32) * 49 < 4900;
                        outer && !inner
                    }
                    Pose::Closed => (31..34).contains(&y),
                    Pose::Half => (26..38).contains(&y),
                    Pose::TickleA | Pose::TickleB => {
                        // Filled five-point star, wobbling with the giggle.
                        let bounce = if pose == Pose::TickleA { -2 } else { 2 };
                        star_pixel(dx, y - 32 - bounce)
                    }
                    Pose::Happy => {
                        let curve = 25 + dx.abs() / 2;
                        y >= curve && y < curve + 4
                    }
                    _ => (20..44).contains(&y) && !(dx.abs() >= 8 && !(22..42).contains(&y)),
                };
                if lit {
                    let x = (center + offset + dx) as usize;
                    frame[(y as usize / 8) * 128 + x] |= 1 << (y % 8);
                }
            }
        }
    }
    frame
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn debounce_rejects_bounce_and_hold() {
        let mut d = Debounce::new(false);
        assert_eq!(d.update(true, 10), None);
        assert_eq!(d.update(false, 20), None);
        assert_eq!(d.update(true, 25), None);
        assert_eq!(d.update(true, 54), None);
        assert_eq!(d.update(true, 55), Some(true));
        assert_eq!(d.update(true, 100), None);
    }
    #[test]
    fn lift_reaction_is_distinct_and_returns() {
        let mut e = Eyes::new(0);
        e.trigger(Event::Lifted, 10);
        assert_eq!(e.update(10), Pose::LiftLeft);
        assert_eq!(e.update(110), Pose::LiftRight);
        assert_ne!(render(Pose::LiftLeft), render(Pose::LiftRight));
        assert_ne!(render(Pose::LiftLeft), render(Pose::TickleA));
        assert_eq!(e.update(1060), Pose::Normal);
    }
    #[test]
    fn animation_interrupts_and_returns() {
        let mut e = Eyes::new(0);
        assert_eq!(e.update(2500), Pose::Right);
        assert_eq!(e.update(3050), Pose::Normal);
        assert_eq!(e.update(3999), Pose::Normal);
        assert_eq!(e.update(4000), Pose::Half);
        e.trigger(Event::Head, 4010);
        assert_eq!(e.update(4010), Pose::Happy);
        e.trigger(Event::Chest, 4020);
        assert_eq!(e.update(4020), Pose::TickleA);
        assert_eq!(e.update(4100), Pose::TickleB);
        assert_ne!(render(Pose::TickleA), render(Pose::Left));
        assert_ne!(render(Pose::TickleA), render(Pose::TickleB));
        assert_eq!(e.update(4920), Pose::Normal);
        assert!((8920..=11920).contains(&e.next));
        for pose in [
            Pose::TickleA,
            Pose::TickleB,
            Pose::Normal,
            Pose::Happy,
            Pose::Left,
            Pose::Right,
            Pose::Half,
            Pose::Closed,
        ] {
            assert!(render(pose).iter().any(|v| *v != 0));
        }
    }
}
