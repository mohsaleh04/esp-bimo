use super::control::Direction;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Stage {
    LeftForward,
    LeftReverse,
    RightForward,
    RightReverse,
}

impl Stage {
    pub(super) const ALL: [Self; 4] = [
        Self::LeftForward,
        Self::LeftReverse,
        Self::RightForward,
        Self::RightReverse,
    ];

    pub(super) fn directions(self) -> [Direction; 2] {
        use Direction::{Negative, Positive, Stopped};
        match self {
            Self::LeftForward => [Positive, Stopped],
            Self::LeftReverse => [Negative, Stopped],
            Self::RightForward => [Stopped, Positive],
            Self::RightReverse => [Stopped, Negative],
        }
    }
}

pub(super) fn selected_stage(
    live_feature: bool,
    both_feature: bool,
    configured: Option<Stage>,
) -> Option<Stage> {
    if live_feature && !both_feature {
        configured
    } else {
        None
    }
}

#[derive(Default)]
pub(super) struct Confirmation {
    matched: usize,
    finished: bool,
}

impl Confirmation {
    pub(super) fn push(&mut self, byte: u8) -> Option<bool> {
        const TOKEN: &[u8] = b"RUN";
        if self.finished {
            return Some(false);
        }
        if byte == b'\r' || byte == b'\n' {
            self.finished = true;
            return Some(self.matched == TOKEN.len());
        }
        if TOKEN.get(self.matched) == Some(&byte) {
            self.matched += 1;
            return None;
        }
        self.finished = true;
        Some(false)
    }
}
