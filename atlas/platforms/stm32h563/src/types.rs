//! In-memory update progress and command-loop control decisions.

/// Track this CAN session; distinct from Embassy's persistent swap state.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum UpdateState {
    /// No image has been announced in this session.
    Idle,
    /// An image is being received or still requires verification.
    Updating,
    /// The full staged image passed CRC and vector checks.
    Verified,
}

impl UpdateState {
    /// Encode the state byte returned by GET_STATUS.
    pub const fn code(self) -> u8 {
        match self {
            Self::Idle => 0,
            Self::Updating => 1,
            Self::Verified => 2,
        }
    }
}

/// Tell the top-level loop whether to keep serving, jump, or reset.
pub enum Control {
    Continue,
    StartApp,
    Reset,
}

/// Metadata for one sequential image transfer; reset discards this progress.
pub struct Context {
    pub state: UpdateState,
    pub image_size: u32,
    pub expected_crc: u32,
    /// DFU-relative byte offset expected by the next write request.
    pub next_offset: u32,
}

impl Context {
    /// Begin with no announced or verified image.
    pub const fn new() -> Self {
        Self {
            state: UpdateState::Idle,
            image_size: 0,
            expected_crc: 0,
            next_offset: 0,
        }
    }

    /// Replace transfer metadata; flash erasure happens when blocks are written.
    pub fn start(&mut self, image_size: u32, expected_crc: u32) {
        self.state = UpdateState::Updating;
        self.image_size = image_size;
        self.expected_crc = expected_crc;
        self.next_offset = 0;
    }
}
