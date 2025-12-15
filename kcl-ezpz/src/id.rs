/// The ID of a variable which could be constrained,
/// whose value could be found by ezpz.
pub type Id = u32;

/// Generates an incrementing sequence of IDs starting from 0.
#[derive(Default)]
pub struct IdGenerator {
    next: Id,
}

impl IdGenerator {
    /// Generates an incrementing sequence of IDs starting from 0.
    pub fn next_id(&mut self) -> Id {
        let out = self.next;
        self.next += 1;
        out
    }
}
