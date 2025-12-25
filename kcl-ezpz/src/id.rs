/// The ID of a variable which could be constrained,
/// whose value could be found by ezpz.
pub type Id = u32;

/// Generates an incrementing sequence of IDs starting from 0.
/// ```
/// use kcl_ezpz::{datatypes::inputs::DatumPoint, IdGenerator};
/// let mut ids = IdGenerator::default();
/// // This point has two variables, its X and Y.
/// let p = DatumPoint::new(&mut ids);
/// // This point also has two variables.
/// let q = DatumPoint::new(&mut ids);
/// // So there should be 4 variables generated so far,
/// // i.e. 0, 1, 2 and 3.
/// // This means the next ID should be 4.
/// assert_eq!(ids.next_id(), 4);
/// ```
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
