#![no_main]

use arbitrary::Arbitrary;
use kcl_ezpz::{Constraint, Id};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|setup: Setup| {
    // fuzzed code goes here
    let guesses: Vec<_> = setup
        .guesses
        .into_iter()
        .enumerate()
        .map(|(i, v)| (Id::try_from(i).unwrap(), v))
        .collect();
    let constraints = &setup.constraints;
    let _ = kcl_ezpz::solve(&constraints, guesses, Default::default());
});

#[derive(Debug, Arbitrary)]
struct Setup {
    constraints: Vec<Constraint>,
    guesses: Vec<f64>,
}
