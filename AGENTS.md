# AGENTS.md

## Setup commands
- Run tests: `cargo nextest run -p kcl-ezpz`
- Get test coverage: `cargo llvm-cov nextest -p kcl-ezpz`
- Don't modify the `newtonls-faer` crate.
- Always run `cargo fmt` when finishing a task.

## Code style
- This is highly performance-sensitive.
- Try to minimize clones, copies and allocations.
- Functions should not return Vecs or other collections, instead, the caller
  should provide the function with one to mutate, to avoid allocations.
- When initializing vectors, prefer `with_capacity()` over `new()`.
- Don't use .unwrap(), return errors via Results.
- Check for divide-by-zero with the EPSILON constant for values near zero.
- 
