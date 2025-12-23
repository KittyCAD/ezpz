## Setting up your dev environment

* We use `just` to run scripts, so please [install just](https://github.com/casey/just?tab=readme-ov-file#installation)
* This is written in Rust, so please [install Rust via Rustup](https://rustup.rs/)
* Install `cargo binstall` to make installing other deps easier: `cargo install cargo-binstall`
* Then install other tools for development: `cargo binstall cargo-criterion typos-cli cargo-nextest cargo-llvm-cov flamegraph cargo-sort`

If everything was successful, `just check-most` should succeed.

## Development flow

Before opening PRs, please run `just bench` on `main`, then again on your branch, and include any speedup/slowdown in your PR description.

If you see there's a slowdown, run `just flamegraph` and open the resulting `flamegraph.svg` to see where the program is spending its time.

Run `just test` for our test suite. You should also run `cargo llvm-cov nextest --open` to make sure your changes are tested properly. Ideally new PRs wouldn't lower the test coverage.

So far, both benchmarks and tests read a text file with constraint system (e.g. [`test_cases/tiny/problem.md`](https://github.com/KittyCAD/ezpz/blob/main/test_cases/tiny/problem.md)). Then they run that constraint system and either check its outputs (tests) or note its performance (benchmarks). If you want to add a new one, make a new directory under `test_cases/YOUR_TEST/problem.md` and then update either [tests.rs](https://github.com/KittyCAD/ezpz/blob/main/kcl-ezpz/src/tests.rs) or [solver_bench.rs](https://github.com/KittyCAD/ezpz/blob/main/kcl-ezpz/benches/solver_bench.rs). Just copy how other tests/benches work.
