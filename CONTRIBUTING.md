## Setting up your dev environment

* We use `just` to run scripts, so please [install just](https://github.com/casey/just?tab=readme-ov-file#installation)
* Install `cargo binstall` to make installing other deps easier: `cargo install cargo-binstall`
* Then install other tools for development: `cargo binstall cargo-criterion typos-cli cargo-nextest cargo-llvm-cov flamegraph cargo-sort`

If everything was successful, `just check-most` should succeed.

## Development flow

Before opening PRs, please run `just bench` on `main`, then again on your branch, and include any speedup/slowdown in your PR description.

If you see there's a slowdown, run `just flamegraph` and open the resulting `flamegraph.svg` to see where the program is spending its time.

You should also run `cargo llvm-cov nextest --open` to make sure your changes are tested properly. Ideally new PRs wouldn't lower the test coverage.
