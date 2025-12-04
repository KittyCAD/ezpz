clippy-flags := "--workspace --tests --benches --examples --all-targets"
gen := "test_cases/massive_parallel_system/gen_big_problem.py"

# Check most of CI, but locally.
@check-most:
    just lint
    just check-wasm
    just test
    just fmt-check
    just fuzz-check

lint:
    cargo clippy {{clippy-flags}} -- -D warnings
    cargo clippy {{clippy-flags}} --features dbg-jac -- -D warnings

# Fix some lints automatically.
lint-fix:
    cargo clippy {{clippy-flags}} --fix -- -D warnings

# Check our WASM projects build properly.
check-wasm:
    cargo check -p ezpz-wasm --target wasm32-unknown-unknown
    cd ezpz-wasm; wasm-pack build --target web --dev; cd -

test:
    cargo nextest run --all-features

# Run unit tests, output coverage to `lcov.info`.
test-with-coverage:
    cargo llvm-cov nextest --all-features --workspace --lcov --output-path lcov.info

# Flamegraph our benchmarks
flamegraph:
    cargo flamegraph -p kcl-ezpz --bench solver_bench

# Run benchmarks
bench:
    cargo criterion -p kcl-ezpz --bench solver_bench
    git restore test_cases/massive_parallel_system/problem.md

# Check formatting and typos.
fmt-check:
    cargo fmt --check
    cargo sort --check
    typos

# Generate a constraint system with varying number of lines.
@regen-massive-test num_lines:
    python3 {{gen}} {{num_lines}} > test_cases/massive_parallel_system/problem.md

# Generate an overconstraint system with varying number of lines.
@regen-massive-test-overconstrained num_lines:
    python3 {{gen}} {{num_lines}} true > test_cases/massive_parallel_system/problem.md

# Install the ezpz CLI.
# The output text will tell you where it got installed.
# Probably in ~/.cargo/bin/ezpz
install:
    cargo install --path ezpz-cli

# Like `install` but faster.
@reinstall:
    cargo install --path ezpz-cli --quiet --offline --force

# Create a new test case
new-test name:
    mkdir test_cases/{{name}}
    touch test_cases/{{name}}/problem.md

[linux]
[windows]
fuzz:
    cargo +nightly fuzz run fuzz_target_1

[macos]
fuzz:
    cargo +nightly fuzz run fuzz_target_1 --target aarch64-apple-darwin

[linux]
[windows]
fuzz-check:
    cargo +nightly fuzz check

[macos]
fuzz-check:
    cargo +nightly fuzz check --target aarch64-apple-darwin

# Install dependencies needed for ezpz's CLI to render graphics
[linux]
install-viz-deps:
    sudo apt install -y pkg-config libfontconfig1-dev
