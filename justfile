clippy-flags := "--workspace --tests --benches --examples"
gen := "test_cases/massive_parallel_system/gen_big_problem.py"

# Check most of CI, but locally.
@check-most:
    just lint
    just check-wasm
    just check-typos
    just test
    just fmt-check

lint:
    cargo clippy {{clippy-flags}} -- -D warnings

# Fix some lints automatically.
lint-fix:
    cargo clippy {{clippy-flags}} --fix -- -D warnings

# Check our WASM projects build properly.
check-wasm:
    cargo check -p ezpz-wasm --target wasm32-unknown-unknown
    cd ezpz-wasm; wasm-pack build --target web --dev; cd -

check-typos:
    typos

test:
    cargo nextest run --all-features
    cargo test --doc --workspace --exclude newton_faer

# Run unit tests, output coverage to `lcov.info`.
test-with-coverage:
    cargo llvm-cov nextest --all-features --workspace --lcov --output-path lcov.info

# Flamegraph our benchmarks
flamegraph:
    cargo flamegraph -p --root --bench solver_bench

# Run benchmarks
bench:
    cargo criterion -p kcl-ezpz --bench solver_bench
    git restore test_cases/massive_parallel_system/problem.txt

# Check formatting and typos.
fmt-check:
    cargo fmt --check
    cargo sort --check
    typos

# Generate a constraint system with varying number of lines.
@regen-massive-test num_lines:
    python3 {{gen}} {{num_lines}} > test_cases/massive_parallel_system/problem.txt

# Generate an overconstraint system with varying number of lines.
@regen-massive-test-overconstrained num_lines:
    python3 {{gen}} {{num_lines}} true > test_cases/massive_parallel_system/problem.txt

# Install the ezpz CLI.
# The output text will tell you where it got installed.
# Probably in ~/.cargo/bin/ezpz
install:
    cargo install --path ezpz-cli

# Like `install` but faster.
@reinstall:
    cargo install --path ezpz-cli --quiet --offline

# Create a new test case
new-test name:
    mkdir test_cases/{{name}}
    touch test_cases/{{name}}/problem.txt

[linux]
[windows]
fuzz:
    cargo +nightly fuzz run fuzz_target_1

[macos]
fuzz:
    cargo +nightly fuzz run fuzz_target_1 --target aarch64-apple-darwin
