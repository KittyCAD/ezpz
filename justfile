clippy-flags := "--workspace --tests --benches --examples"

lint:
    cargo clippy {{clippy-flags}} -- -D warnings

lint-fix:
    cargo clippy {{clippy-flags}} --fix -- -D warnings

check-wasm:
    cargo check -p ezpz-wasm --target wasm32-unknown-unknown
    cd ezpz-wasm; wasm-pack build --target web --dev; cd -

check-typos:
    typos

test:
    cargo nextest run --all-features
    cargo test --doc

# Run unit tests, output coverage to `lcov.info`.
test-with-coverage:
    cargo llvm-cov nextest --all-features --workspace --lcov --output-path lcov.info

# Flamegraph our benchmarks
flamegraph:
    cargo flamegraph -p --root --bench solver_bench

bench:
    cargo criterion -p --bench solver_bench

