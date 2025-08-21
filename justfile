cnr := "cargo nextest run --no-fail-fast"
cita := "cargo insta test --accept"

# Run the same lint checks we run in CI.
lint:
    cargo clippy --workspace --all-targets --all-features -- -D warnings
    # Ensure we can build without extra feature flags.
    cargo clippy -p kcl-lib --all-targets -- -D warnings

lint-fix:
    cargo clippy --workspace --all-targets --all-features --fix

test:
    cnr
