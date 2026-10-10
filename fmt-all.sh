#!/bin/bash
set -e

# Format the rp-hub75 library and its example packages. The examples are
# separate crates (not a Cargo workspace), so each is formatted in its own
# directory. There is no `rustfmt.toml`; every crate uses the default style.
echo "=========================================="
echo "Formatting (library): cargo fmt"
echo "=========================================="
cargo fmt

for dir in examples/*/; do
    echo "=========================================="
    echo "Formatting: ${dir}cargo fmt"
    echo "=========================================="
    (cd "$dir" && cargo fmt)
done

echo "=========================================="
echo "Checking formatting: cargo fmt --check"
echo "=========================================="
cargo fmt --check

for dir in examples/*/; do
    (cd "$dir" && cargo fmt --check)
done

echo "=========================================="
echo "All formatting succeeded!"
echo "=========================================="
