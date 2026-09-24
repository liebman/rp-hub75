#!/bin/bash
set -e

# Format the rp-hub75 library and its examples (rustfmt.toml at the repo root).
echo "=========================================="
echo "Formatting: cargo fmt"
echo "=========================================="
cargo fmt --all

echo "=========================================="
echo "Checking formatting: cargo fmt --check"
echo "=========================================="
cargo fmt --all --check

echo "=========================================="
echo "All formatting succeeded!"
echo "=========================================="
