#!/bin/bash
set -e

# Clippy the rp-hub75 library and its examples.
#
# `--lib --examples` rather than `--all-targets`: the test harness needs
# `core::test`, which does not exist for a no_std bare-metal target.
echo "=========================================="
echo "Clippy (library): cargo clippy --release --lib"
echo "=========================================="
cargo clippy --release --lib

echo "=========================================="
echo "Clippy (examples): cargo clippy --release --examples"
echo "=========================================="
cargo clippy --release --examples

echo "=========================================="
echo "All clippy runs succeeded!"
echo "=========================================="
