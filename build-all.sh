#!/bin/bash
set -e

# Build the rp-hub75 library and its examples for the RP2350 (Cortex-M33).
echo "=========================================="
echo "Building (library): cargo build-arm"
echo "=========================================="
cargo build-arm

echo "=========================================="
echo "Building (examples): cargo build-arm --examples"
echo "=========================================="
cargo build-arm --examples

echo "=========================================="
echo "All builds succeeded!"
echo "=========================================="
