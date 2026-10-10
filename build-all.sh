#!/bin/bash
set -e

# Build the rp-hub75 library and its example packages.
#
# The library is built once per library feature combination: `invert-clock` and
# `invert-oe` select code paths in the library itself. The `build-rp2350` alias
# names the board's config file and features (no feature is a default).
#
# The examples are separate crates under `examples/`, not members of a Cargo
# workspace, so each is built in its own directory through its own `build-*`
# aliases. Each names the HAL features a binary needs — the RT vector table, the
# picotool header, the RP2040's second-stage bootloader and the critical-section
# impl — in its own `Cargo.toml`, so the library forces none of them.
#
# `20mhz` is an example feature — it sets the rate the examples ask for through
# `cfg!`, which compiles both arms in every build — so it is not part of the loop.
for features in "" "invert-clock" "invert-oe" "invert-clock,invert-oe"; do
    if [ -n "$features" ]; then
        extra=(--features "$features")
    else
        extra=()
    fi
    echo "=========================================="
    echo "Building (library): cargo build-rp2350 ${extra[*]}"
    echo "=========================================="
    cargo build-rp2350 "${extra[@]}"
done

echo "=========================================="
echo "Building (embassy library): cargo build-embassy"
echo "=========================================="
cargo build-embassy

echo "=========================================="
echo "Building (RP2040 library): cargo build-rp2040 --lib --features invert-clock,invert-oe"
echo "=========================================="
cargo build-rp2040 --lib --features invert-clock,invert-oe

# Each example crate, in its own package.
for dir in examples/*/; do
    echo "=========================================="
    echo "Building (example): ${dir}cargo build-rp2350"
    echo "=========================================="
    (cd "$dir" && cargo build-rp2350)
done

echo "=========================================="
echo "Building (RP2040 example): examples/gradient-latched cargo build-rp2040"
echo "=========================================="
(cd examples/gradient-latched && cargo build-rp2040)

echo "=========================================="
echo "All builds succeeded!"
echo "=========================================="
