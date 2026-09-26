#!/bin/bash
set -e

# Build the rp-hub75 library and its examples for the RP2350 (Cortex-M33).
#
# The library is built once per library feature combination: `invert-clock` and
# `invert-oe` select code paths in the library itself. The examples go through
# `Hub75` and name no polarity, so they are built once.
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
    echo "Building (library): cargo build-arm ${extra[*]}"
    echo "=========================================="
    cargo build-arm "${extra[@]}"
done

echo "=========================================="
echo "Building (examples): cargo build-arm --examples"
echo "=========================================="
cargo build-arm --examples

# The library logs through defmt by default; `--no-default-features` is the
# silent build, and it is a different set of code paths (the `log_info!` that
# discards its arguments), so it is built too.
echo "=========================================="
echo "Building (library, no logging): cargo build-arm --no-default-features"
echo "=========================================="
cargo build-arm --no-default-features

echo "=========================================="
echo "Building (examples, no logging): cargo build-arm --examples --no-default-features"
echo "=========================================="
cargo build-arm --examples --no-default-features

echo "=========================================="
echo "All builds succeeded!"
echo "=========================================="
