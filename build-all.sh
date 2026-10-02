#!/bin/bash
set -e

# Build the rp-hub75 library and its examples for the RP2350 (Cortex-M33).
#
# The `build-rp2350` alias names the board's config file and features (no feature is
# a default). The library is built once per library feature combination:
# `invert-clock` and `invert-oe` select code paths in the library itself. The
# examples go through `Hub75` and name no polarity, so they are built once.
#
# `20mhz` is an example feature — it sets the rate the examples ask for through
# `cfg!`, which compiles both arms in every build — so it is not part of the loop.
# The library does not log; the examples log over defmt, from a dev-dependency,
# and they are built with the rest.
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
echo "Building (examples): cargo build-rp2350 --examples"
echo "=========================================="
cargo build-rp2350 --examples

echo "=========================================="
echo "All builds succeeded!"
echo "=========================================="
