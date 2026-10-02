#!/bin/bash
set -e

# Clippy the rp-hub75 library and its examples.
#
# `--lib --examples` rather than `--all-targets`: the test harness needs
# `core::test`, which does not exist for a no_std bare-metal target.
#
# The `clippy-rp2350` alias names the board's config file and features (no feature
# is a default). The library is linted once per library feature combination,
# because `invert-clock` and `invert-oe` select library code paths. The examples
# name no polarity, so they are linted once — and `20mhz`, the examples' feature,
# is read through `cfg!`, so both of its arms are linted in that one run.
for features in "" "invert-clock" "invert-oe" "invert-clock,invert-oe"; do
    if [ -n "$features" ]; then
        extra=(--features "$features")
    else
        extra=()
    fi
    echo "=========================================="
    echo "Clippy (library): cargo clippy-rp2350 --lib ${extra[*]}"
    echo "=========================================="
    cargo clippy-rp2350 --lib "${extra[@]}"
done

echo "=========================================="
echo "Clippy (examples): cargo clippy-rp2350 --examples"
echo "=========================================="
cargo clippy-rp2350 --examples

echo "=========================================="
echo "All clippy runs succeeded!"
echo "=========================================="
