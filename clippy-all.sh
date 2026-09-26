#!/bin/bash
set -e

# Clippy the rp-hub75 library and its examples.
#
# `--lib --examples` rather than `--all-targets`: the test harness needs
# `core::test`, which does not exist for a no_std bare-metal target.
#
# The library is linted once per library feature combination, because
# `invert-clock` and `invert-oe` select library code paths. The examples name no
# polarity, so they are linted once — and `20mhz`, the examples' feature, is read
# through `cfg!`, so both of its arms are linted in that one run.
for features in "" "invert-clock" "invert-oe" "invert-clock,invert-oe"; do
    if [ -n "$features" ]; then
        extra=(--features "$features")
    else
        extra=()
    fi
    echo "=========================================="
    echo "Clippy (library): cargo clippy --release --lib ${extra[*]}"
    echo "=========================================="
    cargo clippy --release --lib "${extra[@]}"
done

echo "=========================================="
echo "Clippy (examples): cargo clippy --release --examples"
echo "=========================================="
cargo clippy --release --examples

# The silent build is a different set of code paths (`log_info!` discarding its
# arguments), so it is linted too.
echo "=========================================="
echo "Clippy (library, no logging): cargo clippy --release --lib --no-default-features"
echo "=========================================="
cargo clippy --release --lib --no-default-features

echo "=========================================="
echo "Clippy (examples, no logging): cargo clippy --release --examples --no-default-features"
echo "=========================================="
cargo clippy --release --examples --no-default-features

echo "=========================================="
echo "All clippy runs succeeded!"
echo "=========================================="
