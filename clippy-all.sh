#!/bin/bash
set -e

# Clippy the rp-hub75 library and its example packages.
#
# The library is linted once per library feature combination, because
# `invert-clock` and `invert-oe` select library code paths. `--lib` on the
# library, not `--all-targets`: the test harness needs `core::test`, which does
# not exist for this target.
#
# The examples are separate crates under `examples/`, each linted in its own
# directory through its own `clippy-rp2350` alias.
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
echo "Clippy (embassy library): cargo clippy-embassy --lib"
echo "=========================================="
cargo clippy-embassy --lib

# Each example crate, in its own package.
for dir in examples/*/; do
    echo "=========================================="
    echo "Clippy (example): ${dir}cargo clippy-rp2350"
    echo "=========================================="
    (cd "$dir" && cargo clippy-rp2350)
done

echo "=========================================="
echo "All clippy runs succeeded!"
echo "=========================================="
