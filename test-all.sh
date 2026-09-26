#!/bin/bash
set -e

# Host unit tests for the rp-hub75 library.
#
# The library is no_std, but the parts that do not touch the hardware — today the
# PIO programs, whose bit accounting the panel is unforgiving about — are worth
# testing on the machine you are sitting at. `thumbv8m` is set as the default
# target in .cargo/config.toml and there is no way to *run* tests there, so this
# script asks cargo to build for the host triple instead.
#
# The polarity features change what the crate compiles — the clock edge, and the
# level that blanks the panel — so the tests run once per combination. `20mhz` is
# an example feature and changes nothing here.
HOST_TARGET=$(rustc -vV | sed -n 's/^host: //p')

for features in "" "invert-clock" "invert-oe" "invert-clock,invert-oe"; do
    if [ -n "$features" ]; then
        extra=(--features "$features")
    else
        extra=()
    fi
    echo "=========================================="
    echo "Host tests: cargo test --lib --target $HOST_TARGET ${extra[*]}"
    echo "=========================================="
    cargo test --lib --target "$HOST_TARGET" "${extra[@]}"
done

echo "=========================================="
echo "All tests succeeded!"
echo "=========================================="
