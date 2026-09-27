#!/bin/bash
set -e

# Host unit tests for the rp-hub75 library.
#
# The library is no_std, but the parts that do not touch the hardware are worth
# testing on the machine you are sitting at. `thumbv8m` is set as the default
# target in .cargo/config.toml and there is no way to *run* tests there, so this
# script asks cargo to build for the host triple instead.
#
# One pass, not one per feature. No test reads a feature flag — the polarity
# features change what the library compiles, not what a test observes — so
# running the suite once per combination runs it four times with the same
# result. `build-all.sh` is what builds every combination. `20mhz` is an example
# feature and changes nothing here, and the library does not log: the examples
# own the log, through defmt as a dev-dependency.
HOST_TARGET=$(rustc -vV | sed -n 's/^host: //p')

echo "=========================================="
echo "Host tests: cargo test --lib --target $HOST_TARGET"
echo "=========================================="
cargo test --lib --target "$HOST_TARGET"

echo "=========================================="
echo "All tests succeeded!"
echo "=========================================="
