#!/bin/bash
set -e

# Host unit tests for the rp-hub75 library.
#
# The library is no_std, but the parts that do not touch the hardware are worth
# testing on the machine you are sitting at. There is no default target — each
# board's target lives in its `.cargo/config-*.toml`, loaded only by the aliases —
# so the script pins the host triple explicitly.
#
# One pass, not one per feature. No test reads a feature flag — the polarity
# features change what the library compiles, not what a test observes — so
# running the suite once per combination runs it four times with the same
# result. `build-all.sh` is what builds every combination. `20mhz` is an example
# feature and changes nothing here, and the library does not log: the examples
# own the log, through defmt as a dev-dependency.
#
# The `test-rp2350` alias names the board feature `rphal-rp2350` (no feature is a
# default), because the library needs a HAL to compile; the host triple is added
# here.
HOST_TARGET=$(rustc -vV | sed -n 's/^host: //p')

echo "=========================================="
echo "Host tests: cargo test-rp2350 --target $HOST_TARGET"
echo "=========================================="
cargo test-rp2350 --target "$HOST_TARGET"

echo "=========================================="
echo "All tests succeeded!"
echo "=========================================="
