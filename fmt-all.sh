#!/bin/bash
set -e

# Format the rp-hub75 library and its example packages, or, with `--check`,
# report the files that are not formatted without writing to them. The examples
# are separate crates (not a Cargo workspace), so each is handled in its own
# directory. There is no `rustfmt.toml`; every crate uses the default style. CI
# passes `--check`, which must not write.
check=0
[ "$1" = "--check" ] && check=1

if [ "$check" -eq 0 ]; then
    echo "=========================================="
    echo "Formatting (library): cargo fmt"
    echo "=========================================="
    cargo fmt

    for dir in examples/*/; do
        echo "=========================================="
        echo "Formatting: ${dir}cargo fmt"
        echo "=========================================="
        (cd "$dir" && cargo fmt)
    done
fi

echo "=========================================="
echo "Checking formatting: cargo fmt --check"
echo "=========================================="
cargo fmt --check

for dir in examples/*/; do
    echo "=========================================="
    echo "Checking formatting: ${dir}cargo fmt --check"
    echo "=========================================="
    (cd "$dir" && cargo fmt --check)
done

echo "=========================================="
echo "All formatting succeeded!"
echo "=========================================="
