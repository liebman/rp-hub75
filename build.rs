//! Places the right linker memory map for the selected target.
//!
//! `cortex-m-rt`'s `link.x` does `INCLUDE memory.x`, so the file must be on the
//! linker's search path under that name. The two boards need different maps —
//! the RP2350's carries the boot-ROM `.start_block`/`.bi_entries` sections, the
//! RP2040's the second-stage bootloader — so this copies the one the `rp2040` or
//! `rp2350` feature selects into `OUT_DIR` as `memory.x` and puts `OUT_DIR` on
//! the search path.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    let source = if env::var_os("CARGO_FEATURE_RP2040").is_some() {
        "memory-rp2040.x"
    } else {
        "memory-rp2350.x"
    };
    println!("cargo:rerun-if-changed={source}");
    fs::copy(source, out.join("memory.x")).expect("the memory map is readable");
    println!("cargo:rustc-link-search={}", out.display());
}
