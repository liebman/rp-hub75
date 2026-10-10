# AGENTS.md

Working agreement for coding agents in `rp-hub75`. Architecture and orientation live
in [`README.md`](./README.md); do not duplicate them here. This file holds the rules a
change must not break, and the commands that verify it.

## Invariants

* A framebuffer word **is** the bus. With `hub75-framebuffer`'s `interstate75` layout,
  bit *n* is pin *n*, so one `out pins, 14` drives the six colour bits, the row
  address, the latch and the output enable. See `src/backend/programs.rs`.
* The sequence **is** the frame. The driver plays the framebuffer's own BCM sequence
  from a descriptor table — one entry per run, then a terminator — and knows nothing
  about planes, rows or their lengths. See `src/descriptor.rs`.
* A HUB75 bus is **fourteen consecutive** GPIOs: `out pins, 14` writes
  `[base, base + 13]`, and no PIO instruction moves that window. A port to another
  board is a *base pin* plus the same bus order
  (`R0 G0 B0 R1 G1 B1 A B C D E STB OE`), clock apart. GP0–GP13 is the reference
  wiring and the examples' default, clock GP11; do not let new code assume it. A
  non-zero base is exercised in `examples/gradient-latched` (GP2–GP9, clock GP10).
* `Hub75Pins16::new` takes any GPIOs the PIO can drive — thirteen `PinId`s and a
  `PioPin` bound, not `Gpio0`–`Gpio13` — derives the base from the *lowest* pin, and
  checks at startup that each signal sits on the pin its word bit is driven to
  (`pins::LAYOUT`, `src/pins.rs`) and that the window fits the PIO's five-bit pin
  fields. The word layout is `hub75-framebuffer`'s `interstate75` feature; base,
  wiring and layout are independent axes. `LAYOUT` is this crate's copy of the
  layout's numbers, pinned to `hub75-framebuffer::pinmap::PINMAP` by a `const _`
  check in `src/pins.rs` and held honest by the host test there.
* The clock is the one pin that is not a word bit: a side-set drives it, so it may be
  `pins::RESERVED` (where the word holds zero) or any pin outside the window. A clock
  on a signal pin is refused — the side-set wins an overlap and would overwrite that
  signal silently.
* Two buses, and the `Hub75Pins` trait carries the difference: `Hub75Pins16` is the
  fourteen-pin direct-drive bus, `Hub75Pins8` the eight-signal latched one
  (`R1 G1 B1 R2 G2 B2 LAT OE`, row address in trailing `Address` bytes). A `Hub75`
  requires the pin set's `Word` to equal the framebuffer's, so a mismatched pair does
  not type-check; the word width selects the program (`programs::stream_for`), so
  `out pins, N` cannot fall out of step with the word. The 8-bit window has no
  reserved bit, so its clock is always outside it (`pins::latched`, pinned to
  `LATCHED_PINMAP`).
* The backend is a build-time choice on two features named together — backend
  (`rphal` or `embassy`) and chip (`rp2040` or `rp2350`); neither is a default.
  `src/backend/mod.rs` is the only place the choice is read; `src/driver.rs`,
  `src/descriptor.rs` and `src/pins.rs` carry no `#[cfg(feature = ...)]`. The two
  `rp-hal` parts share `rp-hal-common`, and `embassy-rp` uses the same PAC block and
  the same `pio` crate, so `src/backend/programs.rs` is shared; each backend's
  `Hub75::new` still takes its own peripheral types.
* The library names only what the library needs. A binary's RT vector table and entry
  point, picotool header, RP2040 second-stage bootloader and critical-section
  implementation are named by its example package. A library does not supply
  `critical-section` (`critical-section`'s README requires this). Examples are
  separate packages, not `[[example]]` targets: each PAC emits a `device.x` under
  `rt`, and an rp-hal binary and an embassy binary cannot share a link.
* The library does not log, and nothing logs from `on_dma_irq`: `defmt`'s `acquire`
  is a critical section and the picture waits on the handler. `defmt-rtt` keeps
  `disable-blocking-mode`. Two defmt majors cannot share an image, so neither the
  HAL's nor the framebuffer's own defmt feature is enabled.
* The RP2040 has two DMA interrupt lines (the RP2350 has four) and no atomic
  compare-and-swap; both are handled in `src/driver.rs` and `Cargo.toml`.
* A new library tuning feature is forwarded by every example package and listed in
  the README feature tables.
* A board is not a panel. The Interstate 75 W is a HUB75 **header**: panel geometry is
  the framebuffer type's and the example's (`PANEL_WIDTH`, `PANEL_HEIGHT`,
  `compute_rows`, `PLANES`). The library holds no panel constants; the board
  contributes the pin window, the signal order and the crystal.

## Dependencies

`hub75-framebuffer` is the published crate (`0.13.0`, the `interstate75` feature). To
develop against the sibling checkout, uncomment the `[patch.crates-io]` block at the
end of the root `Cargo.toml` — and in each `examples/<name>/Cargo.toml`, which is its
own workspace. The module map is the README `## Layout` table.

## Build, test, verify

Run these from the repository root. Use the scripts instead of hand-rolled cargo
lines: each one is shaped around the bare-metal target.

| command | what, and why |
|---|---|
| `./test-all.sh` | Host unit tests through `cargo test-rp2350` (the alias names the backend and chip features) with `--target` pinned to the host triple. One pass: the polarity features do not change what a test observes. |
| `./build-all.sh` | The library in the four polarity combinations via `cargo build-rp2350`, then each example package with `(cd examples/<name> && cargo build-rp2350)` and the RP2040 example with `cargo build-rp2040`. The library alias names `.cargo/config-rp2350.toml` and `rphal,rp2350`; no feature is a default. |
| `./clippy-all.sh` | `cargo clippy-rp2350 --lib` on the library — not `--all-targets`: the test harness needs `core::test`, which does not exist for this target — and `cargo clippy-rp2350` in each example package. |
| `./fmt-all.sh` | `cargo fmt` in the root and in each example package (they are separate crates), then `--check`. `./fmt-all.sh --check` is the CI pass: it reports and does not write. Do not add a `rustfmt.toml` without agreeing on it. |
| `cd examples/gradient && cargo run-rp2350` | Builds, flashes and runs over the debug probe (`probe-rs run --chip RP235x`), and streams the `defmt` log back on the same terminal. |
| `cd examples/gradient-latched && cargo build-rp2040` | The same, for the RP2040: the example's alias names `.cargo/config-rp2040.toml` (target `thumbv6m-none-eabi`, `probe-rs run --chip RP2040`) and its `rp2040` feature; no feature is a default. Every example defines `build-rp2040`/`run-rp2040`. |
| `cargo build-embassy` (root) / `cd examples/gradient-embassy && cargo run-rp2350` | The embassy-rp backend: the root alias builds the library against `embassy-rp`; the example package builds and flashes the embassy example through its own alias. |
| `cargo doc-rp2350` | Checks that the documentation builds (the alias names the board and the `--no-deps` flag). A `rustdoc` warning is a bug — nothing denies them here, and there is no `cargo xtask` in this crate. |

CI is `.github/workflows/ci.yml`: five jobs on `ubuntu-latest`, one each for
`./fmt-all.sh --check`, `./test-all.sh`, `./clippy-all.sh`, `./build-all.sh` and
`cargo doc-rp2350`, run on every pull request and push to `main`. It hosts no
probe, so it never flashes or runs a binary; a hardware claim beyond the checks
is still one you made by running the command.

## Tests

Host tests cover only what a file can be wrong about by itself: the clock divider, the
descriptor table `fill` and `patch` write, and the pin checks `Hub75Pins16::new` and
`Hub75Pins8::new` make. `src/pins.rs`'s layout tests draw pixels into a real
`hub75-framebuffer` framebuffer of each word width and assert which word bits move; the
layouts are also pinned to the dependency's exported `pinmap` at compile time, so a
layout change fails the build. There are no tests for the stream program (two PIO
instructions) or for the drawing convention the examples use, which lives in bare-metal
examples and cannot be host-tested at all.

## Conventions

* `no_std`, no allocator. Use the dependencies already in `Cargo.toml`; a new one needs
  a reason.
* Module docs are `//!`, item docs are `///`. Host tests sit in `#[cfg(test)] mod tests`
  in the same file as the code they test.
* Edition 2024: unsafe attributes and impls use the new syntax (`#[unsafe(no_mangle)]`,
  `unsafe impl Send for Engine {}`). Read the existing `unsafe` blocks before adding
  one; `src/driver.rs` justifies each.
* Comments and docs state what the code does, and why when the reason is a fact — a
  register, a hazard, a timing limit. Do not explain a choice by arguing about the one
  it displaced (`read from the plan rather than counted …`): state the value and where
  it comes from.
* **Document the hardware, not the bench.** No `## Status`, no claims about what was
  flashed, measured, tested or left unverified, no "looks right". Flash *instructions*
  stay; flash *results* do not.
* Keep `README.md` in step with the code.
* Commit subjects are short, lowercase, and describe the change
  (`add interrupts/sec to gradient`). Do not rewrite published history.

## API documentation

Rustdoc here is **reference material**: an impersonal, third-person description of what
the code does.

* Start with a one-sentence summary in the present tense: `Sets the baud rate.` — not
  `Set the baud rate.`, and not `This function sets the baud rate.` Constructors start
  `Creates a new …`; predicates start `Returns whether …`, with a space before the next
  word. When a function returns a handle, say what it is for: ``Returns a [`Hub75Swap`]
  that can be used to wait for the swap to complete.``
* Do not use `This function`, `This struct`, `This enum`, `Note that`, `allows you to`,
  or contractions. Prefer the present tense to `will`. Keep one verb tense per summary.
* Item docs use rustdoc's `#` sections; module docs use `##` headings. Order:
  `# Examples`, `# Panics`, `# Errors`, `# Cancellation Safety`, `# Safety`,
  `# Undefined Behavior`. `# Errors` names the error type and the condition; `# Panics`
  lists the conditions that panic. Never leave an **empty** section — delete the
  heading. Do not repeat the summary inside a section body.
* One word, one meaning: `Returns` for fallible results, `Creates` for constructors.
  Keep the longer phrase when it is the precise one (`that can be used to`). Keep noun
  phrases to about three words. Hyphenate compound modifiers (`one-time sampling`).
* Spell out an acronym on first use in a module, except where the short form is the
  familiar one (`USB`, `DMA`). Keep TRM register and field names.
* Numbers: numerals for 10 and higher and for measurements (`8-bit`, `115200` baud), a
  space before a unit (`40 MHz`, `85 °C`), `0x` prefix for hex, en dash for ranges
  (`GPIO0–GPIO3`). Oxford comma in lists of three or more.
* American English (`behavior`, `center`, `initialized`), except `colour`, which this
  crate uses throughout — keep it consistent, and do not make a spelling-only change.
* Prefer intra-doc links in code formatting — ``[`Hub75`]`` — to raw URLs. Summaries
  end with ASCII punctuation; no stray characters (`level.ç`).

### After a bulk doc pass

1. Read the `git diff` hunk by hunk. Do not run a repository-wide search-and-replace on
   doc comments without reviewing every hunk.
2. Grep for the usual regressions before committing:
   * merged words: `whetherthe`, `whethersampling`, `with[`, `than can`
   * mixed tense: `(Starts|Checks|Returns).+ and (wait|write|return|copy)\b`
   * typos: `Fow `, `Finishes of`, `behaviour`, `whilst`, `initialised`
   * filler: `This module`, `This function`, `In summary`, `Ultimately`, `seamlessly`,
     `It is important to note`
   * bench diary: `flashed|untested|unverified|unrecorded|measured on|looks? right`
   * empty sections: a `#` heading with no body
   * stray characters: unexpected non-ASCII in `///` or `//!` lines
3. Run `cargo doc --no-deps`.
