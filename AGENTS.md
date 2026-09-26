# AGENTS.md

Working agreement for coding agents in `rp-hub75`. Human-facing orientation is in
[`README.md`](./README.md); the reasoning and the measurements are in
[`NOTES.md`](./NOTES.md) and [`plan.md`](./plan.md).

## Project

A `no_std` HUB75 LED matrix driver for the **RP2350**. The **Pimoroni Interstate
75 W** (64×64, 1/32 scan) is the *reference board* — where the pin map comes from
and where every measurement in `NOTES.md` was taken — not the only target. One
PIO state machine streams a `hub75-framebuffer` bitplane framebuffer straight at
the panel, fed by two DMA channels that hand the picture to each other: one plays
a descriptor table into the other's registers, and the CPU does nothing between
frames. Edition 2024, `rust-version = 1.85`,
no allocator.

Two facts decide most changes:

* A framebuffer word **is** the bus. With `hub75-framebuffer`'s `interstate75`
  layout, bit *n* is pin *n*, so one `out pins, 14` drives the six colour bits,
  the row address, the latch, and the output enable. See `src/programs.rs`.
* The sequence **is** the frame. The driver plays the framebuffer's own BCM
  sequence from a descriptor table — one entry per run, then a terminator — and
  knows nothing about planes, rows, or their lengths. See `src/descriptor.rs`.

### Pins

A HUB75 bus needs **fourteen consecutive** GPIOs: `out pins, 14` writes
`[OUT_BASE, OUT_BASE + 13]`, and no PIO instruction can move that window. A port
to another board is therefore a *base pin* plus the same bus order —
`R0 G0 B0 R1 G1 B1 A B C D E CLK STB OE`. Keep GP0–GP13 as the default and as the
reference wiring, but do not let new code assume it.

Status: the timing paths are board-independent already; the pin selection is not.
`Hub75Pins::new` takes concrete `Pin<Gpio0, …>` … `Pin<Gpio13, …>` arguments, and
`src/driver.rs` programs `out_pins(pins::WORD_BASE, …)` and
`side_set_pin_base(pins::CLK)`. A different consecutive run is an edit to
`src/pins.rs` and the constructor today, not a parameter. Making the base a
constructor argument, with the fourteen pins checked for consecutiveness at
startup, is open work — see `plan.md`.

## Layout

| path | what |
|---|---|
| `src/driver.rs` | `Hub75`, `Hub75Config`, `Hub75Pins`, `Hub75Swap`, `on_dma_irq` |
| `src/descriptor.rs` | the engine: the frame as a descriptor table a control channel plays into the data channel's `AL3` block |
| `src/programs.rs` | the PIO program, the word↔pin layout, the panel model tests |
| `src/color.rs` | `level`, `grey`, `gamma_step` for a `BIT_DEPTH`-bit panel |
| `src/pins.rs` | the reference wiring: the Interstate 75 W pin map (GP0–GP13) |
| `examples/` | `gradient`, `telemetry`, `ghost`, `ladder` — every example goes through `Hub75` |
| `NOTES.md` | measurements, datasheet facts, and the reasoning: the design record |
| `plan.md` | milestones: what is done, in progress, and next |
| `README.md` | orientation for a human reader |

`tools/` (MicroPython bring-up, capture, and GIF helpers) and `upstream/`
(vendored `esp-hub75` and interstate75 reference code) are **gitignored**: they
exist on a developer's machine only. Do not commit them, and do not add files
there.

`hub75-framebuffer` is a `[patch.crates-io]` **path** dependency on the sibling
checkout `../hub75-framebuffer`. A build fails without it.

## Build, test, verify

Run these from the repository root. Use the scripts instead of hand-rolled cargo
lines: each one is shaped around the bare-metal target.

| command | what, and why |
|---|---|
| `./test-all.sh` | Host unit tests. The default target is `thumbv8m.main-none-eabihf`, where tests cannot *run*, so the script asks for the host triple explicitly. Runs the four polarity combinations plus one `--no-default-features` pass. |
| `./build-all.sh` | `cargo build-arm` (= `build --release`) for the library and every example, in the four polarity combinations, plus the `--no-default-features` (silent) library and examples. |
| `./clippy-all.sh` | `cargo clippy --release --lib --examples`. Not `--all-targets`: the test harness needs `core::test`, which does not exist for this target. Same feature passes as `build-all.sh`. |
| `./fmt-all.sh` | `cargo fmt --all`, then `--check`. Do not add a `rustfmt.toml` without agreeing on it. |
| `cargo run-arm --example gradient` | Builds, flashes and runs over the debug probe (`probe-rs run --chip RP235x`), and streams the `defmt` log back on the same terminal. Without a probe, flash over the USB-C bootloader instead: hold BOOT, tap RESET, `picotool load -u -v -x -t elf <elf>`. |
| `cargo doc --no-deps` | Checks that the documentation builds. A `rustdoc` warning is a bug — nothing denies them here, and there is no `cargo xtask` in this crate. |

There is no CI and no `.github/`. Whatever a change claims is verified, you ran.

Hardware is the only judge of the panel. Host tests cover the target-independent
parts — the PIO bit accounting, the engine's weights and run counts, the colour
math. A change to the panel path in `src/programs.rs` or `src/driver.rs` is
unverified until it is flashed; say so rather than implying otherwise.

### Logging

`defmt` over RTT, through the debug probe: `cargo run-arm --example <name>` prints
it. Three rules come with that.

* **The interrupt never logs.** `defmt`'s `acquire` is a critical section, and
  `on_dma_irq` runs once a frame with the picture waiting on it. The driver logs
  its startup and nothing else; examples log from their own loops.
* **`defmt-rtt` must keep `disable-blocking-mode`.** probe-rs puts RTT into
  blocking mode, so a host that stops reading would block a log call — and a
  blocked display loop is the frozen panel this project has already met twice.
  Dropping a frame is the right failure.
* **The library's logging is the `defmt` feature, on by default**, and its call
  sites are gated on the target as well as the feature, because the host test
  harness links without `defmt.x`. `--no-default-features` builds a silent driver;
  both passes are in the scripts.

## Conventions

* `no_std`, no allocator. Use the dependencies already in `Cargo.toml`; a new one
  needs a reason.
* Module docs are `//!`, item docs are `///`. Host tests sit in
  `#[cfg(test)] mod tests` in the same file as the code they test.
* Edition 2024: unsafe attributes and impls use the new syntax
  (`#[unsafe(no_mangle)]`, `unsafe impl Send for Engine {}`). Read the existing
  `unsafe` blocks before adding one; `src/driver.rs` justifies each.
* Comments and docs state what the code does, and why when the reason is a fact —
  a register, a measurement, a hazard, a timing limit. Rejected alternatives
  belong in `NOTES.md`: do not explain a choice by arguing about the one it
  displaced (`read from the plan rather than counted …`), state the value and
  where it comes from.
* Keep `README.md`, `NOTES.md`, and `plan.md` in step with the code. A milestone
  that changes state, or a measurement that settles a question, belongs in
  `plan.md` or `NOTES.md` in the same change.
* Commit subjects are short, lowercase, and describe the change
  (`add interrupts/sec to gradient`). Do not rewrite published history.

## API documentation

Rustdoc here is **reference material**: an impersonal, third-person description
of what the code does. The rules below are adapted from esp-hal's
`DEVELOPER-GUIDELINES.md`, and they follow Simplified Technical English
([ASD-STE100](https://www.asd-ste100.org/)) plus the
[Espressif Manual of Style (EMoS)](https://mos.espressif.com/).

### Summaries

* Start with a one-sentence summary in the third-person present tense:
  `Sets the baud rate.` — not `Set the baud rate.`, and not `This function sets
  the baud rate.`
* Constructors start `Creates a new …`. Predicates start `Returns whether …`,
  with a space before the next word. When a function returns a handle, say what
  it is for: ``Returns a [`Hub75Swap`] that can be used to wait for the swap to
  complete.`` — not `… to wait for the swap to complete.`
* Keep one verb tense per summary, and conjugate coordinated verbs the same way
  (`Starts and waits … and returns …`).
* Hyphenate compound modifiers (`one-time sampling`).
* Do not use `This function`, `This struct`, `This enum`, `Note that`,
  `allows you to`, or contractions. Prefer the present tense to `will`.

### Sections

* Item docs use rustdoc's `#` sections; module docs use `##` headings.
* Order: `# Examples`, `# Panics`, `# Errors`, `# Cancellation Safety`,
  `# Safety`, `# Undefined Behavior`.
* `# Errors` names the error type and the condition
  (``[`Hub75Error::SwapInFlight`] if the previous swap has not been waited
  for.``), rather than a leading `Returns a … if …` paragraph. `# Panics` lists
  the conditions that panic. `# Examples` is for doctests; prefer
  `rust, no_run`.
* Never leave an **empty** section: if `# Examples` has no code fence, or
  `# Errors` no conditions, delete the heading.
* Do not repeat the summary inside a section body.

### Voice: STE and EMoS

* Short sentences, one idea each, about 20 words. Present tense, active voice;
  use the passive only when the actor is irrelevant and the result matters.
* Second person and the imperative mood belong in `# Examples` and module
  `## Examples` only. Everywhere else the voice is impersonal — no `you`,
  `users`, `we`, or `I`.
* One word, one meaning: keep `Returns` for fallible results, `Creates` for
  constructors, and so on. Keep the longer phrase when it is the precise one
  (`that can be used to`). Keep noun phrases to about three words.
* American English: `behavior`, `center`, `initialized` — not `behaviour`,
  `whilst`, `initialised`.
* Spell out an acronym on first use in a module (`two-wire automotive interface
  (TWAI)`), except where the short form is the familiar one (`USB`, `DMA`). Keep
  the Espressif forms: `eFuse`, `Wi-Fi`, and TRM register and field names.
* Numbers: spell out one through nine, use numerals for 10 and higher and for
  measurements (`8-bit`, `115200` baud). Space before a unit (`40 MHz`, `85 °C`,
  `10 ms`), `0x` prefix for hex, en dash for ranges (`GPIO0–GPIO3`).
* Oxford comma in lists of three or more. Do not use `/` for "or" (`UART or
  SPI`). Title case plus lowercase `mode` for mode names (`Station mode`).

### Links and hygiene

* Prefer intra-doc links in code formatting — ``[`Hub75`]`` — to raw URLs or
  markdown links. Leave a space before the link when it follows a word:
  ``Used with [`Self::register_block`] to access the register block.``
* Summaries end with ASCII punctuation. No stray or accidental characters —
  keyboard typos, dead keys, mojibake, or a letter stuck after the full stop
  (`level.ç`).

### After a bulk doc pass

A style pass alone is not enough, and there is no CI to catch the rest, so read
the changes.

1. Read the `git diff` hunk by hunk. Do not run a repository-wide
   search-and-replace on doc comments without reviewing every hunk.
2. Grep for the usual regressions before committing:
   * merged words: `whetherthe`, `whethersampling`, `with[`, `than can`
   * mixed tense: `(Starts|Checks|Returns).+ and (wait|write|return|copy)\b`
   * typos and British spellings: `Fow `, `Finishes of`, `behaviour`, `whilst`,
     `initialised`
   * empty sections: a `#` heading with no body
   * stray characters: unexpected non-ASCII in `///` or `//!` lines
3. Run `cargo doc --no-deps`.
