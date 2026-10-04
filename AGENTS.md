# AGENTS.md

Working agreement for coding agents in `rp-hub75`. Human-facing orientation is in
[`README.md`](./README.md); the reasoning and the measurements are in
[`NOTES.md`](./NOTES.md) and [`plan.md`](./plan.md).

## Project

A `no_std` HUB75 LED matrix driver for the **RP2040** and **RP2350**. The **Pimoroni Interstate
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
  the row address, the latch, and the output enable. See `src/backend/programs.rs`.
* The sequence **is** the frame. The driver plays the framebuffer's own BCM
  sequence from a descriptor table — one entry per run, then a terminator — and
  knows nothing about planes, rows, or their lengths. See `src/descriptor.rs`.

### Pins

A HUB75 bus needs **fourteen consecutive** GPIOs: `out pins, 14` writes
`[OUT_BASE, OUT_BASE + 13]`, and no PIO instruction can move that window. A port
to another board is therefore a *base pin* plus the same bus order —
`R0 G0 B0 R1 G1 B1 A B C D E STB OE`, with the clock specified apart. Keep
GP0–GP13 as the default and as the reference wiring, but do not let new code
assume it.

Status: the pin selection is a parameter. `Hub75Pins16::new` takes the thirteen
signals in bus order as any GPIOs the PIO can drive — thirteen `PinId`s and a
`PioPin` bound, not `Gpio0`–`Gpio13` — derives the window's base from the *lowest*
pin, and checks at startup that each signal is on the pin its word bit is driven
to (`pins::LAYOUT` in `src/pins.rs`) and that the window fits the PIO's five-bit pin
fields (a wiring mistake stops startup rather than driving the wrong pins, since a
runtime pin number cannot be checked by the type system). `src/driver.rs` programs
`out_pins(pins.base(), …)` and `side_set_pin_base(pins.clock())`. The *word layout*
is still the `interstate75` feature of `hub75-framebuffer`, which is per-board;
base, wiring and layout are independent axes, and `LAYOUT` is the crate's copy of
the layout's numbers, kept honest by the host test in `src/pins.rs` that draws
pixels into a real framebuffer and checks which bits move.

The clock is the one pin that is not a word bit: a side-set drives it, so it has a
pin base of its own and may be the window's reserved pin (`pins::RESERVED`, where
the word holds zero) or any pin outside the window — in which case the reserved pin
is left unwired and out of `set_pindirs`. A clock on a signal pin is refused,
because the side-set wins on an overlap and would overwrite that signal silently.

### The 8-bit latched bus

The crate drives two buses, and a `Hub75Pins` **trait** carries the difference:
`Hub75Pins16` is the fourteen-pin direct-drive bus above, and `Hub75Pins8` is the
eight-signal latched one (`R1 G1 B1 R2 G2 B2 LAT OE`, with the row address in
trailing `Address` bytes rather than in the word). A `Hub75` is generic over its
pin set and requires the set's `Word` to equal the framebuffer's, so an 8-bit pin
set with a 16-bit framebuffer — or the reverse — does not type-check. The trait also
returns the `programs::Stream` (a program with the window width it writes), so
`out pins, 14` and `out pins, 8` cannot fall out of step with their word.

The 8-bit window has **no reserved bit**, so its clock is always outside the window;
`Hub75Pins8::new` refuses a clock inside it, and `pins::latched` is the layout,
held to `hub75-framebuffer` by its own host test. `examples/latched.rs` is the
wiring.

### The target

The backend is a build-time choice on two features named together: the backend,
`rphal` or `embassy`, and the chip, `rp2040` or `rp2350`. `rphal` builds against
`rp2040-hal` / `rp235x-hal` — the chip feature picks which, and each crate is
declared under the chip's target triple so the other is never built; `embassy`
builds against `embassy-rp`, whose chip comes from the same chip feature. The
selected HAL is re-exported as `rp_hub75::hal`, so the driver and examples name
`crate::hal` / `rp_hub75::hal` rather than a HAL crate directly. `src/backend/mod.rs`
is the one place the choice is read: it re-exports the selected HAL, the pin sets,
the PIO bring-up and the DMA helpers, and defines `ChannelSetup`, the single trait the
differing DMA-channel accessors need. `src/driver.rs`, `src/descriptor.rs` and
`src/pins.rs` carry no `#[cfg(feature = ...)]`.

The two `rp-hal` parts share `rp-hal-common`; `embassy-rp` uses the same PAC
register block and the same `pio` crate version, so the stream programs are shared
(`src/backend/programs.rs`). Each backend's `Hub75::new` takes its own peripheral
types — they differ enough (a `Pio` owns the state machines; a DMA channel is a
`ChannelInstance`) that no one signature fits both. `on_dma_irq` and
`wait_for_done` are backend- and executor-agnostic.

The RP2040 has two DMA interrupt lines, not four, and no atomic compare-and-swap;
both are handled in `src/driver.rs` and in `Cargo.toml` (see the `rp2040` notes
there).

GP0–GP13 is the reference wiring and the examples' default, the clock being GP11.
The base is 0 there, so nothing about the reference board changed; a non-zero base
is exercised in M6.

### A board is not a panel

The Interstate 75 W is a HUB75 **header**: the panel plugged into it is the
application's choice, so no panel is a property of the crate and a module named
after a board must not grow one. Panel geometry belongs to the framebuffer type
(`DmaFrameBuffer<NROWS, COLS, PLANES>`), and the **examples** state the panel they
are wired to (`PANEL_WIDTH`, `PANEL_HEIGHT`, `compute_rows`, `PLANES`) and let the
type follow. The library holds no panel constants. What is the board's is the pin
window, the signal order, and the crystal.

## Layout

| path | what |
|---|---|
| `src/driver.rs` | the engine and the shared driver: `Hub75`, `Hub75Config`, `Hub75Pins`, `Hub75Swap`, `on_dma_irq`, the DMA interrupt, and the divider math |
| `src/backend/mod.rs` | the HAL port: the single place the backend feature is read, re-exporting the selected HAL as `hal` and the port surface, and defining `ChannelSetup` |
| `src/backend/rphal.rs` | the `rp2040-hal` / `rp235x-hal` port: `Hub75::new`, `bring_up`, the two pin sets (`Hub75Pins16`/`Hub75Pins8`), `PioPin`, and the DMA channel accessors |
| `src/backend/embassy.rs` | the `embassy-rp` port: `Hub75::new`, `bring_up`, `Hub75Pins16`/`Hub75Pins8` (holding PIO `Pin`s), `EmbassyPins`, and the DMA channel accessors |
| `src/backend/programs.rs` | the two PIO stream programs (`hub75_stream16`, `hub75_stream8`) and `Stream`, shared by both backends (they use the same `pio` crate) |
| `src/backend/window.rs` | the bus-window checks (`check_bus`, `check_bus8`), shared by both backends |
| `src/descriptor.rs` | the engine: the frame as a descriptor table a control channel plays into the data channel's `AL3` block, plus `BusWord` (the framebuffer word's width and DMA `DATA_SIZE`) |
| `src/pins.rs` | the 16-bit HUB75 word layout as data (`pins`) and the 8-bit one (`pins::latched`), the window's bounds, and the host tests that pin each layout to `hub75-framebuffer` |
| `.cargo/config-rp2040.toml`, `.cargo/config-rp2350.toml` | the per-chip target, runner, and the `-L` path to the chip's `memory.x`; the chip config is shared by both backends |
| `memory/rp2040/memory.x`, `memory/rp2350/memory.x` | the linker memory maps, each named `memory.x` so `link.x`'s `INCLUDE memory.x` finds the chip's |
| `examples/` | `gradient`, `telemetry`, `ghost`, `ladder`, `latched` (rp-hal) and `gradient-embassy` (embassy-rp) — every example goes through `Hub75` |
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
| `./test-all.sh` | Host unit tests through `cargo test-rp2350` (the alias names the backend and chip features) with `--target` pinned to the host triple. One pass: the polarity features do not change what a test observes. |
| `./build-all.sh` | `cargo build-rp2350` for the library and every example, in the four polarity combinations. The alias names `.cargo/config-rp2350.toml` and `rphal,rp2350`; no feature is a default. |
| `./clippy-all.sh` | `cargo clippy-rp2350 --lib --examples`. Not `--all-targets`: the test harness needs `core::test`, which does not exist for this target. Same feature passes as `build-all.sh`. |
| `./fmt-all.sh` | `cargo fmt --all`, then `--check`. Do not add a `rustfmt.toml` without agreeing on it. |
| `cargo run-rp2350 --example gradient` | Builds, flashes and runs over the debug probe (`probe-rs run --chip RP235x`), and streams the `defmt` log back on the same terminal. Without a probe, flash over the USB-C bootloader instead: hold BOOT, tap RESET, `picotool load -u -v -x -t elf <elf>`. |
| `cargo build-rp2040` / `cargo run-rp2040` (e.g. `--example latched`) | The same, for the RP2040: the alias names `.cargo/config-rp2040.toml` (target `thumbv6m-none-eabi`, `probe-rs run --chip RP2040`) and the `rphal,rp2040` features; no feature is a default. `build-rp2350`/`run-rp2350` are the symmetric names. |
| `cargo build-embassy` / `cargo run-embassy` | The embassy-rp backend: the aliases name the RP2350 chip config (`.cargo/config-rp2350.toml`, the same target and probe the rp-hal build uses) and the `embassy,rp2350` features. `cargo run-embassy --example gradient-embassy` (or the `run-gradient-embassy` alias) flashes and runs the embassy example. |
| `cargo doc-rp2350` | Checks that the documentation builds (the alias names the board and the `--no-deps` flag). A `rustdoc` warning is a bug — nothing denies them here, and there is no `cargo xtask` in this crate. |

There is no CI and no `.github/`. Whatever a change claims is verified, you ran.

Hardware is the only judge of the panel. Host tests cover only what a file can be
wrong about by itself — the clock divider, the descriptor table `fill` and `patch`
write, and the pin checks `Hub75Pins16::new` and `Hub75Pins8::new` make. The last
of those includes `src/pins.rs`'s layout tests, which draw pixels into a real
`hub75-framebuffer` framebuffer of each word width and assert which word bits move:
that is what keeps `LAYOUT` and `pins::latched::LAYOUT` from
drifting away from the dependency they describe. There are no tests for the stream
program or for the drawing convention the examples use: the programs are two PIO
instructions, and the emulator that used to step it supplied its own state-machine
configuration, so it could not have caught a driver that configured the machine
differently; the colour arithmetic now lives in bare-metal examples, which cannot be
host-tested at all. A change to the panel path in `src/backend/rphal` or
`src/driver.rs` is unverified until it is flashed; say so rather than implying
otherwise.

### Logging

`defmt` over RTT, through the debug probe: `cargo run-rp2350 --example <name>` prints
it. The examples own the log — `defmt`, `defmt-rtt` and `panic-probe` are
dev-dependencies, and a log line is a plain `defmt::info!` — so three rules come
with that.

* **The library does not log, and nothing logs from an interrupt.** `defmt`'s
  `acquire` is a critical section, and `on_dma_irq` runs once a frame with the
  picture waiting on it. The driver reports its numbers through accessors —
  `divider`, `panel_clock`, `refresh_hz`, `runs_per_refresh` and `pins` — and
  examples read them from their own loops.
* **`defmt-rtt` must keep `disable-blocking-mode`.** probe-rs puts RTT into
  blocking mode, so a host that stops reading would block a log call — and a
  blocked display loop is the frozen panel this project has already met twice.
  Dropping a frame is the right failure.
* **A log line formats the examples' primitives only.** `rp235x-hal` wants
  `defmt <0.4` and `hub75-framebuffer` wants `1.0`, and two defmt majors cannot
  share an image, so neither crate's own defmt feature can be turned on — a value
  out of either has no `Defmt` impl, and `to_Hz()` of a `Rate` is how the driver's
  own numbers reach the log.

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
