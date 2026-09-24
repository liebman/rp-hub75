# rp-hub75 — working notes

Hardware bring-up notes for the Pimoroni **Interstate 75 W (RP2350)** and the
plan for a Rust HUB75 driver in this repo.

## Hardware

| | |
|---|---|
| Board | Pimoroni Interstate 75 W (RP2350), SKU PIM727 — USB-C, RM2 wireless, buttons A/B/BOOT/RESET |
| Panel | single 64x64 HUB75 (1/32 scan, 5 address lines A–E) |
| USB serial | `/dev/cu.usbmodem*` (MicroPython), or a `RP2350` mass-storage volume in bootloader mode |
| Default CPU clock | 150 MHz (MicroPython); heap ~443 KB free at rest |
| Flash | 2 MB littlefs partition in this build; treat as 4 MB total until `picotool info` confirms |

## Phase 1: MicroPython bring-up (done)

Official firmware, from <https://github.com/pimoroni/interstate75/releases/latest>
(v0.0.5). The Interstate 75 builds moved out of `pimoroni-pico` into this repo.

```
i75w_rp2350-v0.0.5-micropython-with-filesystem.uf2   # firmware + every example
i75w_rp2350-v0.0.5-micropython.uf2                   # firmware only
```

Flashing: hold **BOOT**, tap **RESET**, then copy the `.uf2` to `/Volumes/RP2350`.
The `-with-filesystem` build **erases files already on the board**.

### Verified configuration

```
DISPLAY      = DISPLAY_INTERSTATE75_64X64
PANEL_TYPE   = Interstate75.PANEL_GENERIC      # PANEL_FM6126A not needed
COLOR_ORDER  = Interstate75.COLOR_ORDER_RGB
STB_INVERT   = False
```

Confirmed on hardware with `tools/panel_bringup.py`: each solid colour matches
its label, the border/diagonals/centre marker are where they should be, the
left/right row stubs pair up every 8 rows, and A/B/BOOT drive the onboard
WS2812 and serial log.

### Measured performance (64x64, MicroPython, 150 MHz)

From `tools/example_reel.sh` — these are the numbers a Rust driver should beat:

| Example | Measured | Notes |
|---|---|---|
| `rainbow.py` | 43 ms/frame (~23 fps) | full-frame `create_pen_hsv` per pixel |
| `fast_fire.py` | 49 ms/frame (~20 fps) | viper-optimised fire, 258x68 heat array |
| `gol.py` | 64 ms/frame (~15.6 fps) | Game of Life, viper + ptr32 tricks |
| `gif.py` | ~4.8 fps | PNG decode per frame, not a driver limit |

### Modules in the v0.0.5 RP2350 build

| Present | Missing |
|---|---|
| `interstate75`, `hub75`, `picographics`, `picovector`, `plasma`, `pngdec`, `adcfft`, `pimoroni`, `pimoroni_i2c`, `qwstpad`, `ezwifi` | `ulab`, `jpegdec` |

So `glorious_fire.py` and `ulab/the_matrix.py` cannot run on this build, and
`cover-art.py` needs its own PNGs.

### API notes learned the hard way

* `SWITCH_A` / `SWITCH_B` / `SWITCH_BOOT` are **module-level** constants in
  `interstate75`, not attributes of the `Interstate75` class.
* `Interstate75(display=…, panel_type=…, stb_invert=…, color_order=…, pen_type=…)`,
  then `i75.update()`, `i75.set_led(r, g, b)`, `i75.switch_pressed(SWITCH_A)`, `i75.i2c`.
* `DISPLAY_INTERSTATE75_128X128` is a special case that drives a 256x64 output.

## Pin map (from the upstream driver)

Pimoroni's C driver (`pimoroni-pico`, `drivers/hub75/hub75.{hpp,cpp,pio}`) has no
RP2350 conditionals, so this is the shared layout — confirmed working on this
board by the Phase 1 tests:

| Function | GPIO |
|---|---|
| R0 G0 B0 R1 G1 B1 (data) | 0 1 2 3 4 5 |
| A B C D E (row select) | 6 7 8 9 10 |
| CLK (active high) | 11 |
| STB / LAT (active high) | 12 |
| OE (active low) | 13 |
| Buttons A / B / BOOT | 14 / 15 / 22 |
| WS2812 status LED | 18 |
| I2C (Qw/ST) SDA / SCL | 20 / 21 |
| Wireless (RM2 / CYW43-class) | 23 24 25 29 |

PIO: **pio0**, SM0 = data, SM1 = row select. Bit depth **10** (BCM) with a
10-bit gamma LUT, row pairs interlaced (row *N* and *N*+height/2 adjacent).

## Tools in this repo

| File | Purpose |
|---|---|
| `tools/panel_bringup.py` | Self-labelling colour screens, geometry and row tests, button/LED check. Run with `mpremote run` |
| `tools/example_reel.sh` | Plays a sequence of upstream examples on the panel, N seconds each |
| `tools/make_gif_frames.py` | Generates a dependency-free 64x64 PNG animation for `gif.py` |
| `tools/upload_gif_frames.sh` | Pushes those frames to `/gif` on the board |
| `tools/capture_usb.py` | Reads N seconds of USB CDC output from the Rust firmware |
| `upstream/interstate75` | Pimoroni's MicroPython firmware repo, branch `i75w-64x64` (all examples patched from 128x128/256x64 to 64x64) |

Host tooling is in `~/.venvs/mpremote` (`mpremote 1.29.0`); `picotool 2.3.1` is
installed via Homebrew. Note `mpremote` takes one file per invocation and needs
a moment to release the port, so scripts retry.

## Phase 2: Rust driver — constraints already known

* No Rust HUB75 crate targets RP2350 today: `hub75-pio` is RP2040 + `rp2040-hal 0.9`,
  `pimoroni_interstate75` is an RP2040 BSP with a blinky example. This repo is
  where that gap gets filled.
* `hub75-framebuffer 0.12.0` (used by `esp-hub75`) is MCU-agnostic — only
  `bitfield`, `embedded-dma`, `embedded-graphics` and optional `defmt` — so its
  `plain` / `bitplane` / `latched` / `tiling` code and `FrameBuffer::BCM_SEQUENCE`
  model can be reused directly on RP2350.
* **Bit depth ceiling: 8 planes, not the 10 the reference driver uses.** Every
  bitplane framebuffer asserts `PLANES` is `1..=8`
  (`bitplane/{plain,latched}/{row,frame}.rs`), `plain.rs` asserts `BITS <= 8`,
  and `BCM_SEQUENCE_CAPACITY` is 10 — "8 bit-planes plus an inter-row gap plus an
  end-of-row trailer" — while row-major framebuffers need `PLANES + gap + tail`
  slots. The framebuffer colour type is also 8 bits per channel, so extra planes
  would not add range by themselves. Beat Pimoroni's 10-bit gamma table with a
  gamma LUT applied at draw time; going beyond 8 planes means extending
  `hub75-framebuffer` (raise the asserts, grow the capacity, widen the pixel
  type) and doubling the MSB-plane reps, which costs refresh — which is exactly
  why Pimoroni's own driver caps `brightness` to 6 bits on RP2350 and why
  `matrix-clock` runs 1–3 planes on ESP32.
* Target `thumbv8m.main-none-eabihf` (Cortex-M33, installed) or
  `riscv32imac-unknown-none-elf` (Hazard3, installed).
* **GP0 and GP1 are HUB75 data lines**, so UART0 logging over the debug pins is
  not an option: log over USB CDC, or SWD/RTT with a CMSIS-DAP probe.
* Flashing a Rust binary replaces the MicroPython firmware; re-flash the UF2 to
  get the examples back.

## Phase 2 — M0 status (built and verified, not yet flashed)

Crate skeleton at the repo root:

| File | What it holds |
|---|---|
| `Cargo.toml` | `rp-hub75` (lib `rp_hub75`), edition 2024, deps `rp235x-hal 0.4.0` (`binary-info`, `critical-section-impl`, `rt`), `hub75-framebuffer 0.12.0`, `embedded-graphics 0.8.2` |
| `rust-toolchain.toml` | stable + `thumbv8m.main-none-eabihf` |
| `.cargo/config.toml` | target, `--nmagic -Tlink.x`, `cortex-m33`, `picotool` runner, `build-arm` / `run-arm` aliases |
| `memory.x` | RP2350 4 MiB flash, 512 KB RAM, `.start_block` / `.bi_entries` / `.end_block` for the boot ROM and picotool |
| `src/pins.rs` | the verified I75W pin map as constants |
| `src/lib.rs` | re-exports `hub75_framebuffer` as `framebuffer`, panel constants (`PANEL_WIDTH/HEIGHT`, `ROW_PAIRS`, `BIT_DEPTH`) |
| `examples/hello_usb.rs` | USB CDC banner + heartbeat, the M0 proof of life |
| `build-all.sh` / `clippy-all.sh` / `fmt-all.sh` | same shape as the esp-hub75 scripts |

Verified without hardware: `cargo build-arm --examples` clean (no linker
warnings), `clippy --lib --examples` clean, `cargo fmt --check` clean, and
`picotool info -a` on the ELF reports `name: hello_usb`, `version: 0.1.0`,
`target chip: RP2350`, `image type: ARM Secure`.

Verified on hardware: flashed with the `picotool` cargo runner (no debug probe
needed), re-enumerated as `/dev/cu.usbmodemrp_hub751`, and streamed the banner
plus a once-a-second heartbeat (`1 2 3 … 11` — it keeps running).

That first flash exposed a real bug worth remembering: `SerialPort::write` only
accepts what fits in the endpoint buffer and returns how much of it it took, so
the banner lost its tail. `UsbLog`, the writer in `hello_usb.rs` (destined for
the crate itself), now polls the USB stack until every byte is queued. That
matters because USB CDC is the driver's only logging channel — GP0/GP1 are
HUB75 data lines.

Re-flashed with the fix: the whole banner arrives (230 bytes vs 152 before).

```
rp-hub75 on an Interstate 75 W (RP2350)
panel    64x64 (32 row pairs, 10 bit BCM)
data     GP0-GP5
rowsel   GP6-GP10
clk/stb/oe  GP11/12/13
buttons  GP14/15/22, status LED GP18, Qw/ST GP20/GP21
heartbeat: 1 2 3 4 5 6 7 8 9
```

The board also ran continuously for 2.5 minutes across a capture session
(heartbeat 12 → 153), so clock setup and USB are stable.

One dead end recorded for the future: `rp235x-hal 0.4` has no
`rom_data::reset_to_usb_boot` (unlike rp2040-hal), so reflashing still needs the
BOOT+RST button dance.

Flashing (wipes MicroPython):

```
# hold BOOT, tap RESET, then:
cargo run-arm --example hello_usb
cat /dev/cu.usbmodem*          # the banner and heartbeat
```

Notes about the toolchain choice:

* Edition 2024 needs `#[unsafe(link_section = "…")]` for the boot block and
  binary-info statics.
* We are on `stable` (1.98.1), so the nightly-only `rustfmt.toml` options used
  by the other repos are not applied here. Adding that file (and a nightly
  toolchain) is a one-line change if you want the same formatting rules.
* `clippy --all-targets` cannot work on bare metal (`core::test` does not
  exist), hence `--lib --examples` in `clippy-all.sh`.

## Phase 2 — M1/M2 status (built, flashed, M2b rebuilt after a real bug)

| File | What it holds |
|---|---|
| `src/programs.rs` | `hub75_data`: the colour-data PIO program, **one 32-bit FIFO word per panel column**, plus `ROW_CLOCKED_IRQ` and a host test that walks the assembled program |
| `examples/panel_static.rs` | M1 (blanked clock), M2a (row pair 0 white), M2b (two diagonals over every row pair) |
| `test-all.sh` | host unit tests (`cargo test --lib --target <host>`; the default target in `.cargo/config.toml` is `thumbv8m` and cannot be run) |

### What the panel showed, and why (the lesson worth keeping)

M2a was right: row pair 0 lit white on both halves, so data order, clock, latch and
address lines were all correct. M2b was not. The photo showed staircases with a
period of about 16 columns, in **all six colours instead of white and blue**, with
lit pixels in *both* halves where the code had lit only one, and segments that did
not chain up into the intended diagonal.

Cause: the stream packed six colour bits per pixel into continuous 32-bit words.
The PIO's OSR refills only at `pull_threshold`, and autopull here leaves it at its
reset value, 32 bits (`pull_threshold` defaults to 0, which the hardware reads as
32; Pimoroni's SDK config spells the same thing with an explicit `32`). Six does
not divide 32, so a word boundary lands mid-pixel — and an `out` that reaches the
end of a word *cannot* stall halfway: it shifts out the two bits it has left plus
four zeroes. So each word handed the panel 36 bits instead of 32, a row became
**72 clock edges instead of 64**, and the pixel phase drifted four bits per word,
re-aligning every three words = every 16 pixels. That is the 16-column period in
the photo, and the reason the segments never chained up.

Why it looked like confetti rather than a shifted image: a panel only keeps the
**last 64 bits per channel**, so everything before that window is gone, and the six
colour bits are `R0 G0 B0` for the top half then `R1 G1 B1` for the bottom half — a
pixel field that straddles a boundary puts some of its bits in the top half and the
rest in the bottom half, at different columns, in whatever colour the misplaced
bits happen to spell. Hence "white" pixels showing up as red, green and blue.

Fix: **one FIFO word per column, every time.** Pimoroni's reference driver does
exactly this (`hub75_data_rgb888` takes one 32-bit record per pixel, RGB in three
10-bit fields), and it is why that driver never has this class of bug. The column
program is four instructions — `out null, 9` (drop the entry's address, latch,
dummy and OE bits), `out pins, 6` (colour to GP0-GP5, CLK low), `nop side 1` (CLK
rising edge, so the panel shifts the column in), `out null, 17` — 9 + 6 + 17 = 32,
so the OSR refills on the next column's word and the fields cannot drift. The word
layout is the one `hub75-framebuffer`'s bitplane entries use, so the hand-built
words in the example are already the words the DMA backend will stream.

The other half of the fix is *when* the CPU may latch. The program counts columns
(`x`, 31 down; `y`, two halves, because `set` immediates are five bits and
`set x, 63` does not assemble) and raises `irq 0` after the last column's clock
edge, so the CPU knows the shift registers hold exactly one row before it touches
the address lines or the strobe. Note what does *not* need blanking: the panel
keeps displaying the previous row pair while the next one is clocked in, because
HUB75 LEDs are driven from the output latches, not from the shift registers. Only
the address move and the latch pulse are blanked (~1.2 µs per row), so the pattern
runs at ~2.6% duty — the 1/32 scan limit — instead of the ~0.7% the first attempt
managed by holding OE high for the whole shift. Sequence: blank → address → latch
→ unblank, since the strobe captures the address lines as well as the colour data.

Numbers: four PIO cycles per column at a `/6` divider (25 MHz PIO) → 6.25 MHz
panel clock, 40 ns clock-high, 160 ns per column, 10.24 µs per row, ~370 µs per
frame (~2.7 kHz).

### First read back from the panel (M2b, after the stream fix)

The fix landed: no confetti, both halves lit, and each arm exactly where the code
put it — the white arm in the top half starting in the upper-left corner, the blue
arm in the bottom half starting at the bottom-centre and running up to the right.
That settles three hardware unknowns:

* **no column mirroring** — the first column clocked is the panel's leftmost column
  (the white arm starts at the upper-left corner, not the upper-right);
* **no half swap** — `R0 G0 B0` lights the top half and `R1 G1 B1` the bottom half;
* **the row order is straight** — the arm descends one column per row pair, and
  neither arm runs off the end of the panel.

What looked wrong was the *pattern*, not the driver. The two arms were drawn at
mirrored x one half apart (`x == row` in the top half, `x == 63 - row` in the bottom
half), so by construction they cannot touch: the white arm ends at the panel centre
and the blue arm sits in the bottom-right quadrant. A poor choice for a test that is
supposed to be unambiguous. M2b now draws **one diagonal from the top-left corner to
the bottom-right corner** — white where `x == row` (top half), blue where
`x == 32 + row` (bottom half) — so the two arms join at the middle of the panel. A
column shift or a mirrored half kinks or mirrors the line, a wrong row order breaks
it, and the colour change marks exactly where the halves meet.

### Gotchas worth remembering

* `PIOBuilder::from_installed_program` defaults to `ShiftDirection::Right`
  (least-significant bit first, what the packed words assume). The deprecated
  `from_program` defaults to `Left`. `panel_static` sets it explicitly now.
* `set` immediates are five bits: `set x, 63` is a compile error ("SET argument
  out of range: 63"), hence the nested loop.
* The `pio` crate is a *dependency* now, not just a dev-dependency, because the
  program lives in the library. Its `pio_asm!` assembles at compile time, so a
  program-level mistake is a build error, not a panel-shaped mystery.
* Host tests do work for this crate — `.cargo/config.toml` scopes its rustflags
  to `thumbv8m`, so `test-all.sh` (`cargo test --lib --target <host triple>`)
  runs the PIO program test on the laptop. `clippy --all-targets` still cannot.

### Constraints found for M3/M4 (DMA + BCM)

*(Written before the crate's `interstate75` feature existed. The first two bullets
describe the default layout and the problem it caused, both of which the feature
solved — the M3 section below is what happened in the end. The BCM budget
arithmetic is still the one that matters.)*

* **Entry layout** (`hub75-framebuffer 0.12.0`, `bitplane::plain`, 16-bit words),
  in the crate's **default** (ESP32-ish) order: bits 0-4 row address, bit 5 latch,
  bits 6-7 dummy, bit 8 output enable (active low), bits 9-14 `R1 G1 B1 R2 G2 B2`,
  bit 15 dummy. RP2350 DMA can widen a 16-bit source into the PIO's 32-bit FIFO,
  so the colours land exactly where `out null, 9; out pins, 6` expects them.
* **PIO pin mapping on this board** is the awkward part: `out` has one contiguous
  range from a base (GP0-5 = data ✔), `set` is exactly five pins (GP6-10 = the
  address ✔) and side-set can take GP11/GP12/GP13 (CLK, strobe, OE —
  consecutive ✔), but side-set values are per-instruction constants. So the entry's
  own latch/OE bits cannot drive those pins directly the way they do on ESP32's
  parallel bus; the choices are a conditional branch in the data program or a
  second state machine, which is what Pimoroni does (`hub75_row` runs the address
  with `out pins, 5` and the latch/OE with side-set, while its data SM builds the
  six GPIO bits in the ISR and `mov pins, ::isr`). The addressing in the entries is
  also *below* the colours (bits 0-4), the opposite order to the I75W's pins (data
  GP0-5, address GP6-10), so streaming a word straight at GP0-13 with one `out`
  cannot work: entry bit 11 is B1 but GP11 is CLK.
* **BCM budget.** The row-major bitplane layout streams
  `2^0 + … + 2^(PLANES-1)` column passes per row (255 at 8 planes), ~522k column
  clocks per panel refresh. At this milestone's 6.25 MHz that is 83 ms (12 fps), so
  either the column rate comes up (25 MHz → 21 ms, 48 fps) or the plane count comes
  down. The DMA also has to keep the FIFO fed: a starved state machine stalls, which
  costs brightness rather than correctness, so an underfeed shows up as flicker, not
  as corruption.

### M3: one state machine, one DMA channel, the framebuffer out as-is

`src/programs.rs` holds **one** program, `hub75_stream`, and
`examples/panel_static.rs` streams a real `hub75-framebuffer` bitplane
framebuffer through it with **one** DMA channel. M1 and M2a are still there as
CPU-driven bring-up phases (a blanked clock, then row pair 0 white at full
brightness, latched by the CPU); M3 hands every HUB75 pin to PIO and never
touches them again.

**The word layout is what collapsed four state machines into one.** `out pins, N`
writes `[OUT_BASE, OUT_BASE + N - 1]` and `OUT_BASE` can only be set when the
state machine is built — no PIO instruction moves it. A word that puts the
address in bits 0-4 and the colours in bits 9-14, as the crate's own default
layout does, therefore cannot be streamed at a panel wired `R0 G0 B0 R1 G1 B1
A-E CLK LAT OE`: the fields are in the opposite order, so each contiguous window
needs its own machine (which is what the first cut of M3 did, and what Pimoroni's
driver still does in its own way). The crate's **`interstate75` feature** changes
the word instead: bit *n* is pin *n* for the whole bus —

| word bits | pins | signal |
|---|---|---|
| 0-2 | GP0-2 | `R0 G0 B0` |
| 3-5 | GP3-5 | `R1 G1 B1` |
| 6-10 | GP6-10 | `A B C D E` |
| 11 | GP11 | reserved — the PIO drives it |
| 12 | GP12 | `STB`, the latch |
| 13 | GP13 | `OE`, active low |

— so one `out pins, 14` puts a whole word on the bus, and the only thing the PIO
still *generates* rather than copies out of the word is the clock.

**The program is two instructions**, one word per panel column:

```text
.wrap_target
    out pins, 14     side 0   ; the word -> GP0-13, clock low
    nop              side 1   ; rising edge: the panel shifts this column in
.wrap
```

The panel's clock is edge triggered, so two cycles are enough: the word changes
with the clock at its idle level (low for this panel, the HUB75 convention) and a
full PIO cycle later the rising edge shifts it into the panel.
`hub75_stream(true)` is the same two instructions with the side-set values
swapped — clock high between words, and the panel takes the column on the falling
edge — so the only difference between the two variants is one bit of side-set.

**The one invariant** (tested on the host, see below): *one FIFO word per
column*. `pull_threshold` is 14, the bits one `out pins, 14` shifts out, so the
OSR refills exactly on a column boundary. Sixteen — the obvious value, one whole
word — would leave the word's two spare top bits in the OSR and put every
following word two bits out of place. That is the M2b bug in a new costume, and
the emulator catches it: it asserts that every word is consumed exactly.

**The one hair: bit 11.** The reserved clock slot sits inside the `out` window, so
every word drives GP11 as well as the side-set does, and bit 11 is always zero. On
the normal variant that costs nothing, because the `out` runs on the cycle whose
side-set holds the clock low and the two agree. The inverted variant is where they
disagree: its `out` runs with the side-set asking for the clock high while the word
writes a zero there. **The PIO's rule settles it** — RP2350 datasheet §11.5.6: *"If
a side-set overlaps with an OUT/SET performed by that state machine on the same
cycle, the side-set takes precedence in the overlapping region"* (§11.2.6 states
the same thing for the pin-mapping chapter) — so GP11 ends that cycle high, the
clock falls cleanly on the next one, and the inverted program is exactly the normal
one with a bit of side-set flipped. The emulator applies the side-set over the
instruction's own pin write to model that.

The emulator also asserts that the two windows only ever overlap on the clock pin.
A side-set aimed at another bus pin would *win* against the word on every cycle —
silent corruption of that pin rather than an error — which is exactly the kind of
fault worth catching on the host, over a real framebuffer's 2048 words and word by
word.

**The other rule from the same section, for when a second machine joins in.** The
one above settles a single state machine's own overlap. When two machines write the
same pin, §11.2.6 says: *"For each individual GPIO output (level and direction
separately), PIO considers all 8 writes that may have occurred on that cycle, and
applies the write from the highest-numbered state machine."* So a second machine
(M6's latched framebuffer, or the latch-on-its-own-machine fallback for the strobe)
only has a say on a pin the streaming machine also writes if its index is *higher*
than the streaming machine's. Worth knowing before it surprises someone.

**The host emulation earned its keep twice.** The first time it caught two bugs
before any flashing, both in the four-machine design this replaced: `hub75_addr`
had four instructions instead of five, so the address machine ran a word ahead of
the data machine, and the latch stayed asserted into the *next* word's clock edge.

The second time was the panel model itself, and it is the more interesting
lesson: the emulator's first panel had **a latch per row pair**, which looks
plausible and is wrong. A HUB75 panel has *one* set of column latches and a row
decoder — the LEDs are driven from the latches, and the address lines pick which
row pair those latches light — so every pair shows the same latched content, and
only one pair is lit at a time. With a latch per pair the address timing is
invisible: the emulator "latches" each row into its own storage, and the crate's
deferred address change looks like a bug (an earlier draft of these notes claimed
the picture comes out one row out because of it). With one shared latch set the
same stream comes out **in place**, and the reason is worth stating plainly: the
address the crate writes while a row is lit is the address of the row *already*
in the latches, which is exactly the address a panel needs. The address is not
lagging — it is one slot behind the data being clocked in, which is what the
display wants, and `slot_s_carries_the_address_of_the_row_it_displays` pins that
down.

The emulator now models the panel as one shift register per half, one shared pair
of output latches, the address lines selecting which pair is lit, and OE gating
the light — plus a `Strobe` choice for *when* the strobe copies the shift
registers, which is the one thing bus timing cannot settle (see below). It runs a
real `DmaFrameBuffer<32, 64, 1>` drawn with `embedded-graphics` through the real
assembled program, one PIO cycle at a time, and asserts the resulting picture:
*the framebuffer's words are what the panel shows*.

**The DMA is free-running.** One channel reads half-words from the framebuffer
segment and writes them to the PIO FIFO, paced by its DREQ. `TRANS_COUNT.MODE =
TRIGGER` makes it reload its own count when it is triggered, and `CHAIN_TO`
pointing at itself makes completion the trigger, so the block repeats for ever: no
CPU, no interrupts, no descriptors. Pacing is not optional — a machine whose FIFO
runs dry stalls, and a stalled machine means a stalled clock, which is a frozen
panel. (With one machine there is no pin sharing left to get wrong; PIO pin
*values* are shared across the state machines of a block, which is why the
four-machine design had to keep every program inside its own window.)

**Numbers.** Two PIO cycles per word at a `/3` divider (50 MHz PIO) gives a 40 ns
word and a **25 MHz panel clock** with a 20 ns clock-high and a full cycle of
setup before the edge that samples it. One plane is one pass per row, so a
refresh is 32 x 64 = 2048 words = 82 µs (12.2 kHz), and the 1/32 scan gives each
pixel 63 of its pair's 64 words of slot time — about 1.9% duty. Eight planes is
255 passes per row = 522 240 words = 21 ms (48 fps): the BCM walker's problem, and
the reason the single-plane case is a useful halfway house. The divider is the
knob if the ribbon misbehaves — `/2` is 37.5 MHz, `/4` is 18.75 MHz, which is the
15 MHz-odd comfort zone the earlier milestones ran in.

**What to look for on hardware.** The panel now shows a ladder of five patterns,
two seconds each, drawn one after another into the framebuffer the DMA is
already streaming:

| # | pattern | what it should show | what a wrong picture means |
|---|---|---|---|
| A | every row pair, every column, both halves white | all 64 rows white | only rows 0 and 32 white: the address lines are stuck; all dark: the words are not arriving |
| B | top half white | rows 0-31 white, 32-63 dark | both halves lit: the halves are crossed; a left/right split: the colour pins are not where `pins.rs` says |
| C | bottom half white | A's mirror | as B |
| D | red, green, blue columns at x = 0, 1, 2 | three one-pixel vertical lines down the left edge | at the *right* edge: the shift direction is reversed; a staircase: the columns slide per row; more than three dots per row: the word's fields are landing in the wrong places; **dots at x = 1, 2, 3: the panel latched on the strobe's rising edge — see below** |
| E | the M2b diagonal | one unbroken 45° line, corner to corner, white above the join and blue below, `x == y` **in place** | a kink, a mirrored arm or a column shift: a real fault; every row shifted by a whole row: the address lines are a row out |

Every rung is drawn identically into every row pair, so what they test is the
plumbing, one piece at a time.

**The one thing the ladder has to tell us: the strobe.** `hub75-framebuffer` puts
the latch bit in the last column's word, so on the bus `STB` rises at the start of
that word — one cycle *before* its clock edge — and falls at the start of the next
row's first word. That is right if `STB` is the column shift registers' latch clock
(`RCK` on a 74HC595 and its relatives) and the copy completes when the strobe is
released: by then the last column is in, and the row is whole. A panel that
transferred on the strobe's *rising* edge would capture one column early, which
the ladder shows as rung D's three dots at x = 1, 2, 3 instead of 0, 1, 2 (and as
a stray dot at the left edge where the shift register holds a stale bit from the
row before). The emulator models both — `Strobe::AtRelease` and `Strobe::AtRise`
in `programs.rs` — and if the hardware turns out to want the latch a word later,
the crate already has the feature for it (`tail-closes-latch`), with a second
state machine for the latch alone as the fallback.

### M3 on hardware, first cut: M2a passed, the stream stopped after one pass

*(History: this is the four-state-machine design that the `interstate75` layout
and the one-machine program replaced. The diagnosis it produced — and the DMA ring
it led to — still stand.)*

M1 and M2a behaved exactly as before. M3 did not: the panel ended up **frozen**,
with two rows lit (0 and 32 — the pair address 0 selects) carrying a handful of
saturated dots each and everything else dark.

That picture is a stopped stream, not a scrambled picture. A stopped stream
leaves the state machines holding their last pin values, so the address
lines stop on one value and the panel scans only that row pair — which is why
exactly two rows are lit and the rest of the panel is black. The row pair is the
one the address lines froze on; the dots in it are whatever the output latches
happened to catch before the stream died, and their colours are all saturated
because a `PLANES == 1` framebuffer only carries the MSB of each channel
(`set_pixel` uses bit 7), so the only representable colours are the eight
primaries. That last detail is worth keeping: **saturated-only colours are the
signature of a bitplane framebuffer with few planes, not of a broken one.**

### The bug: a self-trigger does not rewind the read address

`stream_forever` looped the pass with `TRANS_COUNT.MODE = TRIGGER_SELF` and
`CHAIN_TO` pointing at the channel itself — the documented idiom for an endless
ring-buffer transfer, and *only half* of it was implemented. The RP2350 register
descriptions in `rp235x-pac` (they carry the datasheet text) settle it:

* the trigger aliases say "Alias for channel 0 TRANS_COUNT register. This is a
  trigger register (0xc). Writing a nonzero value will **reload the channel
  counter** and start the channel" — the counter, not the address;
* `TRANS_COUNT.MODE = 0x1` re-triggers the channel when the count hits zero, and
  its documentation describes the use case as "an endless **ring-buffer** DMA",
  which is the tell: a ring is what makes the address wrap;
* `RING_SIZE` is "Size of address wrap region. If 0, don't wrap. For values n > 0,
  only the lower n bits of the address will change. This wraps the address on a
  `(1 << n)` byte boundary" — **note `1 << n`, where the RP2040 used
  `1 << (n + 2)`** — and "ring sizes between 2 and 32768 bytes are possible".

Without a ring, pass two starts at the end of the plane and reads the 4096 bytes
*after* the framebuffer — BSS and stack — so the stream is garbage from then on
until the DMA wedges. The fix, now in the example:

* `RING_SEL = 0` (wrap the *read* address) and `RING_SIZE = log2(bytes)`, which is
  12 for one plane of a 64x64 panel: exactly 4096 bytes;
* the segment therefore has to be a power of two no larger than 32768 bytes *and*
  aligned to its own size, which is what `#[repr(align(4096))] struct Aligned(Fb)`
  is for; `stream_forever` asserts both, so a framebuffer that cannot be ringed
  fails loudly instead of freezing the panel;
* the M4 plan does not change: one ring loops one segment, but a multi-plane
  refresh needs a *different repetition count per segment*, which is a DMA restart
  (or a descriptor chain) rather than a free-running loop.

### The host side has a panel

`programs.rs` grew a panel emulator, which is what turned "the panel froze" from a
guess into a diagnosis and what makes the expected picture executable. The model,
after the correction described above:

* **one shared pair of output latches** — the panel has one set, not one per row
  pair — and one shift register per half. The latches are what the LEDs are driven
  from, the five address lines select which row pair they light, and OE gates the
  light. So a row is shown *after* the row that clocked it in, and the address on
  the wires has to be the one whose data is in the latches, which is what the
  crate writes.
* **a `Strobe` choice**, because the one thing pin levels cannot settle is *when*
  the strobe copies the shift registers into the latches (see above).
  `Strobe::AtRelease` gives the in-place diagonal; `Strobe::AtRise` gives the
  one-column-right picture that ladder rung D would reveal.
* `the_panel_shows_the_diagonal_where_the_framebuffer_put_it` runs the real words
  through the real program, one PIO cycle at a time, throws the first pass away
  (that is the panel filling its latches) and asserts the second: `x == row` on all
  64 rows, one lit pixel per row, and 63 of a slot's 64 words of duty per pair —
  the 1/32 scan limit, less the one word each row needs blanked for its latch and
  address change. It prints the picture too, so it doubles as the reference for a
  photo.
* The test harness keeps the pins and the state machine across passes, because the
  DMA's ring never restarts anything: the last latch of a pass is released by the
  first word of the next one. A harness that reset the pins at the seam lost that
  edge and produced a convincing off-by-one-row picture that had nothing to do with
  the driver.
* `slot_s_carries_the_address_of_the_row_it_displays` pins the address field down
  to the words themselves: slot 0 carries address 31, slot 1 carries 0, and so on,
  because the row being displayed is the previous slot's and that is the row the
  address has to name.

Still unverified on hardware: which of GP0/GP1 is red and which is green (rung D
is the first pattern that can tell them apart), the panel's clock ceiling at two
PIO cycles a word, and which edge this panel's strobe takes the row on.

Next: flash `examples/panel_static.rs` and report **which rung of the ladder
misbehaves** — a photo per pattern, or just the odd ones out, is ideal. If A shows
all 64 rows white the stream is running and E should already be the diagonal; if A
stops at one row pair the stream is stopping again, and the next step is to look at
CLK, LAT, OE and GP0 on the logic analyser (or read back `ch_read_addr`, the
transfer count and the PIO's `FDEBUG`/`TXF` over USB CDC, which is what
`tools/capture_usb.py` exists to print: GP0/GP1 belong to the panel, so there is no
UART). Then M4 — the BCM walker: stream a multi-plane framebuffer's `BCM_SEQUENCE`
(row by row, segment by segment, with each segment's repetition count), one DMA
restart per segment, and draw it with `embedded-graphics`. Pimoroni's reference PIO
programs are in `pimoroni-pico/drivers/hub75/hub75.pio` for comparison (their
buffer format and BCM scheme differ: bit-sliced RGB888 plus OE-pulse-width
weighting, one scan per plane), and `drivers/plasma/ws2812.pio` is there for the
onboard status LED.
