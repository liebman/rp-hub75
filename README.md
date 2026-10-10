# rp-hub75

[![Crates.io](https://img.shields.io/crates/v/rp-hub75.svg)](https://crates.io/crates/rp-hub75)
[![Documentation](https://docs.rs/rp-hub75/badge.svg)](https://docs.rs/rp-hub75)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](README.md)

A `no-std` driver for HUB75-style LED matrix panels on the RP2040 and RP2350.

The panel is refreshed over DMA with almost no CPU involvement. One PIO state
machine streams the framebuffer, two DMA channels play its Binary Code Modulation
(BCM) sequence, and the CPU takes one interrupt per frame. Pick the backend with
a single target feature:

- **`rphal-rp2350`**: the `rp-hal` backend on the RP2350 (`rp235x-hal`)
- **`rphal-rp2040`**: the `rp-hal` backend on the RP2040 (`rp2040-hal`)
- **`embassy-rp2350`**: the `embassy-rp` backend on the RP2350
- **`embassy-rp2040`**: the `embassy-rp` backend on the RP2040

Exactly one is required and none is a default.

The framebuffers work with `embedded-graphics`, so you can draw shapes, text and
images on the display, and they go to the PIO as-is; there is no per-frame
conversion step.

## Framebuffers

Use the **bitplane** framebuffers from the `hub75-framebuffer` crate. Each is
either direct-drive or latched, and its type fixes the bus:

- **Direct drive** (`framebuffer::bitplane::plain::DmaFrameBuffer`, driven by
  `Hub75Pins16`) is a 16-bit word: the colour bits, the row address, the latch and
  the output enable all go out with every pixel. It works with any standard HUB75
  panel and needs the widest pin window.
- **Latched** (`framebuffer::bitplane::latched::DmaFrameBuffer`, driven by
  `Hub75Pins8`) is an 8-bit word and needs an external address-latch circuit. The
  row address rides apart from the word, in trailing address bytes, so it uses
  fewer GPIO pins and less memory. See the `hub75-framebuffer` crate for the
  circuit.

A `Hub75` requires the pin set's `Word` to equal the framebuffer's, so an 8-bit
pin set with a 16-bit framebuffer is a compile error rather than a scrambled
picture.

The type also fixes the geometry: `NROWS` is `compute_rows(PANEL_HEIGHT)`, `COLS`
the panel width times the panels chained across, and `PLANES` the BCM depth. A
64×64, 1/32 scan panel is `DmaFrameBuffer<compute_rows(64), 64, 6>`; a 64×32 cut
is `DmaFrameBuffer<compute_rows(32), 64, 6>`.

## Refresh Rate

The panel shift clock is the word rate: one word, one column, per period. The
refresh rate is that clock divided by the words the framebuffer's BCM sequence
streams, and `Hub75::refresh_hz()` reports it. It follows the geometry, the BCM
depth (`PLANES`), and the panel clock: six planes is 63 panel scans a frame, where
eight is several times the words and visibly slower, so the examples draw at six.

The panel clock is a request, not a guarantee: the PIO divider is an integer
rounded up, so `Hub75Config::with_panel_clock_hz` lands on the requested clock or
the next slower one, and `Hub75::panel_clock_hz()` reports which.

## Hardware Requirements

- An RP2040 or RP2350 board wired to the panel. The **Pimoroni Interstate 75 W**
  is the reference board: its pin map is the wiring the examples default to, and
  its 64×64, 1/32 scan panel is the one they drive. It is not the only target —
  the pin base, the bus width, and the panel geometry are all parameters.
- A HUB75 RGB matrix panel.
- A 5 V supply capable of several amps for a large panel.
- A CMSIS-DAP debug probe for `defmt` over RTT. There is no UART to log to,
  because GP0/GP1 are panel data.

The panel's logic is 5 V while the RP2040 and RP2350 are 3.3 V; a level shifter
(for example a 74HCT245) is recommended unless the board provides one.

## Pin Configurations

A HUB75 bus is fourteen consecutive GPIOs. `Hub75Pins16::new` takes the thirteen
signals in bus order — `R0 G0 B0 R1 G1 B1 A B C D E STB OE` — plus the clock,
which is the one pin the PIO generates rather than copies out of a word:

- The window's base is the *lowest* signal pin, so any run of consecutive GPIOs
  works. The reference board starts at GP0; `gradient-latched` puts an 8-bit
  window on GP2–GP9.
- The clock is a side-set, so it may be the window's reserved pin (word bit 11,
  where the framebuffer holds zero) or any pin outside the window. A clock on a
  signal pin is refused: the side-set would overwrite that signal.
- `Hub75Pins8::new` takes `R1 G1 B1 R2 G2 B2 LAT OE` plus the clock. An 8-bit
  window has no reserved slot, so its clock is always outside it.

## Examples

Each example is its own package under `examples/<name>/`:

- `gradient`: the `esp-hub75` gradient demo — three colour ramps and a status
  block whose labels are left-justified and values right-justified: refresh rate,
  render time, the descriptor table's runs a frame, and a 10 Hz counter.
- `telemetry`: the same display with `wait_for_done().await`, logging
  frames/interrupts/swaps once a second over `defmt` → RTT.
- `ghost`: a ghosting/bleed probe — a blinking dot, a raster, a diagonal and a
  sweeping bar — that make row and column bleed visible and name their direction.
- `starfield`: depth-field stars, projected and tinted by depth; the flight speed
  derives from `refresh_hz()`, so `20mhz` does not change the motion.
- `starfield-embassy`: `starfield` on the embassy-rp backend, with
  `wait_for_done().await`.
- `ladder`: the read-back ladder — five patterns whose *failure modes* name a
  fault rather than just looking wrong. A regression tool; `gradient` and `ghost`
  are the same questions at the eye's resolution.
- `gradient-embassy`: `gradient` on the embassy-rp backend, with a
  `bind_interrupts!` handler calling `on_dma_irq` and `wait_for_done().await`.
- `gradient-latched`: the **8-bit latched** bus, on `bitplane::latched` and
  `Hub75Pins8`, through an external latch circuit on GP2–GP9 (clock GP10).

Every example drives the panel through `Hub75`, the only API this crate offers.
`build-all.sh` and `clippy-all.sh` walk the directory.

### Running the examples

The aliases name a chip config and one target feature. With a probe attached, one
command flashes, runs, and streams the `defmt` log:

```console
cd examples/gradient && cargo run-rp2350
cd examples/gradient-latched && cargo run-rp2040
```

An embassy example runs the same way:

```console
cd examples/gradient-embassy && cargo run-rp2350
```

An example forwards every library tuning, so a build selects a panel's tuning:

```console
cd examples/gradient && cargo run-rp2350 --features invert-oe,lead-blank-4
```

Each example also defines `20mhz`, its panel-clock choice: 10 MHz without it,
20 MHz with it. The divider is an integer rounded up, so 20 MHz here lands on
18.75 MHz.

### Flashing

Without a probe — the board alone on USB-C — put it in BOOTSEL (hold BOOT, tap
RESET, or hold BOOT while plugging in) and let `picotool` write the ELF's program
headers. Build first, because a `run-*` alias needs the probe for its runner:

```console
cd examples/gradient
cargo build-rp2350
picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/gradient
```

The ELF is `target/<triple>/release/<package>`: `thumbv8m.main-none-eabihf` for
the RP2350 and `thumbv6m-none-eabi` for the RP2040, the file named for the
package (`gradient`, `gradient-latched`, …). `-u` skips flash sectors that already
match, `-v` verifies the write, and `-x` reboots into the image; `-t elf` names
the type because Cargo emits no extension for picotool to read. There is no RTT
without the probe, so this flashes and runs blind — the probe is what carries the
log back.

There is no UART to log to — GP0/GP1 are panel data — so the log is `defmt` over
RTT, back up the probe's wire. `defmt-rtt` keeps `disable-blocking-mode`:
probe-rs puts RTT in blocking mode, so a host that stops reading would otherwise
block a log call, and a blocked display loop is a frozen panel.

The examples own the log: `defmt`, `defmt-rtt` and `panic-probe` are each example
package's own dependencies and a log line is a plain `defmt::info!`. The library
logs nothing, so its numbers reach the log through its accessors (`divider`,
`panel_clock_hz`, `refresh_hz`, `runs_per_refresh`), which is where `telemetry`'s
startup line reads them. Neither the hal's nor the framebuffer's own defmt feature
can be turned on — the hal wants `defmt <0.4`, the framebuffer `1.0`, and two
majors cannot share an image.

## Crate Features

Exactly one target feature is required, naming the backend and the chip together:

- `rphal-rp2350`: the `rp-hal` backend against `rp235x-hal`, for the RP2350
  (Cortex-M33), `thumbv8m.main-none-eabihf`.
- `rphal-rp2040`: the `rp-hal` backend against `rp2040-hal`, for the RP2040
  (Cortex-M0+), `thumbv6m-none-eabi`. Also the RP2040 `memory.x`; the examples
  place the second-stage bootloader in `.boot2`.
- `embassy-rp2350`: the `embassy-rp` backend, built with `embassy-rp/rp235xa`.
- `embassy-rp2040`: the `embassy-rp` backend, built with `embassy-rp/rp2040`.

None is a default: the aliases in `.cargo/config.toml` name a feature and the chip
config file together, so a build missing the choice stops instead of picking a
board. The `rphal-*` features build against `rp2040-hal` / `rp235x-hal` (which
share `rp-hal-common`); the `embassy-*` features against `embassy-rp`. The PIO and
DMA register blocks are the same shape on both chips and both HALs use the same
`pio` crate, so the engine and the stream programs are shared.

The library's tuning features:

- `invert-clock`: inverts the clock pin's output in the GPIO block (`IO_BANK0`
  `GPIOx` `CTRL` `OUTOVER`) for a panel that takes a column on the clock's falling
  edge. The clock is the one signal the PIO generates rather than copies out of a
  word, so its polarity is the pin's and one program runs either way.
- `invert-oe`: forwards to `hub75-framebuffer/invert-oe` for a panel whose output
  enable is active-high. The crate's words are the only place that polarity
  lives, so there is one switch.
- `skip-black-pixels`, `tail-closes-latch`, `lead-blank-1` through
  `lead-blank-32`, `trail-blank-1` through `trail-blank-32`, `inter-row-blank-4`
  through `inter-row-blank-32`, and `reverse-row-order`: forward to
  `hub75-framebuffer` and change the words the driver streams — the framebuffer's
  layout, and the blanking it inserts around rows. [`Cargo.toml`](Cargo.toml) is
  the list.

Each *example* package under `examples/` is its own crate, not a Cargo workspace
member, so it carries `[profile]` and `[patch]` of its own, and it names the HAL
features a binary needs: the RT vector table and entry point, the picotool header,
the critical-section implementation, and, on the RP2040, the second-stage
bootloader. The library's base features force none of them, as
`critical-section`'s README requires of a library (two implementations in one
binary fail to link), so a downstream consumer supplies its own.

## Layout

| path | what |
|---|---|
| `src/driver.rs` | `Hub75`, `Hub75Config`, `Hub75Error`, the `Hub75Pins` trait, `Hub75Swap`, `on_dma_irq`, and the divider math |
| `src/descriptor.rs` | the engine: the frame as a descriptor table — one entry a run, then the terminator that ends it — the swap's delta applied in place, `BusWord` (the word width and its DMA `DATA_SIZE`), and the `Hub75Descriptors` storage `hub75_descriptors!` allocates |
| `src/backend/` | the HAL port: `mod.rs` chooses the backend and re-exports the selected HAL and pin sets; `rphal.rs` / `embassy.rs` hold the two `Hub75Pins16` / `Hub75Pins8` sets, `PioPin`, `bring_up` and the DMA accessors; `programs.rs` the two PIO stream programs (16-bit `out pins, 14`, 8-bit `out pins, 8`) and the `Stream`; `window.rs` the bus-window checks |
| `src/pins.rs` | the 16-bit HUB75 word layout as data, the `pins::latched` 8-bit layout, the pin window's bounds, and the compile-time pins and host tests that hold each layout to `hub75-framebuffer` |
| `examples/<name>/` | the example packages (`gradient`, `ghost`, `starfield`, `ladder`, `gradient-latched`, `telemetry`, `gradient-embassy`, `starfield-embassy`), each its own crate with a `Cargo.toml`, `src/main.rs` and its own `.cargo/` config; not Cargo workspace members |
| `.cargo/config-rp2040.toml`, `.cargo/config-rp2350.toml` | per-chip target, rustflags and probe runner, plus the `-L` path to the chip's `memory.x`; named by the aliases in `.cargo/config.toml` |
| `memory/rp2040/memory.x`, `memory/rp2350/memory.x` | the linker memory maps, each named `memory.x` so `link.x`'s `INCLUDE memory.x` finds the chip's |
| `test-all.sh`, `build-all.sh`, `clippy-all.sh`, `fmt-all.sh` | host tests and the target builds |

## License

This project is dual-licensed under either of the following:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

## Contributing

Contributions are welcome. Please open a pull request.

By contributing, you agree that your submissions will be licensed under both the
Apache-2.0 and MIT licenses.

## Support

If you need help, please open an issue on the repository.
