# rp-hub75

A HUB75 LED matrix driver for the **RP2040** and **RP2350**. The **Pimoroni
Interstate 75 W** is the *reference board* — where the pin map and the
measurements come from, and the (64×64, 1/32 scan) panel the examples default to
— not the only target: the pin base, the bus wiring, the 8- or 16-bit bus, and
the panel geometry are all parameters.

One PIO state machine streams a `hub75-framebuffer` bitplane framebuffer at the
panel; the CPU takes one interrupt per frame.

```rust
let hub75 = Hub75::new(pio, ch0, ch1, &mut resets, pins, clocks.system_clock.freq(),
                       Hub75Config::new(pac::Interrupt::DMA_IRQ_1), fb0,
                       hub75_descriptors!(Fb))?;
// ch0 streams the panel; ch1 plays the descriptor table.

let mut fb = fb1;                        // fb0 is on the panel now
loop {
    draw_into(fb);
    fb = hub75.swap(fb)?.wait();         // hand over, get the other one back
}
```

## How it works

* Framebuffer bit *n* is window pin *n* (`hub75-framebuffer`'s `interstate75`
  layout), so one PIO `out pins, 14` drives the six colour bits, the row address,
  the latch and the output enable — no conversion pass
  ([`programs`](src/backend/programs.rs)). The clock is a side-set: the one pin
  passed apart from the window (`Hub75Pins16::new`'s last argument), wired to the
  word bit `pins::RESERVED` or to a pin outside the window. The **8-bit latched**
  bus is `out pins, 8` (`Hub75Pins8`): two sub-pixels' colour, the latch and the
  output enable, the row address in trailing `Address` bytes, and — having no
  reserved bit — the clock always outside the window.
* The control channel plays a descriptor table into the data channel's `AL3`
  register block; the data channel chains back on completion
  ([`descriptor`](src/descriptor.rs)). One entry per BCM `reps`, then a
  terminator. No ring size, alignment rule or power-of-two rule. The driver knows
  no planes or rows, so plane-major, row-major, threshold and remapped
  framebuffers all work; the layout only changes the cost. The table is the
  caller's storage: `hub75_descriptors!(Fb)` sizes a `static` to the frame length,
  and the driver keeps a pointer.
* The table terminator is the only interrupt, once a frame, at the boundary. It
  depends on neither the layout nor the framebuffer's link address. A blocked
  interrupt delays the boundary; it cannot corrupt the picture.
* A swap is one address delta, added to the table at that boundary. No tearing, no
  memcpy; `swap().wait()` returns the outgoing buffer.
* Six planes is 63 scans per refresh, which is why the examples draw at six; eight
  flickers at the example clock. Depth is the framebuffer's `PLANES`.

## Cargo features

The library's `Cargo.toml` has two groups. Only the library features change how
the crate drives the panel; the target features pick the HAL. Each example package
under `examples/` has its own small feature set on top (the chip and `20mhz`).

### Target features

The target is two features named together, one from each axis: the backend
(`rphal` or `embassy`) and the chip (`rp2040` or `rp2350`). None is a default:
the aliases in `.cargo/config.toml` name a pair and the config file together, so
a build missing an axis stops instead of picking a board. `rphal` builds against
`rp2040-hal` / `rp235x-hal` (which share `rp-hal-common`); `embassy` against
`embassy-rp`. The PIO and DMA register blocks are the same shape and both HALs
use the same `pio` crate, so the engine and the stream programs are shared and
only the port differs. `.cargo/config-rp2040.toml` and
`.cargo/config-rp2350.toml` carry the per-chip target and runner settings, shared
by both backends (see the build table in [`AGENTS.md`](AGENTS.md)).

Each chip's `rp-hal` crate is declared under that chip's target triple, so
`rphal` can name both without building the wrong one — a build is
`--features rphal,rp2350`, `rphal,rp2040`, `embassy,rp2350`, or
`embassy,rp2040`.

Each *example* under `examples/` is its own package — not a workspace member, so
it carries `[profile]` and `[patch]` of its own — and it names the HAL features a
binary needs: the RT vector table and entry point, the picotool header, the
critical-section implementation, and, on the RP2040, the second-stage bootloader.
The library's base features force none of them, as `critical-section`'s README
requires of a library (two implementations in one binary fail to link), so a
downstream consumer supplies its own. Each example also declares its own `defmt`,
`panic-probe`, `embedded-graphics` and `static_cell`; the framebuffer crate is not
named — its types arrive through `rp_hub75::framebuffer`.

| feature | effect |
|---|---|
| `rphal` | the backend against `rp2040-hal` / `rp235x-hal`, which the chip feature selects |
| `embassy` | the backend against `embassy-rp`, whose chip the chip feature selects |
| `rp2350` | the RP2350 (Cortex-M33), `thumbv8m.main-none-eabihf`, `rp235x-hal` |
| `rp2040` | the RP2040 (Cortex-M0+), `thumbv6m-none-eabi`, `rp2040-hal`. Also the RP2040 `memory.x`; the examples place the second-stage bootloader in `.boot2` |

### Library features

| feature | effect |
|---|---|
| `invert-clock` | inverts the clock pin's output in the GPIO block (`IO_BANK0` `GPIOx` `CTRL` `OUTOVER`) for a panel that takes a column on the clock's falling edge. The clock is the one signal the PIO generates rather than copies out of a word, so its polarity is the pin's and one program runs either way. |
| `invert-oe` | forwards to `hub75-framebuffer/invert-oe` for a panel whose output enable is active-high. The crate's words are the only place that polarity lives, so there is one switch. |

The rest forward to `hub75-framebuffer` and change the words the driver streams —
the framebuffer's layout, and the blanking it inserts around rows:
`skip-black-pixels`, `tail-closes-latch`, `lead-blank-1` through `lead-blank-32`,
`trail-blank-1` through `trail-blank-32`, `inter-row-blank-4` through
`inter-row-blank-32`, and `reverse-row-order`. [`Cargo.toml`](Cargo.toml) is the
list.

### Example features

Each example package defines its own `rp2350`/`rp2040` (forwarded to the library),
its own `20mhz`, and a pass-through for every **Library feature** above —
`invert-clock`, `invert-oe`, `skip-black-pixels`, `tail-closes-latch`,
`lead-blank-1` through `lead-blank-32`, `trail-blank-1` through `trail-blank-32`,
`inter-row-blank-4` through `inter-row-blank-32`, and `reverse-row-order` — so an
example build selects a panel's tuning, e.g.
`cargo run-rp2350 --features invert-oe,lead-blank-4`.

| feature | effect |
|---|---|
| `20mhz` | the examples' panel clock: 10 MHz without it, 20 MHz with it, as the esp-hub75 examples do. The divider is an integer rounded *up*, so 20 MHz here lands on 18.75 MHz |
| (the tunings) | each forwards to the library, so an example build selects a panel's tuning |

## Examples

Each row is its own package under `examples/<name>/`; run one with
`cd examples/<name> && cargo run-rp2350` (or `cargo run-rp2040`, where the board
is wired for it). `build-all.sh` and `clippy-all.sh` walk the directory.

| example | what |
|---|---|
| `gradient` | the display from `esp-hub75`'s gradient demo: three colour ramps (red, green, blue, one light level per column) and a status block whose labels are left-justified while the values are right-justified — refresh rate, render time, the descriptor table's runs a frame (`runs_per_refresh()`, 32 here), and a counter ticking ten times a second, which is also the heartbeat |
| `telemetry` | the same engine and panel pattern, with `wait_for_done().await` and frames/interrupts/swaps once a second over `defmt` → RTT (the digit on the panel is the loop’s frame count). `irqs` should track the frame count one for one: one interrupt a frame is the whole engine |
| `ghost` | a ghosting / bleed probe: four phases — a blinking dot, a raster, a diagonal and a sweeping bar — that make row bleed (adjacent, and across the frame boundary) and column bleed visible, and name their direction |
| `starfield` | depth-field stars: integer projection, an inline `xorshift32` for placement, tinted by depth. Flight speed is per second, derived from `refresh_hz()`, so `20mhz` does not change the motion |
| `starfield-embassy` | `starfield` on the embassy-rp backend: the same projection, on `#[embassy_executor::main]` with `wait_for_done().await`. Built and run with `cd examples/starfield-embassy && cargo run-rp2350` |
| `ladder` | the read-back ladder: five patterns, two seconds each, whose *failure modes* name a fault rather than just looking wrong — one row pair lit means the stream stopped, columns at x = 1, 2, 3 mean the panel latches on `STB`'s rising edge, the diagonal shifted means the address lines are a row out. A regression tool; `gradient` and `ghost` are the same questions at the eye's resolution |
| `gradient-embassy` | `gradient` on the embassy-rp backend: `#[embassy_executor::main]`, a `bind_interrupts!` handler calling `on_dma_irq`, and `wait_for_done().await`. Built and run with `cd examples/gradient-embassy && cargo run-rp2350` (the `embassy` backend) |
| `gradient-latched` | the **8-bit latched** bus: the `gradient` display on `bitplane::latched` and `Hub75Pins8`, drawn through an external latch circuit on a non-zero window base (GP2–GP9, clock GP10). Needs the breadboard latch, not just the panel header |

Every example here drives the panel through [`Hub75`](src/driver.rs), which is the
only API this crate offers.

Flashing and running, with the CMSIS-DAP probe attached:

```console
cd examples/gradient && cargo run-rp2350     # flash, run, and stream the defmt log
```

Without a probe, flash over the USB-C bootloader instead (hold BOOT, tap RESET,
then `picotool load -u -v -x -t elf <elf>`).

There is no UART to log to — GP0/GP1 are panel data — so the log is `defmt` over
RTT, back up the probe's wire. `defmt-rtt` keeps `disable-blocking-mode`:
probe-rs puts RTT in blocking mode, so a host that stops reading would otherwise
block a log call, and a blocked display loop is a frozen panel.

The examples own the log: `defmt`, `defmt-rtt` and `panic-probe` are each example
package's own dependencies and a log line is a plain `defmt::info!`. The library
logs nothing, so its numbers reach the log through its accessors (`divider`,
`panel_clock`, `refresh_hz`, `runs_per_refresh`), which is where `telemetry`'s
startup line reads them. Neither the hal's nor the framebuffer's own defmt feature
can be turned on — the hal wants `defmt <0.4`, the framebuffer `1.0`, and two majors
cannot share an image.

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
