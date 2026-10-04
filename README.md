# rp-hub75

A HUB75 LED matrix driver for the **RP2040** and **RP2350**, developed against the
**Pimoroni Interstate 75 W** (64×64, 1/32 scan).

One PIO state machine streams a `hub75-framebuffer` bitplane framebuffer straight
at the panel, and one interrupt a frame is all the CPU does about it. A **control**
DMA channel plays a descriptor table — the framebuffer's own BCM sequence, one
entry per run, then a terminator — into the data channel's `AL3` register block,
and the data channel chains back to it on every completion. The sequence *is* the
frame, so the driver knows nothing about planes or rows.

```rust
let hub75 = Hub75::new(pio, ch0, ch1, &mut resets, pins, clocks.system_clock.freq(),
                       Hub75Config::new(pac::Interrupt::DMA_IRQ_1), fb0)?;
// ch0 streams the panel; ch1 plays the descriptor table.

let mut fb = fb1;                        // fb0 is on the panel now
loop {
    draw_into(fb);
    fb = hub75.swap(fb)?.wait();         // hand over, get the other one back
}
```

## How it works

* A framebuffer word *is* the bus: with `hub75-framebuffer`'s `interstate75`
  layout, bit *n* is pin *n*, so one PIO `out pins, 14` drives the six colour
  bits, the row address, the latch and the output enable. No conversion pass —
  see [`programs`](src/programs.rs). The clock is the exception: the PIO
  generates it with a side-set, so it is the one pin specified apart from the
  window (`Hub75Pins16::new`'s last argument) and the one word bit
  (`pins::RESERVED`) that carries no signal. The **8-bit latched** bus is the same
  idea at half the width: one `out pins, 8` drives two sub-pixels' colour, the
  latch and the output enable (`Hub75Pins8`), the row address rides in trailing
  address bytes, and — having no reserved bit — the clock is always a pin outside
  the window.
* **Two channels and no CPU between frames.** The control channel plays a
  descriptor table — one entry per repetition of every segment, then a terminator
  — into the data channel's `AL3` register block, and the data channel chains back
  to it on completion ([`descriptor`](src/descriptor.rs)). The chain re-arms itself
  as often as the frame needs, so a `reps` count costs a table entry rather than an
  interrupt: there is no ring size, no alignment rule and no power-of-two rule
  anywhere. Nothing in the driver knows what a plane or a row is, or how long one
  is, so plane-major, row-major, threshold and remapped framebuffers all work — the
  layout only changes the cost. The table itself is **the caller's storage**:
  `hub75_descriptors!(Fb)` sizes a `static` to the frame's own length (33 entries,
  268 bytes, for a six-plane plane-major frame) and the driver keeps only a pointer,
  so its own static is 44 bytes.
* **One interrupt a frame.** It is the table's terminator, at the boundary, and it
  depends neither on the layout (32 runs a refresh plane-major, 1024 row-major for
  this panel) nor on where the linker put the framebuffer: 73/s at the examples' 10
  MHz setting, measured. A blocked interrupt delays a frame boundary; it cannot
  corrupt the picture.
* A framebuffer swap is **one delta**: the incoming buffer differs from the one on
  the panel by a single number, and the interrupt adds it to the table's addresses
  at the boundary — one store per run, on the frames that swap and no others. No
  tearing, no memcpy, and `swap().wait()` hands the outgoing buffer back.
* Six planes is 63 panel scans a refresh, so the frame is the panel clock over
  129 024 words: 13.8 ms (≈72 Hz) at the examples' default 10 MHz, and 6.9 ms
  (≈145 Hz) with the `20mhz` feature, which the divider rounds to 18.75 MHz.
  Eight planes flickers at this clock, which is why the examples draw at six;
  the depth is the framebuffer's `PLANES`, not a driver constant. The crate docs
  have the geometry-to-framebuffer mapping.

## Cargo features

There are three groups. Only the library features change how the crate drives the
panel; the target features pick the HAL, and the example features the examples.

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

| feature | effect |
|---|---|
| `rphal` | the backend against `rp2040-hal` / `rp235x-hal`, which the chip feature selects |
| `embassy` | the backend against `embassy-rp`, whose chip the chip feature selects |
| `rp2350` | the RP2350 (Cortex-M33), `thumbv8m.main-none-eabihf`, `rp235x-hal` |
| `rp2040` | the RP2040 (Cortex-M0+), `thumbv6m-none-eabi`, `rp2040-hal`. Also the RP2040 `memory.x`; the examples place the second-stage bootloader in `.boot2` |

### Library features

| feature | effect |
|---|---|
| `invert-clock` | inverts the clock pin's output in the GPIO block (`IO_BANK0` `GPIOx` `CTRL` `OUTOVER`) for a panel that takes a column on the clock's falling edge. The clock is the one signal the PIO generates rather than copies out of a word, so its polarity is the pin's and one program runs either way. RP2350-only, like the driver. |
| `invert-oe` | forwards to `hub75-framebuffer/invert-oe` for a panel whose output enable is active-high. The crate's words are the only place that polarity lives, so there is one switch. |

The rest forward to `hub75-framebuffer` and change the words the driver streams —
the framebuffer's layout, and the blanking it inserts around rows:
`skip-black-pixels`, `tail-closes-latch`, `lead-blank-1` through `lead-blank-32`,
`trail-blank-1` through `trail-blank-32`, `inter-row-blank-4` through
`inter-row-blank-32`, and `reverse-row-order`. [`Cargo.toml`](Cargo.toml) is the
list.

### Example features

| feature | effect |
|---|---|
| `20mhz` | the examples' panel clock: 10 MHz without it, 20 MHz with it, as the esp-hub75 examples do. The divider is an integer rounded *up*, so 20 MHz here lands on 18.75 MHz — the clock every rate in [`NOTES.md`](NOTES.md) was measured at |

## Examples

| example | what |
|---|---|
| `gradient` | the display from `esp-hub75`'s gradient demo: three colour ramps (red, green, blue, one light level per column) and a status block whose labels are left-justified while the values are right-justified — refresh rate, render time, the descriptor table's runs a frame (`runs_per_refresh()`, 32 here), and a counter ticking ten times a second, which is also the heartbeat |
| `telemetry` | the same engine and panel pattern, with `wait_for_done().await` and frames/interrupts/swaps once a second over `defmt` → RTT (the digit on the panel is the loop’s frame count). `irqs` should track the frame count one for one: one interrupt a frame is the whole engine |
| `ghost` | a ghosting / bleed probe: four phases — a blinking dot, a raster, a diagonal and a sweeping bar — that make row bleed (adjacent, and across the frame boundary) and column bleed visible, and name their direction |
| `starfield` | the view out of a spaceship's window: stars live in a depth field ahead of the camera, project out from the center as their depth falls, and are re-spawned on the far plane when they pass the near plane or leave the panel. Each star takes a tint from a small palette (blue-white through white and yellow to red) and is faded by depth, so distant stars are dim and near ones bright. Integer projection (no FPU) and an inline `xorshift32` for placement — no new dependency, and the flight speed is per second, derived from `refresh_hz()`, so the `20mhz` feature does not change it |
| `ladder` | the read-back ladder: five patterns, two seconds each, whose *failure modes* name a fault rather than just looking wrong — one row pair lit means the stream stopped, columns at x = 1, 2, 3 mean the panel latches on `STB`'s rising edge, the diagonal shifted means the address lines are a row out. A regression tool; `gradient` and `ghost` are the same questions at the eye's resolution |
| `gradient-embassy` | `gradient` on the embassy-rp backend: `#[embassy_executor::main]`, a `bind_interrupts!` handler calling `on_dma_irq`, and `wait_for_done().await`. Built and run with `cargo build-embassy` / `cargo run-embassy --example gradient-embassy` (the `embassy,rp2350` features) |
| `latched` | the **8-bit latched** bus: the `gradient` display on `bitplane::latched` and `Hub75Pins8`, drawn through an external latch circuit on a non-zero window base (GP2–GP9, clock GP10). Needs the breadboard latch, not just the panel header |

Three earlier engines lived in examples too — M3's raw single-channel stream, M4a's
six-channel ring, and the one-channel/descriptor pair that replaced them — and so
did M3's original ladder; the losers are gone from the tree, what they taught is in
[NOTES.md](NOTES.md) and in git, and the ladder is back as `examples/ladder.rs`.
Every example here drives the panel through [`Hub75`](src/driver.rs), which is the
only API this crate offers.

Flashing and running, with the CMSIS-DAP probe attached:

```console
cargo run-rp2350 --example gradient        # flash, run, and stream the defmt log
```

Without a probe, flash over the USB-C bootloader instead (hold BOOT, tap RESET,
then `picotool load -u -v -x -t elf <elf>`).

There is no UART to log to — GP0/GP1 are panel data — so the log is `defmt` over
RTT, back up the probe's wire. The old path was USB CDC, which needed no probe but
could stall the display loop on a host that stopped reading; RTT drops a frame
instead, and that is the one thing a log must never do to a panel.

The examples own the log: `defmt`, `defmt-rtt` and `panic-probe` are
dev-dependencies and a log line is a plain `defmt::info!`. The library logs nothing,
so its numbers reach the log through its accessors (`divider`, `panel_clock`,
`refresh_hz`, `runs_per_refresh`), which is where `telemetry`'s startup line reads
them. Neither dependency's own defmt feature can be turned on — the hal wants
`defmt <0.4`, the framebuffer `1.0`, and two majors cannot share an image.

## Status

The engine is the descriptor one, and it is flashed: 32 runs a refresh played by a
control channel, **one interrupt a frame**, `irqs` == `swap_waited` == `frames` at
73/s measured (`telemetry`, two minutes), with `gradient` and `ghost` flashed and
looking right — straight colour ramps and no row or column bleed. The per-run
engine it replaced measured 985/s on the same panel: 13.5 interrupts a refresh,
where its own banner said 17, because the ring it used depended on where the linker
put the framebuffers. Neither that engine nor the ring is in the crate any more.
See [plan.md](plan.md) for what is done and what is next, and [NOTES.md](NOTES.md)
for the measurements and the datasheet facts the design rests on. The ladder has now
been read off the panel — all five rungs as they should be, which closes M3's
read-back and settles the latch edge (this panel takes the row on `STB`'s falling
edge) and the R0/G0/B0 colour order. What is still unverified is the panel clock
ceiling: the ladder at a faster divider, and an analyser for the number.

The **8-bit latched** path and the **RP2040** build are new and compiled — the
`latched` example links for both `thumbv6m-none-eabi` and
`thumbv8m.main-none-eabihf`, and the host tests cover the 8-bit word layout — but
they are **not yet flashed**: they need the external latch circuit on a breadboard,
and no RP2040 board has driven a panel here. Their numbers belong in
[NOTES.md](NOTES.md) once they are.

## Layout

| path | what |
|---|---|
| `src/driver.rs` | `Hub75`, `Hub75Config`, the `Hub75Pins` trait, `Hub75Pins16`, `Hub75Pins8`, `Hub75Swap`, `PioPin`, `on_dma_irq` |
| `src/descriptor.rs` | the engine: the frame as a descriptor table — one entry a run, then the terminator that ends it — the swap's delta applied in place, `BusWord` (the word width and its DMA `DATA_SIZE`), and the `Hub75Descriptors` storage `hub75_descriptors!` allocates |
| `src/programs.rs` | the two PIO stream programs (16-bit `out pins, 14`, 8-bit `out pins, 8`), the `Stream` that carries a program with its window, and the word↔pin layout |
| `src/pins.rs` | the 16-bit HUB75 word layout as data, the `pins::latched` 8-bit layout, the pin window's bounds, and the host tests that hold each layout to `hub75-framebuffer` |
| `.cargo/config-rp2040.toml`, `.cargo/config-rp2350.toml` | per-board target, rustflags and probe runner, named by the aliases in `.cargo/config.toml` |
| `.cargo/config-rp2040.toml`, `.cargo/config-rp2350.toml` | per-chip target, rustflags and probe runner, plus the `-L` path to the chip's `memory.x`; named by the aliases in `.cargo/config.toml` |
| `memory/rp2040/memory.x`, `memory/rp2350/memory.x` | the linker memory maps, each named `memory.x` so `link.x`'s `INCLUDE memory.x` finds the chip's |
| `tools/` | MicroPython bring-up scripts, capture and GIF helpers |
| `test-all.sh`, `build-all.sh`, `clippy-all.sh`, `fmt-all.sh` | host tests and the target builds |
