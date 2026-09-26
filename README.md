# rp-hub75

A HUB75 LED matrix driver for the **RP2350**, developed against the **Pimoroni
Interstate 75 W** (64×64, 1/32 scan).

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
  see [`programs`](src/programs.rs).
* **Two channels and no CPU between frames.** The control channel plays a
  descriptor table — one entry per repetition of every segment, then a terminator
  — into the data channel's `AL3` register block, and the data channel chains back
  to it on completion ([`descriptor`](src/descriptor.rs)). The chain re-arms itself
  as often as the frame needs, so a `reps` count costs a table entry rather than an
  interrupt: there is no ring size, no alignment rule and no power-of-two rule
  anywhere. Nothing in the driver knows what a plane or a row is, or how long one
  is, so plane-major, row-major, threshold and remapped framebuffers all work — the
  layout only changes the cost.
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
  (Eight planes flickers at this clock; see `BIT_DEPTH`.)

## Cargo features

The two groups are separate: only the library features change what the crate
compiles and how the panel is driven.

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
| `telemetry` | the same engine and panel pattern, with `wait_for_done().await` and fps/interrupt/swap counters over USB CDC (the digit on the panel is the loop’s frame count). `irqs` should track `frame` one for one: one interrupt a frame is the whole engine |
| `ghost` | a ghosting / bleed probe: four phases — a blinking dot, a raster, a diagonal and a sweeping bar — that make row bleed (adjacent, and across the frame boundary) and column bleed visible, and name their direction |

Two earlier engines lived in examples too — M3's raw single-channel stream and
M4a's six-channel ring — and so did the one-channel/descriptor pair that replaced
them; the losers are gone from the tree and what they taught is in
[NOTES.md](NOTES.md) and in git. Every example here drives the panel through
[`Hub75`](src/driver.rs), which is the only API this crate offers.

Flashing (board in BOOTSEL: hold BOOT, tap RESET):

```console
cargo run-arm --example gradient
```

There is no UART to log to — GP0/GP1 are panel data — so telemetry goes over USB
CDC (or SWD/RTT).

## Status

The engine is the descriptor one, and it is flashed: 32 runs a refresh played by a
control channel, **one interrupt a frame**, `irqs` == `swap_waited` == `frames` at
73/s measured (`telemetry`, two minutes), with `gradient` and `ghost` flashed and
looking right — straight colour ramps and no row or column bleed. The per-run
engine it replaced measured 985/s on the same panel: 13.5 interrupts a refresh,
where its own banner said 17, because the ring it used depended on where the linker
put the framebuffers. Neither that engine nor the ring is in the crate any more.
See [plan.md](plan.md) for what is done and what is next, and [NOTES.md](NOTES.md)
for the measurements and the datasheet facts the design rests on — including what
is still unverified: the panel clock ceiling, and the strobe edge and address order
as *measurements* rather than as a picture that looks right.

## Layout

| path | what |
|---|---|
| `src/driver.rs` | `Hub75`, `Hub75Config`, `Hub75Pins`, `Hub75Swap`, `on_dma_irq` |
| `src/descriptor.rs` | the engine: the frame as a descriptor table — one entry a run, then the terminator that ends it — and the swap's delta applied in place |
| `src/color.rs` | `level`, `grey`, `gamma_step` for a `BIT_DEPTH`-bit panel |
| `src/programs.rs` | the PIO program, the word↔pin layout, and the panel model tests |
| `src/pins.rs` | the Interstate 75 W pin map |
| `tools/` | MicroPython bring-up scripts, capture and GIF helpers |
| `test-all.sh`, `build-all.sh`, `clippy-all.sh`, `fmt-all.sh` | host tests and the target builds |
