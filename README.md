# rp-hub75

A HUB75 LED matrix driver for the **RP2350**, developed against the **Pimoroni
Interstate 75 W** (64×64, 1/32 scan).

One PIO state machine streams a `hub75-framebuffer` bitplane framebuffer straight
at the panel, fed by two DMA channels that ping-pong with each other. While the
panel refreshes, **the CPU does nothing**: no interrupts, no descriptor tables, no
per-column work. It draws, and occasionally swaps buffers.

```rust
let hub75 = Hub75::new(pio, ch_ctrl, ch_data, &mut resets, pins, clocks.system_clock.freq(),
                       Hub75Config::new(), fb0)?;

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
* One DMA channel streams the framebuffer's **own BCM sequence**, run by run: its
  completion interrupt advances the cursor and re-arms it
  ([`engine`](src/engine.rs)). Nothing in the driver knows what a plane or a row
  is, or how long one is, so plane-major, row-major, threshold and remapped
  framebuffers all work — the layout only changes the cost.
* Where the layout allows it — a segment that is a power of two and aligned to it,
  checked per run — the DMA's ring repeats the segment in hardware, so `reps`
  costs no interrupts at all.
* A framebuffer swap is **one delta**: segment offsets are layout constants, so
  the interrupt applies the switch at a frame boundary. No tearing, no memcpy,
  and `swap().wait()` hands the outgoing buffer back.
* The interrupt is load-bearing — it *is* the refresh (32 runs per refresh
  plane-major, 1024 row-major for this panel). A blocked interrupt pauses the
  picture; it cannot corrupt it. `Hub75::runs_per_refresh()` reports the cost of
  whichever layout you chose.
* Six planes is 63 panel scans a refresh, so the frame is the panel clock over
  129 024 words: 13.8 ms (≈72 Hz) at the examples' default 10 MHz, and 6.9 ms
  (≈145 Hz) with the `20mhz` feature, which the divider rounds to 18.75 MHz.
  (Eight planes flickers at this clock; see `BIT_DEPTH`.)

## Cargo features

| feature | effect |
|---|---|
| `20mhz` | the examples' panel clock: 10 MHz without it, 20 MHz with it, as the esp-hub75 examples do. The divider is an integer rounded *up*, so 20 MHz here lands on 18.75 MHz — the clock every rate in [`NOTES.md`](NOTES.md) was measured at |
| `invert-oe` | forwards to `hub75-framebuffer/invert-oe` for a panel whose output enable is active-high. The crate's words are the only place that polarity lives, so there is one switch. |

## Examples

| example | what |
|---|---|
| `gradient` | the display from `esp-hub75`'s gradient demo: three colour ramps (red, green, blue, one light level per column) and a status block whose labels are left-justified while the values are right-justified — refresh rate, render time, the interrupt rate, and a counter ticking ten times a second, which is also the heartbeat. The interrupt rate is the panel's own check on the engine: measured, not derived, so it should read `runs_per_refresh()` × the refresh rate — and a lost DMA ring would show as 32 runs a refresh instead of 10 |
| `telemetry` | the same engine and panel pattern, with `wait_for_done().await` and fps/interrupt/swap counters over USB CDC (the digit on the panel is the loop’s frame count) |
| `ghost` | a ghosting / bleed probe: four phases — a blinking dot, a raster, a diagonal and a sweeping bar — that make row bleed (adjacent, and across the frame boundary) and column bleed visible, and name their direction |

The two earlier engines (M3's raw single-channel stream and M4a's six-channel ring)
lived in examples too; those are gone from the tree — what they taught is in
[NOTES.md](NOTES.md) and in git. Every example here now drives the panel through
[`Hub75`](src/driver.rs), which is the only API this crate offers.

Flashing (board in BOOTSEL: hold BOOT, tap RESET):

```console
cargo run-arm --example gradient
```

There is no UART to log to — GP0/GP1 are panel data — so telemetry goes over USB
CDC (or SWD/RTT).

## Status

M5: the driver, double buffering and the interrupt path are implemented and
host-tested; the panel has been flashed with M4a's engine (which rendered
correctly). See [plan.md](plan.md) for what is done and what is next, and
[NOTES.md](NOTES.md) for the measurements and the datasheet facts the design
rests on — including the parts that are still unverified on hardware (the strobe
edge, the clock ceiling, and which of GP0/GP1 is red).

## Layout

| path | what |
|---|---|
| `src/driver.rs` | `Hub75`, `Hub75Config`, `Hub75Pins`, `Hub75Swap`, `on_dma_irq` |
| `src/engine.rs` | the crate's BCM sequence as a plan, the cursor that walks it, and the ring decision |
| `src/color.rs` | `level`, `grey`, `gamma_step` for a `BIT_DEPTH`-bit panel |
| `src/programs.rs` | the PIO program, the word↔pin layout, and the panel model tests |
| `src/pins.rs` | the Interstate 75 W pin map |
| `tools/` | MicroPython bring-up scripts, capture and GIF helpers |
| `test-all.sh`, `build-all.sh`, `clippy-all.sh`, `fmt-all.sh` | host tests and the target builds |
