//! A HUB75 LED matrix driver for the RP2350, developed against the Pimoroni
//! Interstate 75 W.
//!
//! The panel is refreshed from PIO plus DMA with no CPU involvement at all,
//! and the framebuffers come from the [`framebuffer`] crate that backs
//! `esp-hub75` — it is MCU-agnostic, so the Binary Code Modulation (BCM)
//! machinery, the tiling helpers and the `embedded-graphics` integration are
//! reused rather than rewritten.
//!
//! The reference wiring is the Interstate 75 W's, whose signals [`pins`] names as
//! offsets in bus order. On that board the window starts at GP0:
//!
//! | | |
//! |---|---|
//! | data (R0 G0 B0 R1 G1 B1) | GP0–GP5 |
//! | row select (A–E) | GP6–GP10 |
//! | clock / strobe / output-enable | GP11 / GP12 / GP13 |
//! | buttons, status LED, Qw/ST | GP14, GP15, GP22 / GP18 / GP20 + GP21 |
//!
//! # Using it
//!
//! Two framebuffers and their descriptor table, one driver, and a swap per
//! frame:
//!
//! ```text
//! let hub75 = Hub75::new(pio, ch_ctrl, ch_data, &mut resets, pins, sys_clock,
//!                        Hub75Config::new(), fb0, hub75_descriptors!(Fb))?;
//! let mut fb = fb1;                       // fb0 is on the panel now
//! loop {
//!     draw(fb);
//!     fb = hub75.swap(fb)?.wait();        // the other buffer comes back
//! }
//! ```
//!
//! `swap` arms the switch and hands the outgoing buffer back once nothing is
//! reading it, so the buffer being drawn and the buffer on screen are never the
//! same one. With a wake interrupt configured, `wait_for_done().await` parks the
//! caller instead of spinning — [`driver`] has the whole picture.
//!
//! # Geometry
//!
//! A panel **is** the framebuffer type, and its three const generics are the
//! whole of it. There is nothing else to configure:
//!
//! | panel | framebuffer | runs a refresh |
//! |---|---|---|
//! | 64×64, 1/32 | `DmaFrameBuffer<compute_rows(64), 64, 6>` | 32, plane-major |
//! | 64×32, 1/16 | `DmaFrameBuffer<compute_rows(32), 64, 6>` | 32, plane-major |
//! | two 64×64 chained | `DmaFrameBuffer<compute_rows(64), 128, 6>` | 32, plane-major |
//!
//! `NROWS` is [`compute_rows`](framebuffer::compute_rows) of the panel height,
//! `COLS` its width times the number of panels chained across, and `PLANES` the
//! BCM depth. Chaining is just a wider `COLS`, and the run count does not change
//! at any geometry for a plane-major frame; [`Hub75::runs_per_refresh`] reports
//! what it is for the framebuffer on the panel. Six planes is the examples'
//! depth — 63 panel scans a frame, ≈145 Hz at 18.75 MHz, where eight visibly
//! flickers. A bitplane framebuffer holds up to eight.
//!
//! The library holds no panel: each example states the one it is wired to and
//! derives its framebuffer type from it, because the board — the Interstate
//! 75 W — is a HUB75 header with no panel attached. What is the board's is the
//! **pin window** and the **signal order**: [`Hub75Pins::new`] takes fourteen
//! GPIOs in bus order, derives the window's base from the first one, and checks
//! that the rest follow it, while the order within that window is
//! `hub75-framebuffer`'s `interstate75` feature. Base and order are independent:
//! any fourteen consecutive GPIOs carry the same signals in the same order.
//!
//! # How the panel is driven
//!
//! The words in a `hub75-framebuffer` bitplane framebuffer *are* the pin levels
//! the panel receives. With the crate's `interstate75` layout, bit *n* of a word
//! is pin *n* of the HUB75 window, so a single PIO instruction — `out pins, 14` —
//! puts a whole word on the fourteen pins: the six colour bits, the row address,
//! the latch and the output enable. **One** state machine streams them, one word
//! per panel column, and the only thing the PIO still generates rather than
//! copies out of the word is the clock. There is no conversion pass, and nothing
//! on the CPU touches the panel while the refresh runs. See [`programs`] for the
//! program, the pin layout and the one invariant it rests on.
//!
//! Feeding that FIFO is [`driver`]'s pair of DMA channels: a **control** channel
//! plays a [`descriptor`] table — the framebuffer's own BCM sequence, one entry
//! per run, then a terminator — into the data channel's `AL3` register block,
//! and the data channel chains back to it on every completion. The CPU does
//! nothing between frames: the chain re-arms itself as often as the frame needs,
//! and the terminator's zero word is the one interrupt, at the frame boundary
//! where a buffer swap belongs. The table is storage the caller owns, sized at
//! compile time by [`descriptor::descriptor_count`] and allocated by
//! [`hub75_descriptors!`]: 33 entries cover a six-plane plane-major frame at any
//! panel geometry, and `32 × NROWS + 1` a row-major one.
//!
//! The sequence *is* the frame: brightness lives in the framebuffer's memory
//! layout rather than in a register, so a plane-major, row-major, threshold or
//! remapped framebuffer all drive the same code and the layout only changes the
//! cost, which [`Hub75::runs_per_refresh`] reports. Six planes at an 18.75 MHz
//! panel clock is 63 panel scans a refresh — 6.9 ms, ≈145 Hz — for 32 runs
//! plane-major, or 1024 row-major.
//!
//! See `NOTES.md` for the measurements and the register facts this rests on,
//! and `plan.md` for what is done and what is next.

#![no_std]

pub mod descriptor;
pub mod driver;
pub mod pins;
pub mod programs;

pub use descriptor::Hub75Descriptors;
pub use driver::Hub75;
pub use driver::Hub75Config;
pub use driver::Hub75Error;
pub use driver::Hub75Pins;
pub use driver::Hub75Swap;
pub use driver::PioPin;
pub use driver::on_dma_irq;

/// Framebuffer implementations shared with `esp-hub75`.
///
/// `plain`, `bitplane`, `latched` and `tiling` modules with
/// `FrameBuffer`/`MutableFrameBuffer` traits; see the crate docs for the BCM
/// segment model the DMA backend will consume.
pub use hub75_framebuffer as framebuffer;

/// The `StaticCell` [`hub75_descriptors!`] allocates its storage in.
///
/// Re-exported so that macro can name it from a downstream crate.
pub use static_cell;

/// Crystal frequency on the Interstate 75 W, in Hz. Used when configuring
/// the PLLs.
pub const XTAL_FREQ_HZ: u32 = 12_000_000;
