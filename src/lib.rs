//! A HUB75 LED matrix driver for the **RP2040** and **RP2350**. The Pimoroni
//! Interstate 75 W is the reference board — the source of the pin map and the
//! measurements — not a requirement: the pin base, wiring, bus width, and panel
//! geometry are all parameters.
//!
//! The panel is refreshed from PIO plus DMA with no CPU involvement at all,
//! and the framebuffers come from the [`framebuffer`] crate that backs
//! `esp-hub75` — it is MCU-agnostic, so the Binary Code Modulation (BCM)
//! machinery, the tiling helpers and the `embedded-graphics` integration are
//! reused rather than rewritten.
//!
//! # Two buses, two word widths
//!
//! A framebuffer word *is* the bus, so the framebuffer type picks the bus. Two
//! widths are supported, each with its own pin set and PIO program:
//!
//! | bus | framebuffer | pins | program |
//! |---|---|---|---|
//! | 16-bit direct drive | [`framebuffer::bitplane::plain`] | [`Hub75Pins16`] | `out pins, 14` |
//! | 8-bit latched | [`framebuffer::bitplane::latched`] | [`Hub75Pins8`] | `out pins, 8` |
//!
//! The latched bus needs an external latch circuit (the row address rides in
//! trailing address bytes, not in the word). A [`Hub75`] is generic over the pin
//! set and requires its `Word` to equal the framebuffer's, so a mismatched pair
//! does not type-check.
//!
//! # The target
//!
//! The HAL is a build-time choice, selected by the `rp2040` or `rp2350` feature
//! (exactly one, and neither is a default), and re-exported as [`hal`]. The PIO
//! and DMA code is the same on both parts — the register blocks are the same
//! shape — so the crate builds for `thumbv6m-none-eabi` and
//! `thumbv8m.main-none-eabihf` from one source. `.cargo/config-rp2040.toml` and
//! `.cargo/config-rp2350.toml` carry the per-board target and probe settings; the
//! aliases in `.cargo/config.toml` name the config file and the feature together.
//!
//! # The reference wiring
//!
//! The Interstate 75 W's, whose signals [`pins`] names as offsets in bus order.
//! On that board the window starts at GP0:
//!
//! | | |
//! |---|---|
//! | data (R0 G0 B0 R1 G1 B1) | GP0–GP5 |
//! | row select (A–E) | GP6–GP10 |
//! | clock / strobe / output-enable | GP11 / GP12 / GP13 |
//! | buttons, status LED, Qw/ST | GP14, GP15, GP22 / GP18 / GP20 + GP21 |
//!
//! GP11 is the word's one reserved bit, which is where the clock is wired there;
//! [`Hub75Pins16::new`] takes the clock as its own argument and accepts a pin
//! outside the window instead. The 8-bit latched bus has no reserved slot, so its
//! clock is always outside the window — see [`pins::latched`] and
//! `examples/latched.rs`.
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
//! **pin window** and the **word layout**: [`Hub75Pins16::new`] takes the thirteen
//! signals in bus order plus the clock, derives the window's base from the lowest
//! pin, and checks every signal against [`pins::LAYOUT`] — so a wiring mistake
//! stops startup rather than driving the wrong pins — while the word layout
//! itself is `hub75-framebuffer`'s `interstate75` feature. Base and wiring are
//! independent: any fourteen consecutive GPIOs carry the same signals in the same
//! order, and the clock may be one of them or a pin of its own.
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
//! on the CPU touches the panel while the refresh runs. See [`backend`] for the
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

// The target is a build-time choice on two independent axes, and exactly one of
// each is needed: the backend family (`rphal` or `embassy`) and the chip
// (`rp2040` or `rp2350`). Without one there is no HAL to drive the panel; with
// two of a kind the two HALs collide.
#[cfg(not(any(feature = "rphal", feature = "embassy")))]
compile_error!("enable one backend feature: `rphal` or `embassy`");
#[cfg(all(feature = "rphal", feature = "embassy"))]
compile_error!("enable exactly one backend feature: `rphal` or `embassy`");

#[cfg(not(any(feature = "rp2040", feature = "rp2350")))]
compile_error!("enable one chip feature: `rp2040` or `rp2350`");
#[cfg(all(feature = "rp2040", feature = "rp2350"))]
compile_error!("enable exactly one chip feature: `rp2040` or `rp2350`");

pub mod backend;
pub mod descriptor;
pub mod driver;
pub mod pins;

pub use backend::Hub75Pins8;
pub use backend::Hub75Pins16;
pub use backend::PioPin;
pub use backend::hal;

pub use descriptor::Hub75Descriptors;
pub use driver::Hub75;
pub use driver::Hub75Config;
pub use driver::Hub75Error;
pub use driver::Hub75Pins;
pub use driver::Hub75Swap;
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
