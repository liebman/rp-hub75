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
//! The window starts at GP0: the data is GP0–GP5, the row select GP6–GP10, and the
//! clock, strobe and output-enable GP11–GP13. GP11 is the word's one reserved bit,
//! where the clock is wired there; [`Hub75Pins16::new`] takes the clock as its own
//! argument and accepts a pin outside the window instead. The 8-bit latched bus
//! has no reserved slot, so its clock is always outside the window — see
//! [`pins::latched`].
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
//! A panel **is** the framebuffer type. `NROWS` is
//! [`compute_rows`](framebuffer::compute_rows) of the panel height, `COLS` its
//! width times the number of panels chained across, and `PLANES` the BCM depth:
//!
//! | panel | framebuffer |
//! |---|---|
//! | 64×64, 1/32 | `DmaFrameBuffer<compute_rows(64), 64, 6>` |
//! | 64×32, 1/16 | `DmaFrameBuffer<compute_rows(32), 64, 6>` |
//! | two 64×64 chained | `DmaFrameBuffer<compute_rows(64), 128, 6>` |
//!
//! The library holds no panel: each example states the one it is wired to and
//! derives its framebuffer type from it, because the board is a HUB75 header with
//! no panel attached. What is the board's is the **pin window** and the **word
//! layout**: [`Hub75Pins16::new`] takes the thirteen signals in bus order plus the
//! clock, derives the window's base from the lowest pin, and checks every signal
//! against [`pins::LAYOUT`], so a wiring mistake stops startup rather than driving
//! the wrong pins, while the word layout itself is `hub75-framebuffer`'s
//! `interstate75` feature. Base and wiring are independent: any fourteen
//! consecutive GPIOs carry the same signals in the same order, and the clock may
//! be one of them or a pin of its own.
//!
//! # How the panel is driven
//!
//! One state machine streams the framebuffer's words at the pin window with a
//! single `out pins, 14` — the words *are* the pin levels — while a control DMA
//! channel plays the framebuffer's own BCM sequence into the data channel. The
//! CPU does nothing between frames, and one interrupt a frame is all it does
//! about it. [`backend`] has the program and the layout, [`driver`] the channels
//! and the interrupt, and [`descriptor`] the table.

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
