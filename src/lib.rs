//! A HUB75 LED matrix driver for the RP2040 and RP2350.
//!
//! The Pimoroni Interstate 75 W is the reference wiring and the source of the
//! measurements, not a crate invariant. The pin base, bus width and panel
//! geometry are parameters.
//!
//! PIO streams a [`framebuffer`] bitplane buffer. Two DMA channels play that
//! buffer's own Binary Code Modulation (BCM) sequence; the CPU takes one
//! interrupt per frame. The framebuffer crate backs `esp-hub75` and is
//! MCU-agnostic, so its BCM machinery, tiling and `embedded-graphics` support
//! are shared.
//!
//! # Two buses, two word widths
//!
//! A framebuffer word is the bus, so the framebuffer type picks the bus:
//!
//! | bus | framebuffer | pins | program |
//! |---|---|---|---|
//! | 16-bit direct drive | [`framebuffer::bitplane::plain`] | [`Hub75Pins16`] | `out pins, 14` |
//! | 8-bit latched | [`framebuffer::bitplane::latched`] | [`Hub75Pins8`] | `out pins, 8` |
//!
//! The latched bus needs an external address latch: the row address rides in
//! trailing address bytes, not in the word. A [`Hub75`] requires the pin set's
//! `Word` to equal the framebuffer's, so a mismatched pair is a compile error.
//!
//! # The target
//!
//! Exactly one backend feature (`rphal` or `embassy`) and one chip feature
//! (`rp2040` or `rp2350`); neither is a default. The selected HAL is re-exported
//! as [`hal`]. The PIO and DMA register blocks have the same shape on both chips,
//! so one source builds for `thumbv6m-none-eabi` and `thumbv8m.main-none-eabihf`.
//! `.cargo/config-rp2040.toml` and `.cargo/config-rp2350.toml` carry the per-chip
//! target and probe settings; the aliases in `.cargo/config.toml` name the config
//! file and the features together.
//!
//! # The reference wiring
//!
//! The Interstate 75 W's, whose signals [`pins`] names as offsets in bus order.
//! The window starts at GP0: data GP0–GP5, row select GP6–GP10,
//! clock/strobe/output-enable GP11–GP13. GP11 is the word's one reserved bit.
//! [`Hub75Pins16::new`] takes the clock as its own argument and accepts a pin
//! outside the window instead. The 8-bit latched bus has no reserved slot, so its
//! clock is always outside the window — see [`pins::latched`].
//!
//! # Usage
//!
//! Two framebuffers, one descriptor table, one driver, a swap per frame:
//!
//! ```text
//! let hub75 = Hub75::new(pio, ch_ctrl, ch_data, &mut resets, pins, sys_clock,
//!                        Hub75Config::new(pac::Interrupt::DMA_IRQ_1), fb0,
//!                        hub75_descriptors!(Fb))?;
//! let mut fb = fb1;                       // fb0 is on the panel now
//! loop {
//!     draw(fb);
//!     fb = hub75.swap(fb)?.wait();        // the other buffer comes back
//! }
//! ```
//!
//! `swap` arms the switch and returns the outgoing buffer once the interrupt has
//! applied it at a frame boundary. `wait_for_done().await` parks the caller
//! instead of spinning. [`driver`] has the channels and the interrupt,
//! [`descriptor`] the table, and [`backend`] the PIO program and pin layout.
//!
//! # Geometry
//!
//! A panel is the framebuffer type. `NROWS` is
//! [`compute_rows`](framebuffer::compute_rows) of the panel height, `COLS` its
//! width times the number of panels chained across, and `PLANES` the BCM depth:
//!
//! | panel | framebuffer |
//! |---|---|
//! | 64×64, 1/32 | `DmaFrameBuffer<compute_rows(64), 64, 6>` |
//! | 64×32, 1/16 | `DmaFrameBuffer<compute_rows(32), 64, 6>` |
//! | two 64×64 chained | `DmaFrameBuffer<compute_rows(64), 128, 6>` |
//!
//! The library holds no panel constants. Each example states its panel and
//! derives the framebuffer type from it. The board contributes the pin window and
//! the word layout: [`Hub75Pins16::new`] takes the thirteen signals in bus order
//! plus the clock, derives the base from the lowest pin, and checks each signal
//! against [`pins::LAYOUT`], so a wiring mistake stops startup. The word layout is
//! `hub75-framebuffer`'s `interstate75` feature; base and wiring are independent,
//! so any fourteen consecutive GPIOs carry the same signals in the same order.

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

/// Re-exports the framebuffer crate (`hub75-framebuffer`) used with `esp-hub75`.
///
/// `plain`, `bitplane`, `latched` and `tiling` modules, with the `FrameBuffer`
/// and `MutableFrameBuffer` traits. Its BCM segment layout is what
/// [`descriptor::fill`] walks.
pub use hub75_framebuffer as framebuffer;

/// The `StaticCell` [`hub75_descriptors!`] allocates its storage in.
///
/// Re-exported so that macro can name it from a downstream crate.
pub use static_cell;
