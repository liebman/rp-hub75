//! # rp-hub75
//!
//! A `no-std` driver for HUB75-style LED matrix panels on the RP2040 and
//! RP2350.
//!
//! One PIO state machine streams the framebuffer; two DMA channels play its
//! Binary Code Modulation (BCM) sequence, and the CPU takes one interrupt per
//! frame. Pick the backend with a single target feature:
//!
//! - `rphal-rp2350`: the `rp-hal` backend on the RP2350 (`rp235x-hal`)
//! - `rphal-rp2040`: the `rp-hal` backend on the RP2040 (`rp2040-hal`)
//! - `embassy-rp2350`: the `embassy-rp` backend on the RP2350
//! - `embassy-rp2040`: the `embassy-rp` backend on the RP2040
//!
//! Exactly one is required and none is a default. Both chips share the same PIO
//! and DMA register blocks, so one source builds for `thumbv6m-none-eabi` and
//! `thumbv8m.main-none-eabihf`. The selected HAL is re-exported as [`hal`].
//!
//! The framebuffers work with `embedded-graphics`, so you can draw shapes, text
//! and images on the display, and they go to the PIO as-is: there is no
//! per-frame conversion step.
//!
//! ## Framebuffers
//!
//! Use the **bitplane** framebuffers from the [`framebuffer`] crate
//! (`hub75-framebuffer`). The framebuffer type fixes the bus:
//!
//! | bus | framebuffer | pins | program |
//! |---|---|---|---|
//! | 16-bit direct drive | [`framebuffer::bitplane::plain`] | [`Hub75Pins16`] | `out pins, 14` |
//! | 8-bit latched | [`framebuffer::bitplane::latched`] | [`Hub75Pins8`] | `out pins, 8` |
//!
//! The direct-drive bus carries the row address in the word. The latched bus
//! needs an external address latch, and its row address rides apart from the
//! word, in trailing address bytes. A [`Hub75`] requires the pin set's `Word` to
//! equal the framebuffer's, so a mismatched pair is a compile error.
//!
//! The type also fixes the geometry. `NROWS` is
//! [`compute_rows`](framebuffer::compute_rows) of the panel height, `COLS` its
//! width times the panels chained across, and `PLANES` the BCM depth:
//!
//! | panel | framebuffer |
//! |---|---|
//! | 64×64, 1/32 | `DmaFrameBuffer<compute_rows(64), 64, 6>` |
//! | 64×32, 1/16 | `DmaFrameBuffer<compute_rows(32), 64, 6>` |
//! | two 64×64 chained | `DmaFrameBuffer<compute_rows(64), 128, 6>` |
//!
//! The library holds no panel constants; the application states its panel and
//! derives the type from it. The window's wiring is [`Hub75Pins16::new`]'s
//! business: it takes the thirteen signals in bus order plus the clock, derives
//! the base from the lowest pin, and checks each signal against
//! [`pins::LAYOUT`], so a wiring mistake stops startup.
//!
//! ## Usage
//!
//! The application owns the DMA vector, draws into whichever buffer is off the
//! panel, and swaps at a frame boundary:
//!
//! ```rust,ignore
//! #![no_std]
//! #![no_main]
//!
//! use embedded_graphics::Drawable;
//! use embedded_graphics::geometry::Point;
//! use embedded_graphics::mono_font::MonoTextStyleBuilder;
//! use embedded_graphics::mono_font::ascii::FONT_5X7;
//! use embedded_graphics::pixelcolor::Rgb888;
//! use embedded_graphics::pixelcolor::RgbColor;
//! use embedded_graphics::text::Alignment;
//! use embedded_graphics::text::Text;
//! use rp_hub75::Hub75;
//! use rp_hub75::Hub75Config;
//! use rp_hub75::Hub75Pins16;
//! use rp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
//! use rp_hub75::framebuffer::compute_rows;
//! use rp_hub75::hal;
//! use rp_hub75::hub75_descriptors;
//!
//! use hal::clocks::Clock;
//! use hal::dma::DMAExt;
//! use hal::gpio::Pins;
//!
//! const PANEL_WIDTH: usize = 64;
//! const PANEL_HEIGHT: usize = 64;
//! const NROWS: usize = compute_rows(PANEL_HEIGHT);
//! const PLANES: usize = 6;
//!
//! type Fb = DmaFrameBuffer<NROWS, PANEL_WIDTH, PLANES>;
//!
//! macro_rules! mk_static {
//!     ($t:ty,$val:expr) => {{
//!         static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
//!         #[deny(unused_attributes)]
//!         let x = STATIC_CELL.uninit().write($val);
//!         x
//!     }};
//! }
//!
//! // The application owns the vector; the line must match `Hub75Config`.
//! #[unsafe(no_mangle)]
//! extern "C" fn DMA_IRQ_1() {
//!     rp_hub75::on_dma_irq();
//! }
//!
//! #[hal::entry]
//! fn main() -> ! {
//!     let mut pac = hal::pac::Peripherals::take().unwrap();
//!     let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
//!     let clocks = hal::clocks::init_clocks_and_plls(
//!         12_000_000,
//!         pac.XOSC,
//!         pac.CLOCKS,
//!         pac.PLL_SYS,
//!         pac.PLL_USB,
//!         &mut pac.RESETS,
//!         &mut watchdog,
//!     )
//!     .unwrap();
//!     let sio = hal::Sio::new(pac.SIO);
//!     let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
//!     let dma = pac.DMA.split(&mut pac.RESETS);
//!
//!     // One buffer is on the panel; the other is yours to draw into.
//!     let fb0: &'static mut Fb = mk_static!(Fb, Fb::new());
//!     let fb1: &'static mut Fb = mk_static!(Fb, Fb::new());
//!
//!     let hub75 = Hub75::new(
//!         pac.PIO0,
//!         dma.ch0, // streams the panel
//!         dma.ch1, // plays the descriptor table
//!         &mut pac.RESETS,
//!         Hub75Pins16::new(
//!             pins.gpio0, pins.gpio1, pins.gpio2, pins.gpio3,
//!             pins.gpio4, pins.gpio5, pins.gpio6, pins.gpio7,
//!             pins.gpio8, pins.gpio9, pins.gpio10,
//!             pins.gpio12, // STB
//!             pins.gpio13, // OE
//!             pins.gpio11, // clock, driven by the side-set
//!         ),
//!         clocks.system_clock.freq().to_Hz(),
//!         Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_1),
//!         fb0,
//!         hub75_descriptors!(Fb),
//!     )
//!     .expect("failed to start the panel");
//!
//!     let style = MonoTextStyleBuilder::new()
//!         .font(&FONT_5X7)
//!         .text_color(Rgb888::WHITE)
//!         .background_color(Rgb888::BLACK)
//!         .build();
//!
//!     let mut fb = fb1;
//!     loop {
//!         Text::with_alignment("rp-hub75", Point::new(32, 32), style, Alignment::Center)
//!             .draw(fb)
//!             .ok();
//!
//!         // Hand the finished buffer over; the other one comes back.
//!         match hub75.swap(fb) {
//!             Ok(swap) => fb = swap.wait(),
//!             Err((_, returned)) => fb = returned,
//!         }
//!     }
//! }
//! ```
//!
//! `swap` arms the switch and returns once the interrupt has applied it at a
//! frame boundary; [`Hub75Swap::wait`] then hands the outgoing buffer back. On an
//! executor, [`Hub75Swap::wait_for_done`] parks the caller instead of spinning,
//! then `wait` reclaims the buffer. [`driver`] has the channels and the
//! interrupt, [`descriptor`] the table, and [`backend`] the PIO program and pin
//! layout.
//!
//! ## Crate Features
//!
//! Exactly one target feature names the backend and the chip together:
//!
//! - `rphal-rp2350` (the RP2350) / `rphal-rp2040` (the RP2040): the `rp-hal`
//!   backend, against `rp235x-hal` / `rp2040-hal`.
//! - `embassy-rp2350` / `embassy-rp2040`: the `embassy-rp` backend.
//!
//! The library's tuning features:
//!
//! - `invert-clock`: inverts the clock pin's output in the GPIO block for a
//!   panel that takes a column on the clock's falling edge.
//! - `invert-oe`: forwards to `hub75-framebuffer/invert-oe` for a panel whose
//!   output enable is active-high.
//! - `skip-black-pixels`, `tail-closes-latch`, `lead-blank-1` through
//!   `lead-blank-32`, `trail-blank-1` through `trail-blank-32`,
//!   `inter-row-blank-4` through `inter-row-blank-32`, and `reverse-row-order`:
//!   forward to `hub75-framebuffer` and change the words the driver streams. The
//!   crate's `Cargo.toml` lists them.
//!
//! ## Safety
//!
//! The crate uses `unsafe` to drive the PIO and DMA registers; it exposes a
//! safe, high-level API.

#![no_std]

// The target is a build-time choice on one axis: a backend family and a chip in
// one feature. Exactly one of the four is needed. Without one there is no HAL to
// drive the panel; with two of a kind the two HALs collide.
#[cfg(not(any(
    feature = "rphal-rp2040",
    feature = "rphal-rp2350",
    feature = "embassy-rp2040",
    feature = "embassy-rp2350",
)))]
compile_error!(
    "enable one target feature: `rphal-rp2040`, `rphal-rp2350`, `embassy-rp2040` or `embassy-rp2350`"
);

#[cfg(any(
    all(feature = "rphal-rp2040", feature = "rphal-rp2350"),
    all(feature = "rphal-rp2040", feature = "embassy-rp2040"),
    all(feature = "rphal-rp2040", feature = "embassy-rp2350"),
    all(feature = "rphal-rp2350", feature = "embassy-rp2040"),
    all(feature = "rphal-rp2350", feature = "embassy-rp2350"),
    all(feature = "embassy-rp2040", feature = "embassy-rp2350"),
))]
compile_error!("enable exactly one target feature");

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
