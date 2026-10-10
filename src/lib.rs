//! # RP-HUB75
//!
//! A `no-std` driver for HUB75-style LED matrix panels on the RP2040 and
//! RP2350.
//!
//! The panel is refreshed over DMA with almost no CPU involvement. One PIO
//! state machine streams the framebuffer and two DMA channels play its Binary
//! Code Modulation (BCM) sequence, so the CPU takes one interrupt per frame.
//! Pick the backend with a single target feature:
//!
//! - **`rphal-rp2350`**: the `rp-hal` backend and the RP2350 (`rp235x-hal`)
//! - **`rphal-rp2040`**: the `rp-hal` backend and the RP2040 (`rp2040-hal`)
//! - **`embassy-rp2350`**: the `embassy-rp` backend and the RP2350
//! - **`embassy-rp2040`**: the `embassy-rp` backend and the RP2040
//!
//! Exactly one is required, and none is a default.
//!
//! ## Framebuffers
//!
//! Use the **bitplane** framebuffers from the `hub75-framebuffer` crate.
//! They come in two variants: direct-drive (16-bit, no external latch) and
//! latched (8-bit, needs an external address-latch circuit). Both can be
//! handed to the PIO as-is; there is no extra formatting step.
//!
//! Bitplane framebuffers (`framebuffer::bitplane::plain::DmaFrameBuffer` /
//! `framebuffer::bitplane::latched::DmaFrameBuffer`) store only one bit per
//! pixel per plane. The driver assembles the BCM output on the fly with DMA
//! descriptors, so RAM use stays low without losing visual quality.
//!
//! ## Usage
//!
//! Example for the RP2350 with the `rp-hal` backend:
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
//! ## Crate Features
//!
//! - `rphal-rp2350`: Enable the `rp-hal` backend for the RP2350 (`rp235x-hal`).
//! - `rphal-rp2040`: Enable the `rp-hal` backend for the RP2040 (`rp2040-hal`).
//! - `embassy-rp2350`: Enable the `embassy-rp` backend for the RP2350.
//! - `embassy-rp2040`: Enable the `embassy-rp` backend for the RP2040.
//! - `invert-clock`: Invert the clock signal by enabling the GPIO output
//!   inverter on the clock pin. The default drives data that changes on the
//!   falling edge of the clock so that it is stable when the panel latches on the
//!   rising edge; enable this if the panel requires the opposite polarity.
//! - `invert-oe`: Forwards to the `hub75-framebuffer` crate, inverting the
//!   output-enable (OE) signal in the generated data stream, for a panel whose
//!   output enable is active-high.
//! - `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling an
//!   optimization that skips writing black pixels to the framebuffer.
//! - `tail-closes-latch`: Forwards to the `hub75-framebuffer` crate. Appends a
//!   tail word at the end of each bit-plane that parks the bus with the latch
//!   closed and the output blanked. Does not apply to latched framebuffers.
//! - `lead-blank-1/2/4/8/16/32` / `trail-blank-1/2/4/8/16/32`: Forwards to
//!   `hub75-framebuffer`. Control the number of pixel-clock cycles of blanking
//!   inserted before and after a row address change. Higher values reduce
//!   ghosting at the cost of slightly less brightness.
//! - `inter-row-blank-4/8/16/32`: Forwards to `hub75-framebuffer`. Insert
//!   additional dead clock cycles at the end of each row, giving slow panels
//!   more time to finish blanking.
//! - `reverse-row-order`: Forwards to `hub75-framebuffer`. Stores the rows of the
//!   framebuffer in reverse scan order so that the stream renders the last panel
//!   row first and row 0 last.
//!
//! ## Safety
//!
//! The crate uses `unsafe` code to interface with hardware peripherals, but it
//! exposes a safe, high-level API.

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
