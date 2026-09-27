//! A HUB75 LED matrix driver for the RP2350, developed against the Pimoroni
//! Interstate 75 W.
//!
//! The panel is refreshed from PIO plus DMA with no CPU involvement at all,
//! and the framebuffers come from the [`framebuffer`] crate that backs
//! `esp-hub75` — it is MCU-agnostic, so the Binary Code Modulation (BCM)
//! machinery, the tiling helpers and the `embedded-graphics` integration are
//! reused rather than rewritten.
//!
//! Everything here targets the Interstate 75 W wiring, which is described in
//! [`pins`]:
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
//! Two framebuffers, one driver, and a swap per frame:
//!
//! ```text
//! let hub75 = Hub75::new(pio, ch_ctrl, ch_data, &mut resets, pins, sys_clock,
//!                        Hub75Config::new(), fb0)?;
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
//! # How the panel is driven
//!
//! The words in a `hub75-framebuffer` bitplane framebuffer *are* the pin levels
//! the panel receives. With the crate's `interstate75` layout, bit *n* of a word
//! is pin *n* of the HUB75 bus, so a single PIO instruction — `out pins, 14` —
//! puts a whole word on GP0-13: the six colour bits, the row address, the latch
//! and the output enable. **One** state machine streams them, one word per panel
//! column, and the only thing the PIO still generates rather than copies out of
//! the word is the clock. There is no conversion pass, and nothing on the CPU
//! touches the panel while the refresh runs. See [`programs`] for the program,
//! the pin layout and the one invariant it rests on.
//!
//! Feeding that FIFO is [`driver`]'s pair of DMA channels: a **control** channel
//! plays a [`descriptor`] table — the framebuffer's own BCM sequence, one entry
//! per run, then a terminator — into the data channel's `AL3` register block,
//! and the data channel chains back to it on every completion. The CPU does
//! nothing between frames: the chain re-arms itself as often as the frame needs,
//! and the terminator's zero word is the one interrupt, at the frame boundary
//! where a buffer swap belongs. [`descriptor::MAX_DESCRIPTORS`] bounds the
//! table; 32 entries cover a six-plane plane-major frame at any panel geometry.
//!
//! The sequence *is* the frame: brightness lives in the framebuffer's memory
//! layout rather than in a register, so a plane-major, row-major, threshold or
//! remapped framebuffer all drive the same code and the layout only changes the
//! cost, which [`Hub75::runs_per_refresh`] reports. Six planes at an 18.75 MHz
//! panel clock is 63 panel scans a refresh — 6.9 ms, ≈143 Hz — for 32 runs
//! plane-major, or 1024 row-major.
//!
//! See `NOTES.md` for the measurements and the register facts this rests on,
//! and `plan.md` for what is done and what is next.

#![no_std]

pub mod descriptor;
pub mod driver;
pub mod pins;
pub mod programs;

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

/// Crystal frequency on the Interstate 75 W, in Hz. Used when configuring
/// the PLLs.
pub const XTAL_FREQ_HZ: u32 = 12_000_000;

/// Geometry of the panel this driver is being developed against.
pub const PANEL_WIDTH: usize = 64;
/// See [`PANEL_WIDTH`].
pub const PANEL_HEIGHT: usize = 64;

/// Number of row pairs a 1/32-scan 64x64 panel lights at once.
///
/// The driver shifts the top half of the panel and the bottom half
/// simultaneously, so this is also the number of rows the DMA backend has to
/// write per BCM bit-plane step.
pub const ROW_PAIRS: usize = PANEL_HEIGHT / 2;

/// BCM bit depth, i.e. the number of bit-planes the driver runs.
///
/// **Six**, not eight, and the ceiling is not the reason: flicker is.
///
/// `hub75-framebuffer` can drive eight planes, and this driver did — but eight
/// planes is `2^0 + … + 2^7 = 255` panel scans per frame, and the MSB plane alone
/// is 128 of them. At the ~19-25 MHz panel clock these examples run at, that is a
/// 21 ms frame (~48 Hz), and this panel visibly flickers: the eye integrates a
/// 1/32-scan image over the whole frame, so the frame rate is what it feels.
/// Six planes is `2^0 + … + 2^5 = 63` scans — a 6.9 ms frame (~145 Hz at
/// 18.75 MHz) — and still 64 levels per channel, which with the gamma correction
/// on the drawing side is plenty.
///
/// A tenth plane, or a flicker-free eight, needs the reference driver's other
/// trick: it runs **10-bit gamma-corrected BCM** and modulates the *output
/// enable* pulse width within each scan instead of repeating scans, so brightness
/// costs no extra frame time. That is a different scheme from the BCM sequence
/// `hub75-framebuffer` describes, and out of scope here.
///
/// The crate's own ceiling is worth knowing too: every bitplane framebuffer
/// asserts `PLANES` is `1..=8`, the non-bitplane (full-frame-copy) family asserts
/// `BITS <= 8`, and `BCM_SEQUENCE_CAPACITY` is 10 (documented as "8 bit-planes plus
/// an inter-row gap plus an end-of-row trailer"). Going past eight planes means
/// extending `hub75-framebuffer` itself.
pub const BIT_DEPTH: u8 = 6;
