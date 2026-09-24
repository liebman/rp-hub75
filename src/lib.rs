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
//! One DMA channel feeds that FIFO, DREQ-paced so the machine never runs dry,
//! reading half-words from the framebuffer and looping for ever. With a
//! single-plane framebuffer the whole refresh is one contiguous segment, and on
//! RP2350 a channel whose transfer count is in `TRIGGER_SELF` mode and which
//! chains to itself repeats that segment without a single interrupt — but only
//! together with a read-ring the size of the segment, because a self-trigger
//! reloads the transfer *count* and not the *read address*. So a static image
//! costs no CPU at all, and the BCM scan patterns of a multi-plane framebuffer
//! need one DMA restart per segment (each segment is a different repetition
//! count, so no single ring covers it).
//!
//! Status: M3. That one machine and one channel stream a single-plane
//! framebuffer; multi-plane BCM (`BIT_DEPTH`), double buffering and the segment
//! walker are next.
//!
//! See `NOTES.md` for the hardware bring-up results this is based on.

#![no_std]

pub mod pins;
pub mod programs;

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
/// The reference implementation for this board (Pimoroni's HUB75 driver) runs
/// **10-bit gamma-corrected BCM**, but that is out of reach with
/// `hub75-framebuffer` as it stands:
///
/// * every bitplane framebuffer asserts `PLANES` is `1..=8`
///   (`bitplane/{plain,latched}/{row,frame}.rs`),
/// * the non-bitplane (full-frame-copy) family has the same ceiling: `plain.rs`
///   asserts `BITS <= 8`, where each bit is a whole copy of the frame in RAM,
/// * `BCM_SEQUENCE_CAPACITY` is 10, documented as "8 bit-planes plus an
///   inter-row gap plus an end-of-row trailer", and row-major framebuffers need
///   `PLANES + gap + tail` slots, so 10 planes would not even fit,
/// * the framebuffer's colour type is 8 bits per channel, so extra planes would
///   not add range on their own.
///
/// So 8 planes is what we can drive today. Going further means extending
/// `hub75-framebuffer` itself (raise the assert, grow the capacity to
/// `PLANES + gap + tail`, and widen the pixel type to actually buy precision).
/// Until then, gamma-correct the colours you draw instead of reaching for extra
/// planes — that recovers most of the low-end fidelity the reference driver gets
/// from its 10-bit gamma table.
pub const BIT_DEPTH: u8 = 8;
