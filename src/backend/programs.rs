//! PIO programs the driver runs against the panel.
//!
//! **One state machine** streams a `hub75-framebuffer` buffer straight at the
//! panel's pins: the words in the framebuffer *are* the pin levels the panel
//! receives, with no per-column CPU work and no conversion pass. The framebuffer's
//! word width picks between the two programs, and a [`Stream`] carries the
//! assembled program and the window width it writes, so the two cannot drift
//! apart:
//!
//! * [`hub75_stream16`] shifts a 16-bit word into the fourteen-signal direct-drive
//!   bus (the `interstate75` layout) with `out pins, 14`.
//! * [`hub75_stream8`] shifts an 8-bit word into an eight-signal latched bus
//!   (the `bitplane::latched` layouts) with `out pins, 8`.
//!
//! # One state machine, because a word is already in pin order
//!
//! `out pins, N` writes pins `[OUT_BASE, OUT_BASE + N - 1]`, and `OUT_BASE` is
//! fixed when the state machine is built: no PIO instruction can move it. So one
//! instruction puts a whole word on the bus only because the word's fields are
//! already in pin order — bit *n* of a word is pin *n* of the window, which the
//! `interstate75` feature of `hub75-framebuffer` arranges. [`crate::pins`] is that
//! layout. The window's lowest pin is the base [`crate::Hub75Pins`] derives, so
//! one program serves any run of fourteen GPIOs, and the only signal the PIO
//! *generates* rather than copies out of the word is the clock.
//!
//! # The program: two cycles per word, one word per column
//!
//! ```text
//! cycle         1                     2
//!               out pins, 14  side 0   nop  side 1
//! window pins   the word               hold
//! clock pin     low                    rising edge
//! panel         data changes           shifts this column in
//! ```
//!
//! The clock is edge triggered, so a word needs exactly two cycles: the word goes
//! out with the clock at its idle level and is stable for a whole PIO cycle, and
//! the edge a cycle later shifts it into the panel. A panel that takes the column
//! on the *falling* edge gets that edge from the clock pin's output override,
//! which inverts the pad rather than the program — [`crate::driver`] sets it under
//! the `invert-clock` feature.
//!
//! # The one invariant
//!
//! **One FIFO word per column.** `pull_threshold` is the stream's window width
//! ([`Stream::threshold`]), the bits one `out pins, N` shifts out, so the OSR
//! refills exactly on a column boundary and the word's fields cannot drift.
//! Sixteen would leave a 14-bit word's two spare top bits in the OSR and put every
//! following word two bits out of place.
//!
//! # The one hair: the reserved bit
//!
//! The slot the framebuffer keeps at zero sits inside the `out pins, 14` window, so
//! every word drives that pin as well as the side-set does, and the bit is always
//! zero. That costs nothing: the `out` runs on the cycle whose side-set holds the
//! clock low, so the two write the same level and either one winning gives the same
//! pins. The clock's polarity is the pad's, which inverts both writes together.
//!
//! An overlap is free only on that pin, which is why the clock is wired there or
//! outside the window. A side-set aimed at another bus pin would fight every word
//! that follows, and the side-set would win — RP2350 datasheet §11.5.6, *"If a
//! side-set overlaps with an OUT/SET performed by that state machine on the same
//! cycle, the side-set takes precedence in the overlapping region"* — so the
//! corruption of that pin would be silent rather than an error. `.side_set 1`
//! against `side_set_pin_base(clock)` in [`crate::driver`] keeps the window to one
//! bit, and [`crate::Hub75Pins16::new`] refuses any clock pin inside the window
//! other than [`pins::RESERVED`].
//!
//! # The latch
//!
//! The latch bit rides in the last column's word, so on the bus `STB` rises at the
//! start of that word and falls at the start of the next row's first word: the
//! strobe is high across the last column's clock edge and never overlaps another
//! one, and the address lines move while the panel is blanked. `hub75-framebuffer`
//! has a `tail-closes-latch` feature if a panel wants the latch a word later.

use crate::pins;
use pio::{Program, RP2040_MAX_PROGRAM_SIZE};

/// A stream program together with the window width it writes.
///
/// The program and the `out pins, N` window are one decision — a program that
/// shifts fourteen bits into an eight-pin window drives the wrong pins — so they
/// travel together and the driver reads both from one place rather than keeping
/// two lists in step.
pub struct Stream {
    program: Program<{ RP2040_MAX_PROGRAM_SIZE }>,
    window: u8,
}

impl Stream {
    /// The assembled program, for the HAL's `PIOExt::install`.
    pub fn program(&self) -> &Program<{ RP2040_MAX_PROGRAM_SIZE }> {
        &self.program
    }

    /// The pins one `out pins, N` writes: the framebuffer word's width. The
    /// window's lowest pin is the base the pin set derives from the pins it is
    /// given.
    pub fn window(&self) -> u8 {
        self.window
    }

    /// `pull_threshold` to configure the state machine with: the word's own
    /// bits, so the OSR refills on a column boundary. Equal to
    /// [`window`](Self::window); see the module docs for why it is not 16.
    pub fn threshold(&self) -> u8 {
        self.window
    }
}

/// The 16-bit stream program: one `out pins, 14` per column, straight out to
/// the fourteen pins of a direct-drive HUB75 window.
///
/// This is the program for the `hub75-framebuffer` layouts whose word *is* the
/// fourteen-signal bus (the `interstate75` layout, [`crate::pins::LAYOUT`]).
/// Configure the state machine with `out_pins(base, stream.window())`,
/// `side_set_pin_base(clock)`, `autopull(true)`,
/// `pull_threshold(stream.threshold())` and
/// `out_shift_direction(ShiftDirection::Right)` — the words are handed out
/// least-significant bit first, and `Left` reverses every word, which turns the
/// picture into noise. Then `set_pindirs` the thirteen signal pins and the clock
/// as outputs; [`crate::pins::RESERVED`] is only an output when the clock is
/// wired there.
///
/// The program is the same for either clock edge: the word changes with the clock
/// at its idle level, so the panel gets a full cycle of setup before the edge it
/// samples. A panel that wants the falling edge gets it from the clock pin's
/// output override rather than from a second program — see [`crate::driver`].
pub fn hub75_stream16() -> Stream {
    Stream {
        program: pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "    out pins, 14     side 0   ; the word -> the window, clock low",
            "    nop              side 1   ; the clock's edge: the panel shifts this column in",
            ".wrap",
        )
        .program,
        window: pins::WORD_PINS,
    }
}

/// The 8-bit stream program: one `out pins, 8` per column, for an 8-bit latched
/// HUB75 window.
///
/// This is the program for the `hub75-framebuffer` latched layouts
/// ([`crate::pins::latched`]), whose word is eight signals — the two sub-pixels'
/// colour, the latch and the output enable — driven through an external latch
/// circuit that gates the clock. Configure and start the state machine exactly
/// as [`hub75_stream16`] describes, with `out_pins(base, stream.window())`; the
/// clock is the side-set pin and, the 8-bit window having no reserved slot, is
/// wired outside it.
pub fn hub75_stream8() -> Stream {
    Stream {
        program: pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "    out pins, 8      side 0   ; the word -> the window, clock low",
            "    nop              side 1   ; the clock's edge: the panel shifts this column in",
            ".wrap",
        )
        .program,
        window: pins::latched::WORD_PINS,
    }
}
