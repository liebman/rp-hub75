//! PIO programs the driver runs against the panel.
//!
//! **One state machine** streams a `hub75-framebuffer` bitplane buffer straight
//! at the panel's fourteen signals. Nothing touches the HUB75 pins while that
//! runs: no per-column CPU work, and no conversion pass — the words in the
//! framebuffer *are* the pin levels the panel receives.
//!
//! # One state machine, because a word is already in pin order
//!
//! `out pins, N` writes pins `[OUT_BASE, OUT_BASE + N - 1]`, and `OUT_BASE` is
//! fixed when the state machine is built: no PIO instruction can move it. So
//! putting a whole word on the bus with one instruction only works if the word's
//! fields are already in pin order, which is what the `interstate75` feature of
//! `hub75-framebuffer` arranges:
//!
//! | word bits | window pins | signal |
//! |---|---|---|
//! | 0-2 | 0-2 | `R0 G0 B0` |
//! | 3-5 | 3-5 | `R1 G1 B1` |
//! | 6-10 | 6-10 | `A B C D E` |
//! | 11 | 11 | reserved, always zero — the clock is outside the word |
//! | 12 | 12 | `STB`, the latch |
//! | 13 | 13 | `OE`, active low |
//!
//! Bit *n* is pin *n* of the window, so one `out pins, 14` puts a whole word on
//! the pins, and the only thing left for the PIO to *generate* rather than copy
//! out of the word is the clock. The window's lowest pin is the base
//! [`crate::Hub75Pins`] derives, so this one table describes any run of fourteen
//! GPIOs; [`crate::pins`] names the word bits, which are also the offsets from
//! that base.
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
//! out with the clock at its idle level, and the edge a cycle later is what shifts
//! it into the panel, with the word stable for a whole PIO cycle before that
//! edge.
//!
//! The program is the same for either clock edge: the word goes out with the
//! clock at its idle level, and a full PIO cycle later the edge shifts it into the
//! panel. A panel that takes the column on the *falling* edge gets that edge from
//! the clock pin's own output override, which inverts the pad rather than the
//! program — [`crate::driver`] sets it under the `invert-clock` feature.
//!
//! # The one invariant
//!
//! **One FIFO word per column.** `pull_threshold` is [`PULL_THRESHOLD`], the
//! fourteen bits one `out pins, 14` shifts out, so the OSR refills exactly on a
//! column boundary and the word's fields cannot drift. Sixteen would leave the
//! word's two spare top bits in the OSR and put every following word two bits out
//! of place, which is the M2b confetti in a new costume.
//!
//! # The one hair: the reserved bit
//!
//! The slot the framebuffer keeps at zero sits inside the `out pins, 14` window,
//! so every word drives that pin as well as the side-set does, and the bit is
//! always zero. That costs nothing: the `out` runs on the cycle whose side-set
//! holds the clock low, so the two write the same level and either one winning
//! gives the same pins. The clock's polarity is the pad's, which inverts both
//! writes together, so the agreement holds for either edge a panel takes.
//!
//! An overlap is only free on that one pin, which is why the clock is wired
//! there or outside the window altogether. A side-set aimed at another bus pin
//! would be a fight with every word that follows, and the side-set would win it —
//! RP2350 datasheet §11.5.6, *"If a side-set overlaps with an OUT/SET performed by
//! that state machine on the same cycle, the side-set takes precedence in the
//! overlapping region"* — so the corruption of that pin would be silent rather
//! than an error. `.side_set 1` against `side_set_pin_base(clock)` in
//! [`crate::driver`] is what keeps the window to one bit, and
//! [`crate::Hub75Pins::new`] refuses any clock pin inside the window other than
//! [`pins::RESERVED`].
//!
//! # The latch
//!
//! The latch bit rides in the last column's word, so on the bus `STB` rises at
//! the start of that word and falls at the start of the next row's first word:
//! the strobe is high across the last column's clock edge and never overlaps
//! another one, and the address lines move while the panel is blanked.
//!
//! Whether that latches a complete row is the *panel's* strobe answering, not
//! ours. `hub75-framebuffer` puts the latch bit in the last column's word, which
//! is right for the usual case — `STB` is the column shift registers' latch
//! clock, and the transfer is complete when the strobe is released, by which
//! time the last column is in. A panel that transferred on the strobe's *rising*
//! edge would be one column short: every row one column to the right, with a
//! dark left edge. Ladder rung D is the reading that tells the two apart, and it
//! came out in favor of this placement; if a panel does want the latch a word
//! later, `hub75-framebuffer` has a feature for it (`tail-closes-latch`).

use crate::pins;
use pio::{Program, RP2040_MAX_PROGRAM_SIZE};

/// `pull_threshold` for the stream program: the fourteen bits of the word that
/// one `out pins, 14` shifts out. See the module docs — this is what keeps one
/// FIFO word per column, and why it is not the whole 16-bit word.
pub const PULL_THRESHOLD: u8 = pins::WORD_PINS;

/// PIO cycles a word costs: the word and the clock edge, in either clock
/// polarity.
pub const CYCLES_PER_WORD: u32 = 2;

/// The stream program: one framebuffer word per panel column, straight out to
/// the fourteen pins of the HUB75 window, with the panel clock on a pin the
/// side-set drives.
///
/// Configure the state machine with `out_pins(base, WORD_PINS)`,
/// `side_set_pin_base(clock)`, `autopull(true)`,
/// `pull_threshold(PULL_THRESHOLD)` and
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
pub fn hub75_stream() -> Program<{ RP2040_MAX_PROGRAM_SIZE }> {
    pio::pio_asm!(
        ".side_set 1",
        ".wrap_target",
        "    out pins, 14     side 0   ; the word -> the window, clock low",
        "    nop              side 1   ; the clock's edge: the panel shifts this column in",
        ".wrap",
    )
    .program
}
