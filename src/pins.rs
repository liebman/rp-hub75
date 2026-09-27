//! The HUB75 bus — the word layout, the pin window it is streamed into, and the
//! reference board's wiring.
//!
//! ## The word layout
//!
//! A HUB75 bus needs **fourteen** driven signals, and with
//! `hub75-framebuffer`'s `interstate75` feature one framebuffer word carries all
//! of them, one bit per signal, in this order:
//!
//! | word bit | signal |
//! |---|---|
//! | 0-2 | `R0 G0 B0`, the top half's colour |
//! | 3-5 | `R1 G1 B1`, the bottom half's colour |
//! | 6-10 | `A B C D E`, the row address |
//! | 11 | no signal — held at zero by the framebuffer |
//! | 12 | `STB`, the row latch |
//! | 13 | `OE`, the output enable, active low |
//!
//! [`LAYOUT`] states that mapping as data, in the order [`crate::Hub75Pins::new`]
//! takes the pins, and that constructor checks the wiring against it. The colours
//! and the address are the *framebuffer's* — this crate never reads a pixel — so
//! the values here are the contract between the two crates rather than an input
//! to the stream program. [`RESERVED`] is the one bit the framebuffer always
//! leaves at zero, which is what decides where the clock may be wired.
//!
//! ## The pin window
//!
//! A HUB75 bus needs **fourteen consecutive** GPIOs. `out pins, 14` writes
//! `[base, base + 13]` and no PIO instruction can move that window, so the
//! signals below are *offsets from the window's first pin*, in bus order. Each
//! offset is also the bit of a framebuffer word that carries that signal:
//! `hub75-framebuffer`'s `interstate75` layout puts word bit *n* on window pin
//! *n* (see [`crate::programs`]).
//!
//! On the Pimoroni Interstate 75 W the window starts at GP0, so every offset
//! here is also a GPIO number: `R0` is word bit 0 and GP0 at once. That is a
//! property of that board's wiring, not of the bus. Another run of fourteen
//! GPIOs uses these same offsets with a different base, which
//! [`Hub75Pins::new`](crate::Hub75Pins::new) derives from the *lowest* pin it is
//! given; [`MAX_WORD_BASE`] is where such a window may start at the latest.
//!
//! ## The clock
//!
//! The clock is the one signal the PIO *generates* rather than copies out of a
//! word: a side-set drives it, and the side-set has a pin base of its own
//! (`SM_PINCTRL`'s `SIDESET_BASE`), independent of the `out` window. The pin it
//! is wired to is therefore an argument of
//! [`Hub75Pins::new`](crate::Hub75Pins::new) rather than a position in the bus
//! order, and it may be any PIO-reachable GPIO: the window pin at [`RESERVED`],
//! where the framebuffer holds bit 11 at zero so the two writes agree, or a pin
//! outside the window entirely, in which case the reserved pin is left unwired.
//!
//! A clock on any *other* window pin is refused, because the side-set takes
//! precedence over an `out` on an overlapping pin (RP2350 datasheet §11.5.6) and
//! would silently overwrite that signal on every cycle.
//!
//! The clock's polarity is the pad's, not the program's: a panel that takes the
//! column on the falling edge gets that edge by inverting the clock pin's output
//! in the GPIO block (the crate's `invert-clock` feature), not from a second
//! program.
//!
//! The values come from Pimoroni's HUB75 driver (`pimoroni-pico`,
//! `drivers/hub75/hub75.hpp`), whose pin definitions are the same for the
//! RP2040 and RP2350 boards, and were confirmed on hardware with
//! `tools/panel_bringup.py` before any Rust was written.
//!
//! ## Logging
//!
//! The bus occupies GP0–GP13 on the reference board, which means GP0/GP1 (the
//! default UART0 TX/RX) are **not** available for logging: use USB CDC, or
//! SWD/RTT with a debug probe.

/// Red data, top half of the panel. Word bit 0.
pub const R0: u8 = 0;
/// Green data, top half of the panel. Word bit 1.
pub const G0: u8 = 1;
/// Blue data, top half of the panel. Word bit 2.
pub const B0: u8 = 2;
/// Red data, bottom half of the panel. Word bit 3.
pub const R1: u8 = 3;
/// Green data, bottom half of the panel. Word bit 4.
pub const G1: u8 = 4;
/// Blue data, bottom half of the panel. Word bit 5.
pub const B1: u8 = 5;

/// Row address line A, the least significant of the five (32 row pairs = 1/32
/// scan). Word bit 6.
pub const ROW_A: u8 = 6;
/// See [`ROW_A`]. Word bit 7.
pub const ROW_B: u8 = 7;
/// See [`ROW_A`]. Word bit 8.
pub const ROW_C: u8 = 8;
/// See [`ROW_A`]. Word bit 9.
pub const ROW_D: u8 = 9;
/// See [`ROW_A`]. Word bit 10.
pub const ROW_E: u8 = 10;

/// The word bit with no signal: `hub75-framebuffer` holds bit 11 at zero, and
/// the PIO drives the panel clock there when the clock is wired inside the
/// window.
///
/// The other side of the same fact is that nothing else may be wired to this
/// pin while the clock is not: `out pins, 14` writes the word's zero to it on
/// every cycle. A clock wired elsewhere therefore leaves this pin unconnected
/// and undriven, which [`crate::Hub75Pins::new`] accepts.
pub const RESERVED: u8 = 11;
/// Strobe / latch, active high. Word bit 12.
///
/// A panel that wants this inverted drives it out of the framebuffer's words,
/// whose polarity `hub75-framebuffer` selects (`stb_invert=True` in the
/// MicroPython API).
pub const STB: u8 = 12;
/// Output enable, active low. Word bit 13.
pub const OE: u8 = 13;

/// Number of pins a framebuffer word drives: one per bit, for PIO
/// `out pins, 14`. See the `interstate75` layout in [`crate::programs`].
pub const WORD_PINS: u8 = 14;

/// Word bit that carries each bus signal, in the order [`crate::Hub75Pins::new`]
/// takes the pins: `R0 G0 B0 R1 G1 B1 A B C D E STB OE`.
///
/// The clock is not in the list. The PIO generates it with a side-set rather
/// than copying it out of a word, so its pin is that constructor's own argument
/// and [`RESERVED`] is the slot it occupies on the reference wiring.
pub const LAYOUT: [u8; 13] = [
    R0, G0, B0, R1, G1, B1, ROW_A, ROW_B, ROW_C, ROW_D, ROW_E, STB, OE,
];

/// Signals a framebuffer word carries: the length of [`LAYOUT`].
pub const SIGNAL_COUNT: usize = LAYOUT.len();

/// Highest GPIO number the PIO can address at all.
///
/// `SM_PINCTRL`'s `OUT_BASE`, `SET_BASE`, `SIDESET_BASE` and `IN_BASE` are
/// **five-bit** fields, so the PIO addresses pins 0–31. An RP2350B's GPIO32–47
/// are out of its reach.
pub const MAX_PIN: u8 = 31;

/// Highest GPIO number the fourteen-pin window can start at: 18.
///
/// A window of [`WORD_PINS`] pins has to fit inside [`MAX_PIN`].
pub const MAX_WORD_BASE: u8 = MAX_PIN - (WORD_PINS - 1);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framebuffer::Color;
    use crate::framebuffer::FrameBuffer;
    use crate::framebuffer::bitplane::plain::DmaFrameBuffer;
    use embedded_graphics::geometry::Point;
    use embedded_graphics::pixelcolor::RgbColor;

    /// The framebuffer the tests draw on: 32 row pairs exercise all five address
    /// bits, and the row is wide enough for the blanking features' delays.
    type Fb = DmaFrameBuffer<32, 128, 6>;
    const NROWS: usize = 32;
    const COLS: usize = 128;
    const PLANES: usize = 6;

    /// The words of one bit-plane, as the DMA streams them.
    fn plane_words(fb: &Fb, plane: usize) -> &[u16] {
        // The last segment covers exactly one plane, so its declared length is
        // that plane's byte length — from the framebuffer's own sequence.
        let words = Fb::BCM_SEQUENCE[PLANES - 1].len / 2;
        // SAFETY: `bcm_segment_ptr` points at that plane's storage inside the
        // framebuffer, which is a `[Row; NROWS]` of 16-bit entries and therefore
        // 2-byte aligned, and `words` is the length the same crate declares for
        // it.
        unsafe { core::slice::from_raw_parts(fb.bcm_segment_ptr(plane).cast::<u16>(), words) }
    }

    /// The bits a drawn pixel sets are the ones [`LAYOUT`] names.
    ///
    /// The framebuffer is the authority for where a signal's word bit lives, and
    /// the driver relies on this table agreeing with it — a disagreement would
    /// swap the panel's colours with no error anywhere.
    #[test]
    fn the_layout_bits_are_the_ones_the_framebuffer_drives() {
        let cases: [(i32, i32, Color, u8); 6] = [
            (1, 1, Color::RED, R0),
            (2, 1, Color::GREEN, G0),
            (3, 1, Color::BLUE, B0),
            (1, NROWS as i32 + 1, Color::RED, R1),
            (2, NROWS as i32 + 1, Color::GREEN, G1),
            (3, NROWS as i32 + 1, Color::BLUE, B1),
        ];
        let blank = Fb::new();
        for (x, y, color, bit) in cases {
            let mut drawn = Fb::new();
            drawn.set_pixel(Point::new(x, y), color);

            for plane in 0..PLANES {
                let mut changed = 0;
                let mut bits = 0u16;
                for (blank_word, drawn_word) in plane_words(&blank, plane)
                    .iter()
                    .zip(plane_words(&drawn, plane))
                {
                    let delta = blank_word ^ drawn_word;
                    if delta != 0 {
                        changed += 1;
                        bits |= delta;
                    }
                }
                // One pixel is one word per plane, and the bit it set is the
                // signal's.
                assert_eq!(changed, 1, "one word per plane carries one pixel");
                assert_eq!(
                    bits,
                    1 << bit,
                    "word bit {} is not the one a drawn {color:?} sets",
                    bit,
                );
            }
        }
    }

    /// The framebuffer drives thirteen of the word's bits and holds
    /// [`RESERVED`] at zero, which is what lets the clock be wired there.
    #[test]
    fn the_reserved_bit_is_the_one_the_framebuffer_never_drives() {
        let mut fb = Fb::new();
        // Everything lit, so a reserved bit that collided with a signal would be
        // driven by that signal somewhere.
        for y in 0..(NROWS as i32 * 2) {
            for x in 0..COLS as i32 {
                fb.set_pixel(Point::new(x, y), Color::WHITE);
            }
        }

        for bit in 0..WORD_PINS {
            let driven = (0..PLANES).any(|plane| {
                plane_words(&fb, plane)
                    .iter()
                    .any(|word| word & (1 << bit) != 0)
            });
            assert_eq!(
                driven,
                bit != RESERVED,
                "word bit {bit} is {}",
                if driven { "driven" } else { "never driven" }
            );
        }
        // The two spare bits above the bus are never driven either.
        assert!(
            plane_words(&fb, 0)
                .iter()
                .all(|word| word >> WORD_PINS == 0)
        );
    }

    /// The control bits are where [`LAYOUT`] says: one latch per row, the address
    /// in the five bits ROW_A..=ROW_E, and the blanking bit between them.
    #[test]
    fn the_control_bits_are_where_the_layout_says() {
        let fb = Fb::new();
        let words = plane_words(&fb, 0);

        // One latch word per row, and the rows are evenly spaced.
        let mut latches = [0usize; NROWS];
        let mut count = 0;
        for (index, &word) in words.iter().enumerate() {
            if word & (1 << STB) != 0 {
                assert!(count < NROWS, "more latch words than rows");
                latches[count] = index;
                count += 1;
            }
        }
        assert_eq!(count, NROWS, "one latch word per row");
        let row = latches[1] - latches[0];
        for pair in latches.windows(2) {
            assert_eq!(pair[1] - pair[0], row, "the rows are evenly spaced");
        }

        // The address is the five bits ROW_A..=ROW_E, and no two rows share one.
        let colour_mask = [R0, G0, B0, R1, G1, B1]
            .iter()
            .fold(0u16, |mask, &bit| mask | (1 << bit));
        let mut addresses = [0u16; NROWS];
        for (slot, &index) in latches.iter().enumerate() {
            let word = words[index] & !(colour_mask | (1 << STB) | (1 << OE));
            assert_eq!(
                word & ((1 << ROW_A) - 1),
                0,
                "the address starts at bit {ROW_A}"
            );
            assert_eq!(word >> (ROW_E + 1), 0, "the address ends at bit {ROW_E}");
            addresses[slot] = word >> ROW_A;
        }
        for (slot, &address) in addresses.iter().enumerate() {
            assert!(address < NROWS as u16, "an address outside the panel");
            assert!(
                !addresses[..slot].contains(&address),
                "two rows share an address"
            );
        }

        // A row's words agree with each other about everything except the
        // blanking bit: the same address, and the latch on one word only.
        for &index in &latches {
            let start = index - latches[0];
            for &other in &words[start..start + row] {
                let differing = (words[index] ^ other) & !(colour_mask | (1 << STB));
                assert!(
                    differing == 0 || differing == (1 << OE),
                    "a row's words differ outside word bits {OE} and {STB}"
                );
            }
        }
    }
}
