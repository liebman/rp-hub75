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
//! | word bits | pins | signal |
//! |---|---|---|
//! | 0-2 | GP0-2 | `R0 G0 B0` |
//! | 3-5 | GP3-5 | `R1 G1 B1` |
//! | 6-10 | GP6-10 | `A B C D E` |
//! | 11 | GP11 | `CLK`, reserved and always zero — the PIO drives it |
//! | 12 | GP12 | `STB`, the latch |
//! | 13 | GP13 | `OE`, active low |
//!
//! Bit *n* is pin *n* across the whole bus, so one `out pins, 14` puts a whole
//! word on the pins, and the only thing left for the PIO to *generate* rather
//! than copy out of the word is the clock.
//!
//! # The program: two cycles per word, one word per column
//!
//! ```text
//! cycle         1                     2
//!               out pins, 14  side 0   nop  side 1
//! GP0-13        the word               hold
//! GP11 (CLK)    low                    rising edge
//! panel         data changes           shifts this column in
//! ```
//!
//! The clock is edge triggered, so a word needs exactly two cycles: the word goes
//! out with the clock at its idle level, and the edge a cycle later is what shifts
//! it into the panel, with the word stable for a whole PIO cycle before that
//! edge.
//!
//! [`hub75_stream`] with `invert_clock` is the same two cycles with the side-set
//! values swapped, for a panel that takes the column on the clock's *falling*
//! edge:
//!
//! ```text
//! cycle         1                     2
//!               out pins, 14  side 1   nop  side 0
//! GP11 (CLK)    high                   falling edge
//! panel         data changes           shifts this column in
//! ```
//!
//! # The one invariant
//!
//! **One FIFO word per column.** `pull_threshold` is [`PULL_THRESHOLD`], the
//! fourteen bits one `out pins, 14` shifts out, so the OSR refills exactly on a
//! column boundary and the word's fields cannot drift. Sixteen — the obvious
//! value, and one word — would leave the word's two spare top bits in the OSR
//! and put every following word two bits out of place, which is the M2b
//! confetti in a new costume.
//!
//! # The one hair: bit 11
//!
//! The reserved clock slot sits inside the `out pins, 14` window, so every word
//! drives GP11 as well as the side-set does, and bit 11 is always zero. On a
//! normal panel that costs nothing: its `out` runs on the cycle whose side-set
//! holds the clock low, so the two write the same level and either one winning
//! gives the same pins.
//!
//! The inverted variant is the one place they disagree — its `out` runs with the
//! side-set asking for the clock *high* while the word writes a zero to that pin —
//! and there the PIO's own rule settles it, RP2350 datasheet §11.5.6: *"If a
//! side-set overlaps with an OUT/SET performed by that state machine on the same
//! cycle, the side-set takes precedence in the overlapping region"* (the same
//! thing stated for the pin-mapping chapter in §11.2.6). So GP11 ends that cycle
//! high, the clock falls cleanly on the next one, and the inverted program is
//! exactly what it looks like: the normal program with one bit of side-set flipped.
//! The emulator implements the rule, by applying the side-set over the instruction's
//! own pin write.
//!
//! The tests still assert that the two windows only ever overlap on the clock pin:
//! a side-set aimed at another bus pin would be a fight with every word that
//! follows, and the side-set would win it — a silent corruption of that pin rather
//! than an error, which is the fault the check exists to catch.
//!
//! # The latch, and the one thing left to see on hardware
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
//! dark left edge. The tests below model both, `NOTES.md` names the ladder rung
//! that tells them apart, and if the hardware does want the latch a word later,
//! `hub75-framebuffer` has a feature for it (`tail-closes-latch`).

use crate::pins;
use pio::{Program, RP2040_MAX_PROGRAM_SIZE};

/// Bits of framebuffer word a column consumes: the whole 16-bit word, which is
/// also the transfer size the DMA moves. The top two bits are spare.
pub const BITS_PER_WORD: u32 = 16;

/// Bit of a framebuffer word that drives [`pins::STB`], the row latch.
pub const LATCH_BIT: u16 = 1 << pins::STB;

/// Bit of a framebuffer word that drives [`pins::OE`].
///
/// The pin is active low, so this bit *set* means the outputs are off — which is
/// what `hub75-framebuffer` uses as its blanking level by default. See
/// [`crate::table::BLANK_WORD`] for the one place the driver depends on that
/// convention.
pub const OE_BIT: u16 = 1 << pins::OE;

/// `pull_threshold` for the stream program: the fourteen bits of the word that
/// one `out pins, 14` shifts out. See the module docs — this is what keeps one
/// FIFO word per column, and why it is not `BITS_PER_WORD`.
pub const PULL_THRESHOLD: u8 = pins::WORD_PINS;

/// PIO cycles a word costs: the word and the clock edge, in either clock
/// polarity.
pub const CYCLES_PER_WORD: u32 = 2;

/// The stream program: one framebuffer word per panel column, straight out to
/// GP0-13, with the panel clock on GP11.
///
/// Configure the state machine with `out_pins(WORD_BASE, WORD_PINS)`,
/// `side_set_pin_base(CLK)`, `autopull(true)`, `pull_threshold(PULL_THRESHOLD)`
/// and `out_shift_direction(ShiftDirection::Right)` — the words are handed out
/// least-significant bit first, and `Left` reverses every word, which turns the
/// picture into noise. Then `set_pindirs` all fourteen pins as outputs.
///
/// `invert_clock` picks the edge the panel takes a column on: `false` for the
/// usual panel, which shifts on the clock's rising edge, and `true` for one that
/// shifts on the falling edge. The two programs are the same two instructions
/// with the side-set values swapped, so the word always changes with the clock at
/// its idle level and the panel always gets a full cycle of setup before the edge
/// it samples on. The inverted one is the variant whose `out` and side-set
/// disagree about GP11 — see the module docs.
pub fn hub75_stream(invert_clock: bool) -> Program<{ RP2040_MAX_PROGRAM_SIZE }> {
    if invert_clock {
        pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "    out pins, 14     side 1   ; the word -> GP0-13, clock high",
            "    nop              side 0   ; CLK falling edge: the panel shifts this column in",
            ".wrap",
        )
        .program
    } else {
        pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "    out pins, 14     side 0   ; the word -> GP0-13, clock low",
            "    nop              side 1   ; CLK rising edge: the panel shifts this column in",
            ".wrap",
        )
        .program
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::{PANEL_HEIGHT, PANEL_WIDTH, ROW_PAIRS};
    use hub75_framebuffer::FrameBuffer;
    use hub75_framebuffer::bitplane::plain::DmaFrameBuffer;
    use pio::{Instruction, InstructionOperands, MovDestination, MovSource, OutDestination};

    /// The fourteen HUB75 pins, GP0-GP13, as the program leaves them.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    struct Pins {
        level: [bool; 14],
    }

    impl Pins {
        /// The pin levels a word means. This is the whole point of the
        /// `interstate75` layout, and the reason the program is three
        /// instructions long: bit *n* is pin *n*.
        fn from_word(word: u16) -> Self {
            let mut level = [false; 14];
            for (pin, pin_level) in level.iter_mut().enumerate() {
                *pin_level = word & (1u16 << pin) != 0;
            }
            Self { level }
        }

        /// The six colour bits, GP0-GP5 = R0 G0 B0 R1 G1 B1.
        fn colour(self) -> u16 {
            let mut value = 0;
            for pin in 0..6 {
                value |= u16::from(self.level[pin]) << pin;
            }
            value
        }

        /// The row address, GP6-GP10 = A B C D E.
        fn addr(self) -> u8 {
            let mut value = 0;
            for pin in 6..11 {
                value |= u8::from(self.level[pin]) << (pin - 6);
            }
            value
        }

        fn latch(self) -> bool {
            self.level[pins::STB as usize]
        }

        fn oe(self) -> bool {
            self.level[pins::OE as usize]
        }

        fn clock(self) -> bool {
            self.level[pins::CLK as usize]
        }
    }

    /// The one state machine: the slice of PIO the stream program uses, stepped
    /// one instruction — one PIO cycle — at a time.
    struct Sm<'a> {
        program: Program<{ RP2040_MAX_PROGRAM_SIZE }>,
        /// The `out pins` window: base and pin count, as the state machine is
        /// configured on real hardware.
        window: (u8, u8),
        /// The side-set window: base and pin count, which here is the clock.
        side_set: (u8, u8),
        words: &'a [u16],
        /// Index of the next word to pull.
        next: usize,
        pc: usize,
        osr: u16,
        /// Bits of `osr` left to shift out.
        remaining: u32,
        /// Whole words pulled off the stream so far.
        pulled: usize,
        /// PIO cycles executed.
        cycles: u32,
    }

    impl<'a> Sm<'a> {
        fn new(
            program: Program<{ RP2040_MAX_PROGRAM_SIZE }>,
            window: (u8, u8),
            side_set_base: u8,
            words: &'a [u16],
        ) -> Self {
            let side_set = (side_set_base, program.side_set.bits());
            Self {
                program,
                window,
                side_set,
                words,
                next: 0,
                pc: 0,
                osr: 0,
                remaining: 0,
                pulled: 0,
                cycles: 0,
            }
        }

        /// Execute one instruction, which is one PIO cycle.
        fn step(&mut self, pins: &mut Pins) {
            let instruction =
                Instruction::decode(self.program.code[self.pc], self.program.side_set).unwrap();

            // The hardware applies the side-set at the top of the instruction.
            if let Some(value) = instruction.side_set {
                for offset in 0..self.side_set.1 {
                    pins.level[(self.side_set.0 + offset) as usize] = value & (1 << offset) != 0;
                }
            }

            match instruction.operands {
                InstructionOperands::OUT {
                    destination,
                    bit_count,
                } => {
                    if self.remaining == 0 {
                        // Autopull: `pull_threshold` bits have been shifted out,
                        // so the OSR takes the next FIFO word. With the
                        // threshold set to the bits one `out` takes that is
                        // exactly once per column — and the word's two spare top
                        // bits go out with it.
                        let word = *self.words.get(self.next).expect("the stream ran dry");
                        self.next += 1;
                        self.pulled += 1;
                        self.osr = word;
                        self.remaining = u32::from(PULL_THRESHOLD);
                    }

                    let bit_count = u32::from(bit_count);
                    assert!(
                        bit_count <= self.remaining,
                        "a program has to consume exactly one whole word per column"
                    );

                    let mask = if bit_count >= 16 {
                        u32::from(u16::MAX)
                    } else {
                        (1 << bit_count) - 1
                    };
                    let value = u32::from(self.osr) & mask;
                    self.osr >>= bit_count;
                    self.remaining -= bit_count;

                    match destination {
                        OutDestination::PINS => {
                            assert_eq!(
                                bit_count,
                                u32::from(self.window.1),
                                "a program writes a different window than its state machine is configured for"
                            );
                            for pin in 0..self.window.1 {
                                let absolute = self.window.0 + pin;
                                let level = value & (1 << pin) != 0;

                                // The PIO applies the side-set over the top of the
                                // instruction's own pin write: "if a side-set
                                // overlaps with an OUT/SET performed by that state
                                // machine on the same cycle, the side-set takes
                                // precedence in the overlapping region" (RP2350
                                // datasheet §11.5.6). That is what lets the
                                // inverted program hold the clock high on a cycle
                                // where the word writes a zero to that pin.
                                //
                                // The only pin they may share is the clock. The
                                // crate's reserved bit 11 is what lets one `out`
                                // write the whole bus, and a side-set anywhere else
                                // in the window would win against a bus pin on
                                // every word — silent corruption rather than an
                                // error, which is what this check is for.
                                if (self.side_set.0..self.side_set.0 + self.side_set.1)
                                    .contains(&absolute)
                                {
                                    assert_eq!(
                                        absolute,
                                        pins::CLK,
                                        "the side-set window has to stay on the clock pin"
                                    );
                                    continue;
                                }

                                pins.level[absolute as usize] = level;
                            }
                        }
                        OutDestination::NULL => {}
                        other => panic!("unexpected out destination {other:?}"),
                    }
                }
                InstructionOperands::MOV {
                    destination: MovDestination::Y,
                    source: MovSource::Y,
                    ..
                } => {
                    // A nop: the idle cycles either side of the clock edge.
                }
                other => panic!("unexpected instruction {other:?}"),
            }

            self.cycles += 1;
            self.pc = if self.pc == usize::from(self.program.wrap.source) {
                usize::from(self.program.wrap.target)
            } else {
                self.pc + 1
            };
        }
    }

    /// `pins` with the clock on `level`, for the comparisons below: a framebuffer
    /// word carries every pin except the one the PIO generates.
    fn with_clock(pins: Pins, level: bool) -> Pins {
        let mut held = pins;
        held.level[pins::CLK as usize] = level;
        held
    }

    /// One word through the machine, one PIO cycle at a time: a whole wrap of the
    /// program. Returns the pin levels after each of those cycles, so a test can
    /// say what is on the pins at the clock edge and what is left at the end of
    /// the word.
    fn clock_out(machine: &mut Sm<'_>, pins: &mut Pins, index: usize) -> std::vec::Vec<Pins> {
        let start = machine.pc;
        let mut after = std::vec::Vec::new();
        while machine.pc != start || after.is_empty() {
            machine.step(pins);
            after.push(*pins);
        }
        assert_eq!(
            machine.remaining, 0,
            "word {index}: the word was not consumed exactly"
        );
        after
    }

    /// Stream `words` through the one state machine and check the contract the
    /// panel depends on:
    ///
    /// * every word costs exactly [`CYCLES_PER_WORD`] cycles and is consumed
    ///   whole, spare bits and all — one FIFO word per column, no drift;
    /// * the `out` is the only instruction that writes the pins, so the word is
    ///   on the bus for the whole of the clock edge: the setup and hold the panel
    ///   needs;
    /// * one clock edge per word, with the colours, the address, the latch and
    ///   OE already on the pins — rising for a normal panel, falling for one
    ///   whose clock is inverted;
    /// * at the end of each word the pins carry that word, bit for bit, which is
    ///   what "the framebuffer goes out as-is" means.
    ///
    /// The words are the shape a framebuffer writes: bit 11 clear, always.
    ///
    /// Returns the cycles run and the number of clock edges seen.
    fn stream(words: &[u16], invert_clock: bool) -> (u32, usize) {
        let mut pins = Pins::default();
        let mut machine = Sm::new(
            hub75_stream(invert_clock),
            (pins::WORD_BASE, pins::WORD_PINS),
            pins::CLK,
            words,
        );
        let mut cycles = 0;
        let mut edges = 0;

        for (index, &word) in words.iter().enumerate() {
            let cycle = clock_out(&mut machine, &mut pins, index);
            cycles += cycle.len() as u32;

            assert_eq!(
                cycle.len() as u32,
                CYCLES_PER_WORD,
                "word {index}: the program costs more cycles than it advertises"
            );

            // The word goes out on the first cycle with the clock at its idle
            // level — low for a normal panel, high for an inverted one — and
            // nothing but the clock moves after that: it is the one pin the
            // program generates rather than copies out of the word.
            assert_eq!(
                cycle[0].clock(),
                invert_clock,
                "word {index}: the word changes with the clock at its idle level"
            );
            for snapshot in &cycle[1..] {
                assert_eq!(
                    *snapshot,
                    with_clock(cycle[0], snapshot.clock()),
                    "word {index}: an idle cycle moved a data pin"
                );
            }

            // The last cycle is the edge the panel takes the column on, with the
            // whole word already on the pins.
            let at_edge = *cycle.last().unwrap();
            let before_edge = cycle[cycle.len() - 2];
            let edge = if invert_clock {
                before_edge.clock() && !at_edge.clock()
            } else {
                !before_edge.clock() && at_edge.clock()
            };
            assert!(
                edge,
                "word {index}: the clock edge is what shifts the column in"
            );
            assert_eq!(
                at_edge.colour(),
                word & 0x3f,
                "word {index}: the colours before the edge"
            );
            assert_eq!(
                at_edge.addr(),
                ((word >> pins::ROW_A) & 0x1f) as u8,
                "word {index}: the address before the edge"
            );
            assert_eq!(
                at_edge.latch(),
                word & (1 << pins::STB) != 0,
                "word {index}: the latch before the edge"
            );
            assert_eq!(
                at_edge.oe(),
                word & (1 << pins::OE) != 0,
                "word {index}: output enable before the edge"
            );
            edges += 1;

            assert_eq!(
                pins,
                with_clock(Pins::from_word(word), at_edge.clock()),
                "word {index}: the pins after the word"
            );
        }

        assert_eq!(
            machine.pulled,
            words.len(),
            "every word reaches the pins exactly once"
        );
        assert_eq!(machine.cycles, words.len() as u32 * CYCLES_PER_WORD);
        assert_eq!(
            hub75_stream(invert_clock).code.len() as u32,
            CYCLES_PER_WORD,
            "the program is one wrap: the word and the clock edge"
        );
        assert_eq!(edges, words.len(), "one clock edge per word");

        (cycles, edges)
    }

    /// Words with every field in both states — colours, address, latch, OE — and
    /// the reserved clock bit (bit 11) clear, which is the contract the
    /// framebuffer writes under.
    const WORDS: [u16; 6] = [0x0000, 0x07ff, 0x1000, 0x2000, 0x1234, 0x37ff];

    /// The M2b diagonal, drawn into a real single-plane framebuffer: white at
    /// `x == row` in the top half, blue where the bottom half's arm continues
    /// it, so the two arms join at the middle and the whole picture is `x == y`.
    fn m2b_diagonal() -> DmaFrameBuffer<ROW_PAIRS, PANEL_WIDTH, 1> {
        use embedded_graphics::Pixel;
        use embedded_graphics::geometry::Point;
        use embedded_graphics::pixelcolor::RgbColor;
        use embedded_graphics::prelude::Drawable;
        use hub75_framebuffer::Color;

        let mut fb = DmaFrameBuffer::<ROW_PAIRS, PANEL_WIDTH, 1>::new();
        for row in 0..ROW_PAIRS as i32 {
            Pixel(Point::new(row, row), Color::WHITE)
                .draw(&mut fb)
                .unwrap();
            Pixel(Point::new(32 + row, 32 + row), Color::BLUE)
                .draw(&mut fb)
                .unwrap();
        }
        fb
    }

    /// The words of a framebuffer's one and only BCM segment.
    ///
    /// # Safety
    ///
    /// The caller must keep `fb` alive for as long as the slice is used.
    unsafe fn segment_words(fb: &DmaFrameBuffer<ROW_PAIRS, PANEL_WIDTH, 1>) -> &[u16] {
        let segment = fb.bcm_segment(0);
        // SAFETY: the segment points into `fb`'s own plane data.
        unsafe { core::slice::from_raw_parts(segment.ptr.cast::<u16>(), segment.len / 2) }
    }

    /// The core of the driver: a word out to the pins unchanged, one word per
    /// column, at a steady [`CYCLES_PER_WORD`] cycles each.
    #[test]
    fn a_word_goes_out_as_its_own_pin_levels() {
        let (cycles, edges) = stream(&WORDS, false);
        assert_eq!(cycles, WORDS.len() as u32 * CYCLES_PER_WORD);
        assert_eq!(edges, WORDS.len());
    }

    /// The same stream with the clock inverted, for a panel that takes the
    /// column on the falling edge: the same words at the same rate, the edge the
    /// panel cares about at the other level, and the side-set values swapped.
    /// Everything else about the two waveforms is asserted identically by
    /// [`stream`], which is the point: invert-clock is one bit of side-set, not a
    /// second design.
    ///
    /// It is also the test of the side-set precedence rule (RP2350 §11.5.6): the
    /// clock has to stay *high* through the cycle whose word writes a zero to that
    /// pin, so [`stream`]'s "the word changes with the clock at its idle level"
    /// only holds if the side-set wins the overlap.
    #[test]
    fn the_inverted_clock_shifts_on_the_falling_edge() {
        let (cycles, edges) = stream(&WORDS, true);
        assert_eq!(cycles, WORDS.len() as u32 * CYCLES_PER_WORD);
        assert_eq!(edges, WORDS.len());
    }

    /// The check that keeps the side-set honest: it may only drive the clock pin,
    /// because any bus pin it touched would be silently rewritten by every word
    /// that follows (the instruction's own pin write lands first, the side-set
    /// goes over the top of it). Here the side-set is aimed at GP6, the address's
    /// A bit, which the stream writes with every word.
    #[test]
    #[should_panic(expected = "the side-set window has to stay on the clock pin")]
    fn a_side_set_off_the_clock_pin_is_rejected() {
        let words = [0x0000u16];
        let mut pins = Pins::default();
        let mut machine = Sm::new(
            hub75_stream(false),
            (pins::WORD_BASE, pins::WORD_PINS),
            pins::ROW_A,
            &words,
        );
        let _ = clock_out(&mut machine, &mut pins, 0);
    }

    /// The real thing: a `hub75-framebuffer` bitplane framebuffer in the
    /// `interstate75` layout, streamed through the one program and checked
    /// against the crate's own words and the panel's pin mapping.
    #[test]
    fn streams_a_bitplane_frame_from_the_framebuffer() {
        let fb = m2b_diagonal();

        // A single-plane plane-major framebuffer is one contiguous segment,
        // which is what lets one free-running DMA channel stream a whole refresh
        // with no CPU involvement.
        assert_eq!(fb.bcm_segment_count(), 1);
        let segment = fb.bcm_segment(0);
        assert_eq!(segment.reps, 1);
        assert_eq!(segment.len, 32 * 64 * 2);

        // SAFETY: the slice is used while `fb` is alive.
        let words = unsafe { segment_words(&fb) };

        // The words the crate really writes leave the reserved clock slot clear,
        // which is what keeps the normal variant independent of *which* of the two
        // writes wins GP11 on an `out` cycle. (The inverted variant is the one
        // that needs the side-set to win — see the module docs.)
        assert!(
            words.iter().all(|word| *word & (1 << pins::CLK) == 0),
            "bit 11 is the PIO's clock and the crate leaves it clear"
        );

        let (cycles, edges) = stream(words, false);

        assert_eq!(edges, words.len(), "one clock edge per column");
        assert_eq!(cycles as usize, words.len() * CYCLES_PER_WORD as usize);
        assert_eq!(
            words
                .iter()
                .filter(|word| *word & (1 << pins::STB) != 0)
                .count(),
            ROW_PAIRS,
            "one latch per row pair"
        );
    }

    /// The address on the wires while a row is *displayed* is that row's
    /// address. The crate puts the previous slot's address in every word of a
    /// slot — one step behind the data being clocked in — because a panel shows
    /// latched data during the *next* row's clocks: the address that belongs
    /// with the data already in the latches is the one that has to be selected
    /// while that data is up. Slot 0 carries 31, slot 1 carries 0, and so on,
    /// which is what puts the panel's picture in place rather than one row out.
    #[test]
    fn slot_s_carries_the_address_of_the_row_it_displays() {
        let fb = m2b_diagonal();
        // SAFETY: the slice is used while `fb` is alive.
        let words = unsafe { segment_words(&fb) };

        let addresses: std::vec::Vec<u8> = words
            .chunks_exact(PANEL_WIDTH)
            .map(|slot| ((slot[0] >> pins::ROW_A) & 0x1f) as u8)
            .collect();

        // One address per slot, and the same one in every word of the slot.
        for (index, slot) in words.chunks_exact(PANEL_WIDTH).enumerate() {
            assert!(
                slot.iter()
                    .all(|word| ((word >> pins::ROW_A) & 0x1f) as u8 == addresses[index]),
                "slot {index}: the address changes inside a row"
            );
        }

        // Slot 0 carries 31, slot 1 carries 0, and so on: one row behind.
        let expected: std::vec::Vec<u8> = (0..ROW_PAIRS)
            .map(|slot| ((slot + ROW_PAIRS - 1) % ROW_PAIRS) as u8)
            .collect();
        assert_eq!(addresses, expected);
    }

    /// When a strobe copies the shift registers into the output latches.
    ///
    /// HUB75's `STB` is the column shift registers' latch clock (`RCK` on a
    /// 74HC595 and its relatives), so the copy is complete when the strobe is
    /// released, by which time the row's last column has been clocked in:
    /// `AtRelease`. A panel that transferred on the strobe's *rising* edge
    /// instead would copy one column early — see
    /// `a_strobe_that_latches_on_its_rising_edge_shifts_every_row`.
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Strobe {
        AtRelease,
        AtRise,
    }

    /// A HUB75 panel's view of the pin levels.
    ///
    /// The panel is one set of column latches plus a row decoder: colour bits
    /// shift into the registers on the clock edge the panel takes them on, the
    /// strobe copies the registers into the output latches, and those latches —
    /// which are what the LEDs are driven from; the shift registers themselves
    /// are not visible — drive the row pair the five address lines select. Only
    /// one pair is lit at a time and *every* pair shows the same latched content,
    /// which is why a panel scans at all, why the address of the row being
    /// displayed has to be on the wires while the previous row is still in the
    /// latches, and why an emulator with a latch per row pair (as an earlier
    /// version of this one had) hides exactly the timing it is meant to check.
    ///
    /// The per-word checks say the words reach the pins; this says what the
    /// panel then does with them, which is where the strobe timing — the one
    /// thing pin levels cannot answer — shows up. It is the closest thing to a
    /// screenshot that exists off-hardware.
    struct Panel {
        /// Colour bits clocked in so far, oldest at column 0, newest at the
        /// right edge.
        shift: [[u8; PANEL_WIDTH]; 2],
        /// The output latches: the row pair's worth of colour the panel is
        /// showing, shared by every pair because the panel has one set of them.
        latched: [[u8; PANEL_WIDTH]; 2],
        /// PIO cycles each pixel spent lit, `[row][column]`.
        lit: [[u32; PANEL_WIDTH]; PANEL_HEIGHT],
        /// When the strobe copies the shift registers.
        strobe: Strobe,
        /// `true` for a panel that takes the column on the clock's falling edge.
        shift_on_falling: bool,
    }

    impl Panel {
        fn new(strobe: Strobe, shift_on_falling: bool) -> Self {
            Self {
                shift: [[0; PANEL_WIDTH]; 2],
                latched: [[0; PANEL_WIDTH]; 2],
                lit: [[0; PANEL_WIDTH]; PANEL_HEIGHT],
                strobe,
                shift_on_falling,
            }
        }

        /// Advance by one PIO cycle. `before` and `after` are the pin levels
        /// either side of it, which is how the edges are seen.
        fn step(&mut self, before: Pins, after: Pins) {
            let clocked = if self.shift_on_falling {
                before.clock() && !after.clock()
            } else {
                !before.clock() && after.clock()
            };
            if clocked {
                let colour = after.colour();
                for half in 0..2 {
                    // The oldest column walks towards the left edge; the newest
                    // lands at the right one.
                    self.shift[half].rotate_left(1);
                    self.shift[half][PANEL_WIDTH - 1] = if half == 0 {
                        (colour & 0b111) as u8
                    } else {
                        ((colour >> 3) & 0b111) as u8
                    };
                }
            }

            let strobed = match self.strobe {
                Strobe::AtRelease => before.latch() && !after.latch(),
                Strobe::AtRise => !before.latch() && after.latch(),
            };
            if strobed {
                self.latched = self.shift;
            }

            // Active low, and the pair a 1/32-scan panel lights together is `a`
            // and `a + ROW_PAIRS`. Both halves show the same latched row.
            if !after.oe() {
                let address = usize::from(after.addr());
                for column in 0..PANEL_WIDTH {
                    self.lit[address][column] += u32::from(self.latched[0][column] != 0);
                    self.lit[address + ROW_PAIRS][column] +=
                        u32::from(self.latched[1][column] != 0);
                }
            }
        }

        /// Forget the light seen so far, so the next refresh can be looked at on
        /// its own: the first one is the panel filling its latches, the second
        /// is the picture the eye settles on.
        fn reset_lit(&mut self) {
            self.lit = [[0; PANEL_WIDTH]; PANEL_HEIGHT];
        }

        /// The refresh as a picture: one line per panel row, `.` for a pixel
        /// that never lit, `1`-`9` for how much of its duty cycle it got.
        fn render(&self) -> std::vec::Vec<std::string::String> {
            let peak = self.lit.iter().flatten().copied().max().unwrap_or(1).max(1);
            (0..PANEL_HEIGHT)
                .map(|row| {
                    (0..PANEL_WIDTH)
                        .map(|column| {
                            let lit = self.lit[row][column];
                            if lit == 0 {
                                '.'
                            } else {
                                char::from(b'0' + ((lit * 9 / peak) as u8).min(9))
                            }
                        })
                        .collect()
                })
                .collect()
        }
    }

    /// The stream as the DMA drives it: one state machine, one set of pin levels,
    /// running for ever over the same words.
    ///
    /// The stream does *not* stop at the seam between two passes — that is the
    /// whole point of the DMA's read ring — so a pass has to be able to follow
    /// another pass on the same pins: the last latch of one pass is released by
    /// the first word of the next, and a harness that restarted the pins at the
    /// seam would drop that edge and lose the row.
    struct Stream<'a> {
        pins: Pins,
        machine: Sm<'a>,
    }

    impl<'a> Stream<'a> {
        fn new(words: &'a [u16], invert_clock: bool) -> Self {
            Self {
                pins: Pins::default(),
                machine: Sm::new(
                    hub75_stream(invert_clock),
                    (pins::WORD_BASE, pins::WORD_PINS),
                    pins::CLK,
                    words,
                ),
            }
        }

        /// One pass over the words, one PIO cycle at a time, with the panel on
        /// the far end of the pins. The read pointer goes back to the start of
        /// the words at the seam, which is what the ring does: in hardware the
        /// state machine never sees the boundary at all.
        fn refresh(&mut self, panel: &mut Panel) {
            self.machine.next = 0;
            let cycles = self.machine.words.len() * self.machine.program.code.len();
            for _ in 0..cycles {
                let before = self.pins;
                self.machine.step(&mut self.pins);
                panel.step(before, self.pins);
            }
        }
    }

    /// The whole point of the one program, and the closest thing to a
    /// screenshot: the M2b diagonal streamed once to fill the panel's latches
    /// and then once more, counting the light every pixel takes. The diagonal is
    /// `x == y` in both halves, so a correct panel shows every row lit at its
    /// own column, one pixel per row — in place, because the address the crate
    /// puts on the wires while a row is lit is that row's own.
    ///
    /// The duty cycle is the 1/32 scan limit less the one word each row pair
    /// needs blanked for its latch and address change: 63 of a slot's 64 words.
    /// The emulator models the default OE polarity (bit 13 high = blanked). With
    /// the crate's `invert-oe` feature the *words* flip that level, and the model
    /// would have to flip with it — the driver only needs the crate and the blank
    /// plane to agree, which is what `table`'s own tests check.
    #[cfg(not(feature = "invert-oe"))]
    #[test]
    fn the_panel_shows_the_diagonal_where_the_framebuffer_put_it() {
        let fb = m2b_diagonal();
        // SAFETY: the slice is used while `fb` is alive.
        let words = unsafe { segment_words(&fb) };

        let mut panel = Panel::new(Strobe::AtRelease, false);
        let mut stream = Stream::new(words, false);
        // The first pass fills the panel's latches; the second is the steady
        // state the eye sees, and the release of the first pass's last latch
        // happens on the first word of it.
        stream.refresh(&mut panel);
        panel.reset_lit();
        stream.refresh(&mut panel);

        for line in panel.render() {
            std::println!("{line}");
        }

        for row in 0..PANEL_HEIGHT {
            let lit: std::vec::Vec<usize> = (0..PANEL_WIDTH)
                .filter(|column| panel.lit[row][*column] != 0)
                .collect();
            assert_eq!(lit, [row], "row {row}: exactly one pixel, in place");
        }

        // Every pixel of the diagonal gets the same slice of the refresh.
        let duties: std::vec::Vec<u32> = (0..PANEL_HEIGHT).map(|row| panel.lit[row][row]).collect();
        assert_eq!(duties[0], (PANEL_WIDTH as u32 - 1) * CYCLES_PER_WORD);

        // A pixel that took light from a row it does not belong to would be out
        // by whole rows of clocking, not by the odd clock at the seam between
        // the two emulated refreshes.
        let spread = duties.iter().max().unwrap() - duties.iter().min().unwrap();
        assert!(
            spread <= CYCLES_PER_WORD,
            "the diagonal's pixels should share one duty cycle: {duties:?}"
        );
    }

    /// The other clock polarity, with a panel that takes the column on the
    /// falling edge: the same words, the same picture. The word changes two
    /// cycles before the edge either way, so inverting the clock moves which
    /// edge the panel takes and nothing else.
    #[test]
    fn the_inverted_clock_draws_the_same_picture() {
        let fb = m2b_diagonal();
        // SAFETY: the slice is used while `fb` is alive.
        let words = unsafe { segment_words(&fb) };

        let mut panel = Panel::new(Strobe::AtRelease, true);
        let mut stream = Stream::new(words, true);
        stream.refresh(&mut panel);
        panel.reset_lit();
        stream.refresh(&mut panel);

        for row in 0..PANEL_HEIGHT {
            let lit: std::vec::Vec<usize> = (0..PANEL_WIDTH)
                .filter(|column| panel.lit[row][*column] != 0)
                .collect();
            assert_eq!(lit, [row], "row {row}: exactly one pixel, in place");
        }
    }

    /// What a panel that transferred on the strobe's *rising* edge would show
    /// instead: the copy happens a cycle before the row's last column is clocked
    /// in, so every row's content is one column to the right and the last row's
    /// pixel shifts off the end. The shift registers are never cleared, so the
    /// left edge also carries the stale bit from the row before — which the crate
    /// fills with the row's own last column, so it lights only where that column
    /// did (once, at the bottom half of pair 0, from the wrap-around).
    ///
    /// This is the test to read next to a photo: if ladder rung D (red, green,
    /// blue at x = 0, 1, 2) puts its dots at x = 1, 2, 3, the strobe wants to be
    /// a word later, which `hub75-framebuffer`'s `tail-closes-latch` feature (or
    /// a latch on its own state machine) gives it.
    /// See the note on the diagonal test: this models the default OE polarity.
    #[cfg(not(feature = "invert-oe"))]
    #[test]
    fn a_strobe_that_latches_on_its_rising_edge_shifts_every_row() {
        let fb = m2b_diagonal();
        // SAFETY: the slice is used while `fb` is alive.
        let words = unsafe { segment_words(&fb) };

        let mut panel = Panel::new(Strobe::AtRise, false);
        let mut stream = Stream::new(words, false);
        stream.refresh(&mut panel);
        panel.reset_lit();
        stream.refresh(&mut panel);

        for line in panel.render() {
            std::println!("{line}");
        }

        for row in 0..PANEL_HEIGHT {
            let lit: std::vec::Vec<usize> = (0..PANEL_WIDTH)
                .filter(|column| panel.lit[row][*column] != 0)
                .collect();

            let mut expected: std::vec::Vec<usize> = std::vec::Vec::new();
            if row != PANEL_HEIGHT - 1 {
                // Its own pixel, one column along.
                expected.push(row + 1);
            }
            if row == ROW_PAIRS {
                // The stale bit, at the left edge, where the wrap-around brings
                // the last row's lit column 63 (the bottom half of the diagonal).
                expected.insert(0, 0);
            }

            assert_eq!(
                lit, expected,
                "row {row}: expected one column to the right, plus the stale bit"
            );
        }
    }
}
