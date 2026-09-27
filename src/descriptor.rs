//! The engine: the frame as a DMA descriptor table, played by a second channel.
//!
//! A **control** DMA channel reads this table and writes each entry into the data
//! channel's `AL3` register block, whose last word (`AL3_READ_ADDR_TRIG`)
//! triggers the run it describes. The data channel chains back to the control
//! channel when the run completes, so the two hand the picture to each other
//! without the CPU:
//!
//! ```text
//! control channel  writes (trans_count, read_addr) into the data channel's AL3
//!                  the read address starts the run
//! data channel     streams the run to the PIO FIFO, paced by its DREQ
//!                  completes, chains to the control channel
//! ```
//!
//! `AL3` is the register block whose trigger word is the last of the two words a
//! run needs, so a descriptor is exactly two words ([`Descriptor`]) and the data
//! channel's `CTRL` is written once at startup. Nothing about a run is
//! configurable per run, which is the point: a segment the crate asks for `reps`
//! times is materialised `reps` times, so this engine has no ring size, no
//! alignment rule and no power-of-two rule anywhere.
//!
//! # One interrupt per frame
//!
//! The table ends with a terminator: a [`Descriptor`] whose two words are zero.
//! Zero is not a trigger, so it starts nothing, and the data channel runs with
//! `IRQ_QUIET` set — RP2350 datasheet §12.6, *"an IRQ is raised when NULL is
//! written to a trigger register, indicating the end of a control block chain"*.
//! One refresh is therefore one interrupt, and by the time it runs the control
//! channel has stopped at the terminator.
//!
//! # The table is a frame, materialised
//!
//! One refresh is the sum of `reps` runs, so the table holds that many
//! descriptors plus the terminator, and
//! [`bcm_rep_count`](hub75_framebuffer::bcm_rep_count) sizes it at compile time
//! without a framebuffer instance. The count is the layout's cost and not the
//! panel's: 32 runs for a six-plane plane-major frame at any geometry, and
//! `32 × NROWS` for the row-major layout. Nor is it the *buffer's*: an entry
//! names its own address, so the count is the same whichever framebuffer is on
//! the panel.
//!
//! # The swap is a delta, applied in place
//!
//! A descriptor holds an absolute address, because the DMA writes each word
//! straight into the data channel's `READ_ADDR` and nothing adds a base. Both
//! framebuffers of a double-buffered driver have the same layout, so a swap is
//! still one number: the interrupt adds it to every run's address through
//! [`patch`] before it restarts the chain. The terminator is never patched.

use hub75_framebuffer::BcmLenReps;
use hub75_framebuffer::FrameBuffer;

/// Bytes in a framebuffer word.
///
/// The driver is constrained to `FrameBuffer<Word = u16>`, so the word width
/// appears here and in the data channel's `DATA_SIZE` and nowhere else; an 8-bit
/// latched layout would change this, the PIO program, and that field.
pub const WORD_BYTES: usize = 2;

/// Largest `TRANS_COUNT` RP2350 can hold: 28 bits, the top four being `MODE`.
pub const MAX_TRANSFER_WORDS: u32 = (1 << 28) - 1;

/// Run descriptors one frame's table holds.
///
/// Sized for the largest sequence this crate's layouts produce for the boards
/// `plan.md` names: `2^(PLANES-1) × NROWS` is `32 × 64` = 2048 for 64 scan rows at
/// [`crate::BIT_DEPTH`], which is 16 KiB of table. A larger layout fails the
/// assertion in the driver's constructor rather than overflowing, and this constant
/// is raised deliberately for such a panel.
pub const MAX_DESCRIPTORS: usize = 2048;

/// Descriptor words per run: `AL3_TRANS_COUNT`, then `AL3_READ_ADDR_TRIG`.
///
/// The control channel transfers this many words per trigger, and wraps its
/// write pointer on a boundary this wide — the driver's control-channel
/// configuration names the same number as the log of the ring size.
pub const DESCRIPTOR_WORDS: u32 = 2;

/// One DMA run: how many words to stream, and where from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct Descriptor {
    /// The run's length in words, written to `AL3_TRANS_COUNT`.
    pub trans_count: u32,
    /// The run's first word, written to `AL3_READ_ADDR_TRIG`.
    ///
    /// A nonzero value also starts the run, which is why this field is written
    /// after `trans_count`.
    pub read_addr: u32,
}

impl Descriptor {
    /// The end of the chain: neither word triggers, and the zero releases the
    /// data channel's deferred interrupt.
    pub const TERMINATOR: Self = Self {
        trans_count: 0,
        read_addr: 0,
    };
}

/// Writes one frame of descriptors and returns the number of runs.
///
/// The frame is the framebuffer's own BCM sequence, unrolled: every segment
/// contributes one descriptor per repetition, in scan order. The terminator is
/// written at `entries[runs]`, so the slice holds `runs + 1` entries.
///
/// # Panics
///
/// Panics if the framebuffer's BCM sequence is empty, if a segment is empty (a
/// zero-word run cannot be told from the terminator, and would end the frame
/// early), if a segment is longer than one [`MAX_TRANSFER_WORDS`] — this engine
/// does not split a segment across descriptors — or if `entries` cannot hold the
/// frame.
#[must_use]
pub fn fill<FB: FrameBuffer<Word = u16>>(fb: &FB, entries: &mut [Descriptor]) -> usize {
    let segments = fb.bcm_segment_count();
    assert!(segments > 0, "the framebuffer's BCM sequence is empty");
    let mut runs = 0;
    for index in 0..segments {
        let BcmLenReps { len, reps } = FB::BCM_SEQUENCE[index % FB::BCM_SEQUENCE_LEN];
        let trans_count = (len / WORD_BYTES) as u32;
        assert!(
            trans_count > 0,
            "a {len}-byte segment is zero words, which would end the frame early"
        );
        assert!(
            trans_count <= MAX_TRANSFER_WORDS,
            "a {len}-byte segment is longer than one transfer count; \
             this engine does not split segments"
        );
        let descriptor = Descriptor {
            trans_count,
            read_addr: fb.bcm_segment_ptr(index) as u32,
        };
        for _ in 0..reps {
            assert!(
                runs < entries.len(),
                "the sequence needs more than {} run descriptors",
                entries.len() - 1
            );
            entries[runs] = descriptor;
            runs += 1;
        }
    }
    assert!(
        runs < entries.len(),
        "the terminator does not fit the table"
    );
    entries[runs] = Descriptor::TERMINATOR;
    runs
}

/// Adds a swap's delta to every run's read address.
///
/// `count` is the number of runs [`fill`] returned, so the terminator at
/// `entries[count]` is left alone. Two patches that cancel restore the table.
pub fn patch(entries: &mut [Descriptor], count: usize, delta: u32) {
    for entry in &mut entries[..count] {
        entry.read_addr = entry.read_addr.wrapping_add(delta);
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use hub75_framebuffer::bitplane::plain::frame;
    use hub75_framebuffer::bitplane::plain::row;

    use super::*;

    const NROWS: usize = crate::ROW_PAIRS;
    const COLS: usize = crate::PANEL_WIDTH;
    const PLANES: usize = crate::BIT_DEPTH as usize;

    /// A plane-major framebuffer: one sequence of `PLANES` suffixes.
    type FrameMajor = frame::DmaFrameBuffer<NROWS, COLS, PLANES>;
    /// A row-major framebuffer: the same trait, one sequence per scan row.
    type RowMajor = row::DmaFrameBuffer<NROWS, COLS, PLANES>;

    /// A filled table at the constant's bound, and the runs it holds.
    fn runs_of<FB: FrameBuffer<Word = u16>>(fb: &FB) -> ([Descriptor; MAX_DESCRIPTORS + 1], usize) {
        let mut entries = [Descriptor::TERMINATOR; MAX_DESCRIPTORS + 1];
        let runs = fill(fb, &mut entries);
        (entries, runs)
    }

    /// Every run reads inside the framebuffer, which is what makes a swap a
    /// delta and what keeps the chain out of unrelated memory, and the table
    /// ends with the terminator that stops it.
    #[test]
    fn every_run_lies_inside_the_framebuffer() {
        fn check<FB: FrameBuffer<Word = u16>>(fb: &FB) {
            let (entries, runs) = runs_of(fb);
            let base = fb as *const FB as u32;
            let end = base + size_of::<FB>() as u32;
            for (index, descriptor) in entries[..runs].iter().enumerate() {
                assert!(
                    descriptor.read_addr >= base,
                    "run {index} starts before the framebuffer"
                );
                assert!(
                    descriptor.read_addr + descriptor.trans_count * WORD_BYTES as u32 <= end,
                    "run {index} runs past the end of the framebuffer"
                );
            }
            assert_eq!(
                entries[runs],
                Descriptor::TERMINATOR,
                "fill ends the chain after the runs it wrote"
            );
        }
        check(&FrameMajor::new());
        check(&RowMajor::new());
    }

    /// A swap is one delta, and two deltas that cancel restore the table.
    #[test]
    fn a_patch_moves_only_the_run_addresses() {
        let fb = FrameMajor::new();
        let (mut entries, runs) = runs_of(&fb);
        let before: std::vec::Vec<Descriptor> = entries[..=runs].to_vec();

        patch(&mut entries, runs, 0x1000);
        for (index, descriptor) in entries[..runs].iter().enumerate() {
            assert_eq!(
                descriptor.read_addr,
                before[index].read_addr + 0x1000,
                "run {index} did not move by the delta"
            );
            assert_eq!(descriptor.trans_count, before[index].trans_count);
        }
        assert_eq!(
            entries[runs],
            Descriptor::TERMINATOR,
            "the terminator is not patched"
        );

        patch(&mut entries, runs, 0u32.wrapping_sub(0x1000));
        assert_eq!(&entries[..=runs], before.as_slice());
    }

    /// A table that cannot hold the frame is a startup failure, not a silent
    /// overflow.
    #[test]
    #[should_panic(expected = "run descriptors")]
    fn a_short_table_is_rejected() {
        let fb = RowMajor::new();
        let mut entries = [Descriptor::TERMINATOR; 4];
        let _ = fill(&fb, &mut entries);
    }
}
