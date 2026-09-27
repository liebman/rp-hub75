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
//! descriptors plus the terminator, and [`descriptor_count`] sizes it at compile
//! time without a framebuffer instance. The storage itself is the caller's, made
//! by [`hub75_descriptors!`](crate::hub75_descriptors): the driver is handed a
//! pointer and keeps nothing larger than that. The count is the layout's cost and
//! not the panel's: 32 runs for a six-plane plane-major frame at any geometry, and
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

use core::marker::PhantomData;
use hub75_framebuffer::BcmLenReps;
use hub75_framebuffer::FrameBuffer;
use hub75_framebuffer::bcm_rep_count;

/// Bytes in a framebuffer word.
///
/// The driver is constrained to `FrameBuffer<Word = u16>`, so the word width
/// appears here and in the data channel's `DATA_SIZE` and nowhere else; an 8-bit
/// latched layout would change this, the PIO program, and that field.
pub const WORD_BYTES: usize = 2;

/// Largest `TRANS_COUNT` RP2350 can hold: 28 bits, the top four being `MODE`.
pub const MAX_TRANSFER_WORDS: u32 = (1 << 28) - 1;

/// Descriptors a frame of framebuffer type `FB` needs: its runs, and the
/// terminator that ends the chain.
///
/// The count is the *layout's*, and it is the crate's own sequence summed:
/// [`bcm_rep_count`] is `2^(PLANES-1)` runs per sequence times `BCM_SEQUENCE_COUNT`
/// scan rows. A six-plane plane-major frame is therefore 33 descriptors (264
/// bytes) at any panel geometry, and a six-plane row-major one is `32 × NROWS + 1`.
/// This is what [`Hub75Descriptors::COUNT`] sizes storage to, so a driver carries
/// the table its own framebuffer needs and no more.
#[must_use]
pub const fn descriptor_count<FB: FrameBuffer>() -> usize {
    bcm_rep_count::<FB>() + 1
}

/// Static descriptor storage for a framebuffer type.
///
/// The array is the caller's `static`, `N` long and typed by the framebuffer it
/// will describe, so storage built for one framebuffer type cannot be handed to a
/// driver configured for another. The driver keeps only a pointer to it, which is
/// why its own static is a few words rather than a table sized for the largest
/// layout.
///
/// Construct only through [`hub75_descriptors!`](crate::hub75_descriptors), which
/// allocates the storage in a `static_cell::StaticCell` and hands back
/// `&'static mut Hub75Descriptors<FB, N>` — the form [`Hub75::new`] takes.
///
/// [`Hub75::new`]: crate::Hub75::new
pub struct Hub75Descriptors<FB, const N: usize> {
    /// The frame's runs, then its terminator.
    entries: [Descriptor; N],
    /// The storage is typed by the framebuffer it describes.
    _fb: PhantomData<fn() -> FB>,
}

impl<FB: FrameBuffer, const N: usize> Hub75Descriptors<FB, N> {
    /// Descriptors a frame of this framebuffer type needs.
    pub const COUNT: usize = descriptor_count::<FB>();

    /// Creates the storage: every entry a terminator, so nothing runs yet.
    ///
    /// # Panics
    ///
    /// Panics if `N` is not [`COUNT`](Self::COUNT). Constructed in a `static`,
    /// where this is meant to be built, that is a compile-time error.
    #[doc(hidden)]
    #[must_use]
    pub const fn new() -> Self {
        assert!(
            N == Self::COUNT,
            "descriptor storage is not the length this framebuffer type needs"
        );
        Self {
            entries: [Descriptor::TERMINATOR; N],
            _fb: PhantomData,
        }
    }

    /// The storage the control channel plays.
    pub(crate) fn entries(&mut self) -> &mut [Descriptor] {
        &mut self.entries
    }
}

/// Allocates static descriptor storage sized for a framebuffer type.
///
/// The length is [`descriptor_count`] at compile time and the type is the
/// framebuffer's, so the storage cannot be under-sized for the frame that will
/// fill it, nor mixed up with another framebuffer's.
///
/// # Examples
///
/// ```rust,ignore
/// type Fb = DmaFrameBuffer<32, 64, 6>;
/// let descriptors = rp_hub75::hub75_descriptors!(Fb);
/// ```
#[macro_export]
macro_rules! hub75_descriptors {
    ($fb_type:ty) => {{
        const __COUNT: usize = $crate::descriptor::descriptor_count::<$fb_type>();
        static __STORAGE: $crate::static_cell::StaticCell<
            $crate::descriptor::Hub75Descriptors<$fb_type, __COUNT>,
        > = $crate::static_cell::StaticCell::new();
        __STORAGE
            .uninit()
            .write($crate::descriptor::Hub75Descriptors::new())
    }};
}

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
    use hub75_framebuffer::compute_rows;

    use super::*;

    /// The panel the tests build their framebuffers for: the same 64×64 at six
    /// planes the examples drive. The tests hold the table to the *type*, so
    /// these only have to be a layout the crate can build.
    const PANEL_WIDTH: usize = 64;
    const NROWS: usize = compute_rows(64);
    const PLANES: usize = 6;

    /// A plane-major framebuffer: one sequence of `PLANES` suffixes.
    type FrameMajor = frame::DmaFrameBuffer<NROWS, PANEL_WIDTH, PLANES>;
    /// A row-major framebuffer: the same trait, one sequence per scan row.
    type RowMajor = row::DmaFrameBuffer<NROWS, PANEL_WIDTH, PLANES>;

    /// The storage each layout asks for: its own frame's length, not a bound.
    type FrameStorage = Hub75Descriptors<FrameMajor, { descriptor_count::<FrameMajor>() }>;
    type RowStorage = Hub75Descriptors<RowMajor, { descriptor_count::<RowMajor>() }>;

    /// One frame's table, filled into storage sized for it: the runs it wrote,
    /// then the terminator.
    fn table_of<FB: FrameBuffer<Word = u16>, const N: usize>(
        fb: &FB,
        storage: &mut Hub75Descriptors<FB, N>,
    ) -> std::vec::Vec<Descriptor> {
        let entries = storage.entries();
        let runs = fill(fb, entries);
        entries[..=runs].to_vec()
    }

    /// The storage a framebuffer type gets is exactly its own frame's length, and
    /// the count is the layout's: one sequence at any geometry plane-major, one
    /// per scan row row-major.
    #[test]
    fn storage_is_sized_for_the_framebuffer() {
        assert_eq!(FrameStorage::COUNT, 32 + 1, "32 runs, plane-major");
        assert_eq!(RowStorage::COUNT, 32 * NROWS + 1, "32 runs per scan row");
        // `new` asserts the two agree, so this is the same check at run time.
        let _ = FrameStorage::new();
        let _ = RowStorage::new();
    }

    /// Storage too short for the frame is a construction failure, not a silent
    /// overflow.
    #[test]
    #[should_panic(expected = "not the length")]
    fn undersized_storage_is_rejected() {
        let _ = Hub75Descriptors::<FrameMajor, 4>::new();
    }

    /// Every run reads inside the framebuffer, which is what makes a swap a
    /// delta and what keeps the chain out of unrelated memory, and the table
    /// ends with the terminator that stops it.
    #[test]
    fn every_run_lies_inside_the_framebuffer() {
        fn check<FB: FrameBuffer<Word = u16>, const N: usize>(
            fb: &FB,
            storage: &mut Hub75Descriptors<FB, N>,
        ) {
            let table = table_of(fb, storage);
            let runs = table.len() - 1;
            let base = fb as *const FB as u32;
            let end = base + size_of::<FB>() as u32;
            for (index, descriptor) in table[..runs].iter().enumerate() {
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
                table[runs],
                Descriptor::TERMINATOR,
                "fill ends the chain after the runs it wrote"
            );
            assert_eq!(
                runs,
                descriptor_count::<FB>() - 1,
                "the table is one run per repetition, and the storage is its length"
            );
        }
        check(&FrameMajor::new(), &mut FrameStorage::new());
        check(&RowMajor::new(), &mut RowStorage::new());
    }

    /// A swap is one delta, and two deltas that cancel restore the table.
    #[test]
    fn a_patch_moves_only_the_run_addresses() {
        let fb = FrameMajor::new();
        let mut table = table_of(&fb, &mut FrameStorage::new());
        let runs = table.len() - 1;
        let before = table.clone();

        patch(&mut table, runs, 0x1000);
        for (index, descriptor) in table[..runs].iter().enumerate() {
            assert_eq!(
                descriptor.read_addr,
                before[index].read_addr + 0x1000,
                "run {index} did not move by the delta"
            );
            assert_eq!(descriptor.trans_count, before[index].trans_count);
        }
        assert_eq!(
            table[runs],
            Descriptor::TERMINATOR,
            "the terminator is not patched"
        );

        patch(&mut table, runs, 0u32.wrapping_sub(0x1000));
        assert_eq!(table, before);
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
