//! The scan engine: which segment to stream, how many times, and what to program
//! the DMA channel with.
//!
//! Nothing here knows what a plane or a row is, or how long one is. Everything
//! comes from [`FrameBuffer::BCM_SEQUENCE`] and
//! [`FrameBuffer::bcm_segment_ptr`], so a *plane-major*
//! (`bitplane::plain::frame`), a *row-major* (`bitplane::plain::row`), a
//! threshold or a remapped framebuffer all drive the same code:
//!
//! ```text
//! for index in 0..FB::bcm_segment_count():
//!     stream FB::bcm_segment(index) — len bytes at ptr — reps times
//! ```
//!
//! The driver never holds `(ptr, len, reps)` as a whole, because the interrupt
//! cannot name the framebuffer's type: it keeps a [`Plan`] (the static
//! `(len, reps)` sequence, copied) and a table of **byte offsets** relative to
//! the framebuffer's base. The offset table is what makes a framebuffer swap a
//! single delta — and it is *checked* at init (every segment must lie inside the
//! framebuffer) rather than assumed.
//!
//! The two things a driver does need to know about a layout are derived here and
//! reported, never hard-coded: [`Plan::runs_per_refresh`] (the interrupt load)
//! and [`shortest_run_bytes`] (how long the interrupt has to re-arm before the
//! PIO starves).

use hub75_framebuffer::BCM_SEQUENCE_CAPACITY;
use hub75_framebuffer::BcmLenReps;
use hub75_framebuffer::FrameBuffer;

/// Bytes in a framebuffer word.
///
/// The driver is constrained to `FrameBuffer<Word = u16>`, so the word width
/// appears here and in the channel's `DATA_SIZE` and nowhere else; an 8-bit
/// latched layout would change this, the PIO program, and that field.
pub const WORD_BYTES: usize = 2;

/// Segments the engine can cache offsets for.
///
/// The crate's worst case is `BCM_SEQUENCE_CAPACITY` (`10`) times
/// `BCM_SEQUENCE_COUNT`, which for this board's 32 row-pairs is 320 — the same
/// bound `esp-hub75` uses. `Hub75::new` asserts it, so a larger layout fails
/// loudly at startup rather than corrupting a picture.
pub const MAX_SEGMENTS: usize = 320;

/// Largest `TRANS_COUNT` RP2350 can hold: 28 bits, the top four being `MODE`.
pub const MAX_TRANSFER_WORDS: u32 = (1 << 28) - 1;

/// A framebuffer's BCM sequence, copied so the interrupt can walk it without
/// naming the framebuffer's type.
#[derive(Clone, Copy)]
pub struct Plan {
    entries: [BcmLenReps; BCM_SEQUENCE_CAPACITY],
    len: usize,
    count: usize,
}

impl Plan {
    /// An empty plan, for the driver's static state.
    pub const EMPTY: Self = Self {
        entries: [BcmLenReps::ZERO; BCM_SEQUENCE_CAPACITY],
        len: 0,
        count: 0,
    };

    /// Copies a framebuffer's sequence.
    pub fn of<FB: FrameBuffer>() -> Self {
        Self {
            entries: FB::BCM_SEQUENCE,
            len: FB::BCM_SEQUENCE_LEN,
            count: FB::BCM_SEGMENT_COUNT,
        }
    }

    /// Builds a plan from an explicit sequence. Used by the tests.
    pub fn new(entries: &[BcmLenReps], count: usize) -> Self {
        let mut plan = Self {
            entries: [BcmLenReps::ZERO; BCM_SEQUENCE_CAPACITY],
            len: entries.len(),
            count,
        };
        plan.entries[..entries.len()].copy_from_slice(entries);
        plan
    }

    /// Segments in one complete refresh.
    pub fn count(&self) -> usize {
        self.count
    }

    /// `(len, reps)` for segment `index`: the sequence repeats if the layout is
    /// row-major (the crate's `index % BCM_SEQUENCE_LEN`).
    pub fn entry(&self, index: usize) -> BcmLenReps {
        self.entries[index % self.len]
    }

    /// Runs the interrupt has to service **per refresh** if nothing is repeated
    /// in hardware — the crate's `bcm_rep_count`.
    ///
    /// Counted over the whole refresh, not one sequence: a row-major layout
    /// repeats its sequence once per row, so its runs are `NROWS` times the
    /// sequence's.
    pub fn runs_per_refresh(&self) -> usize {
        (0..self.count).map(|i| self.entry(i).reps).sum()
    }

    /// Bytes streamed per refresh: `len × reps` over the whole sequence.
    pub fn bytes_per_refresh(&self) -> usize {
        (0..self.count)
            .map(|i| {
                let entry = self.entry(i);
                entry.len * entry.reps
            })
            .sum()
    }

    /// The DMA recipe for one run of segment `index`, read from `address`.
    ///
    /// `address` is the **absolute** address the hardware will read, not an
    /// offset from the framebuffer base. A ring wraps the `1 << RING_SIZE`
    /// boundary *containing a given address*, so the alignment that decides
    /// whether a ring is usable is the address's own.
    pub fn run(&self, index: usize, address: u32) -> Run {
        let entry = self.entry(index);
        let words = (entry.len / WORD_BYTES) as u32;
        // A ring is only worth asking for when the hardware would otherwise have
        // to be re-armed for every repetition, and only usable when the region
        // is a power of two *and* the address is aligned to it — checked here,
        // per run, against what the layout actually produced.
        if entry.reps > 1 {
            let ring = ring_bytes(entry.len, address);
            let repeated = words.saturating_mul(entry.reps as u32);
            if ring != 0 && repeated <= MAX_TRANSFER_WORDS {
                return Run {
                    words: repeated,
                    ring_bytes: ring,
                };
            }
        }
        Run {
            words,
            ring_bytes: 0,
        }
    }
}

/// Bytes in the shortest segment of a layout — the tightest deadline the
/// interrupt has before the PIO runs dry.
pub fn shortest_run_bytes<FB: FrameBuffer>() -> usize {
    (0..FB::BCM_SEQUENCE_LEN)
        .map(|i| FB::BCM_SEQUENCE[i].len)
        .min()
        .unwrap_or(0)
}

/// The ring size the DMA may use to repeat a `len`-byte segment read from
/// `address`, or `0` for none.
///
/// A ring wraps a region that is a power of two **and** the address has to be
/// aligned to it, because the wrap works by masking address bits. `address` must
/// therefore be the address the hardware sees: aligning an *offset* is not the
/// same thing, since a framebuffer base is not necessarily aligned itself.
///
/// Both conditions are tested against the layout the crate actually produced;
/// nothing is assumed about planes, rows or strides. When a ring applies, `reps`
/// costs no interrupts at all. When it does not — the common case for a frame
/// whose segments are suffixes of unequal, mostly non-power-of-two length — the
/// engine re-arms once per repetition. Aligning the framebuffer (a
/// `#[repr(align(N))]` wrapper) is what turns the last, most-repeated segments
/// of such a frame into single runs.
pub fn ring_bytes(len: usize, address: u32) -> u32 {
    if len > 1 && len.is_power_of_two() && len <= 32768 && address % len as u32 == 0 {
        len as u32
    } else {
        0
    }
}

/// One DMA run: how many words to move, and the ring that repeats them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Run {
    /// `TRANS_COUNT` for this run.
    pub words: u32,
    /// `RING_SIZE` region in bytes, or `0` for no ring. When set, this single
    /// run covers every repetition of the segment.
    pub ring_bytes: u32,
}

/// Where the walk is: the segment being streamed, and how many of its runs have
/// been issued.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Cursor {
    index: usize,
    issued: usize,
}

/// What happens after a run completes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// The same segment needs another run.
    Repeat,
    /// Move on to the next segment.
    Advance,
    /// The refresh is complete; the cursor is back at segment 0.
    FrameBoundary,
}

impl Cursor {
    /// A cursor at the start of a refresh.
    pub const fn new() -> Self {
        Self {
            index: 0,
            issued: 0,
        }
    }

    /// The segment being streamed.
    pub const fn index(&self) -> usize {
        self.index
    }

    /// The run for the current segment, read from `address`.
    pub fn run(&self, plan: &Plan, address: u32) -> Run {
        plan.run(self.index, address)
    }

    /// Advances after one run of the current segment has completed.
    ///
    /// `ringed` says whether that run was repeated by the DMA's ring, i.e.
    /// whether this segment's `reps` are already satisfied.
    pub fn advance(&mut self, plan: &Plan, ringed: bool) -> Step {
        let reps = plan.entry(self.index).reps;
        if !ringed && self.issued + 1 < reps {
            self.issued += 1;
            return Step::Repeat;
        }
        self.issued = 0;
        self.index += 1;
        if self.index == plan.count() {
            self.index = 0;
            Step::FrameBoundary
        } else {
            Step::Advance
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use hub75_framebuffer::bcm_rep_count;
    use hub75_framebuffer::bitplane::plain::frame;
    use hub75_framebuffer::bitplane::plain::row;

    use super::*;

    const NROWS: usize = 32;
    const COLS: usize = 64;
    const PLANES: usize = crate::BIT_DEPTH as usize;

    /// A plane-major framebuffer, the layout the crate recommends for DMA.
    type FrameMajor = frame::DmaFrameBuffer<NROWS, COLS, PLANES>;
    /// A row-major framebuffer: the same trait, a completely different shape.
    type RowMajor = row::DmaFrameBuffer<NROWS, COLS, PLANES>;

    /// Byte offsets of every segment of a framebuffer, from its own pointers.
    fn offsets<FB: FrameBuffer>(fb: &FB) -> ([i32; MAX_SEGMENTS], usize) {
        let base = fb as *const FB as u32;
        let mut offsets = [0i32; MAX_SEGMENTS];
        let count = fb.bcm_segment_count();
        assert!(
            count <= MAX_SEGMENTS,
            "{count} segments exceeds MAX_SEGMENTS ({MAX_SEGMENTS})"
        );
        for (index, offset) in offsets.iter_mut().enumerate().take(count) {
            *offset = fb.bcm_segment_ptr(index) as u32 as i32 - base as i32;
        }
        (offsets, count)
    }

    /// Times a byte is streamed per refresh, from the sequence alone: the sum of
    /// `reps` over the segments whose byte range covers it.
    fn exposure(plan: &Plan, offsets: &[i32], byte: u32) -> usize {
        (0..plan.count())
            .filter(|index| {
                let entry = plan.entry(*index);
                let start = offsets[*index] as u32;
                byte >= start && byte < start + entry.len as u32
            })
            .map(|index| plan.entry(index).reps)
            .sum()
    }

    /// The heart of it: whatever the layout, plane `k`'s data is streamed `2^k`
    /// times per refresh, so BCM's weights come out right. Checked against the
    /// crate's own sequence and pointers rather than against any notion of
    /// planes the driver might have.
    fn weights_by_exposure<FB: FrameBuffer>(fb: &FB, plane_bytes: usize) {
        let plan = Plan::of::<FB>();
        let (offsets, _) = offsets(fb);

        // One panel scan is one plane's worth of bytes, and a refresh is
        // 1 + 2 + … + 2^(PLANES-1) of them.
        assert_eq!(
            plan.bytes_per_refresh() / plane_bytes,
            (1 << PLANES) - 1,
            "a refresh should be {} panel scans",
            (1 << PLANES) - 1
        );

        for plane in 0..PLANES {
            let byte = plane * plane_bytes;
            assert_eq!(
                exposure(&plan, &offsets, byte as u32),
                1 << plane,
                "plane {plane} is streamed the wrong number of times"
            );
        }
    }

    #[test]
    fn the_plane_major_sequence_weights_every_plane() {
        let fb = FrameMajor::new();
        weights_by_exposure(&fb, size_of::<FrameMajor>() / PLANES);
    }

    #[test]
    fn the_row_major_sequence_weights_every_plane_of_every_row() {
        let fb = RowMajor::new();
        // Rows are packed, so each row starts at a multiple of one row's planes.
        let row_bytes = size_of::<RowMajor>() / NROWS;
        let plan = Plan::of::<RowMajor>();
        let (offsets, _) = offsets(&fb);
        for row in 0..NROWS {
            for plane in 0..PLANES {
                let byte = (row * row_bytes + plane * (row_bytes / PLANES)) as u32;
                assert_eq!(
                    exposure(&plan, &offsets, byte),
                    1 << plane,
                    "row {row} plane {plane} is streamed the wrong number of times"
                );
            }
        }
    }

    /// Both layouts describe the same panel: the same total data, in a
    /// different order. This is the test that says the driver cannot favour
    /// either one.
    #[test]
    fn the_layouts_agree_on_total_light() {
        let frame_major = Plan::of::<FrameMajor>();
        let row_major = Plan::of::<RowMajor>();
        assert_eq!(
            frame_major.bytes_per_refresh(),
            row_major.bytes_per_refresh()
        );
        // 63 panel scans: 1 + 2 + 4 + 8 + 16 + 32, where a scan is one
        // plane's worth of bytes in either layout.
        let plane_bytes = size_of::<FrameMajor>() / PLANES;
        assert_eq!(plane_bytes, size_of::<RowMajor>() / PLANES);
        assert_eq!(
            frame_major.bytes_per_refresh() / plane_bytes,
            (1 << PLANES) - 1
        );
        assert_eq!(
            row_major.count(),
            frame_major.count() * NROWS,
            "row-major has one sequence per row"
        );
    }

    /// Every segment of both layouts lies inside the framebuffer, which is what
    /// makes a swap a single delta. A layout with a segment outside it (a shared
    /// blank, say) would fail here rather than corrupt a picture at runtime.
    #[test]
    fn every_segment_lies_inside_the_framebuffer() {
        fn check<FB: FrameBuffer>(fb: &FB) {
            let (offsets, count) = offsets(fb);
            let plan = Plan::of::<FB>();
            for index in 0..count {
                let offset = offsets[index];
                assert!(offset >= 0, "segment {index} starts before the framebuffer");
                assert!(
                    offset as usize + plan.entry(index).len <= size_of::<FB>(),
                    "segment {index} runs past the end of the framebuffer"
                );
            }
        }
        check(&FrameMajor::new());
        check(&RowMajor::new());
    }

    /// The engine's walk must replay exactly what the crate describes: same
    /// bytes, same runs, one frame boundary.
    #[test]
    fn walking_the_plan_replays_one_refresh() {
        fn check<FB: FrameBuffer>(fb: &FB) {
            let plan = Plan::of::<FB>();
            let (offsets, _) = offsets(fb);
            let base = fb as *const FB as u32;
            let mut cursor = Cursor::new();
            let mut runs = 0usize;
            let mut bytes = 0usize;
            let mut boundaries = 0usize;
            let limit = plan.count() * (plan.runs_per_refresh() + 1) + 1;
            loop {
                let address = base.wrapping_add(offsets[cursor.index()] as u32);
                let run = cursor.run(&plan, address);
                assert!(run.words > 0, "a zero-length run would stall the engine");
                assert!(run.words <= MAX_TRANSFER_WORDS, "run does not fit");
                runs += 1;
                bytes += run.words as usize * WORD_BYTES;
                let ringed = run.ring_bytes != 0;
                match cursor.advance(&plan, ringed) {
                    Step::FrameBoundary => {
                        boundaries += 1;
                        break;
                    }
                    Step::Repeat | Step::Advance => {}
                }
                assert!(runs <= limit, "the cursor never reached the boundary");
            }
            assert_eq!(boundaries, 1, "one refresh is one boundary");
            assert_eq!(
                bytes,
                plan.bytes_per_refresh(),
                "the engine streamed a different amount of data than the plan"
            );
            // And the refresh really is the number of panel scans BCM needs,
            // where one scan is one plane's worth of bytes.
            assert_eq!(bytes / (size_of::<FB>() / PLANES), (1 << PLANES) - 1);
        }
        check(&FrameMajor::new());
        check(&RowMajor::new());
    }

    /// The cursor's arithmetic, on a sequence small enough to count by hand.
    #[test]
    fn the_cursor_walks_reps_before_advancing() {
        let plan = Plan::new(
            &[
                BcmLenReps { len: 8, reps: 1 },
                BcmLenReps { len: 8, reps: 3 },
                BcmLenReps { len: 8, reps: 2 },
            ],
            3,
        );
        assert_eq!(plan.runs_per_refresh(), 6);

        let mut cursor = Cursor::new();
        let mut steps = std::vec::Vec::new();
        for _ in 0..6 {
            steps.push(cursor.advance(&plan, false));
        }
        assert_eq!(
            steps,
            [
                Step::Advance,
                Step::Repeat,
                Step::Repeat,
                Step::Advance,
                Step::Repeat,
                Step::FrameBoundary,
            ]
        );

        // A ringed run satisfies every repetition of its segment at once.
        let mut cursor = Cursor::new();
        assert_eq!(cursor.advance(&plan, false), Step::Advance);
        assert_eq!(cursor.advance(&plan, true), Step::Advance);
    }

    /// A ring is a promise about the *address*: power of two, and aligned to it.
    /// Everything else falls back to one run per repetition.
    #[test]
    fn the_ring_is_only_offered_when_the_address_allows_it() {
        // The row-major case: a 128-byte plane row at a 128-byte boundary.
        assert_eq!(ring_bytes(128, 0x2000_0000), 128);
        assert_eq!(ring_bytes(128, 0x2000_1000), 128);
        // Misaligned, not a power of two, empty, or wider than a ring can wrap.
        assert_eq!(ring_bytes(128, 0x2000_0040), 0);
        assert_eq!(ring_bytes(384, 0x2000_0000), 0);
        assert_eq!(ring_bytes(0, 0x2000_0000), 0);
        assert_eq!(ring_bytes(65536, 0x2000_0000), 0);
    }

    /// The bug the first hardware run of `telemetry` exposed: an *offset* that
    /// is aligned says nothing about the *address*, because a framebuffer base
    /// is aligned to its own type, not to the segment. Testing the offset let
    /// the DMA wrap a region that started before the segment, and the MSB
    /// plane's passes read a shifted window.
    #[test]
    fn an_aligned_offset_is_not_an_aligned_address() {
        // Frame-major: the last two segments have power-of-two lengths at
        // 8192- and 4096-byte offsets — the offsets are aligned...
        assert_eq!(16384 % 8192, 0);
        assert_eq!(20480 % 4096, 0);
        // ...and on an unaligned framebuffer neither address is, so no ring.
        let base = 0x2000_0004u32;
        assert_eq!(ring_bytes(8192, base + 16384), 0);
        assert_eq!(ring_bytes(4096, base + 20480), 0);
        // Align the framebuffer and the same segments ring in one run each.
        let aligned = 0x2000_0000u32;
        assert_eq!(ring_bytes(8192, aligned + 16384), 8192);
        assert_eq!(ring_bytes(4096, aligned + 20480), 4096);
    }

    /// A ringed run covers every repetition in one go; a segment with a single
    /// repetition never bothers with a ring at all.
    #[test]
    fn a_ring_covers_the_whole_segment_in_one_run() {
        let aligned = Plan::new(
            &[BcmLenReps {
                len: 4096,
                reps: 16,
            }],
            1,
        );
        let run = aligned.run(0, 0);
        assert_eq!(run.ring_bytes, 4096);
        assert_eq!(run.words, (4096 / WORD_BYTES) as u32 * 16);
        let mut cursor = Cursor::new();
        assert_eq!(
            cursor.advance(&aligned, run.ring_bytes != 0),
            Step::FrameBoundary
        );

        // The same segment one byte out of alignment: sixteen separate runs.
        let misaligned = aligned.run(0, 0x2000_0002);
        assert_eq!(misaligned.ring_bytes, 0);
        assert_eq!(misaligned.words, (4096 / WORD_BYTES) as u32);

        let single = Plan::new(&[BcmLenReps { len: 4096, reps: 1 }], 1);
        assert_eq!(
            single.run(0, 0x2000_0000).ring_bytes,
            0,
            "a ring would save nothing"
        );
    }

    /// The interrupt has to re-arm before the PIO starves. Print the numbers for
    /// both layouts and insist on an order of magnitude of margin.
    #[test]
    fn the_shortest_run_leaves_time_to_re_arm() {
        const PANEL_CLOCK_HZ: f64 = 18_750_000.0;
        for (name, bytes, segments) in [
            (
                "frame-major",
                shortest_run_bytes::<FrameMajor>(),
                Plan::of::<FrameMajor>().count(),
            ),
            (
                "row-major",
                shortest_run_bytes::<RowMajor>(),
                Plan::of::<RowMajor>().count(),
            ),
        ] {
            let words = bytes / WORD_BYTES;
            let micros = words as f64 / PANEL_CLOCK_HZ * 1e6;
            std::println!(
                "{name}: {segments} segments, shortest run {bytes} B = {words} words \
                 = {micros:.2} us"
            );
            assert!(
                micros >= 2.0,
                "{name}: a {micros:.2} us run is too short to re-arm"
            );
        }
    }

    /// The crate's own `bcm_rep_count` is what the engine promises, for both
    /// layouts — the number a layout's interrupt load is judged on.
    #[test]
    fn runs_per_refresh_matches_the_crate() {
        assert_eq!(
            Plan::of::<FrameMajor>().runs_per_refresh(),
            bcm_rep_count::<FrameMajor>()
        );
        assert_eq!(
            Plan::of::<RowMajor>().runs_per_refresh(),
            bcm_rep_count::<RowMajor>()
        );

        // The documented shapes, spelled out: 32 runs for a six-plane frame, and
        // 32 per row times 32 rows for the row-major layout.
        assert_eq!(Plan::of::<FrameMajor>().runs_per_refresh(), 32);
        assert_eq!(Plan::of::<RowMajor>().runs_per_refresh(), 32 * NROWS);
        assert!(Plan::of::<RowMajor>().count() <= MAX_SEGMENTS);
    }
}
