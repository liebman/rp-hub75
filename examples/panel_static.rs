//! M1, M2a and M3: bring-up, then a whole framebuffer over DMA.
//!
//! Three phases, each one adding exactly one thing so a failure points
//! somewhere useful:
//!
//! 1. **M1** — the data program clocks zeroes with OE high (panel blanked) for a
//!    second. The panel must stay completely dark; a scope on GP11 sees the
//!    pixel clock. This is the "is anything wired at all" phase.
//! 2. **M2a** — row pair 0 only, every column white, and the CPU moves the
//!    address lines and pulses the latch. Expect panel rows 0 and 32 lit white
//!    at *full* brightness rather than the 1/32 of a scanned row, since nothing
//!    else is being scanned. That proves data order, clock, latch and address.
//! 3. **M3** — the real thing: **one** state machine and **one** DMA channel
//!    stream a `hub75-framebuffer` framebuffer straight at the panel for ever.
//!    One `out pins, 14` puts a whole word on GP0-13 — colours, row address,
//!    latch and output enable — because the crate's `interstate75` layout makes
//!    bit *n* pin *n*, so the only thing the PIO still generates is the clock.
//!    From there the CPU's only job is to change what is in the framebuffer,
//!    which is what the last five patterns do: one at a time, two seconds each,
//!    drawn into the buffer the DMA is already streaming. They are a ladder, and
//!    each rung isolates one thing — whether the address lines advance at all,
//!    which half is which, where the columns land, and whether the panel's strobe
//!    wants the latch a word later than the framebuffer puts it. Each pattern's
//!    doc comment says what a wrong picture means; `NOTES.md` has the hardware
//!    results and what each rung looked like.
//!
//! Note what looping the stream costs: a self-triggered DMA channel reloads its
//! transfer count but *not* its read address, so the pass only repeats inside a
//! ring the size of the segment — which is why the framebuffer is 4096-byte
//! aligned and why `stream_forever` asserts both. See `stream_forever`.
//!
//! See `rp_hub75::programs` for the program, the pin layout and the one
//! invariant it hangs on (one FIFO word per column, which is why the pull
//! threshold is 14 and not 16), and `NOTES.md` for the hardware bring-up this is
//! built on.
//!
//! The latch is the one thing the bus timing cannot settle for us. It rides in
//! the last column's word, so `STB` rises at the start of that word — one cycle
//! before its clock edge — and falls at the start of the next row's first word,
//! one cycle before that one. That is right if `STB` is the shift registers'
//! latch clock and the transfer completes when the strobe is released; a panel
//! that latched on the *rising* edge would show every row one column to the
//! right. Rung D below is the rung that says which one this panel is.
//!
//! ```text
//! # board in BOOTSEL mode:
//! cargo run-arm --example panel_static
//! ```
#![no_std]
#![no_main]

use core::mem::size_of;

use cortex_m::asm::delay;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_hal::digital::OutputPin;
use panic_halt as _;
use rp235x_hal as hal;

use hal::Sio;
use hal::dma::{DMAExt, SingleChannel};
use hal::gpio::{FunctionPio0, Pin};
use hal::pio::{Buffers, PIOBuilder, PIOExt, PinDir, ShiftDirection};

use hub75_framebuffer::Color;
use hub75_framebuffer::FrameBuffer;
use hub75_framebuffer::bitplane::plain::DmaFrameBuffer;
use rp_hub75::programs::{PULL_THRESHOLD, hub75_stream};
use rp_hub75::{PANEL_HEIGHT, PANEL_WIDTH, pins};

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Row pairs a 1/32-scan 64x64 panel lights at once: the rows the framebuffer
/// holds, and half the panel height.
const NROWS: usize = PANEL_HEIGHT / 2;
/// The same number in the width the address lines want.
const ROW_PAIRS: u8 = NROWS as u8;
/// One FIFO word per panel column.
const WORDS_PER_ROW: usize = PANEL_WIDTH;
/// Half the panel's width (and height): the column the bottom half's arm of the
/// diagonal continues from, and the row where the two halves meet.
const HALF_WIDTH: u8 = (PANEL_WIDTH / 2) as u8;
/// PIO clock divider: 150 MHz / 3 = 50 MHz, so a word (two PIO cycles) is 40 ns
/// and the panel clock is 25 MHz with a 20 ns clock pulse and a whole 20 ns cycle
/// of setup before the edge that samples it.
///
/// This is the knob to turn if the panel is unhappy on this ribbon: 2 is
/// 37.5 MHz (fast), 4 is 18.75 MHz, which is the 15 MHz-odd comfort zone the
/// earlier milestones ran in and slower than the panel needs.
const PIO_CLOCK_DIVIDER: u16 = 3;

/// Which clock edge the panel takes a column on: `false` for the usual panel,
/// which shifts on the rising edge, `true` for one that shifts on the falling
/// edge. See `hub75_stream`.
const INVERT_CLOCK: bool = false;

/// The six colour bits of one column. The `interstate75` layout of
/// `hub75-framebuffer` puts them on bits 0..=5, which are also GP0-5: bit *n* is
/// pin *n* for the whole bus, so these are the pin numbers.
const fn pixel(r0: bool, g0: bool, b0: bool, r1: bool, g1: bool, b1: bool) -> u16 {
    ((r0 as u16) << pins::R0)
        | ((g0 as u16) << pins::G0)
        | ((b0 as u16) << pins::B0)
        | ((r1 as u16) << pins::R1)
        | ((g1 as u16) << pins::G1)
        | ((b1 as u16) << pins::B1)
}

/// One bit-plane of a 64x64 panel in `hub75-framebuffer`'s plane-major bitplane
/// layout. With a single plane the whole refresh is one contiguous segment,
/// which is what lets one free-running DMA channel per state machine stream it
/// with no CPU involvement, no descriptors and no interrupts.
type Fb = DmaFrameBuffer<NROWS, WORDS_PER_ROW, 1>;

/// The framebuffer and the 4096-byte alignment its plane needs.
///
/// The alignment is load-bearing, not decoration: the DMA loops the pass by
/// wrapping the read address inside a ring the size of the plane (see
/// [`stream_forever`]), and a ring works by masking the low bits of the
/// address. A buffer that is not aligned to its own size would wrap in the
/// middle of whatever the linker put either side of it.
#[repr(align(4096))]
struct Aligned(Fb);

/// The framebuffer the panel is refreshed from, for ever. A `static` because
/// the DMA reads it long after `main`'s locals would have gone away, and
/// `DmaFrameBuffer::new` is `const`, so there is nothing to set up at run time.
static mut FRAMEBUFFER: Aligned = Aligned(Fb::new());

/// Write the five row-address lines, lowest first (A=GP6 ... E=GP10). Only the
/// M2a phase needs this: from M3 on, the address comes out of the stream.
macro_rules! set_row {
    ($row:expr, $a:expr, $b:expr, $c:expr, $d:expr, $e:expr) => {{
        let r = $row;
        let _ = if r & 0b00001 != 0 {
            $a.set_high()
        } else {
            $a.set_low()
        };
        let _ = if r & 0b00010 != 0 {
            $b.set_high()
        } else {
            $b.set_low()
        };
        let _ = if r & 0b00100 != 0 {
            $c.set_high()
        } else {
            $c.set_low()
        };
        let _ = if r & 0b01000 != 0 {
            $d.set_high()
        } else {
            $d.set_low()
        };
        let _ = if r & 0b10000 != 0 {
            $e.set_high()
        } else {
            $e.set_low()
        };
    }};
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_hub75::XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // All fourteen HUB75 pins belong to PIO0 from the start; the six data pins
    // and the clock go over now, and the state machine's writes to the pins that
    // are still plain GPIOs are ignored. The address, strobe and OE pins stay
    // GPIOs (SIO) for M1 and M2a, where the CPU drives them by hand, and are
    // handed over just before M3.
    let _d0: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
    let _d1: Pin<_, FunctionPio0, _> = pins.gpio1.into_function();
    let _d2: Pin<_, FunctionPio0, _> = pins.gpio2.into_function();
    let _d3: Pin<_, FunctionPio0, _> = pins.gpio3.into_function();
    let _d4: Pin<_, FunctionPio0, _> = pins.gpio4.into_function();
    let _d5: Pin<_, FunctionPio0, _> = pins.gpio5.into_function();
    let _clk: Pin<_, FunctionPio0, _> = pins.gpio11.into_function();

    let mut row_a = pins.gpio6.into_push_pull_output();
    let mut row_b = pins.gpio7.into_push_pull_output();
    let mut row_c = pins.gpio8.into_push_pull_output();
    let mut row_d = pins.gpio9.into_push_pull_output();
    let mut row_e = pins.gpio10.into_push_pull_output();
    let mut stb = pins.gpio12.into_push_pull_output();
    let mut oe = pins.gpio13.into_push_pull_output();

    // Blank the panel and select row pair 0 before anything starts.
    let _ = oe.set_high();
    let _ = stb.set_low();
    set_row!(0, row_a, row_b, row_c, row_d, row_e);

    // The stream program: one 16-bit word per column, straight out to GP0-13 by
    // one `out pins, 14`, with the clock on GP11 and the words out of a FIFO.
    // `out_shift_direction(Right)` because the words arrive least-significant bit
    // first, and `pull_threshold` is the fourteen bits the `out` shifts out —
    // which is what makes the OSR refill exactly on a column boundary. Sixteen,
    // the obvious value, would leave the word's two spare top bits in the OSR and
    // put every following word two bits out of place.
    let (mut pio, sm0, ..) = pac.PIO0.split(&mut pac.RESETS);

    let stream_program = pio.install(&hub75_stream(INVERT_CLOCK)).unwrap();
    let (mut sm0, _, mut tx_data) = PIOBuilder::from_installed_program(stream_program)
        .out_pins(pins::WORD_BASE, pins::WORD_PINS)
        .side_set_pin_base(pins::CLK)
        .clock_divisor_fixed_point(PIO_CLOCK_DIVIDER, 0)
        .out_shift_direction(ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(PULL_THRESHOLD)
        .buffers(Buffers::OnlyTx)
        .build(sm0);

    sm0.set_pindirs((0..pins::WORD_PINS).map(|pin| (pins::WORD_BASE + pin, PinDir::Output)));
    let mut sm0 = sm0.start();

    // -------------------------------------------------------------------
    // M1: clock a whole panel's worth of zeroes with OE high, so the panel
    // stays dark while CLK (GP11) and the data pins visibly toggle. Every
    // field of the word is zero, including the address, latch and OE bits,
    // and those pins are plain GPIOs here — the CPU holds OE high and the
    // strobe low, and the state machine's writes to them go nowhere.
    // -------------------------------------------------------------------
    let m1_end = timer.get_counter().ticks() + 1_000_000;
    while timer.get_counter().ticks() < m1_end {
        for _ in 0..WORDS_PER_ROW {
            while !tx_data.write(0) {}
        }
        wait_for_columns(&mut tx_data);
    }

    // -------------------------------------------------------------------
    // M2a: row pair 0 only, every column white, latched by the CPU. Rows 0
    // and 32 light up together at full brightness, because nothing else is
    // being scanned and the machine then has nothing left to clock.
    // -------------------------------------------------------------------
    let white = pixel(true, true, true, true, true, true);
    for _ in 0..WORDS_PER_ROW {
        while !tx_data.write(u32::from(white)) {}
    }
    wait_for_columns(&mut tx_data);
    latch_row(
        &mut stb, &mut oe, 0, &mut row_a, &mut row_b, &mut row_c, &mut row_d, &mut row_e,
    );

    let m2a_end = timer.get_counter().ticks() + 3_000_000;
    while timer.get_counter().ticks() < m2a_end {}

    // -------------------------------------------------------------------
    // M3: one state machine, one DMA channel, one framebuffer.
    // -------------------------------------------------------------------
    // The address, strobe and OE pins join PIO0, and the state machine gets a
    // clean slate: the words M1 and M2a pushed through it are dropped, the
    // program restarts at its first instruction, and the FIFO is empty. So
    // nothing reaches the pins until the DMA's first word does — and when it
    // does, the address, latch and OE it carries go with it, because they are
    // bits of the same word.
    let _row_a: Pin<_, FunctionPio0, _> = row_a.into_function();
    let _row_b: Pin<_, FunctionPio0, _> = row_b.into_function();
    let _row_c: Pin<_, FunctionPio0, _> = row_c.into_function();
    let _row_d: Pin<_, FunctionPio0, _> = row_d.into_function();
    let _row_e: Pin<_, FunctionPio0, _> = row_e.into_function();
    let _stb: Pin<_, FunctionPio0, _> = stb.into_function();
    let _oe: Pin<_, FunctionPio0, _> = oe.into_function();

    sm0.clear_fifos();
    sm0.restart();

    // The framebuffer, plus the invariants the free-running DMA loop leans on:
    // one contiguous segment, a power of two in size, and aligned to that size,
    // because the read ring that loops each pass works by masking the read
    // address's low bits.
    //
    // SAFETY: single-threaded, and the static is taken exactly once. The DMA
    // reads the same plane through the segment's raw pointer; the CPU only ever
    // writes colour bits into it, which is the point of a live framebuffer, and
    // a half-written word can only disturb one 82 us refresh.
    let fb: &'static mut Fb = unsafe { &mut (*core::ptr::addr_of_mut!(FRAMEBUFFER)).0 };

    assert_eq!(
        fb.bcm_segment_count(),
        1,
        "a single plane is one contiguous segment per refresh"
    );
    let segment = fb.bcm_segment(0);
    assert_eq!(segment.reps, 1);
    assert_eq!(
        segment.len, 4096,
        "one plane is 32 row pairs x 64 columns x 2 bytes, and that is the read ring"
    );
    // SAFETY: the segment points at the framebuffer's own plane data, which
    // lives in a static for the rest of the program.
    let words: &'static [u16] = unsafe {
        core::slice::from_raw_parts(segment.ptr.cast::<u16>(), segment.len / size_of::<u16>())
    };

    // Arm the channel: the machine is already parked at its first `out` with an
    // empty FIFO, so the first word it pulls is the first word the DMA writes,
    // and from then on the DMA paces itself off the FIFO's DREQ. One channel is
    // all it takes, because one state machine is all it takes.
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch0 = dma.ch0;
    // SAFETY: `words` lives in a static for the rest of the program.
    unsafe {
        stream_forever(&mut ch0, words, &tx_data);
    }

    // From here the panel refreshes itself out of FRAMEBUFFER, and the only
    // thing left for the CPU to do is change what is in it.

    // -------------------------------------------------------------------
    // The diagnostics: a panel is the only console this program has
    // -------------------------------------------------------------------
    // Each pattern is drawn straight into the framebuffer the DMA is already
    // streaming, so the panel picks it up within a refresh (2048 words of 40 ns
    // is 82 us) and the stream is never interrupted. Every row pair is drawn
    // identically, so what these rungs test is the plumbing, one piece at a
    // time; the doc comment on each says what the panel should show and what a
    // wrong picture means.
    //
    // One plane stores the top bit of each colour channel and nothing else
    // (`set_pixel` writes bit 7), so every pattern here uses the eight saturated
    // colours a single plane can express. Dim colours would come out black, and
    // that is the crate's bit depth talking, not the panel.
    let phases: [fn(&mut Fb); 5] = [
        every_row_pair_white,
        top_half_white,
        bottom_half_white,
        left_edge_and_colours,
        the_diagonal,
    ];

    loop {
        for draw in phases {
            fb.erase();
            draw(fb);
            let until = timer.get_counter().ticks() + 2_000_000;
            while timer.get_counter().ticks() < until {}
        }
    }
}

/// Wait until the data machine has clocked out everything written so far.
///
/// The FIFO going empty means the last word has been *handed* to the machine,
/// not that it has been clocked, so the delay covers the word still in flight
/// (two PIO cycles is 40 ns; 200 CPU cycles is ample). The M2 milestones used
/// a PIO IRQ for this handshake; the program no longer raises one, because from
/// M3 on nothing on the CPU needs to know where the scan is.
fn wait_for_columns<SM, SZ>(tx: &mut hal::pio::Tx<SM, SZ>)
where
    SM: hal::pio::ValidStateMachine,
    SZ: hal::dma::TransferSize,
{
    while !tx.is_empty() {}
    delay(200);
}

/// Blank the panel, move the address lines to `row`, pulse the latch and
/// unblank — blank, address, latch, unblank, the order the panel needs: the
/// strobe captures the address lines as well as the shifted colour data, and
/// the LEDs must not show stale data while that happens.
// The five address lines are the panel's interface, not a design smell.
#[allow(clippy::too_many_arguments)]
fn latch_row<A, B, C, D, E>(
    stb: &mut impl OutputPin,
    oe: &mut impl OutputPin,
    row: u8,
    row_a: &mut A,
    row_b: &mut B,
    row_c: &mut C,
    row_d: &mut D,
    row_e: &mut E,
) where
    A: OutputPin,
    B: OutputPin,
    C: OutputPin,
    D: OutputPin,
    E: OutputPin,
{
    let _ = oe.set_high();
    set_row!(row, row_a, row_b, row_c, row_d, row_e);
    delay(60);
    let _ = stb.set_high();
    delay(60);
    let _ = stb.set_low();
    delay(60);
    let _ = oe.set_low();
}

/// Point one DMA channel at `words` and let it run for ever.
///
/// Half-word reads walk the framebuffer, and half-word writes go to the same
/// PIO FIFO address every time, paced by that state machine's DREQ. Pacing
/// matters: a state machine that runs out of data stalls, and a stalled machine
/// means a stalled clock, which is a frozen panel.
///
/// # Looping takes two things, not one
///
/// `TRANS_COUNT.MODE = TRIGGER_SELF` with `CHAIN_TO` pointing at the channel
/// itself makes completion re-trigger the channel, and a trigger *reloads the
/// transfer count*, so each pass is the whole segment over again. What a
/// trigger does **not** do is rewind the read address: the pointer carries on
/// from where the last transfer left it ("writing a nonzero value will reload
/// the channel counter and start the channel" — RP2350 DMA alias registers).
/// Without a ring, the second pass therefore reads the bytes *after* the
/// framebuffer, and the stream is garbage from then on: the state machines
/// starve, freeze, and leave the panel showing whatever row pair the address
/// lines stopped on.
///
/// The fix is the read address's own wrapping window: `RING_SIZE` wraps the
/// read address inside a `1 << n` byte region, so with a transfer count equal to
/// the whole region every pass replays exactly `words`. A ring masks the low
/// bits of the address, which means the segment has to be aligned to its own
/// size *and* be a power of two no larger than 32768 bytes — both true of one
/// plane of a 64x64 panel (4096 bytes, hence the `#[repr(align(4096))]` on the
/// framebuffer). Note this is RP2350-specific: `RING_SIZE` here is `1 << n`
/// bytes, where the RP2040's was `1 << (n + 2)`.
///
/// This is also why the segment walker is still to come for BCM: one ring can
/// loop one power-of-two segment, but a multi-plane refresh needs a different
/// repetition count per segment, which is a DMA restart (or a descriptor chain)
/// rather than a free-running loop.
///
/// # Safety
///
/// `words` has to outlive the program: the channel reads it for ever.
unsafe fn stream_forever<CH, SM, SZ>(
    channel: &mut CH,
    words: &'static [u16],
    tx: &hal::pio::Tx<SM, SZ>,
) where
    CH: SingleChannel,
    SM: hal::pio::ValidStateMachine,
    SZ: hal::dma::TransferSize,
{
    let ch = channel.ch();
    let count = u32::try_from(words.len()).expect("one refresh has to fit the transfer count");
    let bytes = core::mem::size_of_val(words);
    assert!(
        bytes.is_power_of_two() && bytes <= 32768,
        "a read ring can only wrap a power-of-two region of at most 32768 bytes"
    );
    assert_eq!(
        words.as_ptr() as usize % bytes,
        0,
        "a read ring wraps by masking address bits, so the segment has to be aligned to its size"
    );
    let ring = bytes.trailing_zeros();

    // SAFETY: these registers belong to this channel alone, which `&mut CH`
    // proves, and nothing else in the program touches them.
    unsafe {
        ch.ch_read_addr().write(|w| w.bits(words.as_ptr() as u32));
        ch.ch_write_addr()
            .write(|w| w.bits(tx.fifo_address() as u32));
        ch.ch_trans_count().write(|w| {
            w.count().bits(count);
            w.mode().trigger_self();
            w
        });
        ch.ch_al1_ctrl().write(|w| {
            w.data_size().bits(1); // 16-bit transfers: one framebuffer word
            w.incr_read().set_bit(); // walk the framebuffer...
            w.incr_write().clear_bit(); // ...and always the same FIFO
            w.ring_sel().clear_bit(); // wrap the read address, not the write
            w.ring_size().bits(ring as u8); // ...inside the segment
            w.treq_sel().bits(tx.dreq_value()); // paced by the state machine
            w.chain_to().bits(channel.id()); // chain to itself: loop for ever
            w.en().set_bit();
            w
        });
    }

    // SAFETY: this writes only this channel's bit of the trigger register.
    unsafe {
        (*hal::pac::DMA::ptr())
            .multi_chan_trigger()
            .write(|w| w.bits(1 << channel.id()));
    }
}

/// **A. Every row pair, every column, both halves white.**
///
/// The pattern that separates "the stream is running" from "the stream has
/// stopped". If the address lines advance, all 64 rows light. Only one row pair
/// lit — 0 and 32, say — means the address lines are stuck, which is what a DMA
/// stream that has walked off the end of its buffer leaves behind. A completely
/// dark panel means the words are not reaching the state machines at all.
fn every_row_pair_white(fb: &mut Fb) {
    for row in 0..i32::from(ROW_PAIRS) {
        for column in 0..PANEL_WIDTH as i32 {
            fb.set_pixel(Point::new(column, row), Color::WHITE);
            fb.set_pixel(Point::new(column, row + i32::from(ROW_PAIRS)), Color::WHITE);
        }
    }
}

/// **B. The top half white, the bottom half dark.**
///
/// `R0 G0 B0` should light rows 0-31 and leave 32-63 alone. All 64 rows lit
/// means the halves are crossed somewhere; a left/right split instead of a
/// top/bottom one means the colour pins are not in the order `pins.rs` claims.
fn top_half_white(fb: &mut Fb) {
    for row in 0..i32::from(ROW_PAIRS) {
        for column in 0..PANEL_WIDTH as i32 {
            fb.set_pixel(Point::new(column, row), Color::WHITE);
        }
    }
}

/// **C. The bottom half white, the top half dark** — [`top_half_white`]'s
/// mirror.
fn bottom_half_white(fb: &mut Fb) {
    for row in 0..i32::from(ROW_PAIRS) {
        for column in 0..PANEL_WIDTH as i32 {
            fb.set_pixel(Point::new(column, row + i32::from(ROW_PAIRS)), Color::WHITE);
        }
    }
}

/// **D. Three single-pixel columns down the left edge: red, green, blue.**
///
/// The column order test and the colour order test in one picture. Three
/// one-pixel-wide vertical lines at the panel's left edge, red then green then
/// blue, and nothing else lit. Dots at the *right* edge mean the shift
/// direction is reversed; a staircase means the columns slide with the row (the
/// M2b class of bug); more than three dots per row means the word's fields are
/// landing in the wrong places. This is also the only phase here that can tell
/// `R0` from `G0` — the diagonal uses white and blue only.
///
/// This rung is also what settles the strobe. Dots at x = 1, 2 and 3 instead of
/// 0, 1 and 2 mean the panel latched on the strobe's *rising* edge and so
/// captured one column early: it wants the latch a word later than
/// `hub75-framebuffer` puts it (`tail-closes-latch`, or a second state machine
/// for the latch alone). The host test
/// `a_strobe_that_latches_on_its_rising_edge_shifts_every_row` is what that
/// picture looks like.
fn left_edge_and_colours(fb: &mut Fb) {
    for row in 0..i32::from(ROW_PAIRS) {
        fb.set_pixel(Point::new(0, row), Color::RED);
        fb.set_pixel(Point::new(1, row), Color::GREEN);
        fb.set_pixel(Point::new(2, row), Color::BLUE);
    }
}

/// **E. The M2b diagonal, streamed out of the framebuffer.**
///
/// One unbroken 45° line from a corner to the opposite one, white in the top
/// half and blue where the bottom half's arm continues it, `x == y` the whole way
/// down: exactly where the framebuffer has it. That is what the crate's address
/// field is for — it puts the *previous* slot's address on the wires, one step
/// behind the data being clocked in, which is the address of the row a panel is
/// showing while that data is being shifted towards the latches. A kink, a
/// mirrored arm or a column shift is a real fault; every row shifted by a whole
/// row means the address lines are a row out. The host tests in `programs.rs`
/// assert this exact picture.
fn the_diagonal(fb: &mut Fb) {
    for row in 0..i32::from(ROW_PAIRS) {
        let column = i32::from(HALF_WIDTH) + row;
        fb.set_pixel(Point::new(row, row), Color::WHITE);
        fb.set_pixel(Point::new(column, column), Color::BLUE);
    }
}

/// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"rp-hub75 M3: one PIO machine, one DMA channel"),
    hal::binary_info::rp_program_build_attribute!(),
];
