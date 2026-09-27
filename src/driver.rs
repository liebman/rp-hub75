//! The driver: one PIO state machine, two DMA channels, and one interrupt a
//! frame.
//!
//! # What runs the panel
//!
//! A single state machine streams framebuffer words straight at the HUB75 pin
//! window (see [`crate::programs`]), the reference board's being GP0-13. A
//! **control** DMA channel plays a [`crate::descriptor`] table into the data
//! channel's `AL3` register block, and the data channel chains back to it on
//! every completion, so the two hand the picture to each other in hardware:
//!
//! ```text
//! control channel  (trans_count, read_addr) → the data channel's AL3 block
//!                  the read address starts the run
//! data channel     streams the run to the PIO FIFO, paced by its DREQ
//!                  completes, chains to the control channel
//! ```
//!
//! The table is the framebuffer's own BCM sequence, materialised by
//! [`crate::descriptor::fill`]: one entry per repetition of every segment, in
//! scan order, then a terminator. Nothing here knows what a plane or a row is: a
//! plane-major (`bitplane::plain::frame`), a row-major (`bitplane::plain::row`),
//! a threshold or a remapped framebuffer all drive the same code, with the same
//! swap, and the layout only changes the *cost*, which
//! [`Hub75::runs_per_refresh`] reports.
//!
//! # One interrupt per frame
//!
//! The only interrupt is the terminator's: zero in a trigger register starts
//! nothing, and on the quiet data channel it releases the one deferred interrupt.
//! The CPU therefore wakes **once per frame** — at the boundary, where the chain
//! has stopped and the swap belongs — and the rate depends neither on the layout
//! (32 runs a refresh plane-major, 1024 row-major for these panels) nor on where
//! the linker put the framebuffer, because an entry carries its own address.
//!
//! Between boundaries the CPU is free: the chain re-arms itself as often as the
//! frame needs, and a `reps` count costs an entry in the table rather than a run
//! of its own, so no ring size, alignment or power-of-two rule appears anywhere.
//!
//! # Double buffering
//!
//! [`Hub75::swap`] takes the incoming framebuffer and remembers the *delta*
//! between it and the one on the panel. Both buffers have the same type and
//! therefore the same layout, so a swap is one number, applied by the interrupt at
//! the next frame boundary. The incoming frame starts exactly then, the outgoing
//! buffer is unreferenced at that instant, and [`Hub75Swap::wait`] hands it back.
//!
//! A descriptor names an absolute address, so the interrupt applies the delta
//! through [`crate::descriptor::patch`] — one store per run, on the frames that
//! swap and no others — and then restarts the chain.
//!
//! # The interrupt
//!
//! The application binds the handler; the driver does not define one, so the
//! line stays the application's choice:
//!
//! ```text
//! #[unsafe(no_mangle)]
//! extern "C" fn DMA_IRQ_1() { rp_hub75::on_dma_irq(); }
//! ```
//!
//! (the handler *is* the vector-table symbol: `rp235x-hal` does not re-export
//! `cortex_m_rt::interrupt`, and that macro in device mode wants a crate named
//! `interrupt` this PAC does not provide). The line is named in
//! [`Hub75Config::new`], which is the only way the driver knows which one to
//! unmask — the two must agree, and a mismatch simply means nothing moves.

use core::cell::Cell;
use core::cell::RefCell;
use core::marker::PhantomData;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::task::Waker;

use critical_section::Mutex;
use rp235x_hal as hal;

use hal::dma::SingleChannel;
use hal::fugit::Rate;
use hal::gpio::DynPinId;
use hal::gpio::FunctionNull;
use hal::gpio::FunctionPio0;
#[cfg(feature = "invert-clock")]
use hal::gpio::OutputOverride;
use hal::gpio::PullNone;
use hal::gpio::PullType;
use hal::pac;
use hal::pio::Buffers;
use hal::pio::PIOBuilder;
use hal::pio::PIOExt;
use hal::pio::PinDir;
use hal::pio::ShiftDirection;
use hub75_framebuffer::FrameBuffer;

use crate::descriptor::DESCRIPTOR_WORDS;
use crate::descriptor::Descriptor;
use crate::descriptor::Hub75Descriptors;
use crate::descriptor::descriptor_count;
use crate::pins;
use crate::programs::PULL_THRESHOLD;
use crate::programs::hub75_stream;

/// `log2` of the data channel's transfer size: 16-bit framebuffer words.
const DATA_SIZE_HALFWORD: u8 = 1;

/// `log2` of the control channel's transfer size: 32-bit descriptor words.
const DATA_SIZE_WORD: u8 = 2;

/// `RING_SIZE` for the control channel's write pointer: `1 << 3` bytes, the two
/// words of one `AL3` descriptor block.
///
/// The ring wraps the write address on an 8-byte boundary, so every trigger
/// writes the same two data-channel registers — the whole reason the control
/// channel needs no per-run address of its own.
const CONTROL_RING_SIZE_LOG2: u8 = 3;

/// `TREQ_SEL` for a channel nothing paces: it transfers as fast as the bus takes.
const TREQ_PERMANENT: u8 = 63;

/// PIO cycles one word costs. The panel clock is therefore the word rate.
const CYCLES_PER_WORD: u32 = crate::programs::CYCLES_PER_WORD;

/// The PIO divider for a requested panel clock, rounded **up**.
///
/// Rounding up rather than truncating matters because a panel's clock rating is a
/// ceiling: the divider has to hold the word clock at or below the clock that was
/// asked for, and truncating overshoots it — 20 MHz from a 150 MHz system clock
/// would divide by 3 rather than 4 and run the panel at 25 MHz. Integer dividers
/// are what the PIO is given here, so a request no divider hits exactly is met
/// with the next slower clock; [`Hub75::panel_clock`] reports what was achieved.
fn divider_for(system_clock_hz: u32, panel_clock_hz: u32) -> u32 {
    // Saturating and at least one, so a degenerate request reaches the caller's
    // assertion as a number rather than as a division by zero.
    let word_clock_hz = panel_clock_hz.saturating_mul(CYCLES_PER_WORD).max(1);
    system_clock_hz.div_ceil(word_clock_hz)
}

/// Panel shift clock a divider achieves: the system clock divided by the divider
/// and the cycles one word takes.
fn achieved_clock(system_clock_hz: u32, divider: u32) -> u32 {
    system_clock_hz / divider / CYCLES_PER_WORD
}

/// Pins in the HUB75 window: the fourteen the framebuffer's word is shifted
/// into, one of which carries no signal.
pub const PIN_COUNT: usize = pins::WORD_PINS as usize;

/// Signals a framebuffer word carries, in bus order.
pub const SIGNAL_COUNT: usize = pins::SIGNAL_COUNT;

/// Signal names, in the order [`Hub75Pins::new`] takes the pins, for the
/// messages [`check_bus`] panics with.
const SIGNAL_NAMES: [&str; SIGNAL_COUNT] = [
    "R0", "G0", "B0", "R1", "G1", "B1", "A", "B", "C", "D", "E", "STB", "OE",
];

/// A GPIO the PIO can drive.
///
/// [`Hub75Pins::new`] takes the bus pins as concrete types, so this bound is
/// what makes handing over a pin the PIO cannot reach a compile error. The pins'
/// GPIO *numbers* are checked at startup instead: each signal must sit on the
/// window pin its word bit is shifted to, and the whole window must fit the
/// PIO's pin fields.
pub trait PioPin: hal::gpio::PinId + hal::gpio::ValidFunction<FunctionPio0> {}

impl<P> PioPin for P where P: hal::gpio::PinId + hal::gpio::ValidFunction<FunctionPio0> {}

/// Checks that a bus-order pin list is a window the PIO can drive and that each
/// signal is where its word bit lands.
///
/// A HUB75 bus is fourteen consecutive GPIOs. `out pins, 14` writes
/// `[base, base + 13]` and window pin *n* receives framebuffer word bit *n*, so
/// the signal on a pin is decided by the word's layout: the pin given for signal
/// *i* must be `base + LAYOUT[i]`. Anything else would drive the wrong pins —
/// swapped colours, a latch that never latches — which is a wiring mistake rather
/// than a dark panel, so it stops startup.
///
/// The clock is checked apart from the signals, being the one pin the side-set
/// drives rather than the word: it may be the reserved window pin
/// ([`pins::RESERVED`], where the word's zero and the clock agree) or any pin
/// outside the window, but not a signal pin — the side-set would win on every
/// cycle and corrupt that signal silently.
///
/// # Panics
///
/// Panics if a signal is not on the window pin its word bit is driven to, if
/// the window does not fit the PIO's five-bit pin fields — a base above
/// [`pins::MAX_WORD_BASE`] — or if the clock pin is a window pin other than
/// [`pins::RESERVED`], or is not a pin the PIO can address.
fn check_bus(numbers: &[u8; SIGNAL_COUNT], clock: u8) {
    let base = *numbers
        .iter()
        .min()
        .expect("a bus has signals, so it has a lowest pin");
    for (index, &number) in numbers.iter().enumerate() {
        let expected = base + pins::LAYOUT[index];
        assert_eq!(
            number,
            expected,
            "{} is wired to pin {number}, but framebuffer word bit {} is driven \
             to pin {expected} of the window starting at {base}: the signal is \
             on the wrong pin",
            SIGNAL_NAMES[index],
            pins::LAYOUT[index],
        );
    }
    assert!(
        base <= pins::MAX_WORD_BASE,
        "a {PIN_COUNT}-pin window cannot start at {base}: the PIO's pin fields \
         are five bits, so it must start at {} or lower",
        pins::MAX_WORD_BASE
    );
    assert!(
        clock <= pins::MAX_PIN,
        "the clock is on pin {clock}, which the PIO cannot address: its pin \
         fields are five bits, so the clock must be on pin {} or lower",
        pins::MAX_PIN
    );
    // The window is fourteen pins wide from `base`, so the checks above have it
    // inside the PIO's range and neither sum can overflow.
    let reserved = base + pins::RESERVED;
    let top = base + (pins::WORD_PINS - 1);
    assert!(
        clock < base || clock == reserved || clock > top,
        "the clock is on pin {clock}, which is inside the window starting at \
         {base} but is not its reserved pin {reserved}: the side-set takes \
         precedence over the word on that pin, so it would overwrite the signal \
         there on every cycle. Wire the clock to pin {reserved} — where the \
         framebuffer word holds zero — or to a pin outside the window"
    );
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// How the panel is driven.
///
/// What a panel wants *electrically* is a build-time choice, not a field: the
/// `invert-clock` feature moves the edge it samples a column on, and
/// `invert-oe` the level that blanks it.
#[derive(Debug, Clone, Copy)]
pub struct Hub75Config {
    /// DMA interrupt line the application bound [`on_dma_irq`] to.
    ///
    /// Required, because the interrupt is what restarts the chain at each frame
    /// boundary: without it the panel plays one frame and stops. The pairing is
    /// the application's — this value and the `#[interrupt]` function calling
    /// [`on_dma_irq`] must name the *same* line, and the driver cannot check that
    /// (nothing in the vector table is readable that way), so a mismatch simply
    /// means a still picture.
    pub interrupt: pac::Interrupt,
    /// Panel shift-clock frequency — one word, one column, per period.
    pub panel_clock: Rate<u32, 1, 1>,
}

impl Hub75Config {
    /// The recommended configuration: an 18.75 MHz panel clock and `interrupt`
    /// as the wake-up line.
    ///
    /// 18.75 MHz is 150 MHz / 4 / 2: a word every two PIO cycles. The refresh
    /// rate then follows the *layout* — one refresh is the framebuffer's whole
    /// BCM sequence — so [`Hub75::refresh_hz`] reports what a given clock
    /// actually gives. It is also below 20 MHz, which keeps the clock legible on
    /// a logic analyser.
    #[must_use]
    pub const fn new(interrupt: pac::Interrupt) -> Self {
        Self {
            interrupt,
            panel_clock: Rate::<u32, 1, 1>::kHz(18_750),
        }
    }

    /// Sets the panel shift clock. See [`Hub75Config::new`].
    ///
    /// The clock is a ceiling. The PIO divider is an integer and is rounded up, so
    /// a request no divider hits exactly runs the panel at the next slower clock
    /// rather than faster than the panel is rated for;
    /// [`Hub75::panel_clock`] reports what was achieved.
    #[must_use]
    pub const fn with_panel_clock(mut self, panel_clock: Rate<u32, 1, 1>) -> Self {
        self.panel_clock = panel_clock;
        self
    }
}

/// What can go wrong.
///
/// There is deliberately not much: the only failure modes are the API being used
/// out of order, or a framebuffer whose layout this driver cannot cache.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hub75Error {
    /// A driver already exists. There is one PIO, two channels and one table,
    /// and the driver runs for ever, so a second instance would be a bug.
    AlreadyInitialised,
    /// A swap is still in flight. Wait for the previous one — which hands the
    /// outgoing framebuffer back — before arming another.
    SwapInFlight,
    /// The interrupt in the configuration is not one of the four DMA lines, so
    /// no completion can ever reach it.
    NotADmaIrq,
}

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

/// The HUB75 pins, converted and owned by the driver.
///
/// The constructor takes the thirteen signals in bus order — `R0 G0 B0 R1 G1 B1
/// A B C D E STB OE` — and the clock separately, because the clock is the one
/// pin the PIO generates rather than copies out of a framebuffer word. It checks
/// that the signals are a fourteen-pin window with each one on the pin its word
/// bit is driven to, and that the clock is either that window's reserved pin or
/// a pin outside it. The handles are kept so nothing else can claim the bus.
/// `Pins::new` yields pins in `FunctionNull`; this does the `into_function` step.
pub struct Hub75Pins {
    /// The GPIO number the fourteen-pin window starts at.
    base: u8,
    /// The window's signal pins, in bus order.
    hold: [hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>; SIGNAL_COUNT],
    /// The panel clock, wherever it is wired.
    clock: hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>,
}

impl Hub75Pins {
    /// Converts the panel's pins to an owned pin set.
    ///
    /// The order is the HUB75 bus: top half (`R0 G0 B0`), bottom half
    /// (`R1 G1 B1`), row address (`A B C D E`), then `STB` and `OE`. The clock is
    /// the last argument and may be any GPIO the PIO can drive: the window pin
    /// [`pins::RESERVED`], where the frame buffer's word holds zero, or a pin
    /// outside the window. The lowest signal pin is the window's base, which
    /// [`base`](Self::base) reports. See [`crate::pins`] for the offsets this
    /// order is made of, and [`pins::LAYOUT`] for the word bit each one carries.
    ///
    /// # Panics
    ///
    /// Panics if a signal is not on the window pin its word bit is driven to, if
    /// the window does not fit the PIO's five-bit pin fields, or if the clock is
    /// a window pin other than [`pins::RESERVED`] or a pin the PIO cannot
    /// address.
    #[allow(clippy::too_many_arguments)]
    pub fn new<I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, P>(
        r0: hal::gpio::Pin<I0, FunctionNull, P>,
        g0: hal::gpio::Pin<I1, FunctionNull, P>,
        b0: hal::gpio::Pin<I2, FunctionNull, P>,
        r1: hal::gpio::Pin<I3, FunctionNull, P>,
        g1: hal::gpio::Pin<I4, FunctionNull, P>,
        b1: hal::gpio::Pin<I5, FunctionNull, P>,
        a: hal::gpio::Pin<I6, FunctionNull, P>,
        b: hal::gpio::Pin<I7, FunctionNull, P>,
        c: hal::gpio::Pin<I8, FunctionNull, P>,
        d: hal::gpio::Pin<I9, FunctionNull, P>,
        e: hal::gpio::Pin<I10, FunctionNull, P>,
        stb: hal::gpio::Pin<I11, FunctionNull, P>,
        oe: hal::gpio::Pin<I12, FunctionNull, P>,
        clk: hal::gpio::Pin<I13, FunctionNull, P>,
    ) -> Self
    where
        I0: PioPin,
        I1: PioPin,
        I2: PioPin,
        I3: PioPin,
        I4: PioPin,
        I5: PioPin,
        I6: PioPin,
        I7: PioPin,
        I8: PioPin,
        I9: PioPin,
        I10: PioPin,
        I11: PioPin,
        I12: PioPin,
        I13: PioPin,
        P: PullType,
    {
        // The panel pins are outputs, so their pull setting is irrelevant; it is
        // normalized away so the driver does not need a type parameter for it.
        fn hold<I: PioPin, P: PullType>(
            pin: hal::gpio::Pin<I, FunctionNull, P>,
        ) -> hal::gpio::Pin<DynPinId, FunctionPio0, PullNone> {
            let pin: hal::gpio::Pin<I, FunctionPio0, PullNone> =
                pin.into_pull_type::<PullNone>().into_function();
            pin.into_dyn_pin()
        }
        let clock = hold(clk);
        let hold = [
            hold(r0),
            hold(g0),
            hold(b0),
            hold(r1),
            hold(g1),
            hold(b1),
            hold(a),
            hold(b),
            hold(c),
            hold(d),
            hold(e),
            hold(stb),
            hold(oe),
        ];
        let numbers: [u8; SIGNAL_COUNT] = core::array::from_fn(|index| hold[index].id().num);
        // Checked here, so a mis-wired bus stops startup; the base the check
        // derives is stored rather than recomputed.
        check_bus(&numbers, clock.id().num);
        let base = *numbers
            .iter()
            .min()
            .expect("a bus has signals, so it has a lowest pin");
        Self { base, hold, clock }
    }

    /// Inverts the clock pin's output in the GPIO block.
    ///
    /// A panel that takes the column on the clock's falling edge gets it here, not
    /// from a second program: the pad inverts whatever the PIO drives on that pin —
    /// the side-set, and the word's own reserved bit where the clock occupies it —
    /// and both write zero on the cycle the word changes, so the two agree and the
    /// whole bus stays in step with the clock.
    #[cfg(feature = "invert-clock")]
    fn invert_clock(&mut self) {
        debug_assert!(
            self.clock() == self.base + pins::RESERVED
                || self.clock() > self.base + (pins::WORD_PINS - 1)
        );
        self.clock.set_output_override(OutputOverride::Invert);
    }

    /// The GPIO number of window pin `index`: the signal [`pins::LAYOUT`] places
    /// there.
    fn window_pin(&self, index: usize) -> u8 {
        self.hold[index].id().num
    }

    /// The GPIO number the fourteen-pin window starts at.
    ///
    /// The lowest signal pin [`new`](Self::new) was given — `R0`'s, on the
    /// reference wiring — which `out pins, 14` writes through `base + 13`.
    pub fn base(&self) -> u8 {
        self.base
    }

    /// The GPIO number the panel clock is wired to.
    ///
    /// [`base`](Self::base) plus [`pins::RESERVED`] on the reference wiring, or a
    /// pin outside the window.
    pub fn clock(&self) -> u8 {
        self.clock.id().num
    }
}

// ---------------------------------------------------------------------------
// The engine: everything the interrupt needs, without a framebuffer type
// ---------------------------------------------------------------------------

/// The engine's state.
///
/// Everything here is deliberately *data*: the interrupt cannot name the
/// framebuffer's type, so the frame's table and the control channel's register
/// addresses are captured at init and used afterwards.
struct Engine {
    /// The frame's runs and their terminator, played by the control channel.
    ///
    /// This points at the caller's [`Hub75Descriptors`], which outlives the
    /// driver, rather than owning a table: the driver's own static stays a few
    /// words instead of carrying storage sized for the largest layout.
    table: *mut Descriptor,
    /// Descriptors at `table`: the runs, plus the terminator.
    table_len: usize,
    /// Runs in the table, i.e. the index of the terminator. Zero means no driver.
    table_count: usize,
    /// The framebuffer being scanned.
    base: u32,
    /// A swap waiting for the next frame boundary: the delta to the incoming
    /// framebuffer.
    pending: Option<i32>,
    /// The control channel's `READ_ADDR`, through the alias that does not
    /// trigger: a frame boundary points it back at the table.
    control_read_addr: *mut u32,
    /// The control channel's `TRANS_COUNT` trigger: one store restarts the walk.
    control_trigger: *mut u32,
    /// The data channel's bit in `INTR`/`INTEn`, and the line it is routed to.
    flag: u32,
    irq_line: u8,
}

// SAFETY (`Send`): required for the static below to be `Sync`. Every access is
// serialized by the critical section, and the register pointers are only
// dereferenced inside it.
unsafe impl Send for Engine {}

impl Engine {
    /// Nothing running. The engine is a handful of words; the table itself is the
    /// caller's storage, pointed at by `table`.
    const EMPTY: Self = Self {
        table: core::ptr::null_mut(),
        table_len: 0,
        table_count: 0,
        base: 0,
        pending: None,
        control_read_addr: core::ptr::null_mut(),
        control_trigger: core::ptr::null_mut(),
        flag: 0,
        irq_line: 0,
    };

    /// Applies a pending swap: the framebuffer base, and the table the control
    /// channel plays next.
    ///
    /// Returns whether a swap was applied. The base moves first, so a swap armed
    /// while this runs is measured from the framebuffer that is about to be
    /// scanned rather than the one leaving the panel.
    fn apply_pending(&mut self) -> bool {
        let Some(delta) = self.pending.take() else {
            return false;
        };
        self.base = self.base.wrapping_add(delta as u32);
        let count = self.table_count;
        let table_len = self.table_len;
        // SAFETY: `table` is the caller's `Hub75Descriptors` storage, `'static`
        // and handed to the driver at init, and `table_len` is its length. The
        // driver holds the only reference to it, and this runs in the critical
        // section the interrupt holds.
        let table = unsafe { core::slice::from_raw_parts_mut(self.table, table_len) };
        crate::descriptor::patch(table, count, delta as u32);
        SWAP_DONE.store(true, Ordering::Release);
        true
    }

    /// One frame: the control channel has stopped at the terminator, so apply a
    /// swap and set the chain going again.
    fn step_frame(&mut self) -> bool {
        let swapped = self.apply_pending();
        self.restart_chain();
        swapped
    }

    /// Points the control channel at the table and starts it.
    ///
    /// The channel is halted at the terminator, so this is the store that begins
    /// the frame: the read address resumes at the table's first run.
    fn restart_chain(&mut self) {
        let table = self.table as u32;
        let read_addr = self.control_read_addr;
        let trigger = self.control_trigger;
        // SAFETY: these are the control channel's own registers, captured at init
        // and written only from here.
        unsafe {
            read_addr.write_volatile(table);
            trigger.write_volatile(DESCRIPTOR_WORDS);
        }
    }
}

/// The one engine. `EMPTY` until a driver exists.
static ENGINE: Mutex<RefCell<Engine>> = Mutex::new(RefCell::new(Engine::EMPTY));

/// Set (with `Release`) once a swap has been applied at a frame boundary, i.e.
/// the outgoing framebuffer will never be read again.
static SWAP_DONE: AtomicBool = AtomicBool::new(false);

/// Set once a driver exists.
static DRIVER_TAKEN: AtomicBool = AtomicBool::new(false);

/// The interrupt line index (0-3) for a `DMA_IRQ_n`.
fn line_index(irq: pac::Interrupt) -> Option<u8> {
    match irq {
        pac::Interrupt::DMA_IRQ_0 => Some(0),
        pac::Interrupt::DMA_IRQ_1 => Some(1),
        pac::Interrupt::DMA_IRQ_2 => Some(2),
        pac::Interrupt::DMA_IRQ_3 => Some(3),
        _ => None,
    }
}

/// A `Waker` the interrupt can poke, without an executor.
///
/// `rp235x-hal`'s equivalent (`async_utils::IrqWaker`) is sealed, but this is
/// all of it: a critical-section cell holding the waker of the waiting task,
/// taken on wake so a spurious second wake costs nothing.
struct IrqWaker {
    waker: Mutex<Cell<Option<Waker>>>,
}

impl IrqWaker {
    const fn new() -> Self {
        Self {
            waker: Mutex::new(Cell::new(None)),
        }
    }

    fn register(&self, waker: &Waker) {
        critical_section::with(|cs| {
            self.waker.borrow(cs).set(Some(waker.clone()));
        });
    }

    fn wake(&self) {
        let waker = critical_section::with(|cs| self.waker.borrow(cs).take());
        if let Some(waker) = waker {
            waker.wake();
        }
    }
}

/// The swap state's waker.
static WAKER: IrqWaker = IrqWaker::new();

/// The DMA completion handler: apply a swap, and set the next frame going.
///
/// The only interrupt the engine raises is the table's terminator, so this runs
/// once a frame — by which time the control channel has stopped — and does four
/// things: acknowledge the data channel, patch the table if a swap is pending,
/// restart the chain, and wake a waiting swap.
///
/// Call it from the handler bound to the line named in [`Hub75Config::new`]:
///
/// ```text
/// #[unsafe(no_mangle)]
/// extern "C" fn DMA_IRQ_1() { rp_hub75::on_dma_irq(); }
/// ```
///
/// It is safe to call for *any* interrupt routed to that line: with no driver
/// running it touches nothing at all.
pub fn on_dma_irq() {
    let swapped = critical_section::with(|cs| {
        let mut engine = ENGINE.borrow_ref_mut(cs);
        let engine = &mut *engine;
        if engine.table_count == 0 {
            // No driver, or an interrupt from a neighbour on the same line.
            return false;
        }

        // Acknowledge *our* channel's completion. `INTR` is write-1-to-clear and
        // is not masked by `INTEn`, so this cannot disturb another channel.
        // SAFETY: the DMA block is a static peripheral.
        unsafe { (*pac::DMA::ptr()).intr().write(|w| w.bits(engine.flag)) };

        // Nothing here may log. defmt's `acquire` is a critical section, and this
        // runs once a frame with the picture waiting on it: a logger that blocks,
        // or a deadlock against a log in thread context, would be the frozen
        // panel this project has already met once. The library does not log at
        // all; the examples log from their own loops.
        engine.step_frame()
    });

    if swapped {
        // Wake outside the critical section: `wake()` runs executor code.
        WAKER.wake();
    }
}

// ---------------------------------------------------------------------------
// The driver
// ---------------------------------------------------------------------------

/// Configures the data channel: the one that streams the panel.
///
/// Written once, and never touched again — every descriptor the control channel
/// plays rewrites `READ_ADDR` and `TRANS_COUNT`, and nothing rewrites `CTRL`.
/// `control` is the channel that this one chains to when a run completes, and
/// `IRQ_QUIET` is set so the end of a run does **not** interrupt: the only
/// interrupt a frame raises is the table's terminator. (A channel that chains to
/// itself does not chain; the reset value 0 would hand every completion to
/// channel 0 instead.)
fn new_data_channel<CH: SingleChannel, CCH: SingleChannel>(
    channel: &CH,
    control: &CCH,
    fifo: u32,
    dreq: u8,
) {
    // SAFETY: the channel belongs to this driver from here on, and these writes
    // are the only configuration it ever gets.
    unsafe {
        let ch = channel.ch();
        ch.ch_write_addr().write(|w| w.bits(fifo));
        // `CTRL` through the alias whose trigger is `TRANS_COUNT`, so this
        // configures without starting anything: 16-bit words, walk the read
        // address, no ring, paced by the state machine.
        ch.ch_al1_ctrl().write(|w| {
            w.data_size().bits(DATA_SIZE_HALFWORD);
            w.incr_read().set_bit();
            w.incr_write().clear_bit();
            w.ring_sel().clear_bit();
            w.ring_size().bits(0);
            w.treq_sel().bits(dreq);
            w.chain_to().bits(control.id());
            w.irq_quiet().set_bit();
            w.en().set_bit();
            w
        });
    }
}

/// Configures the control channel: the one that plays a descriptor table into
/// the data channel's `AL3` register block.
///
/// `target` is the data channel's `AL3_TRANS_COUNT`. The channel walks 32-bit
/// words from the table, and wraps its write pointer on an 8-byte boundary so
/// that every block lands on the same two registers. Nothing paces it: it runs
/// as fast as the bus allows, once per run the data channel completes.
fn new_control_channel<CH: SingleChannel>(channel: &CH, target: u32) {
    // SAFETY: the channel belongs to this driver from here on, and these writes
    // are the only configuration it ever gets.
    unsafe {
        let ch = channel.ch();
        ch.ch_write_addr().write(|w| w.bits(target));
        // `CTRL` through the alias whose trigger is `TRANS_COUNT`, so this
        // configures without starting anything: 32-bit words, both addresses
        // walking, the write address ringed over one descriptor block, no TREQ,
        // and no chain — the trigger word inside the block is what starts the
        // data channel, so a chain of its own would start it a second time.
        ch.ch_al1_ctrl().write(|w| {
            w.data_size().bits(DATA_SIZE_WORD);
            w.incr_read().set_bit();
            w.incr_write().set_bit();
            w.ring_sel().set_bit();
            w.ring_size().bits(CONTROL_RING_SIZE_LOG2);
            w.treq_sel().bits(TREQ_PERMANENT);
            w.high_priority().set_bit();
            // A channel that chains to itself does not chain.
            w.chain_to().bits(channel.id());
            w.irq_quiet().set_bit();
            w.en().set_bit();
            w
        });
    }
}

/// Enables this channel's completion on one of the four DMA interrupt lines.
///
/// The `INTEn` registers are four distinct types in the PAC, so this routes by
/// address and sets one bit.
fn arm(flag: u32, line: u8) {
    let dma = unsafe { &*pac::DMA::ptr() };
    let register = match line {
        0 => dma.inte0().as_ptr(),
        1 => dma.inte1().as_ptr(),
        2 => dma.inte2().as_ptr(),
        _ => dma.inte3().as_ptr(),
    };
    // SAFETY: one of the DMA's interrupt-enable registers, read-modify-written
    // once at init.
    unsafe { register.write_volatile(register.read_volatile() | flag) };
}

/// What the constructors need back from the shared peripheral bring-up.
struct BringUp {
    /// The panel pins, converted and held so nothing else can claim the bus.
    pins: Hub75Pins,
    /// The PIO clock divider in use.
    divider: u16,
    /// The panel clock actually achieved: the request rounded to a divider.
    panel_clock: Rate<u32, 1, 1>,
    /// The state machine's FIFO, which is the data channel's write address.
    fifo: u32,
    /// The state machine's TX DREQ, which paces the data channel.
    dreq: u8,
}

/// Starts the state machine and returns what the channels need to feed it.
///
/// The panel clock is a ceiling: the PIO divider is an integer and is rounded
/// up, so a request no divider hits exactly runs the panel at the next slower
/// clock rather than faster than the panel is rated for.
fn bring_up(
    pio: pac::PIO0,
    resets: &mut pac::RESETS,
    pins: Hub75Pins,
    system_clock: Rate<u32, 1, 1>,
    config: Hub75Config,
) -> BringUp {
    // A word every `CYCLES_PER_WORD` PIO cycles, so the divider is the ratio
    // between the system clock and that many times the panel clock, rounded up
    // so the panel never runs faster than the clock it was given.
    let divider = divider_for(system_clock.to_Hz(), config.panel_clock.to_Hz());
    assert!(
        divider >= 1 && divider <= u16::MAX as u32,
        "the panel clock is not representable: divider {divider} is outside \
         the PIO's 1..=65535 range (system clock {} Hz, panel clock {} Hz)",
        system_clock.to_Hz(),
        config.panel_clock.to_Hz()
    );
    let divider = divider as u16;
    let achieved =
        Rate::<u32, 1, 1>::from_raw(achieved_clock(system_clock.to_Hz(), divider as u32));

    let (mut pio, sm0, _, _, _) = pio.split(resets);

    // The window the framebuffer words are shifted into. `Hub75Pins::new` has
    // already checked that the signals are a fourteen-pin window, each on the pin
    // its word bit is driven to, so the base is the lowest signal pin and the
    // clock is wherever it was told the clock is.
    let base = pins.base();
    let clock = pins.clock();

    // The clock's polarity is the pad's, so the pin is inverted before the
    // machine starts and the first clock level is already the idle one.
    #[cfg(feature = "invert-clock")]
    let mut pins = pins;
    #[cfg(feature = "invert-clock")]
    pins.invert_clock();

    let program = pio
        .install(&hub75_stream())
        .expect("the stream program fits the instruction memory");
    let (mut sm, _, tx) = PIOBuilder::from_installed_program(program)
        .out_pins(base, pins::WORD_PINS)
        .side_set_pin_base(clock)
        .clock_divisor_fixed_point(divider, 0)
        .out_shift_direction(ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(PULL_THRESHOLD)
        .buffers(Buffers::OnlyTx)
        .build(sm0);

    // Every signal pin and the clock are outputs: the machine drives the whole
    // bus. The window's reserved pin is left alone unless the clock is wired
    // there, so a clock outside the window does not drive a pin held at zero.
    sm.set_pindirs(
        (0..SIGNAL_COUNT)
            .map(|index| (pins.window_pin(index), PinDir::Output))
            .chain(core::iter::once((clock, PinDir::Output))),
    );
    // Keep the machine running. The handle has no `Drop`, so dropping it here
    // would be safe; holding it documents who owns the machine.
    let _sm = sm.start();

    BringUp {
        pins,
        divider,
        panel_clock: achieved,
        fifo: tx.fifo_address() as u32,
        dreq: tx.dreq_value(),
    }
}

/// A HUB75 panel, driven by hardware and one interrupt a frame.
///
/// Created once — there is one PIO, two DMA channels and one table — and then
/// refreshes itself: the CPU only writes pixels and arms the occasional swap.
pub struct Hub75<FB: FrameBuffer + 'static> {
    /// PIO clock divider actually used, for reporting.
    divider: u16,
    /// Panel clock actually achieved: the request rounded to a divider.
    panel_clock: Rate<u32, 1, 1>,
    /// Words streamed per refresh, from the framebuffer's own sequence.
    frame_words: u32,
    /// Runs the interrupt services per refresh.
    runs: u32,
    /// The panel pins, held so nothing else can claim the bus.
    pins: Hub75Pins,
    _marker: PhantomData<FB>,
}

impl<FB: FrameBuffer<Word = u16> + 'static> Hub75<FB> {
    /// Starts the panel.
    ///
    /// Takes the PIO block, **two** DMA channels — `channel` streams the panel,
    /// `control` plays the descriptor table — the pins, the system clock the PIO
    /// divider is derived from, the configuration (which names the interrupt line
    /// the application bound [`on_dma_irq`] to), and the first framebuffer to
    /// display. From here the panel refreshes itself; nothing else needs calling,
    /// ever.
    ///
    /// `fb` is taken by exclusive reference, and the first [`swap`](Self::swap)
    /// is what hands it back — so the buffer on screen and the buffer being
    /// drawn are never the same one, by construction.
    ///
    /// # Panics
    ///
    /// Panics if a framebuffer segment is empty (a zero-word run would end the
    /// frame early), or if the framebuffer's BCM sequence is empty. Both are
    /// properties of the layout, and both are better discovered at startup than as
    /// a frozen panel.
    #[allow(clippy::too_many_arguments)]
    pub fn new<CH, CCH, const N: usize>(
        pio: pac::PIO0,
        channel: CH,
        control: CCH,
        resets: &mut pac::RESETS,
        pins: Hub75Pins,
        system_clock: Rate<u32, 1, 1>,
        config: Hub75Config,
        fb: &'static mut FB,
        descriptors: &'static mut Hub75Descriptors<FB, N>,
    ) -> Result<Self, Hub75Error>
    where
        CH: SingleChannel,
        CCH: SingleChannel,
    {
        if DRIVER_TAKEN.swap(true, Ordering::AcqRel) {
            return Err(Hub75Error::AlreadyInitialised);
        }
        let irq_line = match line_index(config.interrupt) {
            Some(line) => line,
            None => return Err(Hub75Error::NotADmaIrq),
        };

        let BringUp {
            pins,
            divider,
            panel_clock,
            fifo,
            dreq,
        } = bring_up(pio, resets, pins, system_clock, config);

        // The control channel writes the data channel's `AL3` block: its last
        // word is that block's trigger, so a descriptor is the two words before
        // it and the ring keeps both stores landing there.
        let target = channel.ch().ch_al3_trans_count().as_ptr() as u32;
        new_control_channel(&control, target);
        new_data_channel(&channel, &control, fifo, dreq);

        Ok(Self::start(
            &channel,
            &control,
            pins,
            divider,
            panel_clock,
            config.interrupt,
            irq_line,
            fb,
            descriptors,
        ))
    }
}

impl<FB: FrameBuffer<Word = u16> + 'static> Hub75<FB> {
    /// Materialises the frame in the caller's descriptor storage and starts it.
    ///
    /// Split out of [`new`](Self::new) so that one function does the peripheral
    /// setup and this one the engine. Init is the only writer of the table until
    /// the interrupt is unmasked, and the interrupt is the only writer afterwards.
    #[allow(clippy::too_many_arguments)]
    fn start<const N: usize>(
        channel: &impl SingleChannel,
        control: &impl SingleChannel,
        pins: Hub75Pins,
        divider: u16,
        panel_clock: Rate<u32, 1, 1>,
        interrupt: pac::Interrupt,
        irq_line: u8,
        fb: &'static mut FB,
        descriptors: &'static mut Hub75Descriptors<FB, N>,
    ) -> Self {
        // The runs, without the terminator that ends the chain.
        let expected = descriptor_count::<FB>() - 1;
        let base = fb as *const FB as u32;
        let channel_id = channel.id();

        // SAFETY: the control channel's registers live for the rest of the
        // program, and the interrupt writes them from here on. `as_ptr` is safe;
        // the pointers it returns are only dereferenced in the interrupt.
        let (control_read_addr, control_trigger) = {
            let ch = control.ch();
            (
                ch.ch_al1_read_addr().as_ptr(),
                ch.ch_al1_trans_count_trig().as_ptr(),
            )
        };

        let (runs, frame_words) = critical_section::with(|cs| {
            let mut engine = ENGINE.borrow_ref_mut(cs);
            let engine = &mut *engine;
            engine.base = base;
            engine.pending = None;
            engine.flag = 1 << channel_id;
            engine.irq_line = irq_line;
            engine.control_read_addr = control_read_addr;
            engine.control_trigger = control_trigger;
            let table = descriptors.entries();
            engine.table = table.as_mut_ptr();
            engine.table_len = table.len();
            let count = crate::descriptor::fill(fb, table);
            engine.table_count = count;
            // The table *is* the frame, so the words a refresh streams are its
            // own counts: no second walk of the sequence.
            let words = table[..count]
                .iter()
                .map(|descriptor| descriptor.trans_count)
                .sum();
            (count as u32, words)
        });
        debug_assert_eq!(runs, expected as u32, "the table is one run per repetition");
        assert!(frame_words > 0, "a refresh of no words has no refresh rate");

        SWAP_DONE.store(false, Ordering::Release);

        // Arm the interrupt, then start the chain. The order matters: the first
        // interrupt is a frame boundary, and the control channel's first
        // descriptor is what puts words on the panel.
        arm(1 << channel_id, irq_line);
        // SAFETY: the application bound `on_dma_irq` to this line.
        unsafe { hal::arch::interrupt_unmask(interrupt) };
        critical_section::with(|cs| ENGINE.borrow_ref_mut(cs).restart_chain());

        Self {
            divider,
            panel_clock,
            frame_words,
            runs,
            pins,
            _marker: PhantomData,
        }
    }

    /// The panel clock actually achieved, after the divider rounding.
    pub fn panel_clock(&self) -> Rate<u32, 1, 1> {
        self.panel_clock
    }

    /// The PIO clock divider in use.
    pub fn divider(&self) -> u16 {
        self.divider
    }

    /// Panel refreshes per second.
    ///
    /// The panel clock is the word rate and one refresh is the framebuffer's
    /// whole BCM sequence, so this is the panel clock divided by the words the
    /// sequence streams.
    pub fn refresh_hz(&self) -> u32 {
        self.panel_clock.to_Hz() / self.frame_words
    }

    /// Runs one refresh is made of: the length of the descriptor table.
    ///
    /// One entry per repetition of every segment, so this is the layout's cost
    /// and not the panel's — 32 for a six-plane plane-major frame at any
    /// geometry, `NROWS` times as many for row-major. Nor is it the *buffer's*:
    /// an entry names its own address, so nothing about this number changes when
    /// a swap puts the other framebuffer on the panel.
    ///
    /// It is also, one for one, the runs a refresh is played in. The interrupts
    /// a second are [`refresh_hz`], because the table's terminator is the only
    /// thing that interrupts.
    ///
    /// [`refresh_hz`]: Self::refresh_hz
    pub fn runs_per_refresh(&self) -> u32 {
        self.runs
    }

    /// The panel pins the driver holds.
    pub fn pins(&self) -> &Hub75Pins {
        &self.pins
    }
}

// ---------------------------------------------------------------------------
// Swapping framebuffers
// ---------------------------------------------------------------------------

impl<FB: FrameBuffer<Word = u16> + 'static> Hub75<FB> {
    /// Arms a switch to `new_fb` at the next frame boundary.
    ///
    /// Returns immediately. The incoming framebuffer differs from the one on the
    /// panel by a single number — the segment offsets are layout constants, so
    /// the whole layout is a constant offset from the base — and the interrupt
    /// applies it at the boundary. The in-flight frame finishes on the outgoing
    /// buffer, the next one starts on `new_fb`, and the outgoing buffer is
    /// unreferenced the instant that happens.
    ///
    /// Hand the outgoing buffer back with [`Hub75Swap::wait`], which returns when
    /// the boundary has passed.
    ///
    /// # Errors
    ///
    /// [`Hub75Error::SwapInFlight`] if the previous swap has not been waited
    /// for, with `new_fb` handed straight back.
    pub fn swap(
        &self,
        new_fb: &'static mut FB,
    ) -> Result<Hub75Swap<FB>, (Hub75Error, &'static mut FB)> {
        let new_ptr = core::ptr::from_mut(new_fb);

        let armed = critical_section::with(|cs| {
            let mut engine = ENGINE.borrow_ref_mut(cs);
            let engine = &mut *engine;
            if engine.table_count == 0 || engine.pending.is_some() {
                return Err(());
            }
            let outgoing = engine.base;
            engine.pending = Some((new_ptr as usize).wrapping_sub(outgoing as usize) as i32);
            SWAP_DONE.store(false, Ordering::Release);
            Ok(outgoing)
        });

        match armed {
            Ok(outgoing) => Ok(Hub75Swap {
                old: outgoing as *mut FB,
                new: new_ptr,
                _marker: PhantomData,
            }),
            Err(()) => Err((Hub75Error::SwapInFlight, new_fb)),
        }
    }
}

/// A swap in flight, and the framebuffer it will hand back.
///
/// The handle is the only way to get the outgoing buffer again, which is what
/// keeps "one writer per framebuffer" true: the buffer on screen is owned by the
/// driver until this handle says otherwise.
pub struct Hub75Swap<FB: 'static> {
    /// The framebuffer being displayed, to be returned when it is safe.
    old: *mut FB,
    /// The framebuffer being switched to.
    new: *mut FB,
    _marker: PhantomData<FB>,
}

// SAFETY: both pointers come from `&'static mut FB` references, and at most one
// `Hub75Swap` exists at a time — `swap()` refuses to arm a second.
unsafe impl<FB: 'static> Send for Hub75Swap<FB> {}

impl<FB: 'static> Hub75Swap<FB> {
    /// Whether the outgoing framebuffer can be reclaimed yet.
    ///
    /// A plain read of the flag the interrupt sets at the frame boundary; it does
    /// not drive anything. (It reads like `esp-hub75`'s, and for the same
    /// reason: there the DMA needs the CPU only at boundaries, here only at
    /// runs.)
    pub fn is_done(&self) -> bool {
        SWAP_DONE.load(Ordering::Acquire)
    }

    /// Waits for the swap to complete and returns the outgoing framebuffer.
    ///
    /// Blocks for at most one frame: the interrupt applies the delta at the
    /// boundary, so the picture changes with the next run and nothing is reading
    /// `old` afterwards.
    pub fn wait(self) -> &'static mut FB {
        while !self.is_done() {
            core::hint::spin_loop();
        }
        // SAFETY: the engine's base has moved to `new` and the run that was in
        // flight has completed, so no read of `old` can begin again. `self` is
        // consumed, so `old` is handed out exactly once.
        let _ = self.new;
        unsafe { &mut *self.old }
    }

    /// Parks the caller until the swap completes.
    ///
    /// The driver's interrupt wakes it at the frame boundary, so this is the
    /// executor-friendly form of [`wait`](Self::wait): a swap completes once per
    /// frame, and the walk that owns `old` is done at that instant.
    pub async fn wait_for_done(&mut self) {
        core::future::poll_fn(|cx| {
            if self.is_done() {
                return core::task::Poll::Ready(());
            }
            WAKER.register(cx.waker());
            // Re-check: the boundary may have passed between the first read and
            // the registration.
            if self.is_done() {
                core::task::Poll::Ready(())
            } else {
                core::task::Poll::Pending
            }
        })
        .await
    }
}

#[cfg(test)]
mod tests {
    use super::SIGNAL_COUNT;
    use super::achieved_clock;
    use super::check_bus;
    use super::divider_for;
    use crate::pins;

    /// The pins of a window starting at `base`, in bus order: signal *i* on the
    /// pin its word bit is driven to.
    fn window(base: u8) -> [u8; SIGNAL_COUNT] {
        core::array::from_fn(|index| base + pins::LAYOUT[index])
    }

    /// Every run of fourteen consecutive GPIOs is a window the PIO can drive:
    /// the reference board's, another run's, and the last one that fits.
    #[test]
    fn a_window_is_accepted() {
        // The reference board: the window starts at GP0 and the clock is its
        // reserved pin.
        let reference = window(0);
        check_bus(&reference, pins::RESERVED);
        // Another run of fourteen GPIOs: the same offsets, a different base.
        let shifted = window(7);
        check_bus(&shifted, 7 + pins::RESERVED);
        // The last base a fourteen-pin window can have; its top pin is 31.
        let top = window(pins::MAX_WORD_BASE);
        check_bus(&top, pins::MAX_WORD_BASE + pins::RESERVED);
        assert_eq!(usize::from(*top.last().unwrap()), 31);
    }

    /// The clock is the one pin that may be specified apart from the window: on
    /// the reserved pin, where the word holds zero, or anywhere outside it.
    #[test]
    fn a_clock_outside_the_window_is_accepted() {
        let signals = window(0);
        // Above the window.
        check_bus(&signals, 14);
        check_bus(&signals, pins::MAX_PIN);
        // And below and above a window that does not start at zero.
        let shifted = window(7);
        check_bus(&shifted, 6);
        check_bus(&shifted, 21);
    }

    /// A signal on the wrong pin is a wiring mistake, and it stops startup
    /// rather than driving the wrong pins.
    #[test]
    #[should_panic(expected = "the signal is on the wrong pin")]
    fn a_signal_on_another_pin_is_rejected() {
        let mut numbers = window(0);
        numbers[0] = 1;
        numbers[1] = 0;
        check_bus(&numbers, pins::RESERVED);
    }

    /// A gap in the window shifts every signal past it.
    #[test]
    #[should_panic(expected = "the signal is on the wrong pin")]
    fn a_gap_in_the_window_is_rejected() {
        let mut numbers = window(0);
        numbers[5] = 9;
        check_bus(&numbers, pins::RESERVED);
    }

    /// Two signals on one pin leave another undriven, which is the same check.
    #[test]
    #[should_panic(expected = "the signal is on the wrong pin")]
    fn a_repeated_pin_is_rejected() {
        let mut numbers = window(0);
        numbers[1] = numbers[0];
        check_bus(&numbers, pins::RESERVED);
    }

    /// A window reaching past the PIO's five-bit pin fields is not drivable.
    #[test]
    #[should_panic(expected = "five bits")]
    fn a_window_past_pin_31_is_rejected() {
        check_bus(&window(pins::MAX_WORD_BASE + 1), pins::MAX_PIN);
    }

    /// The clock on a signal pin would be overwritten by the side-set on every
    /// cycle, silently, so it is refused.
    #[test]
    #[should_panic(expected = "not its reserved pin")]
    fn a_clock_on_a_signal_pin_is_rejected() {
        check_bus(&window(0), pins::OE);
    }

    /// The PIO cannot reach a pin past 31, so a clock there is refused too.
    #[test]
    #[should_panic(expected = "cannot address")]
    fn a_clock_the_pio_cannot_reach_is_rejected() {
        check_bus(&window(0), pins::MAX_PIN + 1);
    }

    /// The divider is a ceiling, not a rounding of the request to nearest.
    ///
    /// This is the examples' board: a 150 MHz system clock, against the panel
    /// clocks the `20mhz` feature chooses between. Truncating rather than rounding
    /// up would divide by 3 for the 20 MHz request and put the panel at 25 MHz.
    #[test]
    fn divider_rounds_up() {
        assert_eq!(divider_for(150_000_000, 10_000_000), 8);
        assert_eq!(divider_for(150_000_000, 20_000_000), 4);
        assert_ne!(divider_for(150_000_000, 20_000_000), 3);
        // A divider that hits exactly is kept: /4 is 37.5 MHz a word, 18.75 MHz a
        // panel clock.
        assert_eq!(divider_for(150_000_000, 18_750_000), 4);
        assert_eq!(divider_for(100_000_000, 25_000_000), 2);
        // From a slower system clock the same request needs a smaller divider.
        assert_eq!(divider_for(48_000_000, 10_000_000), 3);
    }

    /// Whatever the request, the panel ends up at that clock or slower.
    #[test]
    fn achieved_clock_never_exceeds_the_request() {
        const SYSTEM_CLOCK: u32 = 150_000_000;
        for request in [
            1_000_000, 9_375_000, 10_000_000, 18_750_000, 20_000_000, 37_500_000, 75_000_000,
        ] {
            let divider = divider_for(SYSTEM_CLOCK, request);
            let achieved = achieved_clock(SYSTEM_CLOCK, divider);
            assert!(
                achieved <= request,
                "asked for {request} Hz, the panel would run at {achieved} Hz"
            );
        }
    }

    /// A request no divider can serve is reported by the caller's assertion rather
    /// than as a panic here.
    #[test]
    fn degenerate_requests_are_numbers() {
        assert!(divider_for(150_000_000, 0) > u16::MAX as u32);
        assert!(divider_for(150_000_000, u32::MAX) <= u16::MAX as u32);
        assert_eq!(divider_for(150_000_000, u32::MAX), 1);
    }
}
