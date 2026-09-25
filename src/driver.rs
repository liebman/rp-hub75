//! The driver: one PIO state machine, one DMA channel, and an interrupt per run.
//!
//! # What runs the panel
//!
//! A single state machine streams framebuffer words straight at GP0-13 (see
//! [`crate::programs`]), fed by **one** DMA channel whose runs the DMA
//! completion interrupt re-arms:
//!
//! ```text
//!   run: len bytes at the segment's pointer, to the PIO FIFO, DREQ-paced
//!        │
//!        └── completes → IRQ → the ISR programs the next run and returns
//! ```
//!
//! The sequence of runs is the framebuffer's own BCM sequence — the very thing
//! [`FrameBuffer`] describes — walked by [`crate::engine`]:
//!
//! ```text
//! for index in 0..FB::bcm_segment_count():
//!     stream FB::bcm_segment(index) — len bytes — reps times
//! ```
//!
//! Nothing here knows what a plane or a row is: a plane-major
//! (`bitplane::plain::frame`), a row-major (`bitplane::plain::row`), a threshold
//! or a remapped framebuffer all drive the same code, with the same swap, and
//! the layout only changes the *cost* — [`Hub75::runs_per_refresh`] and
//! [`Hub75::shortest_run`] report it.
//!
//! Two registers change per run. `READ_ADDR` is written through a non-trigger
//! alias, then `TRANS_COUNT` through `AL1_TRANS_COUNT_TRIG` — whose trigger
//! register *is* `TRANS_COUNT`, so that store sets the count and starts the run
//! in one go. The channel never chains (`CHAIN_TO` is set to its own index, or
//! the reset value would hand every completion to channel 0), so the interrupt
//! is what keeps the picture moving.
//!
//! `reps` usually means one run per repetition. Where the layout allows it —
//! the region is a power of two *and* the pointer is aligned to it, which is
//! checked per run and never assumed — the driver asks the DMA's ring to repeat
//! the segment instead, and one run covers all `reps`.
//!
//! # The cost, honestly
//!
//! The interrupt is **load-bearing**: it *is* the refresh. A blocked interrupt
//! pauses the picture (the PIO holds its last word and the panel keeps its
//! latched row) rather than corrupting it, and the shortest run of a layout is
//! the deadline the interrupt has to meet — 109 µs plane-major, 3.4 µs
//! row-major on this panel. Steady state is [`Hub75::runs_per_refresh`] runs per
//! refresh: 32 for a six-plane plane-major frame (~4.6 k/s), 1024 for row-major
//! before the ring paths are counted.
//!
//! # Double buffering
//!
//! [`Hub75::swap`] takes the incoming framebuffer and remembers the *delta*
//! between it and the one on the panel: the segment offsets are layout constants
//! (`bcm_segment_ptr` is a field access), so a swap is one number, applied by
//! the interrupt at the next frame boundary. The incoming frame starts exactly
//! then, the outgoing buffer is unreferenced at that instant, and
//! [`Hub75Swap::wait`] hands it back.
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
use hal::gpio::PullNone;
use hal::gpio::PullType;
use hal::pac;
use hal::pio::Buffers;
use hal::pio::PIOBuilder;
use hal::pio::PIOExt;
use hal::pio::PinDir;
use hal::pio::ShiftDirection;
use hub75_framebuffer::FrameBuffer;

use crate::engine::Cursor;
use crate::engine::MAX_SEGMENTS;
use crate::engine::MAX_TRANSFER_WORDS;
use crate::engine::Plan;
use crate::engine::Step;
use crate::engine::WORD_BYTES;
use crate::engine::shortest_run_bytes;
use crate::pins;
use crate::programs::PULL_THRESHOLD;
use crate::programs::hub75_stream;

/// `log2` of the data channel's transfer size: 16-bit framebuffer words.
const DATA_SIZE_HALFWORD: u8 = 1;

/// `CTRL.RING_SIZE` field: bits 8-11.
const RING_SIZE_MASK: u32 = 0x0f << 8;

/// PIO cycles one word costs. The panel clock is therefore the word rate.
const CYCLES_PER_WORD: u32 = crate::programs::CYCLES_PER_WORD;

/// Pins a [`Hub75Pins`] holds: the fourteen of the HUB75 bus.
pub const PIN_COUNT: usize = pins::WORD_PINS as usize;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// How the panel is driven.
#[derive(Debug, Clone, Copy)]
pub struct Hub75Config {
    /// DMA interrupt line the application bound [`on_dma_irq`] to.
    ///
    /// Required, because the interrupt is what re-arms each run: without it the
    /// panel never advances. The pairing is the application's — this value and
    /// the `#[interrupt]` function calling [`on_dma_irq`] must name the *same*
    /// line, and the driver cannot check that (nothing in the vector table is
    /// readable that way), so a mismatch simply means a still picture.
    pub interrupt: pac::Interrupt,
    /// Panel shift-clock frequency — one word, one column, per period.
    pub panel_clock: Rate<u32, 1, 1>,
    /// Which clock edge the panel takes a column on.
    ///
    /// `false` for the usual panel, which shifts on the clock's rising edge.
    /// This only moves which edge the panel samples; the word itself always
    /// changes a cycle earlier, so the panel gets a full cycle of setup either
    /// way (see [`crate::programs`]).
    pub invert_clock: bool,
}

impl Hub75Config {
    /// The recommended configuration: an 18.75 MHz panel clock, the usual clock
    /// edge, and `interrupt` as the wake-up line.
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
            invert_clock: false,
        }
    }

    /// Sets the panel shift clock. See [`Hub75Config::new`].
    #[must_use]
    pub const fn with_panel_clock(mut self, panel_clock: Rate<u32, 1, 1>) -> Self {
        self.panel_clock = panel_clock;
        self
    }

    /// Selects the clock edge the panel takes a column on.
    #[must_use]
    pub const fn with_invert_clock(mut self, invert_clock: bool) -> Self {
        self.invert_clock = invert_clock;
        self
    }
}

/// What can go wrong.
///
/// There is deliberately not much: the only failure modes are the API being used
/// out of order, or a framebuffer whose layout this driver cannot cache.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hub75Error {
    /// A driver already exists. There is one PIO, one channel and one engine,
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

/// The fourteen HUB75 pins, converted and owned by the driver.
///
/// The constructor takes the concrete `Pin<GpioN, …>` types, so handing over
/// the wrong pin number is a compile error rather than a dark panel, and the
/// handles are kept so nothing else can claim the bus. `Pins::new` yields pins
/// in `FunctionNull`; this does the `into_function` step.
pub struct Hub75Pins {
    /// GP0-13 in bus order: `R0 G0 B0 R1 G1 B1 A B C D E CLK STB OE`.
    hold: [hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>; PIN_COUNT],
}

impl Hub75Pins {
    /// Converts the panel's fourteen pins to an owned pin set.
    ///
    /// The order is the HUB75 bus: top half (`R0 G0 B0`), bottom half
    /// (`R1 G1 B1`), row address (`A B C D E`), then `CLK STB OE`. It is
    /// [`crate::pins`] as a signature.
    #[allow(clippy::too_many_arguments)]
    pub fn new<P: PullType>(
        r0: hal::gpio::Pin<hal::gpio::bank0::Gpio0, FunctionNull, P>,
        g0: hal::gpio::Pin<hal::gpio::bank0::Gpio1, FunctionNull, P>,
        b0: hal::gpio::Pin<hal::gpio::bank0::Gpio2, FunctionNull, P>,
        r1: hal::gpio::Pin<hal::gpio::bank0::Gpio3, FunctionNull, P>,
        g1: hal::gpio::Pin<hal::gpio::bank0::Gpio4, FunctionNull, P>,
        b1: hal::gpio::Pin<hal::gpio::bank0::Gpio5, FunctionNull, P>,
        a: hal::gpio::Pin<hal::gpio::bank0::Gpio6, FunctionNull, P>,
        b: hal::gpio::Pin<hal::gpio::bank0::Gpio7, FunctionNull, P>,
        c: hal::gpio::Pin<hal::gpio::bank0::Gpio8, FunctionNull, P>,
        d: hal::gpio::Pin<hal::gpio::bank0::Gpio9, FunctionNull, P>,
        e: hal::gpio::Pin<hal::gpio::bank0::Gpio10, FunctionNull, P>,
        clk: hal::gpio::Pin<hal::gpio::bank0::Gpio11, FunctionNull, P>,
        stb: hal::gpio::Pin<hal::gpio::bank0::Gpio12, FunctionNull, P>,
        oe: hal::gpio::Pin<hal::gpio::bank0::Gpio13, FunctionNull, P>,
    ) -> Self {
        // The panel pins are outputs, so their pull setting is irrelevant; it is
        // normalized away so the driver does not need a type parameter for it.
        fn hold<I, P>(
            pin: hal::gpio::Pin<I, FunctionNull, P>,
        ) -> hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>
        where
            I: hal::gpio::PinId + hal::gpio::ValidFunction<FunctionPio0>,
            P: PullType,
        {
            let pin: hal::gpio::Pin<I, FunctionPio0, PullNone> =
                pin.into_pull_type::<PullNone>().into_function();
            pin.into_dyn_pin()
        }
        Self {
            hold: [
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
                hold(clk),
                hold(stb),
                hold(oe),
            ],
        }
    }

    /// The GPIO numbers held, in bus order (`pins::R0` … `pins::OE`).
    pub fn numbers(&self) -> [u8; PIN_COUNT] {
        core::array::from_fn(|index| self.hold[index].id().num)
    }
}

// ---------------------------------------------------------------------------
// The engine: everything the interrupt needs, without a framebuffer type
// ---------------------------------------------------------------------------

/// The engine's state.
///
/// Everything here is deliberately *data*: the interrupt cannot name the
/// framebuffer's type, so the sequence, the segment offsets and the channel's
/// register addresses are all captured at init and walked afterwards.
struct Engine {
    /// The framebuffer's BCM sequence, copied from the crate.
    plan: Plan,
    /// Byte offsets of every segment, relative to the framebuffer's base.
    ///
    /// Offsets, not pointers, because both framebuffers of a double-buffered
    /// driver have the same type and therefore the same layout —
    /// `bcm_segment_ptr` is a field access — so a swap is one delta added to the
    /// base. `engine.rs`'s tests check the property that makes it valid (every
    /// segment lies inside the framebuffer) for both crate layouts.
    offsets: [i32; MAX_SEGMENTS],
    /// The framebuffer being scanned.
    base: u32,
    /// A swap waiting for the next frame boundary: the delta to the incoming
    /// framebuffer.
    pending: Option<i32>,
    /// Where the walk is.
    cursor: Cursor,
    /// Whether the run just issued asked the DMA's ring to repeat the segment,
    /// so the next advance knows its `reps` are already satisfied.
    ringed: bool,
    /// The data channel's `READ_ADDR`, through the alias whose trigger is
    /// `TRANS_COUNT`, so writing it starts nothing.
    read_addr: *mut u32,
    /// The alias whose trigger register is `TRANS_COUNT`: one store sets the
    /// count and starts the run.
    trans_count_trig: *mut u32,
    /// That alias's `CTRL`, for `RING_SIZE` — a plain register, never a trigger.
    ctrl: *mut u32,
    /// This channel's bit in `INTR`/`INTEn`, and the line it is routed to.
    flag: u32,
    irq_line: u8,
}

// SAFETY (`Send`): required for the static below to be `Sync`. Every access is
// serialized by the critical section, and the register pointers are only
// dereferenced inside it.
unsafe impl Send for Engine {}

impl Engine {
    /// Nothing running. A driver fills this in place, so the 1.3 KB of offsets
    /// never passes through the stack.
    const EMPTY: Self = Self {
        plan: Plan::EMPTY,
        offsets: [0; MAX_SEGMENTS],
        base: 0,
        pending: None,
        cursor: Cursor::new(),
        ringed: false,
        read_addr: core::ptr::null_mut(),
        trans_count_trig: core::ptr::null_mut(),
        ctrl: core::ptr::null_mut(),
        flag: 0,
        irq_line: 0,
    };

    /// Programs the current segment and starts it.
    fn program(&mut self) {
        let offset = self.offsets[self.cursor.index()];
        let run = self.cursor.run(&self.plan, offset as u32);
        let address = self.base.wrapping_add(offset as u32);
        let ring_size = if run.ring_bytes == 0 {
            0
        } else {
            run.ring_bytes.trailing_zeros()
        };

        // SAFETY: these are the data channel's own registers, captured at init
        // and used only from here.
        unsafe {
            // `RING_SIZE` through the alias whose trigger is `TRANS_COUNT`:
            // writing `CTRL` there reconfigures the channel without starting it.
            let ctrl = self.ctrl.read_volatile();
            self.ctrl
                .write_volatile((ctrl & !RING_SIZE_MASK) | (ring_size << 8));
            self.read_addr.write_volatile(address);
            // The trigger: this store sets `TRANS_COUNT` and starts the run.
            self.trans_count_trig.write_volatile(run.words);
        }
        self.ringed = run.ring_bytes != 0;
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

/// The DMA completion handler: advance the walk, apply a swap at the frame
/// boundary, and start the next run.
///
/// This *is* the refresh — it is called once per run, and nothing else re-arms
/// the channel — so it is deliberately short: an acknowledge, a cursor step, and
/// three register stores.
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
        if engine.plan.count() == 0 {
            // No driver, or an interrupt from a neighbour on the same line.
            return false;
        }

        // Acknowledge *our* channel's completion. `INTR` is write-1-to-clear and
        // is not masked by `INTEn`, so this cannot disturb another channel.
        // SAFETY: the DMA block is a static peripheral.
        unsafe { (*pac::DMA::ptr()).intr().write(|w| w.bits(engine.flag)) };

        let ringed = engine.ringed;
        let boundary = engine.cursor.advance(&engine.plan, ringed) == Step::FrameBoundary;

        let mut swapped = false;
        if boundary {
            if let Some(delta) = engine.pending.take() {
                // The frame boundary is the one instant where the incoming
                // framebuffer can take over without tearing: the outgoing frame
                // is done, and the new one starts with the next run.
                engine.base = engine.base.wrapping_add(delta as u32);
                SWAP_DONE.store(true, Ordering::Release);
                swapped = true;
            }
        }

        engine.program();
        swapped
    });

    if swapped {
        // Wake outside the critical section: `wake()` runs executor code.
        WAKER.wake();
    }
}

// ---------------------------------------------------------------------------
// The driver
// ---------------------------------------------------------------------------

/// Configures the one channel that streams the panel.
///
/// Written once, and never again: the interrupt changes only `READ_ADDR`,
/// `TRANS_COUNT` and `RING_SIZE`, through the aliases that do not trigger.
fn new_channel<CH: SingleChannel>(channel: &CH, fifo: u32, dreq: u8) {
    // SAFETY: the channel belongs to this driver from here on, and these writes
    // are the only configuration it ever gets.
    unsafe {
        let ch = channel.ch();
        ch.ch_write_addr().write(|w| w.bits(fifo));
        // `CTRL` through the alias whose trigger is `TRANS_COUNT`, so this
        // configures without starting anything: 16-bit words, walk the read
        // address, no ring by default, paced by the state machine, and one
        // interrupt per run (the run *is* the tick).
        ch.ch_al1_ctrl().write(|w| {
            w.data_size().bits(DATA_SIZE_HALFWORD);
            w.incr_read().set_bit();
            w.incr_write().clear_bit();
            w.ring_sel().clear_bit();
            w.ring_size().bits(0);
            w.treq_sel().bits(dreq);
            // A channel that chains to itself does not chain. The reset value is
            // 0, which would hand every completion to channel 0 instead.
            w.chain_to().bits(channel.id());
            w.irq_quiet().clear_bit();
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

/// A HUB75 panel, driven by hardware and an interrupt.
///
/// Created once — there is one PIO, one channel and one engine — and then
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
    /// Takes the PIO block, **one** DMA channel, the pins, the system clock the
    /// PIO divider is derived from, the configuration (which names the interrupt
    /// line the application bound [`on_dma_irq`] to), and the first framebuffer
    /// to display. From here the panel refreshes itself; nothing else needs
    /// calling, ever.
    ///
    /// `fb` is taken by exclusive reference, and the first [`swap`](Self::swap)
    /// is what hands it back — so the buffer on screen and the buffer being
    /// drawn are never the same one, by construction.
    ///
    /// # Panics
    ///
    /// Panics if the framebuffer needs more segments than the engine can cache
    /// (`engine::MAX_SEGMENTS`), or if its sequence has no segments. Both are
    /// properties of the layout, and both are better discovered at startup than
    /// as a frozen panel.
    #[allow(clippy::too_many_arguments)]
    pub fn new<CH>(
        pio: pac::PIO0,
        channel: CH,
        resets: &mut pac::RESETS,
        pins: Hub75Pins,
        system_clock: Rate<u32, 1, 1>,
        config: Hub75Config,
        fb: &'static mut FB,
    ) -> Result<Self, Hub75Error>
    where
        CH: SingleChannel,
    {
        if DRIVER_TAKEN.swap(true, Ordering::AcqRel) {
            return Err(Hub75Error::AlreadyInitialised);
        }
        let irq_line = match line_index(config.interrupt) {
            Some(line) => line,
            None => return Err(Hub75Error::NotADmaIrq),
        };

        // A word every `CYCLES_PER_WORD` PIO cycles, so the divider is the ratio
        // between the system clock and that many times the panel clock.
        let word_clock = config.panel_clock.to_Hz() * CYCLES_PER_WORD;
        let divider = system_clock.to_Hz() / word_clock;
        assert!(
            divider >= 1 && divider <= u16::MAX as u32,
            "the panel clock is not representable: divider {divider} is outside \
             the PIO's 16-bit range (system clock {} Hz, panel clock {} Hz)",
            system_clock.to_Hz(),
            config.panel_clock.to_Hz()
        );
        let divider = divider as u16;
        let achieved =
            Rate::<u32, 1, 1>::from_raw(system_clock.to_Hz() / divider as u32 / CYCLES_PER_WORD);

        let (mut pio, sm0, _, _, _) = pio.split(resets);

        // The board's wiring, as the driver understands it. The constructor's
        // signature already pins each argument to a GPIO number; this checks the
        // order it means, against `pins.rs`, before anything is driven.
        assert_eq!(
            pins.numbers(),
            [
                pins::R0,
                pins::G0,
                pins::B0,
                pins::R1,
                pins::G1,
                pins::B1,
                pins::ROW_A,
                pins::ROW_B,
                pins::ROW_C,
                pins::ROW_D,
                pins::ROW_E,
                pins::CLK,
                pins::STB,
                pins::OE,
            ],
            "Hub75Pins must be handed over in HUB75 bus order"
        );

        let program = pio
            .install(&hub75_stream(config.invert_clock))
            .expect("the stream program fits the instruction memory");
        let (mut sm, _, tx) = PIOBuilder::from_installed_program(program)
            .out_pins(pins::WORD_BASE, pins::WORD_PINS)
            .side_set_pin_base(pins::CLK)
            .clock_divisor_fixed_point(divider, 0)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(PULL_THRESHOLD)
            .buffers(Buffers::OnlyTx)
            .build(sm0);

        // Every pin of the word is an output: the machine drives the whole bus,
        // clock included.
        sm.set_pindirs(
            (0..pins::WORD_PINS).map(|offset| (pins::WORD_BASE + offset, PinDir::Output)),
        );
        // Keep the machine running. The handle has no `Drop`, so dropping it
        // here would be safe; holding it documents who owns the machine.
        let _sm = sm.start();

        new_channel(&channel, tx.fifo_address() as u32, tx.dreq_value());
        Ok(Self::start(
            &channel,
            pins,
            divider,
            achieved,
            config.interrupt,
            irq_line,
            fb,
        ))
    }
}

impl<FB: FrameBuffer<Word = u16> + 'static> Hub75<FB> {
    /// Builds the engine on `fb` and starts the walk.
    ///
    /// Split out of [`new`](Self::new) so that one function does the peripheral
    /// setup and this one does the engine — and so that the 1.3 KB offset table
    /// is filled in place in the static rather than passed around.
    #[allow(clippy::too_many_arguments)]
    fn start(
        channel: &impl SingleChannel,
        pins: Hub75Pins,
        divider: u16,
        panel_clock: Rate<u32, 1, 1>,
        interrupt: pac::Interrupt,
        irq_line: u8,
        fb: &'static mut FB,
    ) -> Self {
        // The framebuffer's own description of one refresh.
        let plan = Plan::of::<FB>();
        assert!(plan.count() > 0, "the framebuffer's BCM sequence is empty");
        assert!(
            plan.count() <= MAX_SEGMENTS,
            "{} segments exceeds engine::MAX_SEGMENTS ({MAX_SEGMENTS}); raise it \
             for this layout",
            plan.count()
        );
        let frame_words = (plan.bytes_per_refresh() / WORD_BYTES) as u32;
        assert!(
            frame_words <= MAX_TRANSFER_WORDS,
            "a refresh of {frame_words} words does not fit a transfer count"
        );
        let runs = plan.runs_per_refresh() as u32;
        assert!(
            shortest_run_bytes::<FB>() >= WORD_BYTES,
            "the framebuffer has a zero-length segment, which would stall the engine"
        );
        let base = fb as *const FB as u32;
        let channel_id = channel.id();

        // SAFETY: the channel's registers live for the rest of the program, and
        // the engine's interrupt reads them from here on. `as_ptr` is safe; the
        // pointers it returns are only dereferenced in the interrupt.
        let (read_addr, trans_count_trig, ctrl) = {
            let ch = channel.ch();
            (
                ch.ch_al1_read_addr().as_ptr(),
                ch.ch_al1_trans_count_trig().as_ptr(),
                ch.ch_al1_ctrl().as_ptr(),
            )
        };

        critical_section::with(|cs| {
            let mut engine = ENGINE.borrow_ref_mut(cs);
            let engine = &mut *engine;
            engine.plan = plan;
            engine.base = base;
            engine.pending = None;
            engine.cursor = Cursor::new();
            engine.ringed = false;
            engine.read_addr = read_addr;
            engine.trans_count_trig = trans_count_trig;
            engine.ctrl = ctrl;
            engine.flag = 1 << channel_id;
            engine.irq_line = irq_line;
            for index in 0..plan.count() {
                engine.offsets[index] = fb.bcm_segment_ptr(index) as u32 as i32 - base as i32;
            }
        });
        SWAP_DONE.store(false, Ordering::Release);

        // Arm the interrupt, then start the walk. The order matters: the
        // completion of the first run is what programs the second.
        arm(1 << channel_id, irq_line);
        // SAFETY: the application bound `on_dma_irq` to this line.
        unsafe { hal::arch::interrupt_unmask(interrupt) };
        critical_section::with(|cs| ENGINE.borrow_ref_mut(cs).program());

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

    /// Runs the interrupt must service per refresh.
    ///
    /// The honest cost of the framebuffer's layout: 32 for a six-plane
    /// plane-major frame, `32 × NROWS` for a row-major one, before any segment
    /// the DMA's ring can repeat on its own. Multiply by [`refresh_hz`] for the
    /// interrupt rate.
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
            if engine.plan.count() == 0 || engine.pending.is_some() {
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
