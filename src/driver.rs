//! The driver: one PIO state machine, two DMA channels, and one interrupt a
//! frame.
//!
//! # What runs the panel
//!
//! A single state machine streams framebuffer words straight at the HUB75 pin
//! window (see [`crate::backend`]). A **control** DMA channel plays a
//! [`crate::descriptor`] table into the data channel's `AL3` register block, and
//! the data channel chains back to it on every completion, so the two hand the
//! picture to each other in hardware:
//!
//! ```text
//! control channel  (trans_count, read_addr) → the data channel's AL3 block
//!                  the read address starts the run
//! data channel     streams the run to the PIO FIFO, paced by its DREQ
//!                  completes, chains to the control channel
//! ```
//!
//! The table is the framebuffer's own BCM sequence, materialised by
//! [`crate::descriptor::fill`]. Plane-major, row-major, threshold and remapped
//! framebuffers all drive the same code, with the layout only changing the *cost*,
//! which [`Hub75::runs_per_refresh`] reports. [`crate::descriptor`] has the table's
//! shape, the terminator that gives one interrupt a frame, and the delta a swap
//! applies.
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
//! The handler *is* the vector-table symbol: `rp235x-hal` does not re-export
//! `cortex_m_rt::interrupt`, and that macro in device mode wants a crate named
//! `interrupt` this PAC does not provide. The line is named in
//! [`Hub75Config::new`] — the only way the driver knows which one to unmask; a
//! mismatch means nothing moves.

use core::cell::Cell;
use core::cell::RefCell;
use core::marker::PhantomData;
use core::task::Waker;

use portable_atomic::AtomicBool;
use portable_atomic::Ordering;

use crate::backend;
use crate::hal;
use critical_section::Mutex;

use fugit::Rate;
use hal::pac;
use hub75_framebuffer::FrameBuffer;

use crate::descriptor::BusWord;
use crate::descriptor::DESCRIPTOR_WORDS;
use crate::descriptor::Descriptor;
use crate::descriptor::Hub75Descriptors;
use crate::descriptor::descriptor_count;

/// PIO cycles one word costs. The panel clock is therefore the word rate.
const CYCLES_PER_WORD: u32 = 2;

/// The PIO divider for a requested panel clock, rounded **up**.
///
/// Rounding up rather than truncating matters because a panel's clock rating is a
/// ceiling: the divider has to hold the word clock at or below the clock that was
/// asked for, and truncating overshoots it — 20 MHz from a 150 MHz system clock
/// would divide by 3 rather than 4 and run the panel at 25 MHz. Integer dividers
/// are what the PIO is given here, so a request no divider hits exactly is met
/// with the next slower clock; [`Hub75::panel_clock`] reports what was achieved.
pub(crate) fn divider_for(system_clock_hz: u32, panel_clock_hz: u32) -> u32 {
    // Saturating and at least one, so a degenerate request reaches the caller's
    // assertion as a number rather than as a division by zero.
    let word_clock_hz = panel_clock_hz.saturating_mul(CYCLES_PER_WORD).max(1);
    system_clock_hz.div_ceil(word_clock_hz)
}

/// Panel shift clock a divider achieves: the system clock divided by the divider
/// and the cycles one word takes.
pub(crate) fn achieved_clock(system_clock_hz: u32, divider: u32) -> u32 {
    system_clock_hz / divider / CYCLES_PER_WORD
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
    /// as the wake-up line, which is 150 MHz / 4 / 2 — a word every two PIO
    /// cycles, and a refresh rate that follows the framebuffer's own sequence
    /// ([`Hub75::refresh_hz`]).
    #[must_use]
    pub const fn new(interrupt: pac::Interrupt) -> Self {
        Self {
            interrupt,
            panel_clock: Rate::<u32, 1, 1>::kHz(18_750),
        }
    }

    /// Sets the panel shift clock. See [`Hub75Config::new`].
    ///
    /// The clock is a ceiling: the PIO divider is rounded up, so the panel runs at
    /// the requested clock or the next slower one ([`Hub75::panel_clock`] reports
    /// which).
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

/// A HUB75 bus the driver can stream: the framebuffer word it carries and the
/// program that drives it.
///
/// A [`Hub75`] is generic over its pin set and requires the set's
/// [`Word`](Self::Word) to equal the framebuffer's, so 8-bit pins with a 16-bit
/// framebuffer is a compile error rather than a scrambled picture. The pin set
/// also decides the program and the window width, which is what keeps `out pins,
/// N` and the word width in step.
pub trait Hub75Pins: 'static {
    /// The framebuffer word width this bus carries.
    type Word: BusWord;

    /// The stream program and the window width it writes.
    fn stream() -> crate::backend::Stream;

    /// The GPIO number the window's first pin is on.
    fn base(&self) -> u8;

    /// The GPIO number the panel clock is wired to.
    fn clock(&self) -> u8;

    /// The window's signal pins, in bus order.
    fn signal_pins(&self) -> impl Iterator<Item = u8> + '_;

    /// Inverts the clock pin's output in the GPIO block.
    ///
    /// A panel that takes the column on the clock's falling edge gets it here; the
    /// pad inverts whatever the PIO drives on that pin.
    #[cfg(feature = "invert-clock")]
    fn invert_clock(&mut self);
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

/// The interrupt line index for a `DMA_IRQ_n`.
///
/// The RP2350 has four DMA interrupt lines and the RP2040 two, so lines 2 and 3
/// exist only in the `rp2350` build; an application cannot name them on the
/// RP2040.
fn line_index(irq: pac::Interrupt) -> Option<u8> {
    match irq {
        pac::Interrupt::DMA_IRQ_0 => Some(0),
        pac::Interrupt::DMA_IRQ_1 => Some(1),
        #[cfg(feature = "rp2350")]
        pac::Interrupt::DMA_IRQ_2 => Some(2),
        #[cfg(feature = "rp2350")]
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
/// running it touches nothing, and a completion from a neighbouring channel that
/// shares the line is ignored rather than stepping the frame.
pub fn on_dma_irq() {
    let swapped = critical_section::with(|cs| {
        let mut engine = ENGINE.borrow_ref_mut(cs);
        let engine = &mut *engine;
        if engine.table_count == 0 {
            // No driver running.
            return false;
        }

        // Only *our* channel's completion may step the frame. A neighbour on the
        // same line must not restart the chain mid-frame, so a completion that is
        // not the data channel's is left for its owner. `INTR` is not masked by
        // `INTEn`, so the bit is readable whether or not the neighbour armed the
        // line for its own flag.
        if !backend::intr_pending(engine.irq_line, engine.flag) {
            return false;
        }

        // Acknowledge *our* channel's completion. `INTR` is write-1-to-clear and
        // is not masked by `INTEn`, so this cannot disturb another channel.
        backend::clear_intr(engine.irq_line, engine.flag);

        // Nothing here may log. defmt's `acquire` is a critical section, and this
        // runs once a frame with the picture waiting on it: a logger that blocks,
        // or a deadlock against a log in thread context, would freeze the panel.
        // The library does not log at all; the examples log from their own loops.
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

/// Unmasks the DMA interrupt line the driver watches.
///
/// Arm only: the host, where the library's unit tests run, has no NVIC and runs
/// no panel, so there the call is a no-op that only has to type-check. On both
/// the RP2040 and the RP2350 the PAC's `Interrupt` implements `InterruptNumber`,
/// which is why the call is the same on both.
#[cfg(target_arch = "arm")]
fn unmask_interrupt(irq: pac::Interrupt) {
    // SAFETY: the application bound `on_dma_irq` to this line.
    unsafe { cortex_m::peripheral::NVIC::unmask(irq) }
}

#[cfg(not(target_arch = "arm"))]
fn unmask_interrupt(_irq: pac::Interrupt) {}

/// A HUB75 panel, driven by hardware and one interrupt a frame.
///
/// Created once — there is one PIO, two DMA channels and one table — and then
/// refreshes itself: the CPU only writes pixels and arms the occasional swap.
pub struct Hub75<FB: FrameBuffer + 'static, P: Hub75Pins> {
    /// PIO clock divider actually used, for reporting.
    divider: u16,
    /// Panel clock actually achieved: the request rounded to a divider.
    panel_clock: Rate<u32, 1, 1>,
    /// Words streamed per refresh, from the framebuffer's own sequence.
    frame_words: u32,
    /// Runs the interrupt services per refresh.
    runs: u32,
    /// The panel pins, held so nothing else can claim the bus.
    pins: P,
    _marker: PhantomData<FB>,
}

impl<FB, P> Hub75<FB, P>
where
    FB: FrameBuffer + 'static,
    P: Hub75Pins<Word = FB::Word>,
    FB::Word: BusWord,
{
    /// Reserves the one driver and resolves the interrupt line, before any
    /// hardware is touched.
    ///
    /// The backend constructor calls this first, so a second driver is refused
    /// before the PIO is split or a state machine disturbed.
    ///
    /// # Errors
    ///
    /// [`Hub75Error::AlreadyInitialised`] if a driver already exists, and
    /// [`Hub75Error::NotADmaIrq`] if `config.interrupt` is not a DMA line.
    pub(crate) fn reserve(config: Hub75Config) -> Result<u8, Hub75Error> {
        // Resolve the line before claiming the driver, so a configuration the
        // caller can correct does not consume the one driver for good.
        let line = line_index(config.interrupt).ok_or(Hub75Error::NotADmaIrq)?;
        if DRIVER_TAKEN.swap(true, Ordering::AcqRel) {
            return Err(Hub75Error::AlreadyInitialised);
        }
        Ok(line)
    }

    /// The shared tail of the backend constructors: bind the channels to the data
    /// channel's `AL3` block and start the engine.
    ///
    /// Each backend's `new` brings up its own PIO and calls this with the result,
    /// so the differing peripheral shapes never reach the engine.
    ///
    /// # Panics
    ///
    /// Panics if a framebuffer segment is empty (a zero-word run would end the
    /// frame early), or if the framebuffer's BCM sequence is empty. Both are
    /// properties of the layout, and both are better discovered at startup than as
    /// a frozen panel.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn start_from<CH, CCH, const N: usize>(
        channel: CH,
        control: CCH,
        bringup: backend::BringUp<P>,
        config: Hub75Config,
        irq_line: u8,
        fb: &'static mut FB,
        descriptors: &'static mut Hub75Descriptors<FB, N>,
    ) -> Result<Self, Hub75Error>
    where
        CH: backend::ChannelSetup,
        CCH: backend::ChannelSetup,
    {
        let backend::BringUp {
            pins,
            divider,
            panel_clock,
            fifo,
            dreq,
        } = bringup;

        // The control channel writes the data channel's `AL3` block: its last
        // word is that block's trigger, so a descriptor is the two words before
        // it and the ring keeps both stores landing there.
        let target = channel.al3_trans_count();
        control.program_control(target);
        channel.program_data(
            control.id(),
            fifo,
            dreq,
            <FB::Word as BusWord>::DATA_SIZE_LOG2,
        );

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

impl<FB, P> Hub75<FB, P>
where
    FB: FrameBuffer + 'static,
    P: Hub75Pins<Word = FB::Word>,
    FB::Word: BusWord,
{
    /// Materialises the frame in the caller's descriptor storage and starts it.
    ///
    /// Split out of the backend constructor so that one function does the
    /// peripheral setup and this one the engine. Init is the only writer of the
    /// table until the interrupt is unmasked, and the interrupt is the only writer
    /// afterwards.
    #[allow(clippy::too_many_arguments)]
    fn start<const N: usize>(
        channel: &impl backend::ChannelSetup,
        control: &impl backend::ChannelSetup,
        pins: P,
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

        let (control_read_addr, control_trigger) = control.trigger_pointers();

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
        backend::arm(1 << channel_id, irq_line);
        unmask_interrupt(interrupt);
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
    /// One entry per repetition of every segment, so this is the layout's cost and
    /// not the panel's ([`crate::descriptor`] has the shape). The interrupts a
    /// second are [`refresh_hz`], because the table's terminator is the only thing
    /// that interrupts.
    ///
    /// [`refresh_hz`]: Self::refresh_hz
    pub fn runs_per_refresh(&self) -> u32 {
        self.runs
    }

    /// The panel pins the driver holds.
    pub fn pins(&self) -> &P {
        &self.pins
    }
}

// ---------------------------------------------------------------------------
// Swapping framebuffers
// ---------------------------------------------------------------------------

impl<FB, P> Hub75<FB, P>
where
    FB: FrameBuffer + 'static,
    P: Hub75Pins<Word = FB::Word>,
    FB::Word: BusWord,
{
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
// `Hub75Swap` exists at a time — `swap()` refuses to arm a second. The handle
// moves the framebuffer across threads, so it is `Send` only for a `Send` one.
unsafe impl<FB: Send + 'static> Send for Hub75Swap<FB> {}

impl<FB: 'static> Hub75Swap<FB> {
    /// Whether the outgoing framebuffer can be reclaimed yet.
    ///
    /// A plain read of the flag the interrupt sets at the frame boundary; it does
    /// not drive anything.
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
    use super::achieved_clock;
    use super::divider_for;

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

    /// A corrected configuration can still start after a bad one: a failed
    /// reservation does not consume the one driver.
    #[test]
    fn a_bad_interrupt_does_not_consume_the_driver() {
        use super::Hub75Config;
        use super::Hub75Error;
        use crate::Hub75;
        use crate::Hub75Pins16;
        use crate::hal::pac::Interrupt;
        use hub75_framebuffer::bitplane::plain::frame;
        use hub75_framebuffer::compute_rows;

        type Fb = frame::DmaFrameBuffer<{ compute_rows(64) }, 64, 6>;

        assert_eq!(
            Hub75::<Fb, Hub75Pins16>::reserve(Hub75Config::new(Interrupt::TIMER0_IRQ_0)),
            Err(Hub75Error::NotADmaIrq)
        );
        assert_eq!(
            Hub75::<Fb, Hub75Pins16>::reserve(Hub75Config::new(Interrupt::DMA_IRQ_1)),
            Ok(1)
        );
    }
}
