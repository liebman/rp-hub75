//! The `rp2040-hal` / `rp235x-hal` port.
//!
//! The two `rp-hal` parts share `rp-hal-common` and the same register-block shape,
//! so one implementation covers both. The pieces whose shape differs from
//! embassy's are here — the DMA accessors, the pin sets and their `into_function`
//! conversion, and the PIO bring-up — behind names the shared driver uses. The
//! embassy port names them identically.

use super::BringUp;
use super::ChannelSetup;
use super::hal;
use super::programs::stream_for;
use crate::descriptor::BusWord;
use crate::descriptor::Hub75Descriptors;
use crate::driver::Hub75;
use crate::driver::Hub75Config;
use crate::driver::Hub75Error;
use crate::driver::Hub75Pins;
use crate::driver::achieved_clock;
use crate::driver::divider_for;
use hal::dma::SingleChannel;
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

/// Configures `channel` as the data channel.
///
/// Written once, and never touched again — every descriptor the control channel
/// plays rewrites `READ_ADDR` and `TRANS_COUNT`, and nothing rewrites `CTRL`. See
/// [`super::ChannelSetup::program_data`] for what the arguments mean.
pub(super) fn program_data<C: SingleChannel>(
    channel: &C,
    control: u8,
    fifo: u32,
    dreq: u8,
    data_size_log2: u8,
) {
    // SAFETY: the channel belongs to this driver from here on, and these writes
    // are the only configuration it ever gets.
    unsafe {
        let ch = channel.ch();
        ch.ch_write_addr().write(|w| w.bits(fifo));
        // `CTRL` through the alias whose trigger is `TRANS_COUNT`, so this
        // configures without starting anything: the framebuffer's word size,
        // walk the read address, no ring, paced by the state machine.
        ch.ch_al1_ctrl().write(|w| {
            w.data_size().bits(data_size_log2);
            w.incr_read().set_bit();
            w.incr_write().clear_bit();
            w.ring_sel().clear_bit();
            w.ring_size().bits(0);
            w.treq_sel().bits(dreq);
            w.chain_to().bits(control);
            w.irq_quiet().set_bit();
            w.en().set_bit();
            w
        });
    }
}

/// Configures `channel` as the control channel.
///
/// `target` is the data channel's `AL3_TRANS_COUNT`. The channel walks 32-bit
/// words from the table, and wraps its write pointer on an 8-byte boundary so
/// that every block lands on the same two registers. Nothing paces it: it runs
/// as fast as the bus allows, once per run the data channel completes.
pub(super) fn program_control<C: SingleChannel>(channel: &C, target: u32) {
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

/// The channel's `(AL1_READ_ADDR, AL1_TRANS_COUNT_TRIG)`.
///
/// See [`super::ChannelSetup::trigger_pointers`]: the addresses live for the rest
/// of the program, and `as_ptr` is safe; the pointers are only dereferenced in
/// the interrupt.
pub(super) fn trigger_pointers<C: SingleChannel>(channel: &C) -> (*mut u32, *mut u32) {
    let ch = channel.ch();
    (
        ch.ch_al1_read_addr().as_ptr(),
        ch.ch_al1_trans_count_trig().as_ptr(),
    )
}

/// The channel's `AL3_TRANS_COUNT`: the control channel's write target.
///
/// See [`super::ChannelSetup::al3_trans_count`].
pub(super) fn al3_trans_count<C: SingleChannel>(channel: &C) -> u32 {
    channel.ch().ch_al3_trans_count().as_ptr() as u32
}

// ---------------------------------------------------------------------------
// The pin window and the two pin sets
// ---------------------------------------------------------------------------

use super::window::SIGNAL_COUNT;
use super::window::SIGNAL_COUNT8;
use super::window::check_bus;
use super::window::check_bus8;

/// A GPIO the PIO can drive.
///
/// [`Hub75Pins16::new`] takes the bus pins as concrete types, so this bound is
/// what makes handing over a pin the PIO cannot reach a compile error. The pins'
/// GPIO *numbers* are checked at startup instead: each signal must sit on the
/// window pin its word bit is shifted to, and the whole window must fit the
/// PIO's pin fields.
pub trait PioPin: hal::gpio::PinId + hal::gpio::ValidFunction<FunctionPio0> {}

impl<P> PioPin for P where P: hal::gpio::PinId + hal::gpio::ValidFunction<FunctionPio0> {}

/// The HUB75 pins of a **16-bit direct-drive** bus, converted and owned by the
/// driver.
///
/// The constructor takes the thirteen signals in bus order — `R0 G0 B0 R1 G1 B1
/// A B C D E STB OE` — and the clock separately, because the clock is the one
/// pin the PIO generates rather than copies out of a framebuffer word. It checks
/// that the signals are a fourteen-pin window with each one on the pin its word
/// bit is driven to, and that the clock is either that window's reserved pin or
/// a pin outside it. The handles are kept so nothing else can claim the bus.
/// `Pins::new` yields pins in `FunctionNull`; this does the `into_function` step.
pub struct Hub75Pins16 {
    /// The GPIO number the fourteen-pin window starts at.
    base: u8,
    /// The window's signal pins, in bus order.
    hold: [hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>; SIGNAL_COUNT],
    /// The panel clock, wherever it is wired.
    clock: hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>,
}

/// The HUB75 pins of an **8-bit latched** bus, converted and owned by the driver.
///
/// The constructor takes the eight signals in bus order — `R1 G1 B1 R2 G2 B2 LAT
/// OE` — and the clock separately. An 8-bit window has no reserved slot, so the
/// clock must be wired outside it. The handles are kept so nothing else can claim
/// the bus.
pub struct Hub75Pins8 {
    /// The GPIO number the eight-pin window starts at.
    base: u8,
    /// The window's signal pins, in bus order.
    hold: [hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>; SIGNAL_COUNT8],
    /// The panel clock, wherever it is wired.
    clock: hal::gpio::Pin<DynPinId, FunctionPio0, PullNone>,
}

impl Hub75Pins16 {
    /// Converts the panel's pins to an owned pin set.
    ///
    /// The thirteen signals are in bus order — [`crate::pins::LAYOUT`] — with the
    /// clock last: the window pin [`crate::pins::RESERVED`], where the word holds
    /// zero, or a pin outside the window. The lowest signal pin is the window's base,
    /// which [`base`](Self::base) reports.
    ///
    /// # Panics
    ///
    /// Panics if a signal is not on the window pin its word bit is driven to, if
    /// the window does not fit the PIO's five-bit pin fields, or if the clock is
    /// a window pin other than [`crate::pins::RESERVED`] or a pin the PIO cannot
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

    /// The GPIO number of window pin `index`: the signal [`crate::pins::LAYOUT`] places
    /// there.
    fn window_pin(&self, index: usize) -> u8 {
        self.hold[index].id().num
    }
}

impl Hub75Pins for Hub75Pins16 {
    type Word = u16;

    fn base(&self) -> u8 {
        self.base
    }

    fn clock(&self) -> u8 {
        self.clock.id().num
    }

    fn signal_pins(&self) -> impl Iterator<Item = u8> + '_ {
        (0..SIGNAL_COUNT).map(|index| self.window_pin(index))
    }

    #[cfg(feature = "invert-clock")]
    fn invert_clock(&mut self) {
        debug_assert!(
            self.clock() == self.base + crate::pins::RESERVED
                || self.clock() > self.base + (crate::pins::WORD_PINS - 1)
        );
        self.clock.set_output_override(OutputOverride::Invert);
    }
}

impl Hub75Pins8 {
    /// Converts the panel's pins to an owned pin set.
    ///
    /// The eight signals are in bus order — [`crate::pins::latched::LAYOUT`] — with
    /// the clock last, on a GPIO **outside** the window: the latched window has no
    /// reserved slot, so a clock inside it is a signal the side-set overwrites. The
    /// lowest signal pin is the window's base, which [`Hub75Pins::base`] reports.
    ///
    /// # Panics
    ///
    /// Panics if a signal is not on the window pin its word bit is driven to, if
    /// the window does not fit the PIO's five-bit pin fields, or if the clock is
    /// inside the window or not a pin the PIO can address.
    #[allow(clippy::too_many_arguments)]
    pub fn new<I0, I1, I2, I3, I4, I5, I6, I7, I8, P>(
        r1: hal::gpio::Pin<I0, FunctionNull, P>,
        g1: hal::gpio::Pin<I1, FunctionNull, P>,
        b1: hal::gpio::Pin<I2, FunctionNull, P>,
        r2: hal::gpio::Pin<I3, FunctionNull, P>,
        g2: hal::gpio::Pin<I4, FunctionNull, P>,
        b2: hal::gpio::Pin<I5, FunctionNull, P>,
        lat: hal::gpio::Pin<I6, FunctionNull, P>,
        oe: hal::gpio::Pin<I7, FunctionNull, P>,
        clk: hal::gpio::Pin<I8, FunctionNull, P>,
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
            hold(r1),
            hold(g1),
            hold(b1),
            hold(r2),
            hold(g2),
            hold(b2),
            hold(lat),
            hold(oe),
        ];
        let numbers: [u8; SIGNAL_COUNT8] = core::array::from_fn(|index| hold[index].id().num);
        // Checked here, so a mis-wired bus stops startup; the base the check
        // derives is stored rather than recomputed.
        check_bus8(&numbers, clock.id().num);
        let base = *numbers
            .iter()
            .min()
            .expect("a bus has signals, so it has a lowest pin");
        Self { base, hold, clock }
    }

    /// The GPIO number of window pin `index`: the signal
    /// [`crate::pins::latched::LAYOUT`] places there.
    fn window_pin(&self, index: usize) -> u8 {
        self.hold[index].id().num
    }
}

impl Hub75Pins for Hub75Pins8 {
    type Word = u8;

    fn base(&self) -> u8 {
        self.base
    }

    fn clock(&self) -> u8 {
        self.clock.id().num
    }

    fn signal_pins(&self) -> impl Iterator<Item = u8> + '_ {
        (0..SIGNAL_COUNT8).map(|index| self.window_pin(index))
    }

    #[cfg(feature = "invert-clock")]
    fn invert_clock(&mut self) {
        debug_assert!(self.clock() > self.base + (crate::pins::latched::WORD_PINS - 1));
        self.clock.set_output_override(OutputOverride::Invert);
    }
}

// ---------------------------------------------------------------------------
// PIO bring-up
// ---------------------------------------------------------------------------

/// Starts the state machine and returns what the channels need to feed it.
///
/// The panel clock is a ceiling: the PIO divider is an integer and is rounded
/// up, so a request no divider hits exactly runs the panel at the next slower
/// clock rather than faster than the panel is rated for.
pub(crate) fn bring_up<P: Hub75Pins>(
    pio: pac::PIO0,
    resets: &mut pac::RESETS,
    pins: P,
    system_clock_hz: u32,
    config: Hub75Config,
) -> super::BringUp<P> {
    // A word every `CYCLES_PER_WORD` PIO cycles, so the divider is the ratio
    // between the system clock and that many times the panel clock, rounded up
    // so the panel never runs faster than the clock it was given.
    let divider = divider_for(system_clock_hz, config.panel_clock_hz);
    assert!(
        divider >= 1 && divider <= u16::MAX as u32,
        "the panel clock is not representable: divider {divider} is outside \
         the PIO's 1..=65535 range (system clock {system_clock_hz} Hz, panel clock {} Hz)",
        config.panel_clock_hz
    );
    let divider = divider as u16;
    let panel_clock_hz = achieved_clock(system_clock_hz, divider as u32);

    let (mut pio, sm0, _, _, _) = pio.split(resets);

    // The window the framebuffer words are shifted into. `Hub75Pins16::new` has
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

    let stream = stream_for::<P::Word>();
    let program = pio
        .install(stream.program())
        .expect("the stream program fits the instruction memory");
    let (mut sm, _, tx) = PIOBuilder::from_installed_program(program)
        .out_pins(base, stream.window())
        .side_set_pin_base(clock)
        .clock_divisor_fixed_point(divider, 0)
        .out_shift_direction(ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(stream.threshold())
        .buffers(Buffers::OnlyTx)
        .build(sm0);

    // Every signal pin and the clock are outputs: the machine drives the whole
    // bus. A 16-bit window's reserved pin is left alone unless the clock is wired
    // there, so a clock outside the window does not drive a pin held at zero.
    sm.set_pindirs(
        pins.signal_pins()
            .map(|pin| (pin, PinDir::Output))
            .chain(core::iter::once((clock, PinDir::Output))),
    );
    // Keep the machine running. The handle has no `Drop`, so dropping it here
    // would be safe; holding it documents who owns the machine.
    let _sm = sm.start();

    BringUp {
        pins,
        divider,
        panel_clock_hz,
        fifo: tx.fifo_address() as u32,
        dreq: tx.dreq_value(),
    }
}

#[cfg(test)]
mod tests {
    use super::SIGNAL_COUNT;
    use super::check_bus;
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
}

impl<FB, P> Hub75<FB, P>
where
    FB: FrameBuffer + 'static,
    P: Hub75Pins<Word = FB::Word>,
    FB::Word: BusWord,
{
    /// Starts the panel.
    ///
    /// Takes the PIO block, its reset block, **two** DMA channels — `channel`
    /// streams the panel, `control` plays the descriptor table — the pins, the
    /// system clock the PIO divider is derived from, the configuration (which
    /// names the interrupt line the application bound
    /// [`on_dma_irq`](crate::on_dma_irq) to), and the first framebuffer to
    /// display. From here the panel refreshes itself.
    ///
    /// `fb` is taken by exclusive reference, and the first
    /// [`swap`](crate::driver::Hub75::swap) is what hands it back — so the buffer
    /// on screen and the buffer being drawn are never the same one, by
    /// construction.
    ///
    /// # Errors
    ///
    /// [`Hub75Error::AlreadyInitialised`] if a driver already exists, and
    /// [`Hub75Error::NotADmaIrq`] if `config.interrupt` is not a DMA line.
    ///
    /// # Panics
    ///
    /// Panics if the panel clock is not representable by the PIO divider, if a
    /// framebuffer segment is empty, or if the framebuffer's BCM sequence is empty.
    #[allow(clippy::too_many_arguments)]
    pub fn new<CH, CCH, const N: usize>(
        pio: pac::PIO0,
        channel: CH,
        control: CCH,
        resets: &mut pac::RESETS,
        pins: P,
        system_clock_hz: u32,
        config: Hub75Config,
        fb: &'static mut FB,
        descriptors: &'static mut Hub75Descriptors<FB, N>,
    ) -> Result<Self, Hub75Error>
    where
        CH: ChannelSetup,
        CCH: ChannelSetup,
    {
        let irq_line = Self::reserve(config)?;
        let bringup = bring_up(pio, resets, pins, system_clock_hz, config);
        Self::start_from(channel, control, bringup, config, irq_line, fb, descriptors)
    }
}

/// Enables a channel's completion on one of the DMA interrupt lines.
///
/// The `INTEn` registers are distinct types in the PAC, so this routes by address
/// and sets one bit. The RP2350 has four lines and the RP2040 two; the driver's
/// `line_index` only ever returns a line this match handles.
pub(crate) fn arm(flag: u32, line: u8) {
    let dma = unsafe { &*hal::pac::DMA::ptr() };
    let register = match line {
        0 => dma.inte0().as_ptr(),
        1 => dma.inte1().as_ptr(),
        #[cfg(feature = "rphal-rp2350")]
        2 => dma.inte2().as_ptr(),
        #[cfg(feature = "rphal-rp2350")]
        3 => dma.inte3().as_ptr(),
        _ => unreachable!("a DMA interrupt line is 0 to 3"),
    };
    // SAFETY: one of the DMA's interrupt-enable registers. The read-modify-write
    // is serialized against any other arm in a critical section, so a bit set
    // concurrently cannot be lost by a stale read.
    critical_section::with(|_cs| unsafe {
        register.write_volatile(register.read_volatile() | flag)
    });
}

/// Whether a channel's completion is pending on the line's `INTR`.
///
/// `INTR` is not masked by `INTEn`, so a set bit is some channel's completion and
/// not necessarily one this driver armed. The line is unused: this PAC models one
/// `INTR` for every DMA line, as [`clear_intr`] notes.
pub(crate) fn intr_pending(_line: u8, flag: u32) -> bool {
    // SAFETY: the DMA block is a static peripheral, and this only reads.
    unsafe { (*hal::pac::DMA::ptr()).intr().read().bits() & flag != 0 }
}

/// Acknowledges a channel's completion: writes the flag to `INTR`.
///
/// `INTR` is write-1-to-clear and is not masked by `INTEn`, so this cannot
/// disturb another channel. The line is unused: this PAC models one `INTR` for
/// every DMA line.
pub(crate) fn clear_intr(_line: u8, flag: u32) {
    // SAFETY: the DMA block is a static peripheral.
    unsafe { (*hal::pac::DMA::ptr()).intr().write(|w| w.bits(flag)) };
}
