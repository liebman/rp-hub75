//! The `embassy-rp` port.
//!
//! embassy-rp drives the same PIO block and DMA engine through a different API: a
//! [`Pio`] owns the state machines, a [`Pin`] is made from a GPIO before use, and
//! the DMA channel is programmed through the PAC rather than embassy's `Channel`
//! (which reserves the channel and its interrupt for embassy's transfers).
//! Everything here has a twin in `rphal`, so the shared driver calls `backend::…`
//! and names neither HAL.

use embassy_rp::Peri;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::pac;
use embassy_rp::pac::dma::regs::CtrlTrig;
use embassy_rp::pac::dma::vals::DataSize;
use embassy_rp::pac::dma::vals::TreqSel;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::Config;
use embassy_rp::pio::Direction;
use embassy_rp::pio::FifoJoin;
use embassy_rp::pio::Pin;
use embassy_rp::pio::Pio;
use embassy_rp::pio::ShiftConfig;
use embassy_rp::pio::ShiftDirection;
use fugit::Rate;
use hub75_framebuffer::FrameBuffer;

use super::BringUp;
use super::programs::stream_for;
use super::window::SIGNAL_COUNT;
use super::window::SIGNAL_COUNT8;
use super::window::check_bus;
use super::window::check_bus8;
use crate::descriptor::BusWord;
use crate::descriptor::Hub75Descriptors;
use crate::driver::Hub75;
use crate::driver::Hub75Config;
use crate::driver::Hub75Error;
use crate::driver::Hub75Pins;
use crate::driver::achieved_clock;
use crate::driver::divider_for;

/// embassy's PIO-pin bound, re-exported so the crate root's `PioPin` names one
/// thing on every backend.
pub use embassy_rp::pio::PioPin;

// ---------------------------------------------------------------------------
// The two pin sets
// ---------------------------------------------------------------------------

/// The HUB75 pins of a **16-bit direct-drive** bus, converted to PIO pins and
/// owned by the driver.
///
/// The constructor takes the thirteen signals in bus order — `R0 G0 B0 R1 G1 B1
/// A B C D E STB OE` — and the clock separately, then claims each on the given
/// `Pio`. Because embassy makes a PIO pin from the `Pio` instance, the `Pio` is
/// built first and handed to this constructor; the set is then handed to
/// [`Hub75::new`]. The signals are checked to be a fourteen-pin window with each
/// on the pin its word bit is driven to.
pub struct Hub75Pins16 {
    base: u8,
    hold: [Pin<'static, PIO0>; SIGNAL_COUNT],
    clock: Pin<'static, PIO0>,
}

impl Hub75Pins16 {
    /// Converts the panel's pins to an owned pin set.
    ///
    /// The thirteen signals are in bus order ([`crate::pins::LAYOUT`]), with the
    /// clock last: the window pin [`pins::RESERVED`](crate::pins::RESERVED) or any
    /// pin outside the window.
    ///
    /// # Panics
    ///
    /// Panics if a signal is not on the window pin its word bit is driven to, if
    /// the window does not fit the PIO's five-bit pin fields, or if the clock is
    /// a window pin other than the reserved one.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pio: &mut Pio<'static, PIO0>,
        r0: Peri<'static, impl PioPin + 'static>,
        g0: Peri<'static, impl PioPin + 'static>,
        b0: Peri<'static, impl PioPin + 'static>,
        r1: Peri<'static, impl PioPin + 'static>,
        g1: Peri<'static, impl PioPin + 'static>,
        b1: Peri<'static, impl PioPin + 'static>,
        a: Peri<'static, impl PioPin + 'static>,
        b: Peri<'static, impl PioPin + 'static>,
        c: Peri<'static, impl PioPin + 'static>,
        d: Peri<'static, impl PioPin + 'static>,
        e: Peri<'static, impl PioPin + 'static>,
        stb: Peri<'static, impl PioPin + 'static>,
        oe: Peri<'static, impl PioPin + 'static>,
        clk: Peri<'static, impl PioPin + 'static>,
    ) -> Self {
        let clock = pio.common.make_pio_pin(clk);
        let hold = [
            pio.common.make_pio_pin(r0),
            pio.common.make_pio_pin(g0),
            pio.common.make_pio_pin(b0),
            pio.common.make_pio_pin(r1),
            pio.common.make_pio_pin(g1),
            pio.common.make_pio_pin(b1),
            pio.common.make_pio_pin(a),
            pio.common.make_pio_pin(b),
            pio.common.make_pio_pin(c),
            pio.common.make_pio_pin(d),
            pio.common.make_pio_pin(e),
            pio.common.make_pio_pin(stb),
            pio.common.make_pio_pin(oe),
        ];
        let numbers: [u8; SIGNAL_COUNT] = core::array::from_fn(|index| hold[index].pin());
        check_bus(&numbers, clock.pin());
        let base = *numbers
            .iter()
            .min()
            .expect("a bus has signals, so it has a lowest pin");
        Self { base, hold, clock }
    }
}

/// The HUB75 pins of an **8-bit latched** bus, converted to PIO pins.
///
/// The order is `R1 G1 B1 R2 G2 B2 LAT OE`; the clock must be outside the
/// eight-pin window, which has no reserved slot.
pub struct Hub75Pins8 {
    base: u8,
    hold: [Pin<'static, PIO0>; SIGNAL_COUNT8],
    clock: Pin<'static, PIO0>,
}

impl Hub75Pins8 {
    /// Converts the panel's pins to an owned pin set.
    ///
    /// # Panics
    ///
    /// Panics if a signal is not on the window pin its word bit is driven to, or
    /// if the clock is inside the window.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pio: &mut Pio<'static, PIO0>,
        r1: Peri<'static, impl PioPin + 'static>,
        g1: Peri<'static, impl PioPin + 'static>,
        b1: Peri<'static, impl PioPin + 'static>,
        r2: Peri<'static, impl PioPin + 'static>,
        g2: Peri<'static, impl PioPin + 'static>,
        b2: Peri<'static, impl PioPin + 'static>,
        lat: Peri<'static, impl PioPin + 'static>,
        oe: Peri<'static, impl PioPin + 'static>,
        clk: Peri<'static, impl PioPin + 'static>,
    ) -> Self {
        let clock = pio.common.make_pio_pin(clk);
        let hold = [
            pio.common.make_pio_pin(r1),
            pio.common.make_pio_pin(g1),
            pio.common.make_pio_pin(b1),
            pio.common.make_pio_pin(r2),
            pio.common.make_pio_pin(g2),
            pio.common.make_pio_pin(b2),
            pio.common.make_pio_pin(lat),
            pio.common.make_pio_pin(oe),
        ];
        let numbers: [u8; SIGNAL_COUNT8] = core::array::from_fn(|index| hold[index].pin());
        check_bus8(&numbers, clock.pin());
        let base = *numbers
            .iter()
            .min()
            .expect("a bus has signals, so it has a lowest pin");
        Self { base, hold, clock }
    }
}

/// The embassy port's extra view of a pin set: the held [`Pin`] objects the PIO
/// configuration needs, which the shared [`Hub75Pins`] trait (numbers only) does
/// not carry.
pub trait EmbassyPins: Hub75Pins {
    /// The panel clock pin, for the side-set.
    fn clock_pin(&self) -> &Pin<'static, PIO0>;

    /// Every pin the state machine drives, signals then clock, for
    /// `set_pin_dirs`.
    fn dir_pins(&self) -> impl Iterator<Item = &Pin<'static, PIO0>> + '_;
}

/// Drives the `set_pin_dirs` loop: the signals then the clock.
macro_rules! embassy_pins {
    ($set:ty, $word:ty, $count:expr) => {
        impl Hub75Pins for $set {
            type Word = $word;

            fn base(&self) -> u8 {
                self.base
            }

            fn clock(&self) -> u8 {
                self.clock.pin()
            }

            fn signal_pins(&self) -> impl Iterator<Item = u8> + '_ {
                self.hold.iter().map(|pin| pin.pin())
            }

            #[cfg(feature = "invert-clock")]
            fn invert_clock(&mut self) {
                self.clock.set_output_inversion(true);
            }
        }

        impl EmbassyPins for $set {
            fn clock_pin(&self) -> &Pin<'static, PIO0> {
                &self.clock
            }

            fn dir_pins(&self) -> impl Iterator<Item = &Pin<'static, PIO0>> + '_ {
                self.hold.iter().chain(core::iter::once(&self.clock))
            }
        }

        const _: () = assert!($count > 0);
    };
}

embassy_pins!(Hub75Pins16, u16, SIGNAL_COUNT);
embassy_pins!(Hub75Pins8, u8, SIGNAL_COUNT8);

// ---------------------------------------------------------------------------
// DMA channels
// ---------------------------------------------------------------------------

/// A DMA channel the driver owns, addressed by its index.
///
/// embassy's `Channel` binds an interrupt and is reserved for embassy's async
/// transfers, so the driver programs the PAC channel block directly. The index
/// comes from the [`ChannelInstance`] the constructor is handed.
pub(crate) struct DmaChannel {
    id: u8,
}

impl DmaChannel {
    /// The channel's index: its bit in `INTR` and `INTEn`.
    pub(super) fn id(&self) -> u8 {
        self.id
    }

    fn block(&self) -> pac::dma::Channel {
        pac::DMA.ch(self.id as usize)
    }
}

/// Configures `channel` as the data channel.
///
/// The `CTRL` word is built with rp-pac's typed [`CtrlTrig`] register value, whose
/// setters place each field at the bit *this* part uses: the RP2040 and RP2350
/// `CTRL` layouts differ (RP2350 inserts the `INCR_*_REV` bits and shifts every
/// field above bit 4), so a hand-built word is wrong on one of them. The word is
/// written through the non-triggering alias, so this never starts a run; the
/// framebuffer's word size and DREQ are the only fields that vary.
///
/// See [`ChannelSetup::program_data`].
///
/// [`ChannelSetup::program_data`]: super::ChannelSetup::program_data
pub(super) fn program_data(
    channel: &DmaChannel,
    control: u8,
    fifo: u32,
    dreq: u8,
    data_size_log2: u8,
) {
    let block = channel.block();
    block.write_addr().write_value(fifo);
    let mut ctrl = CtrlTrig(0);
    ctrl.set_data_size(DataSize::from_bits(data_size_log2));
    ctrl.set_incr_read(true);
    // Paced by the state machine's TX FIFO.
    ctrl.set_treq_sel(TreqSel::from_bits(dreq));
    ctrl.set_chain_to(control);
    ctrl.set_irq_quiet(true);
    ctrl.set_en(true);
    block.al1_ctrl().write_value(ctrl.0);
}

/// Configures `channel` as the control channel. See [`ChannelSetup::program_control`].
///
/// [`ChannelSetup::program_control`]: super::ChannelSetup::program_control
pub(super) fn program_control(channel: &DmaChannel, target: u32) {
    let block = channel.block();
    block.write_addr().write_value(target);
    let mut ctrl = CtrlTrig(0);
    // 32-bit descriptor words.
    ctrl.set_data_size(DataSize::from_bits(2));
    ctrl.set_incr_read(true);
    ctrl.set_incr_write(true);
    ctrl.set_ring_sel(true);
    // RING_SIZE 3: the write pointer wraps over one 8-byte descriptor block.
    ctrl.set_ring_size(3);
    // TREQ_SEL 63: permanent, so it transfers as fast as the bus takes.
    ctrl.set_treq_sel(TreqSel::from_bits(63));
    ctrl.set_high_priority(true);
    // A channel that chains to itself does not chain.
    ctrl.set_chain_to(channel.id);
    ctrl.set_irq_quiet(true);
    ctrl.set_en(true);
    block.al1_ctrl().write_value(ctrl.0);
}

/// The channel's `(AL1_READ_ADDR, AL1_TRANS_COUNT_TRIG)`.
///
/// See [`ChannelSetup::trigger_pointers`]: the addresses live for the rest of the
/// program, and `as_ptr` is safe; the pointers are only dereferenced in the
/// interrupt.
///
/// [`ChannelSetup::trigger_pointers`]: super::ChannelSetup::trigger_pointers
pub(super) fn trigger_pointers(channel: &DmaChannel) -> (*mut u32, *mut u32) {
    let block = channel.block();
    (
        block.al1_read_addr().as_ptr(),
        block.al1_trans_count_trig().as_ptr(),
    )
}

/// The channel's `AL3_TRANS_COUNT`. See [`ChannelSetup::al3_trans_count`].
///
/// [`ChannelSetup::al3_trans_count`]: super::ChannelSetup::al3_trans_count
pub(super) fn al3_trans_count(channel: &DmaChannel) -> u32 {
    channel.block().al3_trans_count().as_ptr() as u32
}

/// Enables a channel's completion on one of the DMA interrupt lines.
pub(crate) fn arm(flag: u32, line: u8) {
    let inte = pac::DMA.inte(line as usize);
    // A critical section around the read-modify-write, so a bit set concurrently
    // by another arm cannot be lost by a stale read.
    critical_section::with(|_cs| inte.write_value(inte.read() | flag));
}

/// Whether a channel's completion is pending on the line's `INTR`.
///
/// `INTR` is not masked by `INTEn`, so a set bit is some channel's completion and
/// not necessarily one this driver armed.
pub(crate) fn intr_pending(line: u8, flag: u32) -> bool {
    pac::DMA.intr(line as usize).read() & flag != 0
}

/// Acknowledges a channel's completion: writes the flag to the line's `INTR`.
///
/// `INTR` is write-1-to-clear and is not masked by `INTEn`, so this cannot
/// disturb another channel.
pub(crate) fn clear_intr(line: u8, flag: u32) {
    pac::DMA.intr(line as usize).write_value(flag);
}

// ---------------------------------------------------------------------------
// PIO bring-up
// ---------------------------------------------------------------------------

/// Starts the state machine and returns what the channels need to feed it.
///
/// The panel clock is a ceiling: the PIO divider is an integer and is rounded
/// up, so a request no divider hits exactly runs the panel at the next slower
/// clock rather than faster than the panel is rated for.
///
/// The `Pio` is leaked with [`core::mem::forget`]: embassy's `StateMachine`
/// disables itself on drop, and this driver runs for the life of the program.
pub(crate) fn bring_up<P: EmbassyPins>(
    mut pio: Pio<'static, PIO0>,
    pins: P,
    system_clock: Rate<u32, 1, 1>,
    config: Hub75Config,
) -> BringUp<P> {
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

    let stream = stream_for::<P::Word>();
    let loaded = pio.common.load_program(stream.program());

    // The clock's polarity is the pad's, so the pin is inverted before the
    // machine starts and the first clock level is already the idle one.
    #[cfg(feature = "invert-clock")]
    let mut pins = pins;
    #[cfg(feature = "invert-clock")]
    pins.invert_clock();

    let base = pins.base();
    let mut cfg: Config<'static, PIO0> = Config::default();
    cfg.clock_divider = divider.into();
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.shift_out = ShiftConfig {
        threshold: stream.threshold(),
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    cfg.use_program(&loaded, &[pins.clock_pin()]);
    // Set the OUT window by number: the reserved window pin, where the
    // framebuffer's word holds zero, is not a signal the pin set holds.
    // SAFETY: the state machine has not started, and the only fields written are
    // `out_base` and `out_count`, both derived from the checked pin set and the
    // stream's window width.
    unsafe {
        let mut pin_config = cfg.get_pins();
        pin_config.out_base = base;
        pin_config.out_count = stream.window();
        cfg.set_pins(pin_config);
    }

    let (fifo, dreq) = {
        let sm = &mut pio.sm0;
        sm.set_config(&cfg);
        for pin in pins.dir_pins() {
            sm.set_pin_dirs(Direction::Out, &[pin]);
        }
        sm.set_enable(true);
        (sm.tx_fifo_ptr() as u32, u8::from(sm.tx_treq()))
    };

    core::mem::forget(pio);

    BringUp {
        pins,
        divider,
        panel_clock: achieved,
        fifo,
        dreq,
    }
}

// ---------------------------------------------------------------------------
// The constructor
// ---------------------------------------------------------------------------

impl<FB, P> Hub75<FB, P>
where
    FB: FrameBuffer + 'static,
    P: EmbassyPins<Word = FB::Word>,
    FB::Word: BusWord,
{
    /// Starts the panel.
    ///
    /// Takes the [`Pio`] built from the PIO block — embassy makes a PIO pin from
    /// it, so it is created before the pin set and handed over with it — **two**
    /// DMA channel instances, the pins, the system clock the divider is derived
    /// from, the configuration (which names the interrupt line the application
    /// bound [`on_dma_irq`](crate::on_dma_irq) to), and the first framebuffer to
    /// display. From here the panel refreshes itself.
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
        pio: Pio<'static, PIO0>,
        channel: Peri<'static, CH>,
        control: Peri<'static, CCH>,
        pins: P,
        system_clock: Rate<u32, 1, 1>,
        config: Hub75Config,
        fb: &'static mut FB,
        descriptors: &'static mut Hub75Descriptors<FB, N>,
    ) -> Result<Self, Hub75Error>
    where
        CH: ChannelInstance,
        CCH: ChannelInstance,
    {
        // The channel instances are taken so embassy cannot also use them; the
        // index is all the driver needs.
        let data = DmaChannel { id: CH::number() };
        let ctrl = DmaChannel { id: CCH::number() };
        let _ = (channel, control);

        let irq_line = Self::reserve(config)?;
        let bringup = bring_up(pio, pins, system_clock, config);
        Self::start_from(data, ctrl, bringup, config, irq_line, fb, descriptors)
    }
}
