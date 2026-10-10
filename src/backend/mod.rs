//! The HAL port: the one place the target HAL is chosen.
//!
//! The backend feature selects the HAL, so exactly one is compiled. The selected
//! HAL is re-exported as [`hal`], and the operations whose shape differs are
//! hidden here behind names the rest of the crate uses without `cfg`.
//!
//! The driver's register operations use `hal::pac`, whose field names are the
//! same on both parts, so they need no port. The difference is how a HAL hands
//! over a DMA channel: `rp-hal` exposes `SingleChannel::ch` and `id`,
//! `embassy-rp` exposes `Channel::regs` and `number`. [`ChannelSetup`] is that
//! difference, implemented once per backend.

use crate::driver::Hub75Pins;
use fugit::Rate;

#[cfg(feature = "rphal")]
mod rphal;

/// The bus-window checks, shared by both backends.
mod window;
#[cfg(feature = "rphal")]
#[doc(inline)]
pub use rphal::{Hub75Pins8, Hub75Pins16, PioPin};
#[cfg(feature = "rphal")]
pub(crate) use rphal::{arm, clear_intr, intr_pending};

#[cfg(feature = "embassy")]
mod embassy;
#[cfg(feature = "embassy")]
#[doc(inline)]
pub use embassy::{EmbassyPins, Hub75Pins8, Hub75Pins16, PioPin};
#[cfg(feature = "embassy")]
pub(crate) use embassy::{arm, clear_intr, intr_pending};

/// The two PIO stream programs and the `Stream` pairing a program with its
/// window.
///
/// Shared by both backends: they use the same `pio` crate version, so a
/// `pio::pio_asm!` program is the same type for either.
mod programs;

#[cfg(feature = "embassy")]
pub use embassy_rp as hal;
/// The HAL this build targets.
///
/// Selected by the backend and chip features (one of each), so the driver, the
/// examples, and host code all name one `hal` path. An rp-hal build names the
/// chip's HAL; an embassy build names `embassy-rp`, whose chip comes from the
/// chip feature.
#[cfg(all(feature = "rphal", feature = "rp2350"))]
pub use rp235x_hal as hal;
#[cfg(all(feature = "rphal", feature = "rp2040"))]
pub use rp2040_hal as hal;

/// Programs and observes a DMA channel.
///
/// The driver's two channels are configured the same way wherever the backend
/// comes from, but the HAL hands over the channel differently, so the accessors
/// are what this trait hides. Only startup calls it: the interrupt runs off the
/// raw addresses [`trigger_pointers`](Self::trigger_pointers) returns, captured
/// once, so no trait method is ever reached from the handler.
pub trait ChannelSetup {
    /// The channel's index: its bit in `INTR` and `INTEn`.
    fn id(&self) -> u8;

    /// Programs this channel as the data channel: it streams the runs the control
    /// channel plays into its `AL3` block, feeding the PIO FIFO at `fifo` and
    /// paced by `dreq`, and chains to `control` when a run completes.
    ///
    /// `data_size_log2` is the framebuffer word's DMA `DATA_SIZE` (0 = byte,
    /// 1 = halfword), which the pin set's word width decides.
    fn program_data(&self, control: u8, fifo: u32, dreq: u8, data_size_log2: u8);

    /// Programs this channel as the control channel: it plays the two-word
    /// descriptor table into `target`, the data channel's `AL3_TRANS_COUNT`.
    fn program_control(&self, target: u32);

    /// The channel's `(AL1_READ_ADDR, AL1_TRANS_COUNT_TRIG)`, captured at startup
    /// and written by the interrupt from then on.
    fn trigger_pointers(&self) -> (*mut u32, *mut u32);

    /// The channel's `AL3_TRANS_COUNT`: where the control channel writes, and the
    /// trigger that starts a run.
    fn al3_trans_count(&self) -> u32;
}

#[cfg(feature = "rphal")]
impl<C: hal::dma::SingleChannel> ChannelSetup for C {
    fn id(&self) -> u8 {
        hal::dma::SingleChannel::id(self)
    }

    fn program_data(&self, control: u8, fifo: u32, dreq: u8, data_size_log2: u8) {
        rphal::program_data(self, control, fifo, dreq, data_size_log2);
    }

    fn program_control(&self, target: u32) {
        rphal::program_control(self, target);
    }

    fn trigger_pointers(&self) -> (*mut u32, *mut u32) {
        rphal::trigger_pointers(self)
    }

    fn al3_trans_count(&self) -> u32 {
        rphal::al3_trans_count(self)
    }
}

#[cfg(feature = "embassy")]
impl ChannelSetup for embassy::DmaChannel {
    fn id(&self) -> u8 {
        self.id()
    }

    fn program_data(&self, control: u8, fifo: u32, dreq: u8, data_size_log2: u8) {
        embassy::program_data(self, control, fifo, dreq, data_size_log2);
    }

    fn program_control(&self, target: u32) {
        embassy::program_control(self, target);
    }

    fn trigger_pointers(&self) -> (*mut u32, *mut u32) {
        embassy::trigger_pointers(self)
    }

    fn al3_trans_count(&self) -> u32 {
        embassy::al3_trans_count(self)
    }
}

/// What a backend's `bring_up` returns: the started state machine's handoff to
/// the two DMA channels.
pub(crate) struct BringUp<P: Hub75Pins> {
    /// The panel pins, converted and held so nothing else can claim the bus.
    pub pins: P,
    /// The PIO clock divider in use.
    pub divider: u16,
    /// The panel clock actually achieved: the request rounded to a divider.
    pub panel_clock: Rate<u32, 1, 1>,
    /// The state machine's FIFO, which is the data channel's write address.
    pub fifo: u32,
    /// The state machine's TX DREQ, which paces the data channel.
    pub dreq: u8,
}
