//! The HAL port: the one place the target HAL is chosen.
//!
//! `rp-hub75` drives the panel through whichever HAL the build selects, and the
//! selection is a feature, so exactly one backend is compiled. This module is
//! where that choice is written down: it re-exports the selected HAL as [`hal`],
//! and it hides the few operations whose *shape* differs between the HALs behind
//! names the rest of the crate uses with no conditional compilation of its own.
//!
//! The driver's register operations are written against `hal::pac`, whose
//! method and field names are the same on every part (the register blocks are the
//! same shape), so they need no port. What does differ is how a HAL hands over a
//! DMA channel: `rp-hal` exposes `SingleChannel::ch` and `SingleChannel::id`,
//! while `embassy-rp` exposes `Channel::regs` and `Channel::number`. That one
//! difference is [`ChannelSetup`], implemented once per backend.

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
mod rphal;
#[cfg(any(feature = "rp2040", feature = "rp2350"))]
#[doc(inline)]
pub use rphal::{BringUp, Hub75Pins8, Hub75Pins16, PioPin, Stream, bring_up};

/// The HAL this build targets.
///
/// Selected by the `rp2040` or `rp2350` feature (exactly one), so the driver,
/// the examples, and host code all name one `hal` path.
#[cfg(feature = "rp2350")]
pub use rp235x_hal as hal;
#[cfg(feature = "rp2040")]
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

#[cfg(any(feature = "rp2040", feature = "rp2350"))]
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
