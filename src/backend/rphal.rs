//! The `rp2040-hal` / `rp235x-hal` port.
//!
//! The two `rp-hal` parts share `rp-hal-common` and the same register-block
//! shape, so one implementation covers both: the DMA channel is reached through
//! `SingleChannel`. This module holds the operations whose shape differs between
//! the HALs; the embassy port names them identically, so the rest of the crate
//! calls `backend::…` regardless of the build.

use super::hal;
use hal::dma::SingleChannel;

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
