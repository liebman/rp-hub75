//! Colour helpers for a `BIT_DEPTH`-bit BCM panel.
//!
//! The panel is **linear in the level it is sent** — that is the whole point of
//! Binary Code Modulation — so nothing here needs to model light output. What
//! these helpers do is bridge two conventions that do not line up:
//!
//! * A [`Rgb888`] channel has 8 bits; this driver runs [`BIT_DEPTH`] of them.
//!   `hub75-framebuffer` keeps the **top** `PLANES` bits of a drawn byte
//!   (`bit = 8 - PLANES + plane` in its `set_pixel_internal`), so an effective
//!   level of `n` out of [`LEVELS`] has to be shifted up to the top of the byte
//!   before it is drawn. That is [`level`].
//! * Human eyes are not linear in light, so a ramp drawn with raw levels looks
//!   crushed at the dark end. [`gamma_step`] is the classic gamma-2 correction,
//!   applied to the *level* rather than to the framebuffer words, so BCM
//!   correctness (a `level = column` ramp, rung B of the bring-up ladder) is unaffected by it.
//!
//! [`BIT_DEPTH`]: crate::BIT_DEPTH

use embedded_graphics::pixelcolor::Rgb888;

use crate::BIT_DEPTH;

/// Bit-planes the panel is driven with, as a `usize` for the arithmetic below.
pub const PLANES: u32 = BIT_DEPTH as u32;

/// Light levels a channel has: `1 << BIT_DEPTH`, i.e. **64** at depth 6.
///
/// Level `0` is off; level `LEVELS - 1` is that channel's maximum, which is
/// [`level`]`(LEVELS - 1)` as a byte (252 at depth 6, not 255).
pub const LEVELS: u32 = 1 << PLANES;

/// Bits of an 8-bit channel that survive BCM, i.e. how far a level is shifted.
///
/// The crate keeps the *top* [`PLANES`] bits, so the low bits of a drawn byte
/// are dropped. Three at depth 6 — which is why a naive `set_pixel(x, 1)` looks
/// black: the `1` is below the retained bits and nothing is drawn at all.
pub const LEVEL_SHIFT: u32 = 8 - PLANES;

/// The byte value for BCM light level `bits` (`0..LEVELS`).
///
/// Shifted up to the top of the byte because of [`LEVEL_SHIFT`]; levels above
/// `LEVELS - 1` saturate at the maximum rather than wrapping to black.
pub const fn level(bits: u32) -> u8 {
    let bits = if bits < LEVELS { bits } else { LEVELS - 1 };
    (bits << LEVEL_SHIFT) as u8
}

/// A grey at BCM level `bits`, as linear light.
pub const fn grey(bits: u32) -> Rgb888 {
    let value = level(bits);
    Rgb888::new(value, value, value)
}

/// The light for step `step` of a perceptually even ramp: `step² / (LEVELS-1)`.
///
/// Gamma 2.0 — the exponent that makes the *steps look* equal rather than the
/// light, which is what the gamma rung exists to compare against the linear one.
pub const fn gamma_step(step: u32) -> u8 {
    let last = LEVELS - 1;
    level((step * step * last) / (last * last))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shift is exactly the number of bits the crate drops.
    #[test]
    fn level_sits_at_the_top_of_the_byte() {
        assert_eq!(LEVEL_SHIFT, 8 - PLANES);
        assert_eq!(LEVEL_SHIFT + PLANES, 8, "the level fills the retained bits");
        // Depth 6, the depth the panel was tuned to: 63 -> 252, one -> 4.
        assert_eq!(level(0), 0);
        assert_eq!(level(1), 1 << LEVEL_SHIFT);
        assert_eq!(level(LEVELS - 1), ((LEVELS - 1) << LEVEL_SHIFT) as u8);
        assert_eq!(level(LEVELS - 1), 252);
    }

    /// A level above the top of the range saturates instead of wrapping to
    /// black, which would look like a discontinuity in a gradient.
    #[test]
    fn level_saturates() {
        assert_eq!(level(LEVELS), level(LEVELS - 1));
        assert_eq!(level(u32::MAX), level(LEVELS - 1));
    }

    /// Every level is distinct in the byte it draws, so no two BCM levels
    /// collapse onto the same pixel value.
    #[test]
    fn every_level_is_distinct() {
        let mut previous = None;
        for bits in 0..LEVELS {
            let value = level(bits);
            assert!(
                previous.is_none_or(|previous| value > previous),
                "level {bits} did not increase: {value}"
            );
            previous = Some(value);
        }
    }

    /// Gamma is monotone, hits both ends exactly, and is not the identity —
    /// otherwise rungs B and C of the example would be indistinguishable.
    #[test]
    fn gamma_rises_and_keeps_the_endpoints() {
        assert_eq!(gamma_step(0), 0);
        assert_eq!(gamma_step(LEVELS - 1), level(LEVELS - 1));

        let mut previous = 0;
        for step in 0..LEVELS {
            let value = gamma_step(step);
            assert!(value >= previous, "gamma dipped at step {step}");
            previous = value;
        }

        let middle = gamma_step((LEVELS - 1) / 2);
        assert!(
            middle < level((LEVELS - 1) / 2),
            "gamma should darken the middle of the ramp: {middle}"
        );
    }

    /// The mid-ramp value, spelled out: with 64 levels the middle step sits at
    /// a quarter of full light, not half.
    #[test]
    fn gamma_is_quarter_scale_in_the_middle() {
        let last = LEVELS - 1;
        assert_eq!(gamma_step(last / 2), level((31 * 31) / last));
        assert_eq!(gamma_step(last / 2), level(15));
        assert_eq!(gamma_step(last / 2), 60);
    }
}
