//! The HUB75 bus — offsets into the fourteen-pin window — and the reference
//! board's wiring.
//!
//! ## The bus window
//!
//! A HUB75 bus needs **fourteen consecutive** GPIOs. `out pins, 14` writes
//! `[base, base + 13]` and no PIO instruction can move that window, so the
//! signals below are *offsets from the window's first pin*, in bus order. Each
//! offset is also the bit of a framebuffer word that carries that signal:
//! `hub75-framebuffer`'s `interstate75` layout puts word bit *n* on window pin
//! *n* (see [`crate::programs`]).
//!
//! On the Pimoroni Interstate 75 W the window starts at GP0, so every offset
//! here is also a GPIO number: `R0` is word bit 0 and GP0 at once. That is a
//! property of that board's wiring, not of the bus. Another run of fourteen
//! GPIOs uses these same offsets with a different base, which
//! [`Hub75Pins::new`](crate::Hub75Pins::new) derives from the first pin it is
//! given; [`MAX_WORD_BASE`] is where such a window may start at the latest.
//!
//! The values come from Pimoroni's HUB75 driver (`pimoroni-pico`,
//! `drivers/hub75/hub75.hpp`), whose pin definitions are the same for the
//! RP2040 and RP2350 boards, and were confirmed on hardware with
//! `tools/panel_bringup.py` before any Rust was written.
//!
//! ## The board's other pins
//!
//! The buttons, the status LED and the Qw/ST connector are not part of the bus
//! window, so their constants are **absolute GPIO numbers** on the reference
//! board rather than offsets.
//!
//! The bus occupies GP0–GP13 there, which means GP0/GP1 (the default UART0
//! TX/RX) are **not** available for logging: use USB CDC, or SWD/RTT with a
//! debug probe.

/// Red data, top half of the panel. Word bit 0.
pub const R0: u8 = 0;
/// Green data, top half of the panel. Word bit 1.
pub const G0: u8 = 1;
/// Blue data, top half of the panel. Word bit 2.
pub const B0: u8 = 2;
/// Red data, bottom half of the panel. Word bit 3.
pub const R1: u8 = 3;
/// Green data, bottom half of the panel. Word bit 4.
pub const G1: u8 = 4;
/// Blue data, bottom half of the panel. Word bit 5.
pub const B1: u8 = 5;

/// Row address line A, the least significant of the five (32 row pairs = 1/32
/// scan). Word bit 6.
pub const ROW_A: u8 = 6;
/// See [`ROW_A`]. Word bit 7.
pub const ROW_B: u8 = 7;
/// See [`ROW_A`]. Word bit 8.
pub const ROW_C: u8 = 8;
/// See [`ROW_A`]. Word bit 9.
pub const ROW_D: u8 = 9;
/// See [`ROW_A`]. Word bit 10.
pub const ROW_E: u8 = 10;

/// Shift-register clock, active high. Word bit 11.
///
/// The PIO generates this pin rather than copying it out of a framebuffer word,
/// so it is the one signal whose polarity the driver sets itself: a panel that
/// takes the column on the falling edge is met by inverting this pin's output in
/// the GPIO block (the crate's `invert-clock` feature), not by a second program.
pub const CLK: u8 = 11;
/// Strobe / latch, active high. Word bit 12.
///
/// A panel that wants this inverted drives it out of the framebuffer's words,
/// whose polarity `hub75-framebuffer` selects (`stb_invert=True` in the
/// MicroPython API).
pub const STB: u8 = 12;
/// Output enable, active low. Word bit 13.
pub const OE: u8 = 13;

/// Number of pins a framebuffer word drives: one per bit, for PIO
/// `out pins, 14`. See the `interstate75` layout in [`crate::programs`].
pub const WORD_PINS: u8 = 14;

/// Highest GPIO number the fourteen-pin window can start at.
///
/// `SM_PINCTRL`'s `OUT_BASE` and `SIDESET_BASE` are **five-bit** fields, so the
/// PIO addresses pins 0–31 and a window of [`WORD_PINS`] pins has to fit inside
/// that range — here, 18 or lower. An RP2350B's GPIO32–47 are out of the PIO's
/// reach.
pub const MAX_WORD_BASE: u8 = 31 - (WORD_PINS - 1);

/// Button A. `SWITCH_A` in the MicroPython API.
pub const SWITCH_A: u8 = 14;
/// Button B. `SWITCH_B` in the MicroPython API.
pub const SWITCH_B: u8 = 15;
/// The BOOT button, reachable as a user button once running.
pub const SWITCH_BOOT: u8 = 22;

/// Onboard status LED (a single WS2812, so it needs PIO rather than a plain
/// GPIO write).
pub const STATUS_LED: u8 = 18;

/// Qw/ST (Qwiic/STEMMA QT) I2C.
pub const I2C_SDA: u8 = 20;
/// See [`I2C_SDA`].
pub const I2C_SCL: u8 = 21;
