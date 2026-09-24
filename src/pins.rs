//! GPIO assignments for the Pimoroni Interstate 75 W (RP2350).
//!
//! These come from Pimoroni's HUB75 driver (`pimoroni-pico`,
//! `drivers/hub75/hub75.hpp`), whose pin definitions are the same for the
//! RP2040 and RP2350 boards, and were confirmed on hardware with
//! `tools/panel_bringup.py` before any Rust was written.
//!
//! Note that the HUB75 signals occupy GP0–GP13, which means GP0/GP1 (the
//! default UART0 TX/RX) are **not** available for logging on this board: use
//! USB CDC, or SWD/RTT with a debug probe.

/// Red data, top half of the panel.
pub const R0: u8 = 0;
/// Green data, top half of the panel.
pub const G0: u8 = 1;
/// Blue data, top half of the panel.
pub const B0: u8 = 2;
/// Red data, bottom half of the panel.
pub const R1: u8 = 3;
/// Green data, bottom half of the panel.
pub const G1: u8 = 4;
/// Blue data, bottom half of the panel.
pub const B1: u8 = 5;

/// Row address lines A–E (32 row pairs = 1/32 scan).
pub const ROW_A: u8 = 6;
/// See [`ROW_A`].
pub const ROW_B: u8 = 7;
/// See [`ROW_A`].
pub const ROW_C: u8 = 8;
/// See [`ROW_A`].
pub const ROW_D: u8 = 9;
/// See [`ROW_A`].
pub const ROW_E: u8 = 10;

/// Shift-register clock, active high.
pub const CLK: u8 = 11;
/// Strobe / latch, active high. Some panels want this inverted
/// (`stb_invert=True` in the MicroPython API).
pub const STB: u8 = 12;
/// Output enable, active low.
pub const OE: u8 = 13;

/// First of the fourteen pins a whole framebuffer word drives: the HUB75 bus
/// from `R0` (GP0) up to `OE` (GP13).
pub const WORD_BASE: u8 = R0;
/// Number of pins a framebuffer word drives, one per bit, for PIO
/// `out pins, 14`. See the `interstate75` layout in [`crate::programs`].
pub const WORD_PINS: u8 = 14;

/// First of the six contiguous data pins. Only the bring-up milestones need
/// this: from `M3` on the whole word goes out as one `out pins, 14`.
pub const DATA_BASE: u8 = R0;
/// Number of contiguous data pins.
pub const DATA_PINS: u8 = 6;
/// First of the five contiguous row-select pins.
pub const ROWSEL_BASE: u8 = ROW_A;
/// Number of contiguous row-select pins.
pub const ROWSEL_PINS: u8 = 5;

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
