//! The bus-window checks: that a pin list is a window the PIO can drive.
//!
//! Shared by both backends: the check is over GPIO *numbers* and the word
//! layout, and neither depends on which HAL configured the PIO.

use crate::pins;

/// Pins in the HUB75 window: the fourteen the framebuffer's word is shifted
/// into, one of which carries no signal.
pub const PIN_COUNT: usize = pins::WORD_PINS as usize;

/// Signals a framebuffer word carries, in bus order.
pub const SIGNAL_COUNT: usize = pins::SIGNAL_COUNT;

/// Signal names, in the order [`Hub75Pins16::new`] takes the pins, for the
/// messages [`check_bus`] panics with.
const SIGNAL_NAMES: [&str; SIGNAL_COUNT] = [
    "R0", "G0", "B0", "R1", "G1", "B1", "A", "B", "C", "D", "E", "STB", "OE",
];

/// Checks that a bus-order pin list is a window the PIO can drive and that each
/// signal is where its word bit lands.
///
/// A HUB75 bus is fourteen consecutive GPIOs. `out pins, 14` writes
/// `[base, base + 13]` and window pin *n* receives framebuffer word bit *n*, so
/// the signal on a pin is decided by the word's layout: the pin given for signal
/// *i* must be `base + LAYOUT[i]`. Anything else would drive the wrong pins —
/// swapped colours, a latch that never latches — which is a wiring mistake rather
/// than a dark panel, so it stops startup.
///
/// The clock is checked apart from the signals, being the one pin the side-set
/// drives rather than the word: it may be the reserved window pin
/// ([`pins::RESERVED`], where the word's zero and the clock agree) or any pin
/// outside the window, but not a signal pin — the side-set would win on every
/// cycle and corrupt that signal silently.
///
/// # Panics
///
/// Panics if a signal is not on the window pin its word bit is driven to, if
/// the window does not fit the PIO's five-bit pin fields — a base above
/// [`pins::MAX_WORD_BASE`] — or if the clock pin is a window pin other than
/// [`pins::RESERVED`], or is not a pin the PIO can address.
pub(super) fn check_bus(numbers: &[u8; SIGNAL_COUNT], clock: u8) {
    let base = *numbers
        .iter()
        .min()
        .expect("a bus has signals, so it has a lowest pin");
    for (index, &number) in numbers.iter().enumerate() {
        let expected = base + pins::LAYOUT[index];
        assert_eq!(
            number,
            expected,
            "{} is wired to pin {number}, but framebuffer word bit {} is driven \
             to pin {expected} of the window starting at {base}: the signal is \
             on the wrong pin",
            SIGNAL_NAMES[index],
            pins::LAYOUT[index],
        );
    }
    assert!(
        base <= pins::MAX_WORD_BASE,
        "a {PIN_COUNT}-pin window cannot start at {base}: the PIO's pin fields \
         are five bits, so it must start at {} or lower",
        pins::MAX_WORD_BASE
    );
    assert!(
        clock <= pins::MAX_PIN,
        "the clock is on pin {clock}, which the PIO cannot address: its pin \
         fields are five bits, so the clock must be on pin {} or lower",
        pins::MAX_PIN
    );
    // The window is fourteen pins wide from `base`, so the checks above have it
    // inside the PIO's range and neither sum can overflow.
    let reserved = base + pins::RESERVED;
    let top = base + (pins::WORD_PINS - 1);
    assert!(
        clock < base || clock == reserved || clock > top,
        "the clock is on pin {clock}, which is inside the window starting at \
         {base} but is not its reserved pin {reserved}: the side-set takes \
         precedence over the word on that pin, so it would overwrite the signal \
         there on every cycle. Wire the clock to pin {reserved} — where the \
         framebuffer word holds zero — or to a pin outside the window"
    );
}
/// Signals the 8-bit latched bus carries.
pub const SIGNAL_COUNT8: usize = pins::latched::SIGNAL_COUNT;

/// Signal names for the 8-bit bus, in the order [`Hub75Pins8::new`] takes the
/// pins, for the messages [`check_bus8`] panics with.
const SIGNAL_NAMES8: [&str; SIGNAL_COUNT8] = ["R1", "G1", "B1", "R2", "G2", "B2", "LAT", "OE"];

/// Checks an 8-bit latched bus-order pin list, as [`check_bus`] does for the
/// 16-bit bus.
///
/// The window is eight consecutive GPIOs and `out pins, 8` writes
/// `[base, base + 7]`, so each signal must be on `base + LAYOUT8[i]`. The clock
/// is checked apart: the 8-bit window has no reserved slot, so the clock must be
/// wired outside it.
///
/// # Panics
///
/// Panics if a signal is not on the window pin its word bit is driven to, if the
/// window does not fit the PIO's five-bit pin fields — a base above
/// [`pins::latched::MAX_WORD_BASE`] — if the clock is inside the window, or if it
/// is not a pin the PIO can address.
pub(super) fn check_bus8(numbers: &[u8; SIGNAL_COUNT8], clock: u8) {
    let base = *numbers
        .iter()
        .min()
        .expect("a bus has signals, so it has a lowest pin");
    for (index, &number) in numbers.iter().enumerate() {
        let expected = base + pins::latched::LAYOUT[index];
        assert_eq!(
            number,
            expected,
            "{} is wired to pin {number}, but framebuffer word bit {} is driven \
             to pin {expected} of the window starting at {base}: the signal is \
             on the wrong pin",
            SIGNAL_NAMES8[index],
            pins::latched::LAYOUT[index],
        );
    }
    assert!(
        base <= pins::latched::MAX_WORD_BASE,
        "an {SIGNAL_COUNT8}-pin window cannot start at {base}: the PIO's pin \
         fields are five bits, so it must start at {} or lower",
        pins::latched::MAX_WORD_BASE
    );
    assert!(
        clock <= pins::MAX_PIN,
        "the clock is on pin {clock}, which the PIO cannot address: its pin \
         fields are five bits, so the clock must be on pin {} or lower",
        pins::MAX_PIN
    );
    let top = base + (pins::latched::WORD_PINS - 1);
    assert!(
        clock < base || clock > top,
        "the clock is on pin {clock}, which is inside the window starting at \
         {base}: the 8-bit window has no reserved slot, so the side-set would \
         overwrite a signal on that pin on every cycle. Wire the clock outside \
         the window"
    );
}
