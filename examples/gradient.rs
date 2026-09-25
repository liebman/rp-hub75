//! M5: the `esp-hub75` gradient demo, on the driver.
//!
//! The same display as `../esp-hub75/examples/gradient`: three horizontal colour
//! ramps across the top of the panel — red, then green, then blue, one step per
//! column — and three status lines below them showing the refresh rate, how long
//! the last frame took to draw, and a counter that ticks ten times a second.
//!
//! Labels are left-justified and values right-justified, as two pieces of text on
//! one line, rather than esp's single padded string. That is not a style choice:
//! `FONT_5X7` advances five pixels per character, so esp's `"Refresh:  145"` is 65
//! pixels and loses the last column of its last digit on a 64-pixel panel. Split
//! in two, the label's width no longer constrains the value's: the longest line
//! here is `"Simple:"` plus five digits, 60 pixels, with a gap between them.
//!
//! ```text
//! # board in BOOTSEL mode:
//! cargo run-arm --example gradient
//! ```
#![no_std]
#![no_main]

use embedded_graphics::Drawable;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use panic_halt as _;
use rp235x_hal as hal;

use hal::Sio;
use hal::clocks::Clock;
use hal::dma::DMAExt;
use hal::fugit::Rate;
use hal::gpio::Pins;
use hub75_framebuffer::bitplane::plain::DmaFrameBuffer;

use rp_hub75::Hub75;
use rp_hub75::Hub75Config;
use rp_hub75::Hub75Pins;
use rp_hub75::{BIT_DEPTH, PANEL_HEIGHT, PANEL_WIDTH, ROW_PAIRS, XTAL_FREQ_HZ};

/// The interrupt that walks the framebuffer's BCM sequence (see `ghost`).
#[unsafe(no_mangle)]
extern "C" fn DMA_IRQ_1() {
    rp_hub75::on_dma_irq();
}

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const NROWS: usize = ROW_PAIRS;
const WORDS_PER_ROW: usize = PANEL_WIDTH;
const PLANES: usize = BIT_DEPTH as usize;

type Fb = DmaFrameBuffer<NROWS, WORDS_PER_ROW, PLANES>;

/// Aligned so the DMA's ring can repeat segments (see `ghost`).
#[repr(align(4096))]
struct Aligned(Fb);

/// The DMA ring repeats a segment only while the framebuffer's *address* stays
/// aligned, so the cell has to keep the wrapper's alignment: it does, because
/// alignment is the maximum over a struct's fields and `static_cell`'s
/// `AtomicBool` cannot lower 4096 — which in turn puts the framebuffer behind it
/// on a 4096-byte boundary of its own.
///
/// Checked rather than assumed: the failure would be a quiet one, the panel still
/// working, just with four times the interrupts.
const _: () = assert!(core::mem::align_of::<static_cell::StaticCell<Aligned>>() == 4096);

/// A `'static` place for a value, for the driver's `&'static mut` framebuffers.
///
/// `StaticCell` guarantees the value is taken at most once, so the two
/// framebuffers cannot be confused for one another, and it hands out a `'static`
/// reference without `unsafe`.
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

/// Rows each colour band covers: the panel divided into eight, of which the three
/// colours take the first three — the esp demo's layout.
const BAND_ROWS: i32 = PANEL_HEIGHT as i32 / 8;
/// Baseline of each status line, at the esp demo's offsets (seven pixels apart:
/// a 5×7 font and a gap).
const LINE1: i32 = PANEL_HEIGHT as i32 - 1 - 14;
const LINE2: i32 = PANEL_HEIGHT as i32 - 1 - 7;
const LINE3: i32 = PANEL_HEIGHT as i32 - 1;
/// One ramp step per column. The crate keeps the top `BIT_DEPTH` bits of an 8-bit
/// channel, so `x * (256 / PANEL_WIDTH)` is exactly light level `x` at depth 6 —
/// the ramps are linear in light, which is what makes them a BCM check.
const STEP: u8 = (256 / PANEL_WIDTH) as u8;
/// A counter tick every this many milliseconds.
const COUNTER_MS: u64 = 100;
/// The counter's range, as in the esp demo: five digits, which beside its label
/// still fits inside the panel's 64 pixels.
const COUNTER_MAX: u32 = 99_999;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let dma = pac.DMA.split(&mut pac.RESETS);

    // One framebuffer is the panel's at any moment and the other is this loop's;
    // a swap exchanges them.
    let aligned0 = mk_static!(Aligned, Aligned(Fb::new()));
    let aligned1 = mk_static!(Aligned, Aligned(Fb::new()));
    let fb0: &'static mut Fb = &mut aligned0.0;
    let fb1: &'static mut Fb = &mut aligned1.0;

    let hub75 = Hub75::new(
        pac.PIO0,
        dma.ch0,
        &mut pac.RESETS,
        Hub75Pins::new(
            pins.gpio0,
            pins.gpio1,
            pins.gpio2,
            pins.gpio3,
            pins.gpio4,
            pins.gpio5,
            pins.gpio6,
            pins.gpio7,
            pins.gpio8,
            pins.gpio9,
            pins.gpio10,
            pins.gpio11,
            pins.gpio12,
            pins.gpio13,
        ),
        clocks.system_clock.freq(),
        Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_1)
            .with_panel_clock(Rate::<u32, 1, 1>::MHz(10)),
        fb0,
    )
    .expect("failed to start the panel");

    // fb0 is on the panel now; this loop draws into fb1 until the first swap.
    let mut fb = fb1;
    let mut render_ms = 0u32;
    let mut counter = 0u32;
    let mut counter_start = timer.get_counter().ticks();

    loop {
        let render_start = timer.get_counter().ticks();

        draw(fb, hub75.refresh_hz(), render_ms, counter);

        // How long that took. The number on screen is therefore the previous
        // frame's, which at ~145 Hz is indistinguishable.
        render_ms = ((timer.get_counter().ticks() - render_start) / 1_000) as u32;

        match hub75.swap(fb) {
            Ok(swap) => fb = swap.wait(),
            Err((_error, returned)) => fb = returned,
        }

        // Ten ticks a second, as in the esp demo (whose counter runs to five
        // digits; three is what fits here).
        let now = timer.get_counter().ticks();
        if now - counter_start >= COUNTER_MS * 1_000 {
            counter_start = now;
            counter = if counter >= COUNTER_MAX {
                0
            } else {
                counter + 1
            };
        }
    }
}

/// The display: three colour ramps and the status block.
fn draw(fb: &mut Fb, refresh_hz: u32, render_ms: u32, counter: u32) {
    fb.erase();

    // Red, green and blue ramps. `x * STEP` walks an 8-bit channel, which at this
    // bit depth is exactly one light level per column.
    for x in 0..PANEL_WIDTH as i32 {
        let brightness = x as u8 * STEP;
        let red = Rgb888::new(brightness, 0, 0);
        let green = Rgb888::new(0, brightness, 0);
        let blue = Rgb888::new(0, 0, brightness);
        for y in 0..BAND_ROWS {
            fb.set_pixel(Point::new(x, y), red);
            fb.set_pixel(Point::new(x, y + BAND_ROWS), green);
            fb.set_pixel(Point::new(x, y + 2 * BAND_ROWS), blue);
        }
    }

    // Yellow on black, so a line reads over whatever is behind it.
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Rgb888::YELLOW)
        .background_color(Rgb888::BLACK)
        .build();

    let mut value = [0u8; 16];
    line(
        fb,
        style,
        LINE3,
        "Refresh:",
        decimal(&mut value, refresh_hz, b""),
    );
    line(
        fb,
        style,
        LINE2,
        "Render:",
        decimal(&mut value, render_ms, b"ms"),
    );
    line(
        fb,
        style,
        LINE1,
        "Simple:",
        decimal(&mut value, counter, b""),
    );
}

/// Draws one status line: the label against the left edge, the value against the
/// right one.
///
/// The two are separate pieces of text so that neither constrains the other: a
/// label may be as long as it likes and a value stays fully visible, with the gap
/// between them absorbing the slack. `FONT_5X7` advances five pixels per
/// character, so a five-digit value beside its label — the most these can
/// produce — is 60 of the panel's 64 pixels.
fn line(fb: &mut Fb, style: MonoTextStyle<Rgb888>, baseline: i32, label: &str, value: &str) {
    let left = Point::new(0, baseline);
    let right = Point::new(PANEL_WIDTH as i32 - 1, baseline);
    let _ = Text::with_alignment(label, left, style, Alignment::Left).draw(fb);
    let _ = Text::with_alignment(value, right, style, Alignment::Right).draw(fb);
}

/// Formats `value` as decimal digits followed by `suffix`, into `buf`, and
/// returns it as a string.
///
/// No padding. The value is drawn against the panel's right edge, so leading
/// zeros would only hold the number away from that edge and eat the gap to its
/// label; esp pads with spaces, which lands the digits in the same columns
/// anyway. Three status lines are not worth a formatting dependency, and
/// `core::fmt` would want a buffer regardless: this is that buffer.
fn decimal<'a>(buf: &'a mut [u8; 16], value: u32, suffix: &[u8]) -> &'a str {
    // Written from the end, so the digits need no length in advance. A u32 is at
    // most ten digits, which with a two-byte suffix still leaves the start of the
    // buffer untouched.
    let suffix_at = buf.len() - suffix.len();
    let mut at = suffix_at;
    let mut rest = value;
    loop {
        at -= 1;
        buf[at] = b'0' + (rest % 10) as u8;
        rest /= 10;
        if rest == 0 {
            break;
        }
    }
    buf[suffix_at..].copy_from_slice(suffix);
    core::str::from_utf8(&buf[at..]).unwrap_or("?")
}

/// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"rp-hub75: the esp-hub75 gradient demo (three colour ramps, status text)"
    ),
    hal::binary_info::rp_program_build_attribute!(),
];
