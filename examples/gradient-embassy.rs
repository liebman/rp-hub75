//! The embassy-rp backend: the `esp-hub75` gradient demo, on an embassy executor.
//!
//! The same display as `gradient` — three colour ramps (red, green, blue, one light
//! level per column) and four status lines with the refresh rate, render time, the
//! descriptor table's runs a frame and a counter ticking ten times a second. The
//! differences are all in the setup: embassy's `#[embassy_executor::main]` replaces
//! `#[entry]`; the `Pio` is built before the pin set, because a PIO pin is made from
//! it; the DMA channels are handed over as `ChannelInstance`s; the frame boundary is
//! awaited with `wait_for_done().await`; and the clock is `embassy-time`.
//!
//! ```text
//! cargo run --config .cargo/config-rp2350.toml --release \
//!     --example gradient-embassy --features embassy,rp2350
//! ```
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt::typelevel::Handler;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_rp::pio::Pio;
use embassy_time::Duration;
use embassy_time::Instant;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use fugit::Rate;
use hub75_framebuffer::bitplane::plain::DmaFrameBuffer;
use hub75_framebuffer::compute_rows;

use rp_hub75::Hub75;
use rp_hub75::Hub75Config;
use rp_hub75::Hub75Pins16;
use rp_hub75::hal;
use rp_hub75::hub75_descriptors;
use rp_hub75::on_dma_irq;

/// The driver's frame-boundary work, in the DMA interrupt.
struct DmaIrqHandler;

impl Handler<embassy_rp::interrupt::typelevel::DMA_IRQ_0> for DmaIrqHandler {
    unsafe fn on_interrupt() {
        on_dma_irq();
    }
}

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => DmaIrqHandler;
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// The panel this example is wired to: 64×64, 1/32 scan.
const PANEL_WIDTH: usize = 64;
const PANEL_HEIGHT: usize = 64;
/// Row pairs: a 1/32-scan panel lights its top half and its bottom half at once.
const NROWS: usize = compute_rows(PANEL_HEIGHT);
/// Six planes: 63 panel scans a frame, where eight flickers.
const PLANES: usize = 6;

type Fb = DmaFrameBuffer<NROWS, PANEL_WIDTH, PLANES>;

/// A `'static` place for a value, for the driver's `&'static mut` framebuffers.
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

/// Rows each colour band covers: the panel divided into eight.
const BAND_ROWS: i32 = PANEL_HEIGHT as i32 / 8;
/// Baseline of each status line, seven pixels apart.
const LINE0: i32 = PANEL_HEIGHT as i32 - 1 - 21;
const LINE1: i32 = PANEL_HEIGHT as i32 - 1 - 14;
const LINE2: i32 = PANEL_HEIGHT as i32 - 1 - 7;
const LINE3: i32 = PANEL_HEIGHT as i32 - 1;
/// One ramp step per column.
const STEP: u8 = (256 / PANEL_WIDTH) as u8;
/// A counter tick every this many milliseconds.
const COUNTER_MS: u64 = 100;
/// The counter's range: five digits beside its label.
const COUNTER_MAX: u32 = 99_999;

/// The panel clock, as in the esp-hub75 examples: 10 MHz by default, 20 MHz with
/// the `20mhz` feature. The divider rounds up, so the panel runs at this or the
/// next slower clock — `Refresh:` on the display is the truth.
const PANEL_CLOCK: Rate<u32, 1, 1> =
    Rate::<u32, 1, 1>::MHz(if cfg!(feature = "20mhz") { 20 } else { 10 });

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // The PIO is built first: embassy makes a PIO pin from it.
    let mut pio = Pio::new(p.PIO0, Irqs);
    let pins = Hub75Pins16::new(
        &mut pio, p.PIN_0, p.PIN_1, p.PIN_2, p.PIN_3, p.PIN_4, p.PIN_5, p.PIN_6, p.PIN_7, p.PIN_8,
        p.PIN_9, p.PIN_10,
        // STB and OE are the word's last two signal bits; the clock is not a
        // signal at all — the side-set drives it, and it is wired to GP11.
        p.PIN_12, p.PIN_13, p.PIN_11,
    );

    // One framebuffer is the panel's at any moment and the other is this loop's;
    // a swap exchanges them.
    let fb0: &'static mut Fb = mk_static!(Fb, Fb::new());
    let fb1: &'static mut Fb = mk_static!(Fb, Fb::new());

    // The framebuffers' addresses, taken before they move into the driver. The
    // alignment does not change the picture; the log is where it is checked.
    let fb0_addr = fb0 as *const Fb as usize;
    let fb1_addr = fb1 as *const Fb as usize;

    let hub75 = Hub75::new(
        pio,
        p.DMA_CH0,
        p.DMA_CH1,
        pins,
        Rate::<u32, 1, 1>::from_raw(hal::clocks::clk_sys_freq()),
        Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_0).with_panel_clock(PANEL_CLOCK),
        fb0,
        hub75_descriptors!(Fb),
    )
    .expect("failed to start the panel");

    // The panel's numbers, once, as the esp-hub75 gradient prints them. The display
    // carries the live refresh rate and runs a frame; the log carries what it cannot
    // — the geometry, the divider, the framebuffer's size and where it landed, and
    // the descriptor table's length.
    defmt::info!(
        "{}x{} panel, {} row pairs, {} planes",
        PANEL_WIDTH,
        PANEL_HEIGHT,
        NROWS,
        PLANES
    );
    defmt::info!(
        "panel clock {} Hz -> divider /{} -> {} Hz, {} Hz refresh, {} runs a refresh",
        PANEL_CLOCK.to_Hz(),
        hub75.divider(),
        hub75.panel_clock().to_Hz(),
        hub75.refresh_hz(),
        hub75.runs_per_refresh()
    );
    defmt::info!(
        "framebuffers {} bytes at {=usize:#x} and {=usize:#x}, {} descriptors ({} bytes)",
        core::mem::size_of::<Fb>(),
        fb0_addr,
        fb1_addr,
        rp_hub75::descriptor::descriptor_count::<Fb>(),
        rp_hub75::descriptor::descriptor_count::<Fb>()
            * core::mem::size_of::<rp_hub75::descriptor::Descriptor>()
    );

    // fb0 is on the panel now; this loop draws into fb1 until the first swap.
    let mut fb = fb1;
    let mut render_ms = 0u32;
    let mut counter = 0u32;
    let mut counter_start = Instant::now();

    loop {
        let render_start = Instant::now();

        draw(
            fb,
            hub75.refresh_hz(),
            render_ms,
            hub75.runs_per_refresh(),
            counter,
        );

        // How long that took. The number on screen is therefore the previous
        // frame's, which at panel refresh rates is indistinguishable.
        render_ms = render_start.elapsed().as_micros() as u32 / 1_000;

        match hub75.swap(fb) {
            Ok(mut swap) => {
                swap.wait_for_done().await;
                fb = swap.wait();
            }
            Err((_error, returned)) => fb = returned,
        }

        // Ten ticks a second, as in the esp demo.
        if counter_start.elapsed() >= Duration::from_millis(COUNTER_MS) {
            counter_start = Instant::now();
            counter = if counter >= COUNTER_MAX {
                0
            } else {
                counter + 1
            };
        }
    }
}

/// The display: three colour ramps and the status block.
fn draw(fb: &mut Fb, refresh_hz: u32, render_ms: u32, runs_per_frame: u32, counter: u32) {
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
        "Runs/f:",
        decimal(&mut value, runs_per_frame, b""),
    );
    line(
        fb,
        style,
        LINE0,
        "Simple:",
        decimal(&mut value, counter, b""),
    );
}

/// Draws one status line: the label against the left edge, the value against the
/// right one.
fn line(fb: &mut Fb, style: MonoTextStyle<Rgb888>, baseline: i32, label: &str, value: &str) {
    let left = Point::new(0, baseline);
    let right = Point::new(PANEL_WIDTH as i32 - 1, baseline);
    let _ = Text::with_alignment(label, left, style, Alignment::Left).draw(fb);
    let _ = Text::with_alignment(value, right, style, Alignment::Right).draw(fb);
}

/// Formats `value` as decimal digits followed by `suffix`, into `buf`, and
/// returns it as a string.
fn decimal<'a>(buf: &'a mut [u8; 16], value: u32, suffix: &[u8]) -> &'a str {
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
