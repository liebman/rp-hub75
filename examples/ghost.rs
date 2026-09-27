//! A ghosting / bleed probe: what leaks, in which direction.
//!
//! Ghosting is directional, so the pattern has to be positional. Every phase
//! below puts a *small, bright* thing on an otherwise black panel — bright
//! because a ghost is usually faint, and small because a ghost is only easy to
//! see when there is nothing else to confuse it with.
//!
//! Four phases cycle, and each answers a different question:
//!
//! | phase | what it lights | what it rules in or out |
//! |---|---|---|
//! | **dot** | one pixel at column 8, row 8, blinking | the photographable reference: any *other* lit cell is a leak, and its offset from the dot names it |
//! | **raster** | one pixel stepping one column per step along one row | **horizontal** ghosting: a second pixel leading or trailing the moving one is data/clock timing |
//! | **diagonal** | one pixel per row pair, at that row's own column, sweeping down | **vertical** ghosting: any dot on a row other than the lit one — and the sweep dwells on row 31 then row 0, where a frame-boundary leak shows |
//! | **bar** | a whole row pair, sweeping down | the blunt version of the diagonal: much more light to leak, easy to photograph |
//!
//! What to do with what you see:
//!
//! * A ghost **one column to the side** of a lit pixel is horizontal bleed: the
//!   data moved while the panel was sampling it. That is a timing question — try a
//!   lower panel clock ([`Hub75Config::with_panel_clock`]) and see if it goes.
//! * A ghost **on the row above or below** a lit pixel is row bleed: the panel's
//!   shift register was still holding the previous row's data when the new row was
//!   latched. It is the thing `hub75-framebuffer`'s `inter-row-blank`,
//!   `lead-blank` and `trail-blank` features exist for; that is why this example
//!   exists, so the leak is measured before a feature is added.
//! * A ghost **on the first row while the last row is lit** (or the other way
//!   round) is the frame boundary, which the diagonal and bar phases dwell on.
//! * A ghost on the **other half** of the panel — row `r + 32` while row `r` is
//!   lit — is neither: the two halves share one shift register and one latch, so
//!   that would be an electrical fault, not a scan-order one.
//!
//! ```text
//! # board in BOOTSEL mode:
//! cargo run-arm --example ghost
//! ```
#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use panic_probe as _;
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
use rp_hub75::{BIT_DEPTH, PANEL_WIDTH, ROW_PAIRS, XTAL_FREQ_HZ};

/// The interrupt that walks the framebuffer's BCM sequence (see `gradient`).
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

/// The framebuffer type this example uses: `hub75-framebuffer`'s plane-major
/// layout, whose words are the panel's own pin levels.
///
/// The two of them are plain statics, and that costs nothing: the engine names
/// each run's address in the descriptor table, so neither a framebuffer's address
/// nor its alignment changes the picture or the interrupt load.
type Fb = DmaFrameBuffer<NROWS, WORDS_PER_ROW, PLANES>;

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

/// Steps each phase takes, at [`STEP_MS`] apiece.
const DOT_STEPS: u32 = 20;
const RASTER_STEPS: u32 = PANEL_WIDTH as u32;
/// `ROW_PAIRS` steps down the panel, then a dwell on the last row and another on
/// the first: the two moments a frame-boundary leak would show.
const SWEEP_STEPS: u32 = ROW_PAIRS as u32 + 8;
/// Milliseconds one step is held: slow enough to watch, quick enough to cycle.
const STEP_MS: u64 = 80;
/// The row pair the raster phase moves along — a middle one, away from the edges.
const RASTER_ROW: u32 = 8;

/// The row pair a sweeping phase is on, dwelling at both ends of the panel.
fn sweep_row(step: u32) -> u32 {
    let last = ROW_PAIRS as u32 - 1;
    if step < ROW_PAIRS as u32 {
        step
    } else if step < ROW_PAIRS as u32 + 4 {
        last
    } else {
        0
    }
}

/// The panel clock, as in the esp-hub75 examples: 10 MHz by default, 20 MHz with
/// the `20mhz` feature.
///
/// The driver rounds the divider up, so what the panel runs at is this or the next
/// slower divider's clock — `Refresh:` on the display is the truth, not this
/// number.
const PANEL_CLOCK: Rate<u32, 1, 1> =
    Rate::<u32, 1, 1>::MHz(if cfg!(feature = "20mhz") { 20 } else { 10 });

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
    let fb0: &'static mut Fb = mk_static!(Fb, Fb::new());
    let fb1: &'static mut Fb = mk_static!(Fb, Fb::new());

    let hub75 = Hub75::new(
        pac.PIO0,
        dma.ch0,
        dma.ch1,
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
        // The panel clock is the one knob that trades timing margin for refresh
        // rate: if a phase shows horizontal bleed, halve this and look again.
        Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_1).with_panel_clock(PANEL_CLOCK),
        fb0,
    )
    .expect("failed to start the panel");

    let mut fb = fb1;
    let mut phase = 0u32;
    let mut step = 0u32;

    loop {
        draw(fb, phase, step);

        match hub75.swap(fb) {
            Ok(swap) => fb = swap.wait(),
            Err((_error, returned)) => fb = returned,
        }

        // Advance the probe, and pace it: the panel refreshes at ~145 Hz but an
        // eye reading a photograph needs about ten steps a second.
        step += 1;
        let steps = match phase {
            0 => DOT_STEPS,
            1 => RASTER_STEPS,
            2 => SWEEP_STEPS,
            _ => SWEEP_STEPS,
        };
        if step >= steps {
            step = 0;
            phase = (phase + 1) % 4;
        }

        let until = timer.get_counter().ticks() + STEP_MS * 1_000;
        while timer.get_counter().ticks() < until {}
    }
}

/// One step of the probe. See the module docs for what each phase is for.
fn draw(fb: &mut Fb, phase: u32, step: u32) {
    fb.erase();
    // All planes on: the brightest thing the panel can show, so that whatever
    // does leak has the most light to leak with. The crate keeps the top
    // `BIT_DEPTH` bits of a drawn byte, so 255 is light level 63 — the top of the
    // panel's range — and this is the same light `252` would be.
    let on = Rgb888::WHITE;

    match phase {
        // The dot: a fixed reference, blinking so that even a phase that does
        // not move tells you whether the loop is still running.
        0 => {
            if step % 8 < 6 {
                fb.set_pixel(Point::new(8, 8), on);
            }
        }
        // The raster: one pixel stepping along one row pair, one column per step.
        1 => fb.set_pixel(Point::new(step as i32, RASTER_ROW as i32), on),
        // The diagonal: one pixel per row pair, at that row pair's own column.
        2 => {
            let row = sweep_row(step);
            fb.set_pixel(Point::new(row as i32 * 2, row as i32), on);
        }
        // The bar: a whole row pair at a time. The coarse probe, for a phone
        // camera or a bright room.
        _ => {
            let row = sweep_row(step);
            for column in 0..PANEL_WIDTH as i32 {
                fb.set_pixel(Point::new(column, row as i32), on);
            }
        }
    }
}

/// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"rp-hub75: ghosting / bleed probe"),
    hal::binary_info::rp_program_build_attribute!(),
];
