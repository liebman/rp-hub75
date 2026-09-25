//! M5: the driver API, double buffering, and a tear-free swap.
//!
//! Three lines of setup and then a loop:
//!
//! ```text
//! let hub75 = Hub75::new(pio, ch0, ch1, &mut resets, pins, sys_clock, config, fb0)?;
//! ...
//! fb = hub75.swap(fb)?.wait();   // hand over the buffer you drew, get back the other
//! ```
//!
//! `wait()` blocks until the swap has taken effect: the frame being scanned
//! finishes on the outgoing buffer, the next one shows the incoming buffer, and
//! the outgoing one is then handed straight back. Drawing and display never touch
//! the same framebuffer, and the engine underneath is one DMA channel whose
//! completion interrupt walks the framebuffer’s own BCM sequence — so the panel
//! keeps refreshing while this loop draws, and a swap never tears.
//!
//! What is on the panel: a **static linear ramp** — column *n* is light level
//! *n*, so every step is the same amount of light and the 64 steps can be
//! compared in place. That is the BCM test: a plane with the wrong repetition
//! count kinks the straight line at its boundary (levels 1, 2, 4, 8, 16, 32),
//! and a plane that never lights flattens everything above its level.
//!
//! Gamma is deliberately **not** applied. It would make the ramp look even to the
//! eye, which is what a UI wants, but it compresses the dark end and blurs
//! exactly the steps being checked here. `rp_hub75::color::gamma_step` is there
//! when the picture should look right rather than measure right.
//!
//! One red marker sweeps across it, and that marker is the other test: the loop
//! only advances when each swap completes, so a marker that stops means the swap
//! path has hung, and the frozen ramp makes that obvious.
//!
//! ```text
//! # board in BOOTSEL mode:
//! cargo run-arm --example gradient
//! ```
#![no_std]
#![no_main]

use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::Rgb888;
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
use rp_hub75::color::level;
use rp_hub75::{PANEL_HEIGHT, PANEL_WIDTH, ROW_PAIRS, XTAL_FREQ_HZ};

/// The interrupt that walks the framebuffer's BCM sequence.
///
/// The driver is re-armed from here once per run, so this handler *is* the
/// refresh: the panel stops if it does not run, and the line must be the one
/// named in `Hub75Config::new`.
#[unsafe(no_mangle)]
extern "C" fn DMA_IRQ_1() {
    rp_hub75::on_dma_irq();
}

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// The panel geometry the framebuffer is built for.
const NROWS: usize = ROW_PAIRS;
const WORDS_PER_ROW: usize = PANEL_WIDTH;
const PLANES: usize = rp_hub75::BIT_DEPTH as usize;

/// One framebuffer. No alignment is needed any more: the data channel has no
/// read ring, so a plane can live anywhere and is identified by its address.
type Fb = DmaFrameBuffer<NROWS, WORDS_PER_ROW, PLANES>;

/// A framebuffer aligned so the DMA's ring can repeat its segments: a ring wraps
/// the power-of-two boundary containing an address, so aligning the buffer is
/// what lets `reps` be repeated in hardware — 32 runs per refresh become 10 for
/// this six-plane frame. Unaligned works too, just with more interrupts.
#[repr(align(4096))]
struct Aligned(Fb);

static mut FB0: Aligned = Aligned(Fb::new());
static mut FB1: Aligned = Aligned(Fb::new());

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

    // SAFETY: both statics are taken exactly once, and each is handed to the
    // driver as a `&'static mut` — one of them is owned by the panel from here
    // on, and the other by this loop, strictly alternating.
    let fb0: &'static mut Fb = unsafe { &mut (*core::ptr::addr_of_mut!(FB0)).0 };
    let fb1: &'static mut Fb = unsafe { &mut (*core::ptr::addr_of_mut!(FB1)).0 };

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
            .with_panel_clock(Rate::<u32, 1, 1>::kHz(18_750)),
        fb0,
    )
    .expect("failed to start the panel");

    // ------------------------------------------------------------------
    // The drawing loop: one buffer on the panel, one in hand
    // ------------------------------------------------------------------
    let mut fb = fb1;
    let mut phase = 0u32;
    loop {
        draw(fb, phase);

        // Hand the buffer over and wait for the other one back. `wait()` blocks
        // for up to a frame, which is exactly the time the swap needs.
        match hub75.swap(fb) {
            Ok(swap) => {
                fb = swap.wait();
            }
            // Cannot happen: every swap is waited for before the next.
            Err((error, returned)) => {
                let _ = error;
                fb = returned;
            }
        }

        phase = phase.wrapping_add(1);
        // About 15 frames a second, so a sweep across the panel takes ~4 s and
        // a stop is obvious.
        let until = timer.get_counter().ticks() + 60_000;
        while timer.get_counter().ticks() < until {}
    }
}

/// A linear ramp across the panel, with a marker that sweeps right.
///
/// The ramp is **static and linear**: column *n* is light level *n*, so every step
/// is the same amount of light, and a wrong BCM weight shows as a kink that stays
/// put instead of sliding past. No gamma: it would even out the very steps this
/// exists to check.
///
/// The one moving thing is a red marker, and that is deliberate — it is the test.
/// The loop only advances if each swap completes, so a marker that stops means the
/// swap path has hung, and the frozen ramp makes it unmistakable.
fn draw(fb: &mut Fb, phase: u32) {
    fb.erase();
    let width = PANEL_WIDTH as u32;
    let last = rp_hub75::color::LEVELS - 1;
    for row in 0..PANEL_HEIGHT as i32 {
        for column in 0..PANEL_WIDTH as i32 {
            let value = level(column as u32 * last / (width - 1));
            fb.set_pixel(Point::new(column, row), Rgb888::new(value, value, value));
        }
    }

    // Two columns wide and four rows tall: big enough to spot at a glance, small
    // enough to leave the ramp readable behind it.
    let marker = Rgb888::new(level(last), 0, 0);
    let x = (phase % width) as i32;
    for row in 0..4 {
        for column in 0..2 {
            fb.set_pixel(Point::new((x + column) % width as i32, row), marker);
        }
    }
}
