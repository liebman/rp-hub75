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
//! the outgoing one is then handed straight back. Drawing and display never
//! touch the same framebuffer, and the engine underneath is two DMA channels
//! that hand off to each other for ever — no CPU, no interrupts, no drawing
//! stalls.
//!
//! What is on the panel: a moving gamma ramp with a marker dot, so a torn frame
//! is visible as a discontinuity, a wrong BCM weight as a kink in the ramp, and
//! a swap that never completes as the marker stopping.
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
use rp_hub75::color::gamma_step;
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

static mut FB0: Fb = Fb::new();
static mut FB1: Fb = Fb::new();

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
    let fb0: &'static mut Fb = unsafe { &mut *core::ptr::addr_of_mut!(FB0) };
    let fb1: &'static mut Fb = unsafe { &mut *core::ptr::addr_of_mut!(FB1) };

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

        phase = phase.wrapping_add(3);
        // About 15 frames a second, so the motion is easy to follow.
        let until = timer.get_counter().ticks() + 60_000;
        while timer.get_counter().ticks() < until {}
    }
}

/// A gamma ramp that slides across the panel, with a marker dot.
///
/// The ramp spans all 64 levels through the gamma table, so a wrong BCM weight
/// shows as a kink and a torn frame as a discontinuity. The marker moves with
/// the ramp: if it stands still, a swap hung.
fn draw(fb: &mut Fb, phase: u32) {
    fb.erase();
    let width = PANEL_WIDTH as u32;
    for row in 0..PANEL_HEIGHT as i32 {
        for column in 0..PANEL_WIDTH as i32 {
            let step = (column as u32 + phase) % width;
            let value = gamma_step(step);
            fb.set_pixel(Point::new(column, row), Rgb888::new(value, value, value));
        }
    }

    let marker_x = (width - 4) * phase / width;
    let marker = Rgb888::new(level(rp_hub75::color::LEVELS - 1), 0, 0);
    for row in 0..4 {
        for column in 0..4 {
            let x = (marker_x + column) % width;
            fb.set_pixel(Point::new(x as i32, 2 + row), marker);
        }
    }
}
