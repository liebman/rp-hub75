//! The read-back ladder: five patterns, each one naming a fault.
//!
//! The rungs, re-expressed through [`Hub75`] and kept as a *regression* tool
//! rather than a bring-up one. `gradient` and `ghost` are the
//! eye's version of the same questions; this is the version whose failure modes
//! are specific.
//!
//! | rung | what it lights | a wrong picture means |
//! |---|---|---|
//! | **A** | every pixel white | one row pair only → the stream stopped (DMA or FIFO); all dark → words are not arriving |
//! | **B** | the top half white | both halves lit → the halves are crossed; a left/right split → the colour pins are not where `pins.rs` says |
//! | **C** | the bottom half white | as B, mirrored |
//! | **D** | full-height columns at x = 0, 1, 2 in red, green and blue | columns at x = 1, 2, 3 → **the panel latches on `STB`'s rising edge**, so the last column is captured early; `tail-closes-latch` is the crate's answer. At the *right* edge → the shift direction is reversed; colours out of order → the R0/G0/B0 order |
//! | **E** | the diagonal: one pixel per row, one column to the right per row, white above the join and blue below | a kink or a column shift → a real fault; the lower arm above the join → the halves are crossed; every row shifted a whole row → the address lines are a row out |
//!
//! Two seconds a rung, looping, and the log says which one is up: the patterns
//! carry no label of their own, because rung A has to be uniformly white.
//!
//! The ladder is also the clock question. It takes the same `20mhz` feature as the
//! other examples; rebuild it with a faster divider — a `/3` divider is 25 MHz on
//! the examples' 150 MHz system clock — and the rung that goes wrong first is the
//! ribbon's ceiling, not the driver's.
//!
//! ```text
//! # probe attached:
//! cargo run-rp2350 --example ladder
//! ```
#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use panic_probe as _;
use rp_hub75::hal;

use hal::Sio;
use hal::clocks::Clock;
use hal::dma::DMAExt;
use hal::fugit::Rate;
use hal::gpio::Pins;
use rp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
use rp_hub75::framebuffer::compute_rows;

use rp_hub75::Hub75;
use rp_hub75::Hub75Config;
use rp_hub75::Hub75Pins16;
use rp_hub75::hub75_descriptors;

/// The engine's one interrupt: apply a swap, and set the next frame going.
#[unsafe(no_mangle)]
extern "C" fn DMA_IRQ_1() {
    rp_hub75::on_dma_irq();
}

/// Tell the Boot ROM about our application.
#[cfg(feature = "rp2350")]
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// The RP2040's second-stage bootloader.
///
/// The Boot ROM executes the 256 bytes at the start of flash before anything
/// else, so without them the chip never reaches `main`. `rp2040-boot2` supplies
/// the code for a W25Q-series flash (the reference boards want this one); a board
/// with a different flash needs `BOOT_LOADER_GENERIC_03H` here instead. The
/// RP2350 boots the vector table directly and needs none of this.
#[cfg(feature = "rp2040")]
#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// The panel this example is wired to: 64×64, 1/32 scan.
///
/// This is the whole panel description. Change these for a different panel and
/// the framebuffer type below follows from them.
const PANEL_WIDTH: usize = 64;
const PANEL_HEIGHT: usize = 64;

/// Row pairs: a 1/32-scan panel lights its top half and its bottom half at once.
const NROWS: usize = compute_rows(PANEL_HEIGHT);

/// Six planes: 63 panel scans a frame, where eight flickers. See the crate docs.
const PLANES: usize = 6;

/// The framebuffer type this example uses: `hub75-framebuffer`'s plane-major
/// layout, whose words are the panel's own pin levels.
///
/// The two of them are plain statics, and that costs nothing: the engine names
/// each run's address in the descriptor table, so neither a framebuffer's address
/// nor its alignment changes the picture or the interrupt load.
type Fb = DmaFrameBuffer<NROWS, PANEL_WIDTH, PLANES>;

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

/// The rungs in order, and the line each one logs as it goes up.
const RUNGS: [&str; 5] = [
    "A: every pixel white",
    "B: the top half white",
    "C: the bottom half white",
    "D: red, green and blue columns at x = 0, 1, 2",
    "E: the diagonal, white above the join and blue below",
];

/// How long one rung is held.
const RUNG_MS: u64 = 2_000;

/// Crystal frequency on the Interstate 75 W, in Hz.
///
/// The reference board fits a 12 MHz crystal, and the PLLs are configured from
/// it.
const XTAL_FREQ_HZ: u32 = 12_000_000;

/// The panel clock, as in the other examples: 10 MHz by default, 20 MHz with the
/// `20mhz` feature.
///
/// The driver rounds the divider up, so what the panel runs at is this or the next
/// slower divider's clock — `Hub75::panel_clock` and `Hub75::divider` are the
/// truth, not this number.
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
    #[cfg(feature = "rp2350")]
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    #[cfg(feature = "rp2040")]
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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
        Hub75Pins16::new(
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
            // STB and OE are the word's last two signal bits; the clock is not a
            // signal at all — the side-set drives it, and it is wired to GP11.
            pins.gpio12,
            pins.gpio13,
            pins.gpio11,
        ),
        clocks.system_clock.freq(),
        // The one knob that trades timing margin for refresh rate: a rung that
        // tears or smears only as the clock rises is the ribbon's ceiling.
        Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_1).with_panel_clock(PANEL_CLOCK),
        fb0,
        hub75_descriptors!(Fb),
    )
    .expect("failed to start the panel");

    let mut fb = fb1;
    // Behind by one, so the first rung drawn is A.
    let mut rung = RUNGS.len() - 1;
    let mut deadline = timer.get_counter().ticks();

    loop {
        let now = timer.get_counter().ticks();
        if now >= deadline {
            rung = (rung + 1) % RUNGS.len();
            deadline = now + RUNG_MS * 1_000;
            defmt::info!("{}", RUNGS[rung]);
        }

        draw(fb, rung);

        match hub75.swap(fb) {
            Ok(swap) => fb = swap.wait(),
            Err((_error, returned)) => fb = returned,
        }
    }
}

/// Draws one rung. See the module docs for what each pattern is for.
///
/// Every colour is full intensity: the crate keeps the top `PLANES` bits of a
/// drawn byte, so 255 is light level 63, the top of the panel's range. No rung is
/// a brightness test — `gradient` is.
fn draw(fb: &mut Fb, rung: usize) {
    fb.erase();
    match rung {
        // A: every row pair, every column, both halves.
        0 => fill(
            fb,
            0,
            PANEL_WIDTH as i32,
            0,
            PANEL_HEIGHT as i32,
            Rgb888::WHITE,
        ),
        // B: the top half, and C the bottom half.
        1 => fill(fb, 0, PANEL_WIDTH as i32, 0, 32, Rgb888::WHITE),
        2 => fill(
            fb,
            0,
            PANEL_WIDTH as i32,
            32,
            PANEL_HEIGHT as i32,
            Rgb888::WHITE,
        ),
        // D: three full-height columns down the left edge, in bus order. A row
        // latched one column early puts them at x = 1, 2, 3.
        3 => {
            for (x, colour) in [(0, Rgb888::RED), (1, Rgb888::GREEN), (2, Rgb888::BLUE)] {
                fill(fb, x, x + 1, 0, PANEL_HEIGHT as i32, colour);
            }
        }
        // E: the diagonal — one pixel per row, one column to the right each row,
        // the colour change marking the join between the halves.
        _ => {
            for row in 0..PANEL_HEIGHT as i32 {
                let colour = if row < 32 {
                    Rgb888::WHITE
                } else {
                    Rgb888::BLUE
                };
                fb.set_pixel(Point::new(row, row), colour);
            }
        }
    }
}

/// Fills the rectangle `[x_from, x_to) × [y_from, y_to)`.
///
/// A whole panel is 4096 `set_pixel` calls — a fraction of the 13.7 ms a frame
/// takes here — and it happens once per rung rather than once per frame.
fn fill(fb: &mut Fb, x_from: i32, x_to: i32, y_from: i32, y_to: i32, colour: Rgb888) {
    for y in y_from..y_to {
        for x in x_from..x_to {
            fb.set_pixel(Point::new(x, y), colour);
        }
    }
}

/// Program metadata for `picotool info`.
#[cfg(feature = "rp2350")]
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"rp-hub75: the read-back ladder"),
    hal::binary_info::rp_program_build_attribute!(),
];
