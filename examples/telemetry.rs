//! M5: double buffering, the engine interrupt, and telemetry over RTT.
//!
//! The same driver as `gradient`, with the two things a real application wants
//! on top:
//!
//! * **The interrupt.** One per frame, and it is the only one: the control
//!   channel plays the descriptor table and the data channel chains back to it,
//!   so the handler runs at the terminator — to apply a swap and restart the
//!   chain — and nowhere else. `IRQ_COUNT` counts its entries, so it should track
//!   the frame count one for one.
//! * **Telemetry** over RTT, because GP0/GP1 are panel data and there is no UART:
//!   frames, interrupts and swaps once a second, through `defmt` to the debug
//!   probe. The startup line reads the driver's own numbers — divider, panel
//!   clock, refresh rate and runs a refresh — so one `probe-rs run` shows the
//!   configuration and the counters together.
//!
//! The panel shows a static *linear* ramp — equal light per column, no gamma, so
//! a wrong BCM weight kinks the line — with the loop's frame count on top, and
//! that display is the point: it needs no host to spot a stall. The digit is
//! blurred by the frame rate, so a digit that stops moving while the ramp stays put
//! means the *loop* has stopped even though the panel is still refreshing.
//!
//! ```text
//! # probe attached:
//! cargo run-arm --example telemetry
//! ```
#![no_std]
#![no_main]

use core::future::Future;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use defmt_rtt as _;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use panic_probe as _;
use rp235x_hal as hal;

use hal::Sio;
use hal::clocks::Clock;
use hal::dma::DMAExt;
use hal::fugit::Rate;
use hal::gpio::Pins;
use hub75_framebuffer::bitplane::plain::DmaFrameBuffer;
use hub75_framebuffer::compute_rows;

use rp_hub75::Hub75;
use rp_hub75::Hub75Config;
use rp_hub75::Hub75Pins;
use rp_hub75::hub75_descriptors;

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// The panel this example is wired to: 64×64, 1/32 scan.
///
/// This is the whole panel description. Change these for a different panel and
/// the framebuffer type below follows from them.
const PANEL_WIDTH: usize = 64;
const PANEL_HEIGHT: usize = 64;

/// Row pairs: a 1/32-scan panel lights its top half and its bottom half at once.
const NROWS: usize = compute_rows(PANEL_HEIGHT);

/// The BCM depth. Six planes: 63 panel scans a frame, where eight flickers.
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

/// Interrupts the panel's DMA completion handler has raised.
///
/// One a frame, at the descriptor table's terminator.
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

/// The application's half of the wake-up contract.
///
/// The line must be the one named in `Hub75Config::with_wake_irq`; the driver
/// cannot check that, so the two are documented together here. The `irqs` column
/// reports this handler's own count, so a stalled engine shows up as a number
/// that stops rising.
///
/// The handler *is* the vector-table symbol. `rp235x-hal` does not re-export
/// `cortex_m_rt::interrupt`, and that macro in device mode resolves a crate
/// named `interrupt`, which this PAC does not provide, so naming the weak vector
/// symbol directly is both shorter and exactly what the macro would emit.
#[unsafe(no_mangle)]
extern "C" fn DMA_IRQ_1() {
    IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    rp_hub75::on_dma_irq();
}

/// A one-shot executor: drives a future to completion.
///
/// The waker only has to exist — parking the caller is what it is for, not
/// scheduling, and the interrupt is what ends the wait — so its vtable does
/// nothing and the loop simply re-polls. That is enough for `wait_for_done`,
/// which reads a flag the interrupt sets.
fn block_on<F: Future>(future: F) -> F::Output {
    static DUMMY: () = ();
    const VTABLE: RawWakerVTable = RawWakerVTable::new(|_| raw_waker(), |_| {}, |_| {}, |_| {});
    const fn raw_waker() -> RawWaker {
        RawWaker::new(core::ptr::addr_of!(DUMMY), &VTABLE)
    }
    let mut future = core::pin::pin!(future);
    // SAFETY: the waker's vtable functions ignore the data pointer, which
    // points at a `'static` byte.
    let waker = unsafe { Waker::from_raw(raw_waker()) };
    let mut context = Context::from_waker(&waker);
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut context) {
            return value;
        }
        core::hint::spin_loop();
    }
}

/// Crystal frequency on the Interstate 75 W, in Hz.
///
/// The reference board fits a 12 MHz crystal, and the PLLs are configured from
/// it.
const XTAL_FREQ_HZ: u32 = 12_000_000;

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

    let hub75_pins = Hub75Pins::new(
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
    );

    // One engine, one panel, one interrupt a frame: the control channel plays the
    // descriptor table and the data channel chains back to it.
    let hub75 = Hub75::new(
        pac.PIO0,
        dma.ch0,
        dma.ch1,
        &mut pac.RESETS,
        hub75_pins,
        clocks.system_clock.freq(),
        Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_1).with_panel_clock(PANEL_CLOCK),
        fb0,
        hub75_descriptors!(Fb),
    )
    .expect("failed to start the panel");

    let mut reports: u32 = 0;
    let mut last_report = timer.get_counter().ticks();
    let mut frames: u32 = 0;
    let mut swaps: u32 = 0;

    defmt::info!(
        "telemetry: {}x{}, {} planes, divider /{}, {} Hz panel clock, {} Hz refresh, {} \
         runs a refresh, one interrupt a frame",
        PANEL_WIDTH,
        PANEL_HEIGHT,
        PLANES,
        hub75.divider(),
        hub75.panel_clock().to_Hz(),
        hub75.refresh_hz(),
        hub75.runs_per_refresh()
    );

    let mut fb = fb1;
    loop {
        let now = timer.get_counter().ticks();

        // Draw, then hand the buffer over and park until the swap is applied.
        // The digit on the panel is this counter, so watching it move is watching
        // the loop (and therefore the swaps) run.
        draw(fb, frames);
        frames += 1;

        match hub75.swap(fb) {
            Ok(mut swap) => {
                block_on(swap.wait_for_done());
                fb = swap.wait();
                swaps += 1;
            }
            Err((_error, returned)) => fb = returned,
        }

        // One line a second. One interrupt a frame, so `irqs` and `frames` move
        // together; RTT drops a line rather than blocking if the host is not
        // reading, so this cannot stop the panel.
        if now - last_report >= 1_000_000 {
            last_report = now;
            reports += 1;
            defmt::info!(
                "report {}: {} frames, {} irqs, {} swaps",
                reports,
                frames,
                IRQ_COUNT.load(Ordering::Relaxed),
                swaps
            );
            frames = 0;
        }
    }
}

/// One frame of the animation: a sweeping bar, with the frame digit on top.
/// One frame of the panel's own display: a static **linear** ramp with the frame
/// count on top.
///
/// Same shape as `gradient`, and for the same reason: column *n* is light level
/// *n*, so every step is the same amount of light, which is what makes a wrong BCM
/// weight visible as a kink instead of being smoothed away by gamma. The moving
/// element is the alarm, and here it is the digit — it comes straight from the
/// loop's counter, so a digit that stops ticking means the loop stopped, and a
/// stopped loop over a stable ramp is unmistakable.
fn draw(fb: &mut Fb, frames: u32) {
    fb.erase();
    let width = PANEL_WIDTH as u32;
    // The crate keeps the top `PLANES` bits of a drawn byte, so a light level
    // has to be shifted up to the top of the byte before it is drawn: level
    // `last` is full scale, and the bits below `8 - PLANES` are dropped.
    let last = (1u32 << PLANES) - 1;
    let shift = 8 - PLANES as u32;
    for row in 0..PANEL_HEIGHT as i32 {
        for column in 0..PANEL_WIDTH as i32 {
            let value = ((column as u32 * last / (width - 1)) << shift) as u8;
            fb.set_pixel(Point::new(column, row), Rgb888::new(value, value, value));
        }
    }

    let style = MonoTextStyle::new(&FONT_6X10, Rgb888::new(255, 64, 0));
    let digit = [b'0' + (frames % 10) as u8];
    let _ = Text::new(
        core::str::from_utf8(&digit).unwrap_or("?"),
        Point::new(2, 10),
        style,
    )
    .draw(fb);
}

/// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"rp-hub75 M5: double buffering, wake IRQ, telemetry over RTT"
    ),
    hal::binary_info::rp_program_build_attribute!(),
];
