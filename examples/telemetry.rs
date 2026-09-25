//! M5: double buffering, the engine interrupt, and telemetry over USB CDC.
//!
//! The same driver as `gradient`, with the two things a real application wants
//! on top:
//!
//! * **The interrupt.** One per run, and it is what keeps the picture moving —
//!   the ISR walks the framebuffer's BCM sequence and re-arms the channel, so
//!   this example's handler is the refresh. The `irqs` column is that count:
//!   runs per refresh times frames per second.
//! * **Telemetry** over USB CDC, because GP0/GP1 are panel data and there is no
//!   UART: fps, the interrupt count and the swap count, once a second. The
//!   writes are bounded and drop lines rather than block the loop — telemetry
//!   must never be able to stop the panel.
//!
//! The panel shows the same static *linear* ramp as `gradient` — equal light per
//! column, no gamma, so a wrong BCM weight kinks the line — with the loop’s frame
//! count on top. The bench therefore needs no host at all to spot a stall: the
//! digit ticks continuously (frames modulo ten, about fifteen times a second
//! here), and if it ever stops, the loop has.
//!
//! ```text
//! # board in BOOTSEL mode:
//! cargo run-arm --example telemetry
//! # then, on the host:
//! cat /dev/cu.usbmodem*
//! ```
#![no_std]
#![no_main]

use core::fmt::{self, Write};
use core::future::Future;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use panic_halt as _;
use rp235x_hal as hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

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

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// The panel geometry the framebuffers are built for.
const NROWS: usize = ROW_PAIRS;
const WORDS_PER_ROW: usize = PANEL_WIDTH;
const PLANES: usize = BIT_DEPTH as usize;

type Fb = DmaFrameBuffer<NROWS, WORDS_PER_ROW, PLANES>;
type UsbBus = hal::usb::UsbBus;

/// A framebuffer aligned so the DMA's ring can repeat its segments.
///
/// A ring wraps the power-of-two boundary *containing an address*, so an aligned
/// framebuffer is what lets `reps` be repeated in hardware: with it, this frame's
/// 32 runs per refresh become 10 (planes 4 and 5, the 8- and 16-rep segments,
/// collapse into one run each). Unaligned works exactly the same, just for more
/// interrupts — `Hub75::runs_per_refresh()` reports whichever is true.
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

/// Interrupts the panel's control channel has raised.
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

/// The application's half of the wake-up contract.
///
/// The line must be the one named in `Hub75Config::with_wake_irq`; the driver
/// cannot check that, so the two are documented together here. Counting the
/// interrupts is what makes the handler's effect visible in the telemetry: with
/// no swap in flight there is nothing to count.
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

/// A USB serial writer for telemetry.
///
/// Deliberately **not** the "write every byte, whatever it takes" writer that
/// `hello_usb` uses. This one runs inside the display loop, so a host that stops
/// reading must not stop the panel: the write tries for a bounded number of USB
/// polls and then gives up, dropping the rest of the line. Losing a line of
/// telemetry is fine; a frozen panel is not.
struct UsbLog<'a, 'b> {
    usb_dev: &'a mut UsbDevice<'b, UsbBus>,
    serial: &'a mut SerialPort<'b, UsbBus>,
}

/// USB polls one log write may cost before it gives up.
const LOG_POLL_BUDGET: u32 = 2_000;

impl fmt::Write for UsbLog<'_, '_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let mut remaining = s.as_bytes();
        for _ in 0..LOG_POLL_BUDGET {
            if remaining.is_empty() {
                return Ok(());
            }
            self.usb_dev.poll(&mut [&mut *self.serial]);
            if let Ok(count) = self.serial.write(remaining) {
                remaining = &remaining[count..];
            }
        }
        Err(fmt::Error)
    }
}

/// A one-shot executor: drives a future to completion, servicing the USB stack
/// between polls.
///
/// The swap is driven by the poll itself (and by the interrupt), so the waker
/// only has to exist — parking the caller is what it is for, not scheduling. The
/// `service` closure is what an executor does with the time: keeping the USB
/// stack alive means parking on a swap costs the telemetry nothing.
fn block_on<F: Future, S: FnMut()>(future: F, mut service: S) -> F::Output {
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
        service();
    }
}

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

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("rp-hub75")
            .product("Interstate 75 W (RP2350)")
            .serial_number("rp-hub75")])
        .unwrap()
        .max_packet_size_0(64)
        .unwrap()
        .device_class(2) // CDC
        .build();

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
            .with_panel_clock(Rate::<u32, 1, 1>::kHz(18_750)),
        fb0,
    )
    .expect("failed to start the panel");

    let mut reports: u32 = 0;
    let mut banner_sent = false;
    let mut last_report = 0u64;
    let mut frames: u32 = 0;
    let mut swaps: u32 = 0;

    let mut fb = fb1;
    loop {
        usb_dev.poll(&mut [&mut serial]);
        let now = timer.get_counter().ticks();

        // Draw, then hand the buffer over and park until the swap is applied.
        // The digit on the panel is this counter, so watching it tick is watching
        // the loop (and therefore the swaps) run.
        draw(fb, frames);
        frames += 1;

        match hub75.swap(fb) {
            Ok(mut swap) => {
                // Service USB while parked: a swap is up to a frame of waiting,
                // and the host must keep being answered.
                block_on(swap.wait_for_done(), || {
                    usb_dev.poll(&mut [&mut serial]);
                });
                fb = swap.wait();
                swaps += 1;
            }
            Err((_error, returned)) => fb = returned,
        }

        if !banner_sent && now >= 2_000_000 {
            banner_sent = true;
            last_report = now;
            let mut logger = UsbLog {
                usb_dev: &mut usb_dev,
                serial: &mut serial,
            };
            // A failed write is a dropped line, not a stalled panel.
            let _ = write!(logger, "{}", Banner(&hub75));
        }

        if banner_sent && now - last_report >= 1_000_000 {
            last_report = now;
            reports += 1;
            let mut logger = UsbLog {
                usb_dev: &mut usb_dev,
                serial: &mut serial,
            };
            // The irqs column is now the engine's own heartbeat: one interrupt
            // per run, so it should sit at runs/refresh × frames per second.
            let _ = write!(
                logger,
                "{:>4}  {:>5}  {:>4}  {:>11}\r\n",
                reports,
                frames,
                IRQ_COUNT.load(Ordering::Relaxed),
                swaps,
            );
            frames = 0;
        }
    }
}

/// The one-time description of what the panel is doing.
struct Banner<'a>(&'a Hub75<Fb>);

impl fmt::Display for Banner<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "\r\npanel {}x{}, {} planes, {} Hz refresh, divider /{}\r\n\
             {} runs per refresh — one interrupt each, with the DMA's ring\r\n\
             repeats already folded in — walked from the framebuffer's own BCM\r\n\
             sequence; a swap is armed every frame\r\n\
             report  frame   irqs    swap_waited\r\n",
            PANEL_WIDTH,
            PANEL_HEIGHT,
            BIT_DEPTH,
            self.0.refresh_hz(),
            self.0.divider(),
            self.0.runs_per_refresh(),
        )
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
    let last = rp_hub75::color::LEVELS - 1;
    for row in 0..PANEL_HEIGHT as i32 {
        for column in 0..PANEL_WIDTH as i32 {
            let value = rp_hub75::color::level(column as u32 * last / (width - 1));
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
        c"rp-hub75 M5: double buffering, wake IRQ, telemetry"
    ),
    hal::binary_info::rp_program_build_attribute!(),
];
