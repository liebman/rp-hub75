//! M0: prove the toolchain, the crate and the flash flow.
//!
//! Enumerates as a USB serial device and prints a banner (board identity, panel
//! geometry and the HUB75 pin map the driver will use) followed by a
//! once-a-second heartbeat.
//!
//! USB CDC is not just a convenience here: GP0/GP1 are HUB75 data lines on this
//! board, so the usual UART0 logging pins are occupied.
//!
//! ```text
//! # board in BOOTSEL mode (hold BOOT, tap RESET):
//! cargo run-arm --example hello_usb
//! # then, on the host:
//! cat /dev/cu.usbmodem*
//! ```

#![no_std]
#![no_main]

use core::fmt::{self, Write};

use panic_halt as _;
use rp235x_hal as hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use rp_hub75::{BIT_DEPTH, PANEL_HEIGHT, PANEL_WIDTH, ROW_PAIRS, XTAL_FREQ_HZ, pins};

/// The RP2350's USB peripheral, as used by these examples.
type UsbBus = hal::usb::UsbBus;

/// A writer that does not silently drop bytes.
///
/// `SerialPort::write` accepts only what fits in the endpoint buffer and
/// returns how much of it it took, so writing a banner in a single call loses
/// its tail. This polls the USB stack until every byte has been queued, which
/// is what a logging channel needs to do.
struct UsbLog<'a, 'b> {
    usb_dev: &'a mut UsbDevice<'b, UsbBus>,
    serial: &'a mut SerialPort<'b, UsbBus>,
}

impl fmt::Write for UsbLog<'_, '_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let mut remaining = s.as_bytes();
        while !remaining.is_empty() {
            match self.serial.write(remaining) {
                Ok(count) if count > 0 => remaining = &remaining[count..],
                _ => {}
            }
            self.usb_dev.poll(&mut [&mut *self.serial]);
        }
        Ok(())
    }
}

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

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
        .device_class(2) // CDC, see https://www.usb.org/defined-class-codes
        .build();

    let mut banner_sent = false;
    let mut last_tick = 0u64;
    let mut ticks = 0u32;

    loop {
        // Keep the USB stack serviced or nothing we write ever reaches the host.
        usb_dev.poll(&mut [&mut serial]);

        let now = timer.get_counter().ticks();

        if !banner_sent && now >= 2_000_000 {
            banner_sent = true;
            last_tick = now;

            let mut log = UsbLog {
                usb_dev: &mut usb_dev,
                serial: &mut serial,
            };
            write!(
                log,
                "\r\nrp-hub75 on an Interstate 75 W (RP2350)\r\n\
                 panel    {}x{} ({} row pairs, {} bit BCM)\r\n\
                 data     GP{}-GP{}\r\n\
                 rowsel   GP{}-GP{}\r\n\
                 clk/stb/oe  GP{}/{}/{}\r\n\
                 buttons  GP{}/{}/{}, status LED GP{}, Qw/ST GP{}/GP{}\r\n\
                 heartbeat:",
                PANEL_WIDTH,
                PANEL_HEIGHT,
                ROW_PAIRS,
                BIT_DEPTH,
                pins::DATA_BASE,
                pins::DATA_BASE + pins::DATA_PINS - 1,
                pins::ROWSEL_BASE,
                pins::ROWSEL_BASE + pins::ROWSEL_PINS - 1,
                pins::CLK,
                pins::STB,
                pins::OE,
                pins::SWITCH_A,
                pins::SWITCH_B,
                pins::SWITCH_BOOT,
                pins::STATUS_LED,
                pins::I2C_SDA,
                pins::I2C_SCL,
            )
            .unwrap();
        }

        if banner_sent && now - last_tick >= 1_000_000 {
            last_tick = now;
            ticks += 1;

            let mut log = UsbLog {
                usb_dev: &mut usb_dev,
                serial: &mut serial,
            };
            write!(log, " {ticks}").unwrap();
        }
    }
}

/// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"rp-hub75 M0: USB serial banner and heartbeat"),
    hal::binary_info::rp_program_build_attribute!(),
];
