//! A starfield on the embassy-rp backend: the view out of a spaceship's window.
//!
//! The same display as `starfield` — stars in a depth field ahead of the camera,
//! streaming outward as the camera flies forward — on an embassy executor. A
//! star's distance from the center is `offset * Z_FAR / z`; a star that passes the
//! near plane, or leaves the panel on its way out, is re-spawned on the far plane,
//! so the field never empties.
//!
//! The differences from `starfield` are all in the setup: embassy's
//! `#[embassy_executor::main]` replaces `#[entry]`; the `Pio` is built before the
//! pin set, because a PIO pin is made from it; the DMA channels are handed over as
//! `ChannelInstance`s; the frame boundary is awaited with `wait_for_done().await`;
//! and the seed for the field comes from `embassy-time`. The projection, the
//! palette and the depth ramp are unchanged.
//!
//! ```text
//! # probe attached:
//! cd examples/starfield-embassy && cargo run-rp2350
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
use embassy_time::Instant;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use rp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
use rp_hub75::framebuffer::compute_rows;

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

/// How many stars are in the sky.
const STARS: usize = 32;

/// The far plane, where a star enters: its spawn depth, in field units.
const Z_FAR: i32 = 1024;

/// The near plane, where a star is too close and is recycled.
const Z_NEAR: i32 = 32;

/// The flight speed, in field units a second.
///
/// The speed is per *second*, not per frame, so the panel clock does not change
/// it: a faster clock refreshes more often and each frame steps correspondingly
/// less. `Z_FAR` to `Z_NEAR` is 992 units, so a star that survives the whole fall
/// takes a little over a second and a half.
const SPEED_PER_SEC: i32 = 660;

/// Half-width of the spawn window, in pixels.
///
/// A star enters within this of the panel's center, so the field fills the view
/// at spawn and every star streams outward from there.
const SPAWN_SPREAD: i32 = 30;

/// A star is recycled once its projection is this far past the panel edge, so it
/// does not sit in the array off-screen until it reaches the near plane.
const CULL: i32 = 34;

/// The panel's center, the point every star streams away from.
const CX: i32 = PANEL_WIDTH as i32 / 2;
const CY: i32 = PANEL_HEIGHT as i32 / 2;

/// The brightest a star gets, as an 8-bit channel value: a near star.
///
/// The crate keeps the top `PLANES` bits, so this is light level 63 — the top of
/// the panel's range.
const BRIGHT_MAX: i32 = 255;

/// The dimmest a star gets: a star at the far plane still shows, rather than
/// popping in from black.
const BRIGHT_MIN: i32 = 8;

/// The tints a star can take, from a blue-white through white and yellow to red.
///
/// Real stars are not all white, and a field of one colour reads as flat; this is
/// the whole of the colour in the display. Tints are chosen at spawn and scaled by
/// depth, so a red star is dim until it is close.
const PALETTE: [Rgb888; 7] = [
    Rgb888::new(255, 255, 255), // white
    Rgb888::new(190, 210, 255), // blue-white
    Rgb888::new(140, 175, 255), // blue
    Rgb888::new(255, 240, 205), // warm white
    Rgb888::new(255, 210, 140), // yellow
    Rgb888::new(255, 165, 95),  // orange
    Rgb888::new(255, 110, 80),  // red
];

/// The panel clock, as in the esp-hub75 examples: 10 MHz by default, 20 MHz with
/// the `20mhz` feature.
///
/// The driver rounds the divider up, so what the panel runs at is this or the next
/// slower divider's clock.
const PANEL_CLOCK_HZ: u32 = if cfg!(feature = "20mhz") {
    20_000_000
} else {
    10_000_000
};

/// A star: where it entered the view, how deep it is, and its tint.
struct Star {
    /// Where the star entered, as a pixel offset from the panel's center.
    ox: i32,
    oy: i32,
    /// Depth: `Z_FAR` at spawn, falling by the frame's step each pass.
    z: i32,
    /// The star's full-brightness tint, chosen at spawn.
    color: Rgb888,
}

/// A 32-bit xorshift generator: the field's only source of randomness.
///
/// Not cryptographic and not meant to be — it places stars. It is here, rather
/// than in a dependency, because one display is not worth an RNG crate.
struct Rng(u32);

impl Rng {
    /// Creates a generator from `seed`, replacing a zero seed.
    ///
    /// xorshift cannot leave zero, so a zero seed would draw an empty sky.
    fn new(seed: u32) -> Self {
        Rng(if seed == 0 { 0x2545_f491 } else { seed })
    }

    /// Returns the next 32-bit value.
    fn next(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Returns a value in `lo..hi`.
    fn range(&mut self, lo: i32, hi: i32) -> i32 {
        lo + (self.next() % (hi - lo) as u32) as i32
    }

    /// Returns a star on the far plane at a new spot and with a new tint.
    fn spawn(&mut self) -> Star {
        Star {
            ox: self.range(-SPAWN_SPREAD, SPAWN_SPREAD + 1),
            oy: self.range(-SPAWN_SPREAD, SPAWN_SPREAD + 1),
            z: Z_FAR,
            color: PALETTE[self.next() as usize % PALETTE.len()],
        }
    }

    /// Returns a star anywhere in the field, for the initial fill.
    ///
    /// The first sky must not be a wave: if every star started on the far plane
    /// they would all be recycled together. A random depth staggers them.
    fn spawn_anywhere(&mut self) -> Star {
        let mut star = self.spawn();
        star.z = self.range(Z_NEAR + 1, Z_FAR + 1);
        star
    }
}

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

    let hub75 = Hub75::new(
        pio,
        p.DMA_CH0,
        p.DMA_CH1,
        pins,
        hal::clocks::clk_sys_freq(),
        Hub75Config::new(hal::pac::Interrupt::DMA_IRQ_0).with_panel_clock_hz(PANEL_CLOCK_HZ),
        fb0,
        hub75_descriptors!(Fb),
    )
    .expect("failed to start the panel");

    // The driver's own numbers, once. The display carries the picture; the log
    // carries the configuration behind it.
    defmt::info!(
        "{}x{} panel, {} stars, {} planes",
        PANEL_WIDTH,
        PANEL_HEIGHT,
        STARS,
        PLANES
    );
    defmt::info!(
        "panel clock {} Hz -> divider /{} -> {} Hz, {} Hz refresh, {} runs a refresh",
        PANEL_CLOCK_HZ,
        hub75.divider(),
        hub75.panel_clock_hz(),
        hub75.refresh_hz(),
        hub75.runs_per_refresh()
    );

    // The frame's step. The panel clock sets the refresh rate and one pass of
    // this loop is one refresh, so a per-second speed has to be divided by that
    // rate to stay a wall-clock speed: the `20mhz` feature roughly doubles the
    // refresh, and a fixed per-frame step would fly twice as fast under it.
    // `carry` holds the division's remainder, so the average step is exact
    // rather than rounded down — which is what a coarse step needs at the faster
    // refresh.
    let refresh = (hub75.refresh_hz() as i32).max(1);
    let step = SPEED_PER_SEC / refresh;
    let rem = SPEED_PER_SEC % refresh;
    let mut carry = 0;

    // A seed from the clock, so a re-power draws a different sky.
    let mut rng = Rng::new(Instant::now().as_ticks() as u32);
    let mut stars: [Star; STARS] = core::array::from_fn(|_| rng.spawn_anywhere());

    // fb0 is on the panel now; this loop draws into fb1 until the first swap.
    let mut fb = fb1;

    loop {
        // The step for this frame: the base step, plus one unit on the frames
        // where the carried remainder has added up to a whole one.
        carry += rem;
        let extra = if carry >= refresh {
            carry -= refresh;
            1
        } else {
            0
        };

        advance(&mut stars, &mut rng, step + extra);
        draw(fb, &stars);

        match hub75.swap(fb) {
            Ok(mut swap) => {
                swap.wait_for_done().await;
                fb = swap.wait();
            }
            Err((_error, returned)) => fb = returned,
        }
    }
}

/// Moves the camera one step: every star falls in depth by `step`, and the ones
/// that pass the near plane or leave the view are replaced.
///
/// The step is an argument, not a constant, because it is the per-second speed
/// divided by the refresh rate — the panel clock's rate — and so belongs with the
/// driver's `refresh_hz()` (see `main`).
///
/// The near-plane test comes before the projection, so the division is never by
/// zero or by a star at the near plane.
fn advance(stars: &mut [Star; STARS], rng: &mut Rng, step: i32) {
    for star in stars.iter_mut() {
        star.z -= step;
        if star.z <= Z_NEAR {
            *star = rng.spawn();
            continue;
        }

        let dx = star.ox * Z_FAR / star.z;
        let dy = star.oy * Z_FAR / star.z;
        if dx.abs() > CULL || dy.abs() > CULL {
            *star = rng.spawn();
        }
    }
}

/// Draws the field: every star projected to the panel and faded by its depth.
fn draw(fb: &mut Fb, stars: &[Star; STARS]) {
    fb.erase();

    for star in stars {
        let dx = star.ox * Z_FAR / star.z;
        let dy = star.oy * Z_FAR / star.z;
        let color = tint(star.color, brightness(star.z));
        fb.set_pixel(Point::new(CX + dx, CY + dy), color);
    }
}

/// Maps a star's depth to a brightness: the near plane is [`BRIGHT_MAX`] and the
/// far plane is [`BRIGHT_MIN`].
fn brightness(z: i32) -> u8 {
    let span = Z_FAR - Z_NEAR;
    let near = (Z_FAR - z).clamp(0, span);
    (BRIGHT_MIN + (BRIGHT_MAX - BRIGHT_MIN) * near / span) as u8
}

/// Scales a star's tint by `brightness`, so distance reads as dimness.
fn tint(color: Rgb888, brightness: u8) -> Rgb888 {
    let b = brightness as u16;
    Rgb888::new(
        (color.r() as u16 * b / 255) as u8,
        (color.g() as u16 * b / 255) as u8,
        (color.b() as u16 * b / 255) as u8,
    )
}
