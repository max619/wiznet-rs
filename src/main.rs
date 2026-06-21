#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f1xx_hal::{
    pac::{self},
    prelude::*,
    rcc, spi,
    timer::Timer,
};

mod w6100;
use crate::w6100::{TcpSocket, W6100};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::hse(8.MHz()).sysclk(16.MHz()).pclk1(24.MHz()),
        &mut flash.acr,
    );

    let mut gpio_a = dp.GPIOA.split(&mut rcc);

    let mut rst = gpio_a.pa8.into_push_pull_output(&mut gpio_a.crh);
    let mut cs: stm32f1xx_hal::gpio::Pin<'A', 9, stm32f1xx_hal::gpio::Output> =
        gpio_a.pa9.into_push_pull_output(&mut gpio_a.crh);
    let mut interrupt = gpio_a.pa10.into_pull_up_input(&mut gpio_a.crh);

    let spi_mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    let mut spi = dp.SPI1.spi(
        (Some(gpio_a.pa5), Some(gpio_a.pa6), Some(gpio_a.pa7)),
        spi_mode,
        1.MHz(),
        &mut rcc,
    );

    let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

    let mut chip = W6100::new(
        embedded_hal_bus::spi::ExclusiveDevice::new(
            spi,
            cs,
            Timer::syst(cp.SYST, &rcc.clocks).delay(),
        )
        .expect("Failed to create exclusive device"),
        rst,
        mac,
    )
    .expect("Failed to init W6100");

    let mut rx = [0u8; 512];
    let mut tx = [0u8; 512];

    let sock = TcpSocket::new(1000, &mut rx, &mut tx);

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        // Wait for link
        while !chip.is_link_up().unwrap() {}

        chip.setup_network(
            u32::from_be_bytes([192, 168, 10, 10]),
            u32::from_be_bytes([192, 168, 10, 1]),
            u32::from_be_bytes([255, 255, 255, 0]),
        )
        .unwrap();

        loop {
            if !chip.is_link_up().unwrap() {
                chip.reset().unwrap();
                break;
            }

            chip.run().unwrap();

            chip.open(&sock).unwrap();
        }
    }
}
