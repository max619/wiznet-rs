#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{pac, prelude::*, rcc, spi, timer::Timer};

mod w6100;
use crate::w6100::W6100;

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

    let mut timer = Timer::syst(cp.SYST, &rcc.clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    let mut spi = dp.SPI1.spi(
        (Some(gpio_a.pa5), Some(gpio_a.pa6), Some(gpio_a.pa7)),
        spi_mode,
        1.MHz(),
        &mut rcc,
    );

    let mut chip = W6100::new(
        embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, embedded_hal_bus::spi::NoDelay)
            .unwrap(),
        rst,
    );

    // POR for W6100
    rst.set_low();
    cs.set_high();
    block!(timer.wait()).ok();
    rst.set_high();
    block!(timer.wait()).ok();

    chip.harware_reset().unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        cs.set_low();
        let mut buff = [0x20, 0x16, 0b000_00_0_00, 0x00, 0x00];
        spi.transfer(&mut buff).unwrap();
        cs.set_high();

        block!(timer.wait()).ok();
    }
}
