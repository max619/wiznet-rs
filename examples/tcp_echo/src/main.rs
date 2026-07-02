#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use cortex_m::interrupt::{Mutex, free as interrupt_free};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use panic_halt as _;
use static_cell::StaticCell;
use stm32f1xx_hal::gpio::Floating;
use stm32f1xx_hal::{
    gpio::{Edge, ExtiPin, Input, Output, Pin, PullUp},
    pac::{self, interrupt},
    prelude::*,
    rcc, spi,
    timer::{CounterHz, Event},
};

mod hal_spi;

use crate::hal_spi::{HalSpi, SCRATCH};
use wiznet_rs::{DmaBuffers, NetworkConfig, SocketStatus, TcpSocket, W6100};

// Concrete types of the fully-configured chip, needed to name the `'static`
// singleton storage and the interrupt-shared globals.
type ChipRst = Pin<'A', 8, Output>;
type Chip = W6100<'static, HalSpi, ChipRst>;

// Shared with the interrupt handlers. The chip is only ever touched through
// `&self`, so a shared `&'static` reference is all the ISRs need; the timer and
// INT pin are owned so the ISRs can clear their pending flags.
static CHIP: Mutex<Cell<Option<&'static Chip>>> = Mutex::new(Cell::new(None));
static TIMER: Mutex<RefCell<Option<CounterHz<pac::TIM2>>>> = Mutex::new(RefCell::new(None));
static INT_PIN: Mutex<RefCell<Option<Pin<'A', 10, Input<Floating>>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::hse(8.MHz()).sysclk(24.MHz()).pclk1(24.MHz()),
        &mut flash.acr,
    );

    let mut afio = dp.AFIO.constrain(&mut rcc);
    let mut exti = dp.EXTI;

    let mut gpio_a = dp.GPIOA.split(&mut rcc);
    let mut gpio_c = dp.GPIOC.split(&mut rcc);

    // Onboard LED on PC13 (active low): lit while the socket is connected.
    let mut led = gpio_c.pc13.into_push_pull_output(&mut gpio_c.crh);
    led.set_high(); // off

    let rst = gpio_a.pa8.into_push_pull_output(&mut gpio_a.crh);
    let cs: Pin<'A', 9, Output> = gpio_a.pa9.into_push_pull_output(&mut gpio_a.crh);

    // W6100 INT line (active low) -> EXTI line 10, falling edge.
    let mut int_pin = gpio_a.pa10.into_floating_input(&mut gpio_a.crh);
    int_pin.make_interrupt_source(&mut afio);
    int_pin.trigger_on_edge(&mut exti, Edge::Falling);
    int_pin.enable_interrupt(&mut exti);

    let spi_mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    let spi = dp.SPI1.spi(
        (Some(gpio_a.pa5), Some(gpio_a.pa6), Some(gpio_a.pa7)),
        spi_mode,
        16.MHz(),
        &mut rcc,
    );

    // DMA1: channel 2 = SPI1_RX, channel 3 = SPI1_TX.
    let dma1 = dp.DMA1.split(&mut rcc);
    let sysclk_hz = rcc.clocks.sysclk().raw();

    let mac = [0xfc, 0xd7, 0xfd, 0xab, 0x8b, 0xe4];

    // Buffers and the chip itself live for the whole program in `static` storage
    // so the interrupt handlers can reach the chip via `&'static`.
    static CHIP_CELL: StaticCell<Chip> = StaticCell::new();
    static RX: StaticCell<[u8; 512]> = StaticCell::new();
    static TX: StaticCell<[u8; 512]> = StaticCell::new();
    static RX_SCRATCH: StaticCell<[u8; SCRATCH]> = StaticCell::new();
    static TX_SCRATCH: StaticCell<[u8; SCRATCH]> = StaticCell::new();

    let hal_spi = HalSpi::new(spi, dma1.2, dma1.3, cs, sysclk_hz);

    // Scratch buffers for the driver's DMA transport (header + payload per
    // transfer). The library owns them; the transport borrows them per transfer.
    let scratch = DmaBuffers {
        rx: RX_SCRATCH.init([0u8; SCRATCH]),
        tx: TX_SCRATCH.init([0u8; SCRATCH]),
        len: 0,
    };

    let chip: &'static Chip =
        CHIP_CELL.init(W6100::new(hal_spi, rst, scratch, mac).expect("Failed to init W6100"));

    // Periodic 100 ms tick: drives the non-interrupt transitions (handshake/close
    // polling, TX flush) and backstops any missed INT edge.
    let mut timer = dp.TIM2.counter_hz(&mut rcc);
    timer.start(10.Hz()).unwrap();
    timer.listen(Event::Update);

    // Publish the chip and the interrupt-owned peripherals, then let the ISRs run.
    interrupt_free(|cs| {
        CHIP.borrow(cs).set(Some(chip));
        TIMER.borrow(cs).replace(Some(timer));
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    #[allow(unsafe_code)]
    // SAFETY: enabling the chip's servicing interrupts; nothing relies on these
    // being masked for a critical section.
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
        NVIC::unmask(pac::Interrupt::EXTI15_10);
    }

    // Application thread: no SPI, just react to the socket and sleep. All chip
    // I/O happens in `service` on the interrupts. Handle ops can transiently
    // return `Err(Busy)` if an ISR is mid-servicing the socket — just retry.
    //
    // The 'static buffers can only be handed out once, so the listener is opened
    // on the first link-up and re-armed (not re-created) on later ones.
    let mut socket: Option<TcpSocket<'static>> = None;

    loop {
        // Phase 1: wait for the link to really come up (per the cached link
        // state `service` maintains), sleeping until then.
        while !chip.link_up() {
            cortex_m::asm::wfi();
        }

        // Phase 2: link is up — apply addressing and bring the listener up. Both
        // are SPI-free; `service` applies them on its next tick.
        chip.set_network_config(NetworkConfig {
            ip: u32::from_be_bytes([192, 168, 10, 10]),
            gateway: u32::from_be_bytes([192, 168, 10, 1]),
            subnet: u32::from_be_bytes([255, 255, 255, 0]),
        })
        .expect("Failed to set network config");

        // Echo server: listen on TCP port 5555 and echo whatever arrives. Open
        // the listener on the first link-up (already armed); on later link-ups
        // re-arm the existing one instead of re-creating it.
        if socket.is_none() {
            socket = Some(
                chip.open_tcp_listen(5555, RX.init([0u8; 512]), TX.init([0u8; 512]))
                    .expect("Failed to open socket"),
            );
        } else {
            socket
                .as_ref()
                .unwrap()
                .reconnect()
                .expect("Failed to re-arm socket");
        }

        let sock = socket.as_ref().unwrap();

        // Echo carry-over: bytes read from the rx ring that have not yet been
        // fully accepted by the tx ring. `write` only takes what the ring has
        // room for, so we must retry the remainder instead of dropping it —
        // dropping here is what silently truncated/corrupted the echoed stream.
        // We refill `buf` only once the previous chunk is completely echoed.
        let mut buf = [0u8; 128];
        let mut len = 0usize; // bytes staged in `buf`
        let mut off = 0usize; // bytes of `buf[..len]` already written out

        // Phase 3: handle the connection until the link drops, then loop back to
        // phase 1 and wait for it to return.
        while chip.link_up() {
            match sock.status() {
                Ok(SocketStatus::Established) => {
                    led.set_low(); // LED on while connected (active low)

                    // Drain as much as possible each wake so the RX pipeline
                    // can't back up: refill `buf` from the rx ring only once the
                    // previous chunk is fully echoed (so nothing read is ever lost
                    // to a full tx ring), and keep going until the rx ring is empty
                    // or the tx ring is full.
                    loop {
                        if off == len {
                            len = sock.read(&mut buf).unwrap_or(0);
                            off = 0;
                            if len == 0 {
                                break; // rx ring drained
                            }
                        }
                        let n = sock.write(&buf[off..len]).unwrap_or(0);
                        off += n;
                        if n == 0 {
                            break; // tx ring full — resume next wake
                        }
                    }
                }

                // Client disconnected (or the attempt failed); re-arm to accept
                // the next connection.
                Ok(SocketStatus::Closed) | Ok(SocketStatus::Timeout) | Ok(SocketStatus::Error) => {
                    led.set_high(); // off
                    // Drop any half-echoed carry-over from the old connection so
                    // it can't bleed into the next client.
                    len = 0;
                    off = 0;
                    let _ = sock.reconnect();
                }

                _ => led.set_high(), // off (listening/connecting/busy)
            }

            cortex_m::asm::wfi();
        }

        led.set_high(); // link dropped: off
    }
}

/// Run one background servicing step on the chip (all SPI work).
fn service() {
    let chip = interrupt_free(|cs| CHIP.borrow(cs).get());

    if let Some(chip) = chip {
        let _ = chip.service();
    }
}

/// Periodic tick.
#[interrupt]
fn TIM2() {
    interrupt_free(|cs| {
        if let Some(timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.clear_interrupt(Event::Update);
        }
    });

    service();
}

/// W6100 INT line — low-latency wake on chip events (RX, CON, DISCON, …).
#[interrupt]
fn EXTI15_10() {
    interrupt_free(|cs| {
        if let Some(pin) = INT_PIN.borrow(cs).borrow_mut().as_mut() {
            pin.clear_interrupt_pending_bit();
        }
    });

    service();
}

/// SPI1_RX DMA transfer-complete — finishes the in-flight async bulk payload
/// transfer (the `main` thread ran free while it was clocking) and resumes
/// servicing. The HAL clears the channel flag inside `dma_complete`'s `wait`.
#[interrupt]
fn DMA1_CHANNEL2() {
    let chip = interrupt_free(|cs| CHIP.borrow(cs).get());

    if let Some(chip) = chip {
        chip.dma_complete();
    }
}
