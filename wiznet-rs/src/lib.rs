#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

use core::sync::atomic::{AtomicBool, Ordering};

use bitflags::bitflags;
use embedded_hal::{digital::OutputPin, spi::Operation};

mod error;
pub use error::{DriverError, Error};

mod atomic_cell;
use crate::error::DriverError::{PinError, SpiError};

use self::atomic_cell::{AtomicCell, AtomicError, AtomicMutLock};

mod spi_dma;
pub use spi_dma::*;

mod transiver;
use self::transiver::{Address, BlockAddress, BlockSelectionBits, Transceiver};

mod socket;
pub use self::socket::SocketStatus;
use self::socket::{BulkAction, BulkKind, Socket, SocketBackend, SocketRings};

mod socket_common;
use self::socket_common::{
    SocketCommand, SocketInterrupt, clear_interrupts, send_sock_command, set_rx_read_pointer,
    set_tx_write_pointer,
};

mod spsc_ring;

mod tcp_socket;
pub use self::tcp_socket::TcpSocket;
use self::tcp_socket::TcpSocketState;

const CIDR: Address = Address {
    address: 0x0,
    block: BlockSelectionBits::CommonRegister,
};
const VER: Address = Address {
    address: 0x2,
    block: BlockSelectionBits::CommonRegister,
};

///System Config Register 1
const SYCR1: Address = Address {
    address: 0x2005,
    block: BlockSelectionBits::CommonRegister,
};

/// Source Hardware Address Register (MAC address)
const SHAR: Address = Address {
    address: 0x4120,
    block: BlockSelectionBits::CommonRegister,
};

/// Gateway IP Address Register
const GAR: Address = Address {
    address: 0x4130,
    block: BlockSelectionBits::CommonRegister,
};

/// Subnet Mask Register
const SUBR: Address = Address {
    address: 0x4134,
    block: BlockSelectionBits::CommonRegister,
};

/// IPv4 Source Address Register
const SIPR: Address = Address {
    address: 0x4138,
    block: BlockSelectionBits::CommonRegister,
};

/// Chip Lock Register
const CHPLCKR: Address = Address {
    address: 0x41F4,
    block: BlockSelectionBits::CommonRegister,
};

/// Network Lock Register
const NETLCKR: Address = Address {
    address: 0x41F5,
    block: BlockSelectionBits::CommonRegister,
};

/// SOCKET Interrupt Register
const SIR: Address = Address {
    address: 0x2101,
    block: BlockSelectionBits::CommonRegister,
};

/// SOCKET Interrupt Mask Register
const SIMR: Address = Address {
    address: 0x2114,
    block: BlockSelectionBits::CommonRegister,
};

/// PHY Status Register
const PHYSR: Address = Address {
    address: 0x3000,
    block: BlockSelectionBits::CommonRegister,
};

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct PHYStatusFlags: u8 {
        /// Cable OFF bit
        const CAB_MASK = 1 << 7;
        const CAB_OFF = Self::CAB_MASK.bits();
        const CAB_ON =0;

        const MODE_MASK = 0b111 << 3;
        const AUTO_NEGOTIATION = 0b000 << 3;
        const BASE100_TX_FDX = 0b100 << 3;
        const BASE100_TX_HDX = 0b101 << 3;
        const BASE10_T_FDX = 0b110 << 3;
        const BASE10_T_HDX = 0b111 << 3;

        const DUPLEX_MASK = 1 << 2;
        const DUPLEX_HALF = Self::DUPLEX_MASK.bits();
        const DUPLEX_FULL = 0;

        const SPEED_MASK = 1 << 1;
        const SPEED_10 = Self::SPEED_MASK.bits();
        const SPEED_100 = 0;

        const LINK_MASK = 1 ;
        const LINK_UP = Self::LINK_MASK.bits();
        const LINK_DOWN = 0;
    }
}

pub type MacAddress = [u8; 6];

/// The chip's shared hardware: the SPI transport and the reset pin. Kept behind
/// an `AtomicCell` so the user-facing socket handles (which borrow the `W6100`)
/// can coexist with `&self` driver methods that need exclusive hardware access.
struct Device<Spi: SpiDmaDevice, RstPin: OutputPin> {
    transport: Transceiver<Spi>,
    rst: RstPin,
}

/// IPv4 addressing applied to the chip. Set at runtime (statically by the
/// application, or later by a DHCP client) rather than at initialization.
#[derive(Clone, Copy)]
pub struct NetworkConfig {
    pub ip: u32,
    pub gateway: u32,
    pub subnet: u32,
}

/// Records the single bulk DMA currently in flight (one bus, one transfer). Set
/// by `run` when a socket starts a transfer; read and cleared by `dma_complete`
/// when it finishes. Only ever touched from the servicing/DMA interrupts.
#[derive(Clone, Copy)]
struct BulkOp {
    /// Index of the socket whose transfer this is.
    idx: usize,
    kind: BulkKind,
    block: BlockAddress,
    /// Chip-side buffer pointer the transfer started at.
    pointer: u16,
    /// Payload length in bytes.
    len: usize,
}

pub struct W6100<'a, Spi: SpiDmaDevice, RstPin: OutputPin> {
    device: AtomicCell<Device<Spi, RstPin>>,
    mac: MacAddress,

    // Network addressing: `None` until provided at runtime. `service` (re)applies
    // it to the chip whenever it is set or the link comes back up.
    config: AtomicCell<Option<NetworkConfig>>,
    config_dirty: AtomicBool,
    link_up: AtomicBool,

    sockets: [Socket<'a>; 8],

    // The in-flight bulk transfer, if any. While `Some`, the SPI bus is owned by
    // a DMA and `service` does no SPI until `dma_complete` clears it.
    bulk: AtomicCell<Option<BulkOp>>,
}

impl From<AtomicError> for Error {
    fn from(_: AtomicError) -> Self {
        Error::WouldBlock
    }
}

/// Build one socket slot (protocol cell + empty rings) for the given
/// register/buffer block selectors. The rings get their buffers at `open`.
fn make_socket<'a>(
    reg: BlockSelectionBits,
    tx: BlockSelectionBits,
    rx: BlockSelectionBits,
) -> Socket<'a> {
    Socket {
        backend: AtomicCell::new(SocketBackend::new(BlockAddress { reg, tx, rx })),
        rings: SocketRings::new(),
    }
}

impl<'a, Spi: SpiDmaDevice, RstPin: OutputPin> W6100<'a, Spi, RstPin> {
    pub fn new(
        spi: Spi,
        rst: RstPin,
        scratch_buffers: DmaBuffers,
        mac: MacAddress,
    ) -> Result<Self, Error> {
        let this = W6100 {
            device: AtomicCell::new(Device {
                transport: Transceiver::new(spi, scratch_buffers),
                rst,
            }),
            mac,
            config: AtomicCell::new(None),
            config_dirty: AtomicBool::new(false),
            link_up: AtomicBool::new(false),
            sockets: [
                make_socket(
                    BlockSelectionBits::Socket0Register,
                    BlockSelectionBits::Socket0TxBuffer,
                    BlockSelectionBits::Socket0RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket1Register,
                    BlockSelectionBits::Socket1TxBuffer,
                    BlockSelectionBits::Socket1RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket2Register,
                    BlockSelectionBits::Socket2TxBuffer,
                    BlockSelectionBits::Socket2RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket3Register,
                    BlockSelectionBits::Socket3TxBuffer,
                    BlockSelectionBits::Socket3RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket4Register,
                    BlockSelectionBits::Socket4TxBuffer,
                    BlockSelectionBits::Socket4RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket5Register,
                    BlockSelectionBits::Socket5TxBuffer,
                    BlockSelectionBits::Socket5RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket6Register,
                    BlockSelectionBits::Socket6TxBuffer,
                    BlockSelectionBits::Socket6RxBuffer,
                ),
                make_socket(
                    BlockSelectionBits::Socket7Register,
                    BlockSelectionBits::Socket7TxBuffer,
                    BlockSelectionBits::Socket7RxBuffer,
                ),
            ],
            bulk: AtomicCell::new(None),
        };

        this.reset()?;
        Ok(this)
    }

    pub fn reset(&self) -> Result<(), Error> {
        let mut dev_guard = self.device.lock_mut()?;
        let device = dev_guard.as_mut();

        // Tear down any in-flight bulk DMA so the bus and buffers are reclaimed
        // (e.g. a link-down arriving mid-transfer) and clear its marker.
        device.transport.abort();
        if let Ok(mut bulk) = self.bulk.lock_mut() {
            *bulk.as_mut() = None;
        }

        device.rst.set_low().map_err(|_| Error::Other(PinError))?;

        device
            .transport
            .transaction(&mut [Operation::DelayNs(1_000_000)])
            .map_err(|_| Error::Other(SpiError))?;

        device.rst.set_high().map_err(|_| Error::Other(PinError))?;

        if device.transport.read_u16(&CIDR)? != 0x6100 {
            return Err(Error::Other(DriverError::UnexpectedResponse));
        }

        if device.transport.read_u16(&VER)? != 0x4661 {
            return Err(Error::Other(DriverError::UnexpectedResponse));
        }

        for socket in self.sockets.iter() {
            socket
                .backend
                .lock_mut()?
                .as_mut()
                .reset(&device.transport)?;
        }

        // Unlock SYSR registers
        device.transport.write_u8(&CHPLCKR, 0xCE)?;
        // Enable interrupts, clock-select 100Mhz
        device.transport.write_u8(&SYCR1, 0b10000000)?;
        // Lock SYSR registers
        device.transport.write_u8(&CHPLCKR, 0x00)?;

        // Route every socket's interrupts to the INT pin.
        device.transport.write_u8(&SIMR, 0xFF)?;

        Ok(())
    }

    pub fn setup_network(
        &self,
        source_addr: u32,
        gateway_address: u32,
        mask: u32,
    ) -> Result<(), Error> {
        let mut dev_guard = self.device.lock_mut()?;
        let device = dev_guard.as_mut();

        // Unlock network settings
        device.transport.write_u8(&NETLCKR, 0x3A)?;

        let mac = self.mac;
        device.transport.write(&SHAR, &mac)?;
        device.transport.write_u32(&SIPR, source_addr)?;
        device.transport.write_u32(&GAR, gateway_address)?;
        device.transport.write_u32(&SUBR, mask)?;

        // Lock network settings
        device.transport.write_u8(&NETLCKR, 0xC5)?;

        Ok(())
    }

    pub fn is_link_up(&self) -> Result<bool, Error> {
        let mut dev_guard = self.device.lock_mut()?;
        let status =
            PHYStatusFlags::from_bits_retain(dev_guard.as_mut().transport.read_u8(&PHYSR)?);

        Ok(
            (status & PHYStatusFlags::CAB_MASK) == PHYStatusFlags::CAB_ON
                && (status & PHYStatusFlags::LINK_MASK) == PHYStatusFlags::LINK_UP,
        )
    }

    /// Open a TCP socket that actively connects to `addr:port` from `src_port`.
    /// Allocates a free hardware socket and stages the buffers; the actual chip
    /// `OPEN`/`CONNECT` happens on the next `run`. Returns a handle for I/O.
    pub fn open_tcp_connect(
        &'a self,
        addr: u32,
        port: u16,
        src_port: u16,
        rx: &'a mut [u8],
        tx: &'a mut [u8],
    ) -> Result<TcpSocket<'a>, Error> {
        for socket in self.sockets.iter() {
            let mut guard = match socket.backend.lock_mut() {
                Ok(guard) => guard,
                Err(_) => continue,
            };

            if guard.as_mut().is_free() {
                socket.rings.install(rx, tx);
                guard
                    .as_mut()
                    .claim_tcp(TcpSocketState::connect(addr, port, src_port));
                drop(guard);

                return Ok(TcpSocket::new(socket));
            }
        }

        Err(Error::WouldBlock)
    }

    /// Open a TCP socket that passively listens on `port`. As with
    /// [`open_tcp_connect`](Self::open_tcp_connect), the chip work is deferred to
    /// `run`.
    pub fn open_tcp_listen(
        &'a self,
        port: u16,
        rx: &'a mut [u8],
        tx: &'a mut [u8],
    ) -> Result<TcpSocket<'a>, Error> {
        for socket in self.sockets.iter() {
            let mut guard = match socket.backend.lock_mut() {
                Ok(guard) => guard,
                Err(_) => continue,
            };

            if guard.as_mut().is_free() {
                socket.rings.install(rx, tx);
                guard.as_mut().claim_tcp(TcpSocketState::listen(port));
                drop(guard);

                return Ok(TcpSocket::new(socket));
            }
        }

        Err(Error::WouldBlock)
    }

    pub fn run(&self) -> Result<(), Error> {
        let mut dev_guard = self.device.lock_mut()?;
        let device = dev_guard.as_mut();

        for (idx, socket) in self.sockets.iter().enumerate() {
            // A socket currently held by a user handle is skipped this tick.
            let mut guard = match socket.backend.lock_mut() {
                Ok(guard) => guard,
                Err(_) => continue,
            };

            match guard.as_mut().run(&device.transport, &socket.rings) {
                Ok(BulkAction::None) => (),

                // A bulk DMA was started: it now owns the bus. Record what is in
                // flight so `dma_complete` can finish it, and stop servicing —
                // no further SPI may happen until the transfer completes.
                Ok(BulkAction::Started { kind, pointer, len }) => {
                    let block = guard.as_ref().block();
                    drop(guard);

                    self.set_bulk(BulkOp {
                        idx,
                        kind,
                        block,
                        pointer,
                        len,
                    });

                    return Ok(());
                }

                Err(Error::WouldBlock) => (),
                Err(e) => return Err(e),
            }
        }

        Ok(())
    }

    /// Provide (or replace) the IPv4 addressing. Non-blocking and SPI-free: the
    /// config is staged and applied to the chip by the next `service` tick. This
    /// is the integration point for a static setup at startup or a future DHCP
    /// client handing over a lease.
    pub fn set_network_config(&self, config: NetworkConfig) -> Result<(), Error> {
        *self.config.lock_mut()?.as_mut() = Some(config);
        self.config_dirty.store(true, Ordering::Relaxed);

        Ok(())
    }

    fn network_config(&self) -> Option<NetworkConfig> {
        self.config
            .lock_mut()
            .ok()
            .and_then(|guard| *guard.as_ref())
    }

    /// The PHY link state most recently observed by `service` (cached, no SPI).
    /// Use this from the application thread instead of `is_link_up`, which polls
    /// the chip over the bus.
    pub fn link_up(&self) -> bool {
        self.link_up.load(Ordering::Relaxed)
    }

    /// One background step: manage the PHY link, apply any pending network
    /// configuration, then drive every socket. Does all of its SPI work itself,
    /// so it can run entirely from an interrupt (timer tick and/or the chip's
    /// INT line) while the application thread only touches the socket handles.
    /// Idempotent and non-blocking.
    pub fn service(&self) -> Result<(), Error> {
        // A bulk DMA owns the SPI bus until it completes; defer entirely.
        // `dma_complete` re-drives us once the transfer finishes.
        if self.bulk_in_flight() {
            return Ok(());
        }

        let up = self.is_link_up()?;
        let was_up = self.link_up.swap(up, Ordering::Relaxed);

        if !up {
            // Link just went down: reset the chip (re-arms occupied sockets so
            // they reconnect once the link returns). Network settings are wiped
            // by the reset, so the config must be re-applied on the way back up.
            if was_up {
                self.reset()?;
                self.config_dirty.store(true, Ordering::Relaxed);
            }

            return Ok(());
        }

        // Apply pending network configuration once we actually have some. Until
        // then the chip stays at 0.0.0.0 (e.g. waiting for DHCP).
        if self.config_dirty.load(Ordering::Relaxed) {
            if let Some(config) = self.network_config() {
                self.setup_network(config.ip, config.gateway, config.subnet)?;
                self.config_dirty.store(false, Ordering::Relaxed);
            }
        }

        self.run()?;

        Ok(())
    }

    /// Whether a bulk DMA is currently in flight. A failed lock (which should not
    /// happen — `bulk` is only touched from the interrupts) is treated as "in
    /// flight" so we conservatively skip SPI.
    fn bulk_in_flight(&self) -> bool {
        self.bulk
            .lock_mut()
            .map(|guard| guard.as_ref().is_some())
            .unwrap_or(true)
    }

    fn set_bulk(&self, op: BulkOp) {
        if let Ok(mut guard) = self.bulk.lock_mut() {
            *guard.as_mut() = Some(op);
        }
    }

    /// Finish the in-flight transfer recorded in `op`: collect the captured
    /// payload (read) and commit the chip-side pointers + command. Runs only the
    /// small synchronous epilogue ops over the bus; the bulk payload already
    /// moved by DMA.
    fn finish_bulk(op: BulkOp, trans: &Transceiver<Spi>, rings: &SocketRings) -> Result<(), Error> {
        let end = op.pointer.wrapping_add(op.len as u16);

        match op.kind {
            BulkKind::Receive => {
                trans.finish_read(op.len, |payload| {
                    rings.rx.write(payload);
                })?;

                set_rx_read_pointer(&op.block, trans, end)?;
                send_sock_command(&op.block, trans, SocketCommand::Receive)?;
                clear_interrupts(&op.block, trans, SocketInterrupt::RECV)?;
            }
            BulkKind::Transmit => {
                trans.finish_write()?;

                set_tx_write_pointer(&op.block, trans, end)?;
                send_sock_command(&op.block, trans, SocketCommand::Send)?;
            }
        }

        Ok(())
    }

    /// Completion of an asynchronous bulk transfer — call from the platform's
    /// DMA-complete interrupt. Finishes the recorded transfer (delivers captured
    /// bytes into the socket's lock-free rx ring, commits the chip pointers),
    /// releases the bus, and re-drives `service`. Never touches a socket's
    /// protocol cell, so it cannot contend with the application thread.
    pub fn dma_complete(&self) {
        let op = match self.bulk.lock_mut() {
            Ok(guard) => *guard.as_ref(),
            Err(_) => return,
        };
        let Some(op) = op else {
            return;
        };

        if let Ok(mut dev_guard) = self.device.lock_mut() {
            let device = dev_guard.as_mut();
            let _ = Self::finish_bulk(op, &device.transport, &self.sockets[op.idx].rings);
        }

        // Bus is free again: clear the marker, then resume servicing (which may
        // immediately start the next bulk transfer).
        if let Ok(mut guard) = self.bulk.lock_mut() {
            *guard.as_mut() = None;
        }

        let _ = self.service();
    }
}
