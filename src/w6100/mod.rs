use core::marker::PhantomData;

use bitflags::bitflags;
use embedded_hal::{
    digital::OutputPin,
    spi::{Operation, SpiDevice},
};

mod atomic_cell;

use crate::w6100::{
    socket::{SocketAccess, SocketInternal, SocketProtocolMode},
    socket_common::init_socket,
    transiver::BlockAddress,
};

use self::atomic_cell::{AtomicCell, AtomicError};

mod transiver;
use self::transiver::{Address, BlockSelectionBits, Transceiver};

mod socket;
pub use socket::PinnedSocket;

mod socket_common;

mod tcp_socket;
pub use self::tcp_socket::TcpSocket;

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

struct SocketBackend<'a, Trans: Transceiver> {
    block: BlockAddress,

    accessor: Option<&'a dyn SocketAccess<'a, Trans>>,
}

pub struct Transport<Spi: SpiDevice<u8>> {
    spi: Spi,
}

pub struct W6100<'a, Spi: SpiDevice<u8>, RstPin: OutputPin> {
    transport: Transport<Spi>,
    rst: RstPin,
    mac: MacAddress,

    sockets: [SocketBackend<'a, Transport<Spi>>; 8],
}

#[derive(Debug, Clone, Copy)]
pub enum Error {
    SpiError,
    PinError,

    UnexpectedResponse,

    Busy,
}

impl From<AtomicError> for Error {
    fn from(_: AtomicError) -> Self {
        Error::Busy
    }
}

impl<Spi: SpiDevice<u8>> Transceiver for Transport<Spi> {
    fn read(&mut self, addr: &Address, data: &mut [u8]) -> Result<(), Error> {
        let mut buf = [0u8; 3];

        buf[0..2].copy_from_slice(&addr.address.to_be_bytes());
        buf[2] = (addr.block as u8) << 3;

        self.spi
            .transaction(&mut [Operation::Write(&buf), Operation::Read(data)])
            .map_err(|_| Error::SpiError)
    }

    fn write(&mut self, addr: &Address, data: &[u8]) -> Result<(), Error> {
        let mut buf = [0u8; 3];

        buf[0..2].copy_from_slice(&addr.address.to_be_bytes());
        buf[2] = (addr.block as u8) << 3 | 2;

        self.spi
            .transaction(&mut [Operation::Write(&buf), Operation::Write(data)])
            .map_err(|e| Error::SpiError)
    }
}

impl<'a, Spi: SpiDevice<u8>, RstPin: OutputPin> W6100<'a, Spi, RstPin> {
    pub fn new(spi: Spi, rst: RstPin, mac: MacAddress) -> Result<Self, Error> {
        let mut this = W6100 {
            transport: Transport { spi },
            rst,
            mac,
            sockets: [
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket0Register,
                        tx: BlockSelectionBits::Socket0TxBuffer,
                        rx: BlockSelectionBits::Socket0RxBuffer,
                    },
                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket1Register,
                        tx: BlockSelectionBits::Socket1TxBuffer,
                        rx: BlockSelectionBits::Socket1RxBuffer,
                    },

                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket2Register,
                        tx: BlockSelectionBits::Socket2TxBuffer,
                        rx: BlockSelectionBits::Socket2RxBuffer,
                    },
                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket3Register,
                        tx: BlockSelectionBits::Socket3TxBuffer,
                        rx: BlockSelectionBits::Socket3RxBuffer,
                    },
                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket4Register,
                        tx: BlockSelectionBits::Socket4TxBuffer,
                        rx: BlockSelectionBits::Socket4RxBuffer,
                    },
                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket5Register,
                        tx: BlockSelectionBits::Socket5TxBuffer,
                        rx: BlockSelectionBits::Socket5RxBuffer,
                    },
                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket6Register,
                        tx: BlockSelectionBits::Socket6TxBuffer,
                        rx: BlockSelectionBits::Socket6RxBuffer,
                    },
                    accessor: None,
                },
                SocketBackend {
                    block: BlockAddress {
                        reg: BlockSelectionBits::Socket7Register,
                        tx: BlockSelectionBits::Socket7TxBuffer,
                        rx: BlockSelectionBits::Socket7RxBuffer,
                    },
                    accessor: None,
                },
            ],
        };

        this.reset()?;
        Ok(this)
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.rst.set_low().map_err(|_| Error::PinError)?;

        self.transport
            .spi
            .transaction(&mut [Operation::DelayNs(1_000_000)])
            .map_err(|_| Error::SpiError)?;

        self.rst.set_high().map_err(|_| Error::PinError)?;

        if self.transport.read_u16(&CIDR)? != 0x6100 {
            return Err(Error::UnexpectedResponse);
        }

        if self.transport.read_u16(&VER)? != 0x4661 {
            return Err(Error::UnexpectedResponse);
        }

        for sock in self.sockets.iter_mut().by_ref() {
            sock.reset(&mut self.transport)?;
        }

        // Unlock SYSR registers
        self.transport.write_u8(&CHPLCKR, 0xCE)?;
        // Enable interrupts, clock-select 100Mhz
        self.transport.write_u8(&SYCR1, 0b10000000)?;
        // Lock SYSR registers
        self.transport.write_u8(&CHPLCKR, 0x00)?;

        Ok(())
    }

    pub fn setup_network(
        &mut self,
        source_addr: u32,
        gateway_address: u32,
        mask: u32,
    ) -> Result<(), Error> {
        // Unlock network settings
        self.transport.write_u8(&NETLCKR, 0x3A)?;

        let mac = self.mac;
        self.transport.write(&SHAR, &mac)?;
        self.transport.write_u32(&SIPR, source_addr)?;
        self.transport.write_u32(&GAR, gateway_address)?;
        self.transport.write_u32(&SUBR, mask)?;

        // Lock network settings
        self.transport.write_u8(&NETLCKR, 0xC5)?;

        Ok(())
    }

    pub fn is_link_up(&mut self) -> Result<bool, Error> {
        let status = PHYStatusFlags::from_bits_retain(self.transport.read_u8(&PHYSR)?);

        return Ok(
            (status & PHYStatusFlags::CAB_MASK) == PHYStatusFlags::CAB_ON
                && (status & PHYStatusFlags::LINK_MASK) == PHYStatusFlags::LINK_UP,
        );
    }

    pub fn open<Socket: SocketAccess<'a, Transport<Spi>>>(
        &mut self,
        user_socket: &'a Socket,
    ) -> Result<(), Error> {
        for sock in self.sockets.iter_mut() {
            if sock.is_free_to_use() {
                let result = user_socket
                    .lock_inner()?
                    .as_mut()
                    .init(&sock.block, &mut self.transport);

                match result {
                    Ok(_) => {
                        sock.accessor = Some(user_socket);
                        return Ok(());
                    }
                    Err(e) => match e {
                        Error::Busy => return Err(Error::Busy),
                        e => {
                            sock.reset(&mut self.transport)?;
                            return Err(e);
                        }
                    },
                }
            }
        }

        Err(Error::Busy)
    }

    pub fn run(&mut self) -> Result<(), Error> {
        for sock in self.sockets.iter_mut() {
            match sock.run(&mut self.transport) {
                Ok(()) => (),
                Err(e) => match e {
                    Error::Busy => (),
                    e => return Err(e),
                },
            };
        }

        Ok(())
    }
}

impl<'a, Trans: Transceiver> SocketBackend<'a, Trans> {
    pub fn is_free_to_use(&self) -> bool {
        self.accessor.is_none()
    }

    pub fn run(&mut self, transceiver: &mut Trans) -> Result<(), Error> {
        if let Some(accesor) = self.accessor {
            accesor.lock_inner()?.as_mut().run(&self.block, transceiver)
        } else {
            Ok(())
        }
    }

    pub fn reset(&mut self, transceiver: &mut Trans) -> Result<(), Error> {
        self.accessor = None;

        init_socket(&self.block, transceiver, SocketProtocolMode::CLOSED)?;

        Ok(())
    }
}
