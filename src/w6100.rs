use bitflags::bitflags;
use embedded_hal::{
    digital::OutputPin,
    spi::{Operation, SpiDevice},
};

#[derive(Clone, Copy)]
pub enum BlockSelectionBits {
    CommonRegister = 0,

    Socket0Register = 0b000_01,
    Socket0TxBuffer = 0b000_10,
    Socket0RxBuffer = 0b000_11,

    Socket1Register = 0b001_01,
    Socket1TxBuffer = 0b001_10,
    Socket1RxBuffer = 0b001_11,

    Socket2Register = 0b010_01,
    Socket2TxBuffer = 0b010_10,
    Socket2RxBuffer = 0b010_11,

    Socket3Register = 0b011_01,
    Socket3TxBuffer = 0b011_10,
    Socket3RxBuffer = 0b011_11,

    Socket4Register = 0b100_01,
    Socket4TxBuffer = 0b100_10,
    Socket4RxBuffer = 0b100_11,

    Socket5Register = 0b101_01,
    Socket5TxBuffer = 0b101_10,
    Socket5RxBuffer = 0b101_11,

    Socket6Register = 0b110_01,
    Socket6TxBuffer = 0b110_10,
    Socket6RxBuffer = 0b110_11,

    Socket7Register = 0b111_01,
    Socket7TxBuffer = 0b111_10,
    Socket7RxBuffer = 0b111_11,
}

struct Address {
    address: u16,
    block: BlockSelectionBits,
}

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

pub struct W6100<Spi: SpiDevice<u8>, RstPin: OutputPin> {
    spi: Spi,
    rst: RstPin,
    mac: MacAddress,
}

#[derive(Debug)]
pub enum Error {
    SpiError,
    PinError,

    UnexpectedResponse,
}

macro_rules! impl_read_primitive {
    ($name:ident, $t:ty) => {
        fn $name(&mut self, addr: &Address) -> Result<$t, Error> {
            let mut buf = [0u8; size_of::<$t>()];
            self.read(addr, &mut buf)?;

            Ok(<$t>::from_be_bytes(buf))
        }
    };
}

macro_rules! impl_write_primitive {
    ($name:ident, $t:ty) => {
        fn $name(&mut self, addr: &Address, value: $t) -> Result<(), Error> {
            let buf = value.to_be_bytes();
            self.write(addr, &buf)
        }
    };
}

pub trait Transceiver<Spi: SpiDevice<u8>> {
    fn read(&mut self, addr: &Address, data: &mut [u8]) -> Result<(), Error>;

    fn write(&mut self, addr: &Address, data: &[u8]) -> Result<(), Error>;

    impl_read_primitive!(read_u8, u8);
    impl_read_primitive!(read_u16, u16);
    impl_read_primitive!(read_u32, u32);

    impl_write_primitive!(write_u8, u8);
    impl_write_primitive!(write_u16, u16);
    impl_write_primitive!(write_u32, u32);
}

impl<Spi: SpiDevice<u8>, RstPin: OutputPin> Transceiver<Spi> for W6100<Spi, RstPin> {
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

impl<Spi: SpiDevice<u8>, RstPin: OutputPin> W6100<Spi, RstPin> {
    pub fn new(spi: Spi, rst: RstPin, mac: MacAddress) -> Result<Self, Error> {
        let mut this = W6100 { spi, rst, mac };

        this.reset()?;
        Ok(this)
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.rst.set_low().map_err(|_| Error::PinError)?;

        self.spi
            .transaction(&mut [Operation::DelayNs(1_000_000)])
            .map_err(|_| Error::SpiError)?;

        self.rst.set_high().map_err(|_| Error::PinError)?;

        if self.read_u16(&CIDR)? != 0x6100 {
            return Err(Error::UnexpectedResponse);
        }

        if self.read_u16(&VER)? != 0x4661 {
            return Err(Error::UnexpectedResponse);
        }

        // Unlock SYSR registers
        self.write_u8(&CHPLCKR, 0xCE)?;
        // Enable interrupts, clock-select 100Mhz
        self.write_u8(&SYCR1, 0b10000000)?;
        // Lock SYSR registers
        self.write_u8(&CHPLCKR, 0x00)?;

        Ok(())
    }

    pub fn setup_network(
        &mut self,
        source_addr: u32,
        gateway_address: u32,
        mask: u32,
    ) -> Result<(), Error> {
        // Unlock network settings
        self.write_u8(&NETLCKR, 0x3A)?;

        let mac = self.mac;
        self.write(&SHAR, &mac)?;
        self.write_u32(&SIPR, source_addr)?;
        self.write_u32(&GAR, gateway_address)?;
        self.write_u32(&SUBR, mask)?;

        // Lock network settings
        self.write_u8(&NETLCKR, 0xC5)?;

        Ok(())
    }

    pub fn is_link_up(&mut self) -> Result<bool, Error> {
        let status = PHYStatusFlags::from_bits_retain(self.read_u8(&PHYSR)?);

        return Ok(
            (status & PHYStatusFlags::CAB_MASK) == PHYStatusFlags::CAB_ON
                && (status & PHYStatusFlags::LINK_MASK) == PHYStatusFlags::LINK_UP,
        );
    }
}
