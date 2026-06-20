use embedded_hal::{
    digital::OutputPin,
    spi::{Operation, SpiDevice},
};

enum BlockSelectionBits {
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

const CIDR: u16 = 0x0;
const VER: u16 = 0x2;

pub struct W6100<Spi: SpiDevice<u8>, RstPin: OutputPin> {
    spi: Spi,
    rst: RstPin,
}

pub enum Error<Spi: SpiDevice<u8>> {
    SpiError(Spi::Error),

    UnexpectedResponse,
}

macro_rules! impl_read_primitive {
    ($name:ident, $t:ty) => {
        fn $name(&mut self, addr: u16, block: BlockSelectionBits) -> Result<$t, Error<Spi>> {
            let mut buf = [0u8; size_of::<$t>()];
            self.read(addr, block, &mut buf)?;

            Ok(<$t>::from_be_bytes(buf))
        }
    };
}

pub trait Transceiver<Spi: SpiDevice<u8>> {
    fn read(
        &mut self,
        addr: u16,
        block: BlockSelectionBits,
        data: &mut [u8],
    ) -> Result<(), Error<Spi>>;

    fn write(
        &mut self,
        addr: u16,
        block: BlockSelectionBits,
        data: &[u8],
    ) -> Result<(), Error<Spi>>;

    impl_read_primitive!(read_u8, u8);
    impl_read_primitive!(read_u16, u16);
    impl_read_primitive!(read_u32, u32);
}

impl<Spi: SpiDevice<u8>, RstPin: OutputPin> Transceiver<Spi> for W6100<Spi, RstPin> {
    fn read(
        &mut self,
        addr: u16,
        block: BlockSelectionBits,
        data: &mut [u8],
    ) -> Result<(), Error<Spi>> {
        let mut buf = [0u8; 3];

        buf[0..2].copy_from_slice(&addr.to_be_bytes());
        buf[3] = (block as u8) << 3;

        self.spi
            .transaction(&mut [Operation::Write(&buf), Operation::Read(data)])
            .into()
    }

    fn write(
        &mut self,
        addr: u16,
        block: BlockSelectionBits,
        data: &[u8],
    ) -> Result<(), Error<Spi>> {
        let mut buf = [0u8; 3];

        buf[0..2].copy_from_slice(&addr.to_be_bytes());
        buf[3] = (block as u8) << 3 | 2;

        self.spi
            .transaction(&mut [Operation::Write(&buf), Operation::Write(data)])
            .into()
    }
}

impl<Spi: SpiDevice<u8>, RstPin: OutputPin> W6100<Spi, RstPin> {
    pub fn new(spi: Spi, rst: RstPin) -> Self {
        W6100 { spi, rst }
    }

    pub fn assert_reset(&mut self) -> Result<(), Error<Spi>> {
        self.rst.set_low()?;

        Ok(())
    }

    pub fn harware_reset(&mut self) -> Result<(), Error<Spi>> {
        self.rst.set_high()?;

        if self.get_cidr()? != 0x6100 {
            return Err(Error::UnexpectedResponse);
        }

        if self.get_version()? != 0x4641 {
            return Err(Error::UnexpectedResponse);
        }

        Ok(())
    }

    pub fn get_cidr(&mut self) -> Result<u16, Error<Spi>> {
        self.read_u16(CIDR, BlockSelectionBits::CommonRegister)
    }

    pub fn get_version(&mut self) -> Result<u16, Error<Spi>> {
        self.read_u16(VER, BlockSelectionBits::CommonRegister)
    }
}
