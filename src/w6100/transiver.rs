use crate::w6100::Error;

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

pub struct Address {
    pub(crate) address: u16,
    pub(crate) block: BlockSelectionBits,
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

pub trait Transceiver {
    fn read(&mut self, addr: &Address, data: &mut [u8]) -> Result<(), Error>;

    fn write(&mut self, addr: &Address, data: &[u8]) -> Result<(), Error>;

    impl_read_primitive!(read_u8, u8);
    impl_read_primitive!(read_u16, u16);
    impl_read_primitive!(read_u32, u32);

    impl_write_primitive!(write_u8, u8);
    impl_write_primitive!(write_u16, u16);
    impl_write_primitive!(write_u32, u32);
}
