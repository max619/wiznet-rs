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

pub struct BlockAddress {
    pub(crate) reg: BlockSelectionBits,
    pub(crate) tx: BlockSelectionBits,
    pub(crate) rx: BlockSelectionBits,
}

pub struct Address {
    pub(crate) address: u16,
    pub(crate) block: BlockSelectionBits,
}

/// Build the 3-byte W6100 SPI command header for `addr`. `write` selects the
/// RWB bit; OM is left 0 (variable-length mode).
pub(crate) fn header(addr: &Address, write: bool) -> [u8; 3] {
    let mut h = [0u8; 3];
    h[0..2].copy_from_slice(&addr.address.to_be_bytes());
    h[2] = ((addr.block as u8) << 3) | if write { 0b100 } else { 0 };
    h
}

/// Async DMA capability for the SPI link, kept deliberately platform-close: it
/// moves an opaque `header` (clocked out blocking) plus a DMA payload, manages
/// CS across the transfer, and owns its own scratch. No W6100 concepts here.
///
/// Completion is signalled out-of-band (the platform's DMA interrupt), after
/// which [`finish`](Self::finish) reclaims the bus.
pub trait SpiDma {
    /// Clock out `header`, then DMA-read `len` payload bytes into internal scratch.
    fn start_read(&mut self, header: &[u8], len: usize) -> Result<(), Error>;

    /// Clock out `header`, then DMA-write `data` (copied internally before return).
    fn start_write(&mut self, header: &[u8], data: &[u8]) -> Result<(), Error>;

    /// Wait for the in-flight transfer to finish (instant when called from the
    /// completion interrupt), release CS, and free the bus. No-op if idle.
    fn finish(&mut self) -> Result<(), Error>;

    /// The payload captured by the most recent [`start_read`](Self::start_read),
    /// valid once [`finish`](Self::finish) has run.
    fn read_buffer(&self) -> &[u8];
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

    /// Bulk read via DMA (blocking for now): frames the header and moves the
    /// payload through the [`SpiDma`] path into `dst`.
    fn bulk_read(&mut self, addr: &Address, dst: &mut [u8]) -> Result<(), Error>;

    /// Bulk write via DMA (blocking for now): frames the header and moves `data`
    /// through the [`SpiDma`] path.
    fn bulk_write(&mut self, addr: &Address, data: &[u8]) -> Result<(), Error>;

    impl_read_primitive!(read_u8, u8);
    impl_read_primitive!(read_u16, u16);
    impl_read_primitive!(read_u32, u32);

    impl_write_primitive!(write_u8, u8);
    impl_write_primitive!(write_u16, u16);
    impl_write_primitive!(write_u32, u32);
}
