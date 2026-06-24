use core::mem;

use embedded_hal::spi::Operation;

use crate::{
    DmaBuffers, DriverError, Error, SpiDmaDevice, SpiDmaTransaction,
    atomic_cell::{AtomicCell, AtomicMutLock},
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
pub(crate) fn create_header(addr: &Address, write: bool) -> [u8; 3] {
    let mut h = [0u8; 3];
    h[0..2].copy_from_slice(&addr.address.to_be_bytes());
    h[2] = ((addr.block as u8) << 3) | if write { 0b100 } else { 0 };
    h
}

macro_rules! impl_read_primitive {
    ($name:ident, $t:ty) => {
        pub fn $name(&self, addr: &Address) -> Result<$t, Error> {
            let mut buf = [0u8; size_of::<$t>()];
            self.read(addr, &mut buf)?;

            Ok(<$t>::from_be_bytes(buf))
        }
    };
}

macro_rules! impl_write_primitive {
    ($name:ident, $t:ty) => {
        pub fn $name(&self, addr: &Address, value: $t) -> Result<(), Error> {
            let buf = value.to_be_bytes();
            self.write(addr, &buf)
        }
    };
}

enum DmaState<D: SpiDmaDevice> {
    Pending,
    Idle { dev: D, scratch_buffers: DmaBuffers },
    InFlight(D::Transaction),
}

pub(crate) struct Transceiver<D: SpiDmaDevice> {
    device: AtomicCell<DmaState<D>>,
}

impl<D: SpiDmaDevice> Transceiver<D> {
    pub fn new(dev: D, scratch_buffers: DmaBuffers) -> Self {
        if scratch_buffers.rx.len() != scratch_buffers.tx.len() {
            panic!("Scratch buffers should have the same length");
        }

        Self {
            device: AtomicCell::new(DmaState::Idle {
                dev,
                scratch_buffers,
            }),
        }
    }

    /// This is a sync method
    pub fn transaction(&self, operations: &mut [Operation<'_, u8>]) -> Result<(), Error> {
        let mut guard = self.device.lock_mut()?;
        let mut cell = guard.as_mut();
        let (dev, scratch_buffers) = {
            let current_state = mem::replace(cell, DmaState::Pending);

            match current_state {
                DmaState::Pending | DmaState::InFlight(_) => {
                    let _ = mem::replace(cell, current_state);
                    return Err(nb::Error::WouldBlock);
                }
                DmaState::Idle {
                    dev,
                    scratch_buffers,
                } => (dev, scratch_buffers),
            }
        };

        let (dev, scratch_buffers, error) =
            Self::exec_transaction(dev, scratch_buffers, operations);
        let _ = mem::replace(
            cell,
            DmaState::Idle {
                dev,
                scratch_buffers,
            },
        );

        match error {
            Some(e) => Err(nb::Error::Other(e)),
            None => Ok(()),
        }
    }

    fn exec_transaction(
        dev: D,
        scratch_buffers: DmaBuffers,
        operations: &mut [Operation<'_, u8>],
    ) -> (D, DmaBuffers, Option<DriverError>) {
        let mut index = 0;
        let mut scratch_buffers = scratch_buffers;
        let mut dev = dev;

        while index < operations.len() {
            let mut offset = 0;
            let mut start_index = index;

            while index < operations.len() {
                match &operations[index] {
                    Operation::Read(items) => {
                        if offset + items.len() > scratch_buffers.rx.len() {
                            break;
                        }

                        offset += items.len();
                    }
                    Operation::Write(items) | Operation::Transfer(_, items) => {
                        if offset + items.len() > scratch_buffers.tx.len() {
                            break;
                        }

                        scratch_buffers.tx[offset..offset + items.len()].copy_from_slice(items);
                        offset += items.len();
                    }
                    Operation::TransferInPlace(items) => {
                        if offset + items.len() > scratch_buffers.tx.len() {
                            break;
                        }

                        scratch_buffers.tx[offset..offset + items.len()].copy_from_slice(items);
                        offset += items.len();
                    }

                    Operation::DelayNs(ns) => break,
                }

                index += 1;
            }

            if start_index == index {
                match operations[index] {
                    Operation::DelayNs(ns) => {
                        dev.delay_ns(ns);
                        index += 1;
                        continue;
                    }

                    _ => {
                        return (
                            dev,
                            scratch_buffers,
                            Some(DriverError::ScratchBufferOverrun),
                        );
                    }
                }
            }

            scratch_buffers.len = offset;
            match dev.transceive(scratch_buffers) {
                Ok(transaction) => (dev, scratch_buffers) = transaction.wait(),
                Err((_, dev, buffers)) => {
                    return (dev, buffers, Some(DriverError::SpiError));
                }
            }

            offset = 0;

            while start_index < operations.len() {
                match &mut operations[index] {
                    Operation::Read(items)
                    | Operation::Transfer(items, _)
                    | Operation::TransferInPlace(items) => {
                        items.copy_from_slice(&scratch_buffers.rx[offset..offset + items.len()]);
                        offset += items.len();
                    }

                    Operation::Write(_) | Operation::DelayNs(_) => (),
                }

                start_index += 1;
            }
        }

        (dev, scratch_buffers, None)
    }

    pub fn read(&self, addr: &Address, data: &mut [u8]) -> Result<(), Error> {
        self.transaction(&mut [
            Operation::Write(&create_header(addr, false)),
            Operation::Read(data),
        ])
    }

    pub fn write(&self, addr: &Address, data: &[u8]) -> Result<(), Error> {
        self.transaction(&mut [
            Operation::Write(&create_header(addr, true)),
            Operation::Write(data),
        ])
    }

    impl_read_primitive!(read_u8, u8);
    impl_read_primitive!(read_u16, u16);
    impl_read_primitive!(read_u32, u32);

    impl_write_primitive!(write_u8, u8);
    impl_write_primitive!(write_u16, u16);
    impl_write_primitive!(write_u32, u32);
}
