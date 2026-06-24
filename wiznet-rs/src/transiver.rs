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

                    Operation::DelayNs(_) => break,
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

            while start_index < index {
                match &mut operations[start_index] {
                    Operation::Read(items)
                    | Operation::Transfer(items, _)
                    | Operation::TransferInPlace(items) => {
                        items.copy_from_slice(&scratch_buffers.rx[offset..offset + items.len()]);
                        offset += items.len();
                    }

                    // A write still clocks bytes onto the bus, so it occupies a
                    // slot in the full-duplex rx mirror: advance `offset` past it
                    // to stay aligned with the batching pass above (otherwise the
                    // following reads copy from the wrong window).
                    // NOTE: `Transfer(read, write)` assumes `read.len() ==
                    // write.len()`; the driver only emits `Read`/`Write`.
                    Operation::Write(items) => offset += items.len(),
                    Operation::DelayNs(_) => (),
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

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::{delay::DelayNs, spi::ErrorKind};
    use std::{cell::RefCell, rc::Rc, vec, vec::Vec};

    /// What every mock transfer captures / replays.
    #[derive(Default)]
    struct MockLog {
        /// MOSI bytes (the active `len` window of `tx`) for each `transceive`.
        transfers: Vec<Vec<u8>>,
        /// Recorded `DelayNs` requests.
        delays: Vec<u32>,
    }

    /// A `SpiDmaDevice` that records MOSI and synthesizes a deterministic,
    /// *position-indexed* MISO response: `rx[i] = i`. That pattern is the whole
    /// point — it lets a test tell apart "read the right window" from "read the
    /// header-echo bytes", since the value equals the buffer offset it came from.
    struct MockDevice {
        log: Rc<RefCell<MockLog>>,
    }

    struct MockTransaction {
        dev: MockDevice,
        buffers: DmaBuffers,
    }

    impl SpiDmaTransaction<MockDevice> for MockTransaction {
        fn wait(self) -> (MockDevice, DmaBuffers) {
            (self.dev, self.buffers)
        }
    }

    impl DelayNs for MockDevice {
        fn delay_ns(&mut self, ns: u32) {
            self.log.borrow_mut().delays.push(ns);
        }
    }

    impl SpiDmaDevice for MockDevice {
        type Error = ErrorKind;
        type Transaction = MockTransaction;

        fn transceive(
            self,
            buffers: DmaBuffers,
        ) -> Result<Self::Transaction, (Self::Error, Self, DmaBuffers)> {
            let len = buffers.len;
            self.log.borrow_mut().transfers.push(buffers.tx[..len].to_vec());
            for i in 0..len {
                buffers.rx[i] = i as u8;
            }
            Ok(MockTransaction { dev: self, buffers })
        }
    }

    /// Leak a zeroed buffer so it satisfies the `&'static mut` in `DmaBuffers`.
    fn leak(n: usize) -> &'static mut [u8] {
        vec![0u8; n].leak()
    }

    fn rig(scratch_len: usize) -> (Transceiver<MockDevice>, Rc<RefCell<MockLog>>) {
        let log = Rc::new(RefCell::new(MockLog::default()));
        let dev = MockDevice { log: log.clone() };
        let scratch = DmaBuffers {
            rx: leak(scratch_len),
            tx: leak(scratch_len),
            len: 0,
        };
        (Transceiver::new(dev, scratch), log)
    }

    const ADDR: Address = Address {
        address: 0x1234,
        block: BlockSelectionBits::CommonRegister,
    };

    /// The headline regression: a `read` must copy the payload window
    /// (`rx[3..3+N]`), not the bytes clocked in under the 3-byte header.
    #[test]
    fn read_copies_payload_window_not_header() {
        let (txr, log) = rig(16);
        let mut buf = [0u8; 4];

        txr.read(&ADDR, &mut buf).unwrap();

        // rx[i] == i, header is 3 bytes => payload is rx[3..7].
        assert_eq!(buf, [3, 4, 5, 6]);

        let log = log.borrow();
        assert_eq!(log.transfers.len(), 1, "one full-duplex transfer");
        let mosi = &log.transfers[0];
        assert_eq!(mosi.len(), 3 + 4, "header + read padding");
        assert_eq!(&mosi[..2], &0x1234u16.to_be_bytes(), "address, big-endian");
        assert_eq!(mosi[2] & 0b100, 0, "RWB clear on read");
    }

    /// A `write` stages header + payload as MOSI, sets the RWB bit, and copies
    /// nothing back.
    #[test]
    fn write_stages_header_and_payload() {
        let (txr, log) = rig(16);

        txr.write(&ADDR, &[0xAA, 0xBB]).unwrap();

        let log = log.borrow();
        let mosi = &log.transfers[0];
        assert_eq!(mosi.len(), 3 + 2);
        assert_eq!(mosi[2] & 0b100, 0b100, "RWB set on write");
        assert_eq!(&mosi[3..], &[0xAA, 0xBB]);
    }

    /// Primitive round-trip: `read_u16` decodes big-endian from the payload
    /// window. (rx[3], rx[4]) == (3, 4) => 0x0304.
    #[test]
    fn read_u16_decodes_payload() {
        let (txr, _log) = rig(16);
        assert_eq!(txr.read_u16(&ADDR).unwrap(), 0x0304);
    }

    /// A batch larger than the scratch must split into multiple transfers, and
    /// the read-back offset must reset per chunk. Scratch holds 4 bytes;
    /// `[Write(2), Read(2), Read(2)]` packs the first two ops (offset 2+2=4),
    /// then the trailing read goes in its own transfer.
    #[test]
    fn batch_splits_across_chunks_with_correct_offsets() {
        let (txr, log) = rig(4);
        let mut a = [0u8; 2];
        let mut b = [0u8; 2];

        txr.transaction(&mut [
            Operation::Write(&[0x10, 0x20]),
            Operation::Read(&mut a),
            Operation::Read(&mut b),
        ])
        .unwrap();

        // Chunk 1 = [Write(2), Read(2)] -> a reads rx[2..4] = [2, 3].
        // Chunk 2 = [Read(2)]           -> b reads rx[0..2] = [0, 1].
        assert_eq!(a, [2, 3]);
        assert_eq!(b, [0, 1]);
        assert_eq!(log.borrow().transfers.len(), 2);
    }

    /// A single operation that cannot fit the scratch is unsplittable and must
    /// surface `ScratchBufferOverrun` rather than panic or truncate.
    #[test]
    fn oversized_single_op_reports_overrun() {
        let (txr, _log) = rig(4);
        let err = txr
            .transaction(&mut [Operation::Write(&[0; 8])])
            .unwrap_err();
        assert!(matches!(
            err,
            nb::Error::Other(DriverError::ScratchBufferOverrun)
        ));
    }

    /// `DelayNs` is executed inline and never produces a bus transfer.
    #[test]
    fn delay_is_executed_without_transfer() {
        let (txr, log) = rig(16);

        txr.transaction(&mut [Operation::DelayNs(1_000)]).unwrap();

        let log = log.borrow();
        assert_eq!(log.delays, vec![1_000]);
        assert!(log.transfers.is_empty());
    }
}
