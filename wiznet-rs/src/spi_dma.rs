use embedded_hal::{delay::DelayNs, spi::ErrorKind};

pub struct DmaBuffers {
    pub rx: &'static mut [u8],
    pub tx: &'static mut [u8],

    pub len: usize,
}

/// How a transfer signals completion.
///
/// The mode is chosen by [`SpiDmaDevice::transceive`] because that is the only
/// place that starts the DMA (sets the channel-enable bit), so for `Interrupt`
/// the completion IRQ can be armed *before* the transfer can finish — there is
/// no window where the transfer-complete flag could be raised before the
/// interrupt is enabled.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Completion {
    /// Small/synchronous transfer: the caller blocks in [`SpiDmaTransaction::wait`]
    /// (the device need not raise an interrupt).
    Poll,
    /// Bulk/asynchronous transfer: arm the DMA-complete interrupt so the caller
    /// can return now and finish from the IRQ (`wait` is then instant).
    Interrupt,
}

pub trait SpiDmaTransaction<Device> {
    fn wait(self) -> (Device, DmaBuffers);
}

pub trait SpiDmaDevice: DelayNs + Sized {
    type Error: embedded_hal::spi::Error + From<ErrorKind>;
    type Transaction: SpiDmaTransaction<Self>;

    /// Start one full-duplex DMA of `buffers.len` bytes. For
    /// [`Completion::Interrupt`] the DMA-complete interrupt must be armed before
    /// the transfer can complete.
    fn transceive(
        self,
        buffers: DmaBuffers,
        completion: Completion,
    ) -> Result<Self::Transaction, (Self::Error, Self, DmaBuffers)>;
}
