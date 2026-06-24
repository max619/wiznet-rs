use embedded_hal::{delay::DelayNs, spi::ErrorKind};

pub struct DmaBuffers {
    pub rx: &'static mut [u8],
    pub tx: &'static mut [u8],

    pub len: usize,
}

pub trait SpiDmaTransaction<Device> {
    fn wait(self) -> (Device, DmaBuffers);
}

pub trait SpiDmaDevice: DelayNs + Sized {
    type Error: embedded_hal::spi::Error + From<ErrorKind>;
    type Transaction: SpiDmaTransaction<Self>;

    fn transceive(
        self,
        buffers: DmaBuffers,
    ) -> Result<Self::Transaction, (Self::Error, Self, DmaBuffers)>;
}
