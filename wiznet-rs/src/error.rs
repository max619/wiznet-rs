#[derive(Debug, Clone, Copy)]
pub enum DriverError {
    SpiError,
    PinError,

    ScratchBufferOverrun,

    UnexpectedResponse,
}

pub type Error = nb::Error<DriverError>;
