#[derive(Debug, Clone, Copy)]
pub enum DriverError {
    SpiError,
    PinError,

    UnexpectedResponse,
}

pub type Error = nb::Error<DriverError>;
