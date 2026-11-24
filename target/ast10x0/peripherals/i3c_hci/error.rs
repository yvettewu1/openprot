// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Error types for the AST1080 MIPI I3C HCI driver.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I3cHciError {
    /// DMA backend failure retained for application diagnostics.
    Dma(DmaError),
    InvalidBus,
    InvalidArgs,
    Timeout,
    Busy,
    /// Bus addresses or command ownership are uncertain; recover before submission.
    RecoveryRequired,
    Unsupported,
    NoSpace,
    Transfer(u8),
    TransferStatus(u32),
}

pub type Result<T> = core::result::Result<T, I3cHciError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaError {
    Timeout(&'static str),
    Fault(&'static str),
}
impl DmaError {
    pub const fn message(self) -> &'static str {
        match self {
            Self::Timeout(message) | Self::Fault(message) => message,
        }
    }
}
impl core::fmt::Display for DmaError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.message())
    }
}
impl From<DmaError> for I3cHciError {
    fn from(error: DmaError) -> Self {
        Self::Dma(error)
    }
}
// Bare-metal diagnostic entry points return static messages.
impl From<DmaError> for &'static str {
    fn from(error: DmaError) -> Self {
        error.message()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dma_timeout_conversion_retains_the_phase() {
        let error = DmaError::Timeout("DMA queue reset timeout");
        assert_eq!(I3cHciError::from(error), I3cHciError::Dma(error));
    }
}
