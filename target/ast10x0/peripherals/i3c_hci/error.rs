// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Error types for the AST1080 MIPI I3C HCI driver.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I3cHciError {
    InvalidBus,
    InvalidArgs,
    Timeout,
    Busy,
    Unsupported,
    NoSpace,
    Transfer(u8),
    TransferStatus(u32),
}

pub type Result<T> = core::result::Result<T, I3cHciError>;
