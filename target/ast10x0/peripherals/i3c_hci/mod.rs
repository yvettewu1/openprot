// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 MIPI I3C HCI driver.
//!
//! This module starts with the PAC-backed register facade needed to port
//! Zephyr's drivers/i3c/mipi_hci logic into OpenProt. Protocol layers should
//! build on [I3cHciRegisters] rather than touching PAC-generated names.

pub mod constants;
pub mod core;
pub mod error;
pub mod registers;
pub mod types;

pub use constants::*;
pub use core::{
    discover_sections_from_regs, parse_hci_version, select_command_descriptor, CommandDescriptor,
    HciSections, HciVersion, I3cHciConfig, I3cHciCore, IoMode, TableSection,
};
pub use error::{I3cHciError, Result};
pub use registers::I3cHciRegisters;
pub use types::{InitMode, TargetRole};
