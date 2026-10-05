// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 MIPI I3C HCI driver.
//!
//! This module starts with the PAC-backed register facade needed to port
//! Zephyr's drivers/i3c/mipi_hci logic into OpenProt. Protocol layers should
//! build on [I3cHciRegisters] rather than touching PAC-generated names.

pub mod cmd;
pub mod constants;
pub mod core;
pub mod daa;
pub mod dat;
pub mod dct;
pub mod error;
pub mod master;
pub mod pio;
pub mod registers;
pub mod types;

pub use cmd::{i2c_mode, i3c_sdr_mode, HciCommandKind, HciCommandV1, HciResponse};
pub use constants::*;
pub use core::{
    discover_sections_from_regs, parse_hci_version, select_command_descriptor, CommandDescriptor,
    HciSections, HciVersion, I3cHciConfig, I3cHciCore, IoMode, TableSection,
};
pub use daa::{
    classify_entdaa_response, prepare_entdaa_step, prepare_next_entdaa_step, valid_i3c_address,
    AddressSlots, DaaResponse, DaaStep, I3C_CCC_ENTDAA,
};
pub use dat::{odd_parity7, DatAllocator, DatEntryV1};
pub use dct::{read_dct_entry_v1, DctEntryV1, DCT_V1_ENTRY_SIZE, DCT_V1_ENTRY_WORDS};
pub use error::{I3cHciError, Result};
pub use master::{
    HciDeviceTable, HciDeviceTableIter, HciI3cDevice, I3cHciMaster, I3C_BROADCAST_ADDR,
};
pub use pio::{
    bytes_from_word, validate_pio_transfer, word_from_bytes, I3cHciPio, PioTransfer,
    PioTransferResult,
};
pub use registers::I3cHciRegisters;
pub use types::{InitMode, TargetRole};
