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
pub mod dma;
pub mod error;
pub mod irq;
pub mod master;
pub mod pio;
pub mod registers;
pub mod target;
pub mod types;

pub use cmd::{
    i2c_mode, i3c_sdr_mode, HciCommandKind, HciCommandV1, HciResponse, HciTargetCommand,
    TID_TARGET_IBI, TID_TARGET_RD_DATA,
};
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
pub use dma::{init_rhs, parse_rhs_control, rhs_info, RhsInfo};
pub use error::{I3cHciError, Result};
pub use irq::{
    dispatch_i3c_hci_irq, isr_events, register_i3c_hci_irq_handler, unregister_i3c_hci_irq_handler,
    IsrCtx, IsrEvents,
};
pub use master::{
    HciDeviceTable, HciDeviceTableIter, HciI3cDevice, HciIbiEvent, I3cHciMaster,
    I3C_BROADCAST_ADDR, I3C_CCC_DISEC_BROADCAST, I3C_CCC_DISEC_DIRECT, I3C_CCC_ENEC_BROADCAST,
    I3C_CCC_ENEC_DIRECT, I3C_CCC_EVENT_HJ, I3C_CCC_EVENT_MR, I3C_CCC_EVENT_SIR, I3C_HOT_JOIN_ADDR,
};
pub use pio::{
    bytes_from_word, validate_pio_transfer, word_from_bytes, I3cHciPio, IbiStatus, PioTransfer,
    PioTransferResult, TargetResponse,
};
pub use registers::I3cHciRegisters;
pub use target::{HciTargetConfig, HciTargetEvent, I3cHciTarget};
pub use types::{InitMode, TargetRole};
