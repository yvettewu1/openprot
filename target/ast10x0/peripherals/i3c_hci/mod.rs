// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 MIPI I3C HCI driver.
//!
//! This module starts with the PAC-backed register facade needed to port
//! Zephyr's drivers/i3c/mipi_hci logic into OpenProt. Protocol layers should
//! build on [I3cHciRegisters] rather than touching PAC-generated names.

pub mod ccc;
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

pub use ccc::*;
pub use cmd::{
    HciCommand, HciCommandKind, HciCommandV1, HciCommandV2, HciResponse, HciTargetCommand,
    TID_TARGET_IBI, TID_TARGET_RD_DATA, i2c_mode, i2c_v2_rate, i3c_sdr_mode, i3c_sdr_v2_rate,
};
pub use constants::*;
pub use core::{
    CommandDescriptor, HciSections, HciVersion, I3cHciConfig, I3cHciCore, IoMode, TableSection,
    discover_sections_from_regs, parse_hci_version, select_command_descriptor,
};
pub use daa::{
    AddressSlots, DaaResponse, DaaStep, classify_entdaa_response, prepare_entdaa_step,
    prepare_next_entdaa_step, valid_i3c_address,
};
pub use dat::{
    DAT_0_I2C_DEVICE, DAT_0_IBI_PAYLOAD, DAT_0_MR_REJECT, DAT_0_SIR_REJECT, DAT_V1_ENTRY_SIZE,
    DatAllocator, DatEntryV1, odd_parity7,
};
pub use dct::{DCT_V1_ENTRY_SIZE, DCT_V1_ENTRY_WORDS, DctEntryV1, read_dct_entry_v1};
pub use dma::{
    DmaIbiEvent, DmaRing, DmaRingEntry, RhsInfo, Ring, init_rhs, parse_rhs_control, rhs_info,
};
pub use error::{I3cHciError, Result};
pub use irq::{
    IsrCtx, IsrEvents, dispatch_i3c_hci_irq, isr_events, register_i3c_hci_irq_handler,
    unregister_i3c_hci_irq_handler,
};
pub use master::{
    HciDeviceTable, HciDeviceTableIter, HciI3cDevice, HciIbiEvent, HciTransfer, HciTransferResult,
    I3C_BROADCAST_ADDR, I3C_HOT_JOIN_ADDR, I3cHciMaster, dynamic_address_payload,
    validate_transfer,
};
pub use pio::{
    I3cHciPio, IbiStatus, PioTransferPairResult, TargetResponse, bytes_from_word, word_from_bytes,
};
pub use registers::I3cHciRegisters;
pub use target::{
    HciTargetCcc, HciTargetConfig, HciTargetEvent, I3cHciTarget, classify_target_ccc,
};
pub use types::{InitMode, TargetRole};
