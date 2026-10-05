// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Dynamic address assignment helpers for MIPI I3C HCI.
//!
//! This is the Phase 3 DAA preparation layer. Hardware submission is handled
//! by the PIO path; assigned-target details come from the HCI DCT registers.

use super::cmd::{
    HciCommandKind, HciCommandV1, HciResponse, CMD_0_ROC, CMD_0_TOC, RESP_ERR_ADDR_HEADER,
    RESP_ERR_NACK,
};
use super::dat::{DatAllocator, DatEntryV1};
use super::error::{I3cHciError, Result};

pub const I3C_CCC_ENTDAA: u8 = 0x7;
pub const DEFAULT_DYNAMIC_ADDR_START: u8 = 0x09;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AddressSlots {
    used_lo: u64,
    used_hi: u64,
}

impl AddressSlots {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            used_lo: 0,
            used_hi: 0,
        }
    }

    pub fn mark_used(&mut self, address: u8) -> Result<()> {
        if !valid_i3c_address(address) {
            return Err(I3cHciError::InvalidArgs);
        }
        self.set(address);
        Ok(())
    }

    pub fn mark_free(&mut self, address: u8) -> Result<()> {
        if !valid_i3c_address(address) {
            return Err(I3cHciError::InvalidArgs);
        }
        if address < 64 {
            self.used_lo &= !(1u64 << address);
        } else {
            self.used_hi &= !(1u64 << (address - 64));
        }
        Ok(())
    }

    #[must_use]
    pub fn is_used(&self, address: u8) -> bool {
        if address < 64 {
            (self.used_lo & (1u64 << address)) != 0
        } else if address < 128 {
            (self.used_hi & (1u64 << (address - 64))) != 0
        } else {
            true
        }
    }

    pub fn next_free(&self, start: u8) -> Result<u8> {
        let mut address = start.max(DEFAULT_DYNAMIC_ADDR_START);
        while address < 0x7f {
            if valid_i3c_address(address) && !self.is_used(address) {
                return Ok(address);
            }
            address += 1;
        }
        Err(I3cHciError::NoSpace)
    }

    fn set(&mut self, address: u8) {
        if address < 64 {
            self.used_lo |= 1u64 << address;
        } else {
            self.used_hi |= 1u64 << (address - 64);
        }
    }
}

impl Default for AddressSlots {
    fn default() -> Self {
        Self::new()
    }
}

#[must_use]
pub const fn valid_i3c_address(address: u8) -> bool {
    address > 0x07 && address < 0x7f
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DaaStep {
    pub dat_index: u8,
    pub dynamic_addr: u8,
    pub dat_entry: DatEntryV1,
    pub command: HciCommandV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DaaResponse {
    Assigned,
    NoMoreDevices,
}

pub fn prepare_entdaa_step(dat_index: u8, dynamic_addr: u8, tid: u8) -> Result<DaaStep> {
    if dat_index > 0x7f || !valid_i3c_address(dynamic_addr) || tid > 0x0f {
        return Err(I3cHciError::InvalidArgs);
    }

    let word0 = 0x2
        | ((tid as u32) << 3)
        | ((I3C_CCC_ENTDAA as u32) << 7)
        | ((dat_index as u32) << 16)
        | (1 << 26)
        | CMD_0_ROC
        | CMD_0_TOC;

    Ok(DaaStep {
        dat_index,
        dynamic_addr,
        dat_entry: DatEntryV1::rejected().with_dynamic_addr(dynamic_addr),
        command: HciCommandV1 {
            words: [word0, 0],
            tid,
            kind: HciCommandKind::AddressAssignment,
            read: true,
            data_len: 0,
        },
    })
}

pub fn prepare_next_entdaa_step(
    address_slots: &AddressSlots,
    dat_allocator: &mut DatAllocator,
    preferred_dat_slot: Option<u8>,
    tid: u8,
) -> Result<DaaStep> {
    if tid > 0x0f {
        return Err(I3cHciError::InvalidArgs);
    }

    let dynamic_addr = address_slots.next_free(DEFAULT_DYNAMIC_ADDR_START)?;
    let preferred_slot = if dat_allocator.address_indexed() {
        Some(dynamic_addr)
    } else {
        preferred_dat_slot
    };
    let dat_index = dat_allocator.allocate(preferred_slot)?;
    prepare_entdaa_step(dat_index, dynamic_addr, tid)
}

pub fn classify_entdaa_response(response: HciResponse) -> Result<DaaResponse> {
    if response.success() {
        return Ok(DaaResponse::Assigned);
    }

    if (response.status == RESP_ERR_ADDR_HEADER || response.status == RESP_ERR_NACK)
        && response.data_len == 1
    {
        return Ok(DaaResponse::NoMoreDevices);
    }

    Err(I3cHciError::Transfer(response.status))
}
