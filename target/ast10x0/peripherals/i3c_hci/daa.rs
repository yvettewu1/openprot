// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Dynamic address assignment helpers for MIPI I3C HCI.
//!
//! This module prepares command descriptors and DAT entries. Hardware
//! submission uses the selected PIO/DMA engine. V1 reads identity from DCT;
//! V2 receives it through the paired DAA transfer.

use super::ccc::I3C_CCC_ENTDAA;
use super::cmd::{
    CMD_0_ROC, CMD_0_TOC, HciCommandKind, HciCommandV1, HciResponse, RESP_ERR_ADDR_HEADER,
    RESP_ERR_NACK,
};
use super::dat::{DatAllocator, DatEntryV1};
use super::error::{I3cHciError, Result};

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

/// Address-changing operations become uncertain before touching the bus.
/// Only successful recovery/commit makes cached identities usable again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TopologyState {
    Known,
    RecoveryRequired,
}
impl TopologyState {
    pub fn require_known(self) -> Result<()> {
        match self {
            Self::Known => Ok(()),
            Self::RecoveryRequired => Err(I3cHciError::RecoveryRequired),
        }
    }
}

use super::cmd::{HciCommand, HciCommandV2};
use super::core::CommandDescriptor;
use super::dat::{HciDeviceTable, HciI3cDevice};
use super::master::{HciTransfer, I3cHciMaster};

impl<Y: FnMut(u32)> I3cHciMaster<'_, Y> {
    pub fn entdaa_once(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<Option<HciI3cDevice>> {
        match self.command_descriptor {
            CommandDescriptor::V1 => self.entdaa_once_v1(dat_index, dynamic_addr),
            CommandDescriptor::V2 => self.entdaa_once_v2(dat_index, dynamic_addr),
        }
    }

    pub fn entdaa_all<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
    ) -> Result<usize> {
        table.clear();
        self.entdaa_new_devices(table, dat_allocator)
    }

    pub fn entdaa_new_devices<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
    ) -> Result<usize> {
        let initial_len = table.len();
        let mut address_slots = AddressSlots::new();
        let controller_addr = self.regs.controller_device_addr();
        if controller_addr & super::constants::MASTER_DYNAMIC_ADDR_VALID != 0 {
            let addr = ((controller_addr & super::constants::MASTER_DYNAMIC_ADDR_MASK)
                >> super::constants::MASTER_DYNAMIC_ADDR_SHIFT) as u8;
            address_slots.mark_used(addr)?;
        }

        for device in table.iter() {
            address_slots.mark_used(device.dynamic_addr)?;
        }

        match self.command_descriptor {
            CommandDescriptor::V1 => {
                self.entdaa_new_devices_v1(table, dat_allocator, &mut address_slots, initial_len)
            }
            CommandDescriptor::V2 => {
                self.entdaa_new_devices_v2(table, dat_allocator, &mut address_slots, initial_len)
            }
        }
    }

    fn entdaa_once_v1(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<Option<HciI3cDevice>> {
        let tid = self.next_tid();
        let step = prepare_entdaa_step(dat_index, dynamic_addr, tid)?;
        self.regs.aspeed_prepare_daa(step.dynamic_addr);
        self.write_dat_entry(step.dat_index, step.dat_entry)?;
        self.regs.reset_dct_index();
        let mut rx = [];
        let result = self.submit_raw(HciTransfer {
            command: HciCommand::V1(step.command),
            tx: &[],
            rx: &mut rx,
        })?;

        match classify_entdaa_response(result.response)? {
            DaaResponse::Assigned => self
                .attach_assigned_device_from_dct(step.dat_index, step.dynamic_addr, 0)
                .map(Some),
            DaaResponse::NoMoreDevices => {
                self.write_dat_entry(step.dat_index, DatEntryV1::rejected())?;
                self.recover_after_daa_nack()?;
                Ok(None)
            }
        }
    }

    fn entdaa_once_v2(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<Option<HciI3cDevice>> {
        if dat_index > 0x7f || !valid_i3c_address(dynamic_addr) {
            return Err(I3cHciError::InvalidArgs);
        }

        let mut rx = [0u8; 8];
        match self.entdaa_submit_v2(dynamic_addr, &mut rx)? {
            DaaResponse::Assigned => {
                let device = decode_v2_daa_identity(dat_index, dynamic_addr, &rx)?;
                self.write_dat_entry(
                    dat_index,
                    DatEntryV1::rejected().with_dynamic_addr(dynamic_addr),
                )?;
                Ok(Some(device))
            }
            DaaResponse::NoMoreDevices => Ok(None),
        }
    }

    fn entdaa_new_devices_v1<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
        address_slots: &mut AddressSlots,
        initial_len: usize,
    ) -> Result<usize> {
        loop {
            if table.is_full() {
                return Err(I3cHciError::NoSpace);
            }

            let tid = self.next_tid();
            let step = prepare_next_entdaa_step(address_slots, dat_allocator, None, tid)?;
            self.regs.aspeed_prepare_daa(step.dynamic_addr);
            self.write_dat_entry(step.dat_index, step.dat_entry)?;
            self.regs.reset_dct_index();
            let mut rx = [];
            let result = match self.submit_raw(HciTransfer {
                command: HciCommand::V1(step.command),
                tx: &[],
                rx: &mut rx,
            }) {
                Ok(result) => result,
                Err(error) => {
                    let _ = self.write_dat_entry(step.dat_index, DatEntryV1::rejected());
                    let _ = dat_allocator.free(step.dat_index);
                    return Err(error);
                }
            };

            match classify_entdaa_response(result.response) {
                Ok(DaaResponse::Assigned) => {
                    let dct = match self.read_dct_entry(0) {
                        Ok(dct) => dct,
                        Err(error) => {
                            let _ = self.write_dat_entry(step.dat_index, DatEntryV1::rejected());
                            let _ = dat_allocator.free(step.dat_index);
                            return Err(error);
                        }
                    };
                    // Verify that ENTDAA actually discovered a target.
                    if dct.pid == 0 {
                        self.write_dat_entry(step.dat_index, DatEntryV1::rejected())?;

                        dat_allocator.free(step.dat_index)?;

                        return Err(I3cHciError::TransferStatus(0));
                    }
                    let device = HciI3cDevice {
                        dat_index: step.dat_index,
                        dynamic_addr: step.dynamic_addr,
                        pid: dct.pid,
                        bcr: dct.bcr,
                        dcr: dct.dcr,
                    };

                    if let Err(error) = table.push(device) {
                        let _ = self.write_dat_entry(step.dat_index, DatEntryV1::rejected());
                        let _ = dat_allocator.free(step.dat_index);
                        return Err(error);
                    }
                    address_slots.mark_used(step.dynamic_addr)?;
                    if self.transport.is_dma() {
                        return Ok(table.len() - initial_len);
                    }
                }
                Ok(DaaResponse::NoMoreDevices) => {
                    self.write_dat_entry(step.dat_index, DatEntryV1::rejected())?;
                    dat_allocator.free(step.dat_index)?;
                    self.recover_after_daa_nack()?;
                    return Ok(table.len() - initial_len);
                }
                Err(error) => {
                    let _ = self.write_dat_entry(step.dat_index, DatEntryV1::rejected());
                    let _ = dat_allocator.free(step.dat_index);
                    return Err(error);
                }
            }
        }
    }

    fn entdaa_new_devices_v2<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
        address_slots: &mut AddressSlots,
        initial_len: usize,
    ) -> Result<usize> {
        loop {
            if table.is_full() {
                return Err(I3cHciError::NoSpace);
            }

            let dynamic_addr = address_slots.next_free(super::daa::DEFAULT_DYNAMIC_ADDR_START)?;
            let preferred_slot = if dat_allocator.address_indexed() {
                Some(dynamic_addr)
            } else {
                None
            };
            let dat_index = dat_allocator.allocate(preferred_slot)?;
            let mut rx = [0u8; 8];

            match self.entdaa_submit_v2(dynamic_addr, &mut rx) {
                Ok(DaaResponse::Assigned) => {
                    let device = match decode_v2_daa_identity(dat_index, dynamic_addr, &rx) {
                        Ok(device) => device,
                        Err(error) => {
                            let _ = dat_allocator.free(dat_index);
                            return Err(error);
                        }
                    };
                    if let Err(error) = self.write_dat_entry(
                        dat_index,
                        DatEntryV1::rejected().with_dynamic_addr(dynamic_addr),
                    ) {
                        let _ = dat_allocator.free(dat_index);
                        return Err(error);
                    }
                    if let Err(error) = table.push(device) {
                        let _ = self.write_dat_entry(dat_index, DatEntryV1::rejected());
                        let _ = dat_allocator.free(dat_index);
                        return Err(error);
                    }
                    address_slots.mark_used(dynamic_addr)?;
                }
                Ok(DaaResponse::NoMoreDevices) => {
                    dat_allocator.free(dat_index)?;
                    return Ok(table.len() - initial_len);
                }
                Err(error) => {
                    let _ = dat_allocator.free(dat_index);
                    return Err(error);
                }
            }
        }
    }

    fn entdaa_submit_v2(&mut self, dynamic_addr: u8, rx: &mut [u8; 8]) -> Result<DaaResponse> {
        let read_tid = self.next_tid();
        let assign_tid = self.next_tid();
        let read = HciCommand::V2(HciCommandV2::daa_read_pid(read_tid, rx.len())?);
        let assign = HciCommand::V2(HciCommandV2::daa_assign_address(assign_tid, dynamic_addr)?);
        if let Some(ring) = self.transport.dma_mut() {
            let (first, second) = ring.submit_daa_pair(read, assign, rx)?;
            return match classify_entdaa_response(first)? {
                DaaResponse::Assigned => {
                    let response = second.ok_or(I3cHciError::InvalidArgs)?;
                    if !response.success() {
                        return Err(I3cHciError::Transfer(response.status));
                    }
                    Ok(DaaResponse::Assigned)
                }
                DaaResponse::NoMoreDevices => {
                    ring.recover_after_daa_nack(self.regs)?;
                    Ok(DaaResponse::NoMoreDevices)
                }
            };
        }
        let result = self
            .transport
            .pio_mut()?
            .submit_pair_raw(read, assign, rx)?;

        match classify_entdaa_response(result.first_response)? {
            DaaResponse::Assigned => {
                if !result.second_response.success() {
                    return Err(I3cHciError::Transfer(result.second_response.status));
                }
                if result.rx_len != rx.len() {
                    return Err(I3cHciError::InvalidArgs);
                }
                Ok(DaaResponse::Assigned)
            }
            DaaResponse::NoMoreDevices => Ok(DaaResponse::NoMoreDevices),
        }
    }
}
fn decode_v2_daa_identity(
    dat_index: u8,
    dynamic_addr: u8,
    identity: &[u8; 8],
) -> Result<HciI3cDevice> {
    if dat_index > 0x7f || !valid_i3c_address(dynamic_addr) {
        return Err(I3cHciError::InvalidArgs);
    }

    let pid_low = u32::from_le_bytes([identity[0], identity[1], identity[2], identity[3]]) as u64;
    let upper = u32::from_le_bytes([identity[4], identity[5], identity[6], identity[7]]);
    let pid = (((upper & 0xffff) as u64) << 32) | pid_low;
    let bcr = ((upper >> 16) & 0xff) as u8;
    let dcr = ((upper >> 24) & 0xff) as u8;

    Ok(HciI3cDevice {
        dat_index,
        dynamic_addr,
        pid,
        bcr,
        dcr,
    })
}

#[cfg(test)]
mod topology_tests {
    use super::*;
    #[test]
    fn uncertain_topology_rejects_cached_address_use() {
        assert_eq!(TopologyState::Known.require_known(), Ok(()));
        assert_eq!(
            TopologyState::RecoveryRequired.require_known(),
            Err(I3cHciError::RecoveryRequired)
        );
    }
}
