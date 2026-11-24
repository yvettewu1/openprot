// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI DAT v1 helpers.
//!
//! The bit layout follows Zephyr drivers/i3c/mipi_hci/dat.h and dat_v1.c from
//! AspeedTech-BMC/zephyr aspeed-main-v3.7.0.

use super::error::{I3cHciError, Result};

pub const DAT_0_I2C_DEVICE: u32 = 1 << 31;
pub const DAT_0_SIR_REJECT: u32 = 1 << 13;
pub const DAT_0_MR_REJECT: u32 = 1 << 14;
pub const DAT_0_IBI_PAYLOAD: u32 = 1 << 12;
pub const DAT_0_DYNADDR_PARITY: u32 = 1 << 23;
pub const DAT_0_DYNAMIC_ADDRESS_SHIFT: u32 = 16;
pub const DAT_0_DYNAMIC_ADDRESS_MASK: u32 = 0x7f << DAT_0_DYNAMIC_ADDRESS_SHIFT;
pub const DAT_0_STATIC_ADDRESS_MASK: u32 = 0x7f;
pub const DAT_V1_ENTRY_SIZE: u8 = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DatEntryV1 {
    pub word0: u32,
    pub word1: u32,
}

impl DatEntryV1 {
    #[must_use]
    pub const fn rejected() -> Self {
        Self {
            word0: DAT_0_SIR_REJECT | DAT_0_MR_REJECT,
            word1: 0,
        }
    }

    #[must_use]
    pub const fn with_dynamic_addr(mut self, address: u8) -> Self {
        let addr = (address & 0x7f) as u32;
        self.word0 &= !(DAT_0_DYNAMIC_ADDRESS_MASK | DAT_0_DYNADDR_PARITY);
        self.word0 |= addr << DAT_0_DYNAMIC_ADDRESS_SHIFT;
        if !odd_parity7(address) {
            self.word0 |= DAT_0_DYNADDR_PARITY;
        }
        self
    }

    #[must_use]
    pub const fn with_static_addr(mut self, address: u8) -> Self {
        self.word0 &= !DAT_0_STATIC_ADDRESS_MASK;
        self.word0 |= (address & 0x7f) as u32;
        self
    }

    #[must_use]
    pub const fn with_i2c_device(mut self, address: u8) -> Self {
        self = self.with_static_addr(address);
        self.word0 |= DAT_0_I2C_DEVICE;
        self
    }

    #[must_use]
    pub const fn set_flags(mut self, word0: u32, word1: u32) -> Self {
        self.word0 |= word0;
        self.word1 |= word1;
        self
    }

    #[must_use]
    pub const fn clear_flags(mut self, word0: u32, word1: u32) -> Self {
        self.word0 &= !word0;
        self.word1 &= !word1;
        self
    }

    #[must_use]
    pub const fn dynamic_addr(self) -> u8 {
        ((self.word0 & DAT_0_DYNAMIC_ADDRESS_MASK) >> DAT_0_DYNAMIC_ADDRESS_SHIFT) as u8
    }

    #[must_use]
    pub const fn static_addr(self) -> u8 {
        (self.word0 & DAT_0_STATIC_ADDRESS_MASK) as u8
    }
}

#[must_use]
pub const fn odd_parity7(address: u8) -> bool {
    ((address & 0x7f).count_ones() & 1) != 0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DatAllocator {
    allocated: u128,
    entries: u8,
    address_indexed: bool,
}

impl DatAllocator {
    pub const fn new(entries: u8, address_indexed: bool) -> Self {
        Self {
            allocated: 0,
            entries,
            address_indexed,
        }
    }

    pub fn allocate(&mut self, preferred_slot: Option<u8>) -> Result<u8> {
        if let Some(slot) = preferred_slot {
            if self.slot_available(slot) {
                self.mark(slot);
                return Ok(slot);
            }
            if self.address_indexed {
                return Err(I3cHciError::NoSpace);
            }
        }

        let mut slot = 0;
        while slot < self.entries {
            if self.slot_available(slot) {
                self.mark(slot);
                return Ok(slot);
            }
            slot += 1;
        }
        Err(I3cHciError::NoSpace)
    }

    pub fn free(&mut self, slot: u8) -> Result<()> {
        if !self.slot_in_range(slot) {
            return Err(I3cHciError::InvalidArgs);
        }
        self.allocated &= !(1u128 << slot);
        Ok(())
    }

    #[must_use]
    pub fn is_allocated(&self, slot: u8) -> bool {
        self.slot_in_range(slot) && (self.allocated & (1u128 << slot)) != 0
    }

    #[must_use]
    pub const fn address_indexed(&self) -> bool {
        self.address_indexed
    }

    #[must_use]
    fn slot_available(&self, slot: u8) -> bool {
        self.slot_in_range(slot) && !self.is_allocated(slot)
    }

    #[must_use]
    fn slot_in_range(&self, slot: u8) -> bool {
        slot < self.entries && slot < 128
    }

    fn mark(&mut self, slot: u8) {
        self.allocated |= 1u128 << slot;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciI3cDevice {
    pub dat_index: u8,
    pub dynamic_addr: u8,
    pub pid: u64,
    pub bcr: u8,
    pub dcr: u8,
}

pub struct HciDeviceTable<const N: usize> {
    entries: [Option<HciI3cDevice>; N],
    len: usize,
}

impl<const N: usize> HciDeviceTable<N> {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: [None; N],
            len: 0,
        }
    }

    #[must_use]
    pub const fn capacity(&self) -> usize {
        N
    }

    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[must_use]
    pub const fn is_full(&self) -> bool {
        self.len >= N
    }

    pub fn clear(&mut self) {
        let mut idx = 0;
        while idx < N {
            self.entries[idx] = None;
            idx += 1;
        }
        self.len = 0;
    }

    pub fn push(&mut self, device: HciI3cDevice) -> Result<()> {
        if self.is_full() {
            return Err(I3cHciError::NoSpace);
        }
        self.entries[self.len] = Some(device);
        self.len += 1;
        Ok(())
    }

    pub fn push_or_replace(&mut self, device: HciI3cDevice) -> Result<()> {
        let mut idx = 0;
        while idx < self.len {
            if let Some(entry) = self.entries[idx] {
                if entry.dat_index == device.dat_index
                    || (device.pid != 0 && entry.pid == device.pid)
                {
                    self.entries[idx] = Some(device);
                    return Ok(());
                }
            }
            idx += 1;
        }

        self.push(device)
    }

    #[must_use]
    pub fn by_dynamic_addr(&self, dynamic_addr: u8) -> Option<HciI3cDevice> {
        self.iter()
            .find(|device| device.dynamic_addr == dynamic_addr)
    }

    #[must_use]
    pub fn by_pid(&self, pid: u64) -> Option<HciI3cDevice> {
        self.iter().find(|device| device.pid == pid)
    }

    pub fn iter(&self) -> HciDeviceTableIter<'_, N> {
        HciDeviceTableIter {
            table: self,
            index: 0,
        }
    }
}

impl<const N: usize> Default for HciDeviceTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct HciDeviceTableIter<'a, const N: usize> {
    table: &'a HciDeviceTable<N>,
    index: usize,
}

impl<const N: usize> Iterator for HciDeviceTableIter<'_, N> {
    type Item = HciI3cDevice;

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.table.len {
            let index = self.index;
            self.index += 1;
            if let Some(device) = self.table.entries[index] {
                return Some(device);
            }
        }
        None
    }
}
