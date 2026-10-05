// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Minimal OpenProt-facing MIPI I3C HCI master API.

use super::cmd::{i3c_sdr_mode, HciCommandV1};
use super::core::HciSections;
use super::daa::{
    classify_entdaa_response, prepare_entdaa_step, prepare_next_entdaa_step, AddressSlots,
    DaaResponse,
};
use super::dat::{DatAllocator, DatEntryV1, DAT_V1_ENTRY_SIZE};
use super::dct::{read_dct_entry_v1, DctEntryV1};
use super::error::{I3cHciError, Result};
use super::pio::{I3cHciPio, PioTransfer, PioTransferResult};
use super::registers::I3cHciRegisters;

pub const I3C_BROADCAST_ADDR: u8 = 0x7e;

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

pub struct I3cHciMaster<'a, Y: FnMut(u32)> {
    regs: &'a I3cHciRegisters,
    sections: HciSections,
    pio: I3cHciPio<'a, Y>,
    next_tid: u8,
    i3c_mode: u8,
}

impl<'a, Y: FnMut(u32)> I3cHciMaster<'a, Y> {
    #[must_use]
    pub fn new(
        regs: &'a I3cHciRegisters,
        sections: HciSections,
        i3c_scl_hz: u32,
        yield_us: Y,
    ) -> Self {
        Self {
            regs,
            sections,
            pio: I3cHciPio::new(regs, yield_us),
            next_tid: 0,
            i3c_mode: i3c_sdr_mode(i3c_scl_hz),
        }
    }

    #[must_use]
    pub fn with_timeout_us(mut self, timeout_us: u32) -> Self {
        self.pio = self.pio.with_timeout_us(timeout_us);
        self
    }

    pub fn attach_i3c_device(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<()> {
        self.write_dat_entry(
            dat_index,
            DatEntryV1::rejected().with_dynamic_addr(dynamic_addr),
        )
    }

    pub fn read_dct_entry(&self, index: u8) -> Result<DctEntryV1> {
        read_dct_entry_v1(self.regs, self.sections.dct, index)
    }

    pub fn attach_assigned_device_from_dct(
        &mut self,
        dat_index: u8,
        dynamic_addr: u8,
        dct_index: u8,
    ) -> Result<HciI3cDevice> {
        let dct = self.read_dct_entry(dct_index)?;
        self.attach_i3c_device(dat_index, dynamic_addr)?;
        Ok(HciI3cDevice {
            dat_index,
            dynamic_addr,
            pid: dct.pid,
            bcr: dct.bcr,
            dcr: dct.dcr,
        })
    }

    pub fn entdaa_once(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<Option<HciI3cDevice>> {
        let tid = self.next_tid();
        let step = prepare_entdaa_step(dat_index, dynamic_addr, tid)?;
        self.write_dat_entry(step.dat_index, step.dat_entry)?;
        let mut rx = [];
        let result = self.pio.submit_v1_raw(PioTransfer {
            command: step.command,
            tx: &[],
            rx: &mut rx,
        })?;

        match classify_entdaa_response(result.response)? {
            DaaResponse::Assigned => self
                .attach_assigned_device_from_dct(step.dat_index, step.dynamic_addr, 0)
                .map(Some),
            DaaResponse::NoMoreDevices => {
                self.write_dat_entry(step.dat_index, DatEntryV1::rejected())?;
                Ok(None)
            }
        }
    }

    pub fn entdaa_all<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
    ) -> Result<usize> {
        table.clear();
        let mut address_slots = AddressSlots::new();

        loop {
            if table.is_full() {
                return Err(I3cHciError::NoSpace);
            }

            let tid = self.next_tid();
            let step = prepare_next_entdaa_step(&address_slots, dat_allocator, None, tid)?;
            self.write_dat_entry(step.dat_index, step.dat_entry)?;
            let mut rx = [];
            let result = match self.pio.submit_v1_raw(PioTransfer {
                command: step.command,
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

            match classify_entdaa_response(result.response)? {
                DaaResponse::Assigned => {
                    let dct = self.read_dct_entry(0)?;
                    let device = HciI3cDevice {
                        dat_index: step.dat_index,
                        dynamic_addr: step.dynamic_addr,
                        pid: dct.pid,
                        bcr: dct.bcr,
                        dcr: dct.dcr,
                    };
                    table.push(device)?;
                    address_slots.mark_used(step.dynamic_addr)?;
                }
                DaaResponse::NoMoreDevices => {
                    self.write_dat_entry(step.dat_index, DatEntryV1::rejected())?;
                    dat_allocator.free(step.dat_index)?;
                    return Ok(table.len());
                }
            }
        }
    }

    pub fn private_write(&mut self, dat_index: u8, data: &[u8]) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let mode = self.i3c_mode;
        let command = HciCommandV1::i3c_private_write_regular(dat_index, tid, data.len(), mode)?
            .with_roc()
            .with_toc();
        let mut rx = [];
        self.pio.submit_v1(PioTransfer {
            command,
            tx: data,
            rx: &mut rx,
        })
    }

    pub fn private_read(&mut self, dat_index: u8, out: &mut [u8]) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let mode = self.i3c_mode;
        let command = HciCommandV1::i3c_private_read(dat_index, tid, out.len(), mode)?
            .with_roc()
            .with_toc();
        self.pio.submit_v1(PioTransfer {
            command,
            tx: &[],
            rx: out,
        })
    }

    pub fn private_write_addr<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        dynamic_addr: u8,
        data: &[u8],
    ) -> Result<PioTransferResult> {
        let device = table
            .by_dynamic_addr(dynamic_addr)
            .ok_or(I3cHciError::InvalidArgs)?;
        self.private_write(device.dat_index, data)
    }

    pub fn private_read_addr<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        dynamic_addr: u8,
        out: &mut [u8],
    ) -> Result<PioTransferResult> {
        let device = table
            .by_dynamic_addr(dynamic_addr)
            .ok_or(I3cHciError::InvalidArgs)?;
        self.private_read(device.dat_index, out)
    }

    pub fn private_write_pid<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        pid: u64,
        data: &[u8],
    ) -> Result<PioTransferResult> {
        let device = table.by_pid(pid).ok_or(I3cHciError::InvalidArgs)?;
        self.private_write(device.dat_index, data)
    }

    pub fn private_read_pid<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        pid: u64,
        out: &mut [u8],
    ) -> Result<PioTransferResult> {
        let device = table.by_pid(pid).ok_or(I3cHciError::InvalidArgs)?;
        self.private_read(device.dat_index, out)
    }

    pub fn ccc_broadcast_write(&mut self, ccc: u8, data: &[u8]) -> Result<()> {
        let tid = self.next_tid();
        let mode = self.i3c_mode;
        let command = HciCommandV1::ccc(0, tid, false, ccc, data, mode, None)?.with_toc();
        let mut rx = [];
        if command.expects_response() {
            self.pio
                .submit_v1(PioTransfer {
                    command,
                    tx: data,
                    rx: &mut rx,
                })
                .map(|_| ())
        } else {
            let tx = if data.len() <= 4 { &[] } else { data };
            self.pio.submit_v1_no_response(PioTransfer {
                command,
                tx,
                rx: &mut rx,
            })
        }
    }

    pub fn ccc_direct_write(
        &mut self,
        dat_index: u8,
        ccc: u8,
        data: &[u8],
    ) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let mode = self.i3c_mode;
        let command =
            HciCommandV1::ccc_regular(dat_index, tid, false, ccc, data.len(), mode, None)?
                .with_roc()
                .with_toc();
        let mut rx = [];
        self.pio.submit_v1(PioTransfer {
            command,
            tx: data,
            rx: &mut rx,
        })
    }

    pub fn ccc_direct_read(
        &mut self,
        dat_index: u8,
        ccc: u8,
        defining_byte: Option<u8>,
        out: &mut [u8],
    ) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let mode = self.i3c_mode;
        let command =
            HciCommandV1::ccc_regular(dat_index, tid, true, ccc, out.len(), mode, defining_byte)?
                .with_roc()
                .with_toc();
        self.pio.submit_v1(PioTransfer {
            command,
            tx: &[],
            rx: out,
        })
    }

    pub fn ccc_direct_write_addr<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        dynamic_addr: u8,
        ccc: u8,
        data: &[u8],
    ) -> Result<PioTransferResult> {
        let device = table
            .by_dynamic_addr(dynamic_addr)
            .ok_or(I3cHciError::InvalidArgs)?;
        self.ccc_direct_write(device.dat_index, ccc, data)
    }

    pub fn ccc_direct_read_addr<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        dynamic_addr: u8,
        ccc: u8,
        defining_byte: Option<u8>,
        out: &mut [u8],
    ) -> Result<PioTransferResult> {
        let device = table
            .by_dynamic_addr(dynamic_addr)
            .ok_or(I3cHciError::InvalidArgs)?;
        self.ccc_direct_read(device.dat_index, ccc, defining_byte, out)
    }

    pub fn ccc_direct_write_pid<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        pid: u64,
        ccc: u8,
        data: &[u8],
    ) -> Result<PioTransferResult> {
        let device = table.by_pid(pid).ok_or(I3cHciError::InvalidArgs)?;
        self.ccc_direct_write(device.dat_index, ccc, data)
    }

    pub fn ccc_direct_read_pid<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        pid: u64,
        ccc: u8,
        defining_byte: Option<u8>,
        out: &mut [u8],
    ) -> Result<PioTransferResult> {
        let device = table.by_pid(pid).ok_or(I3cHciError::InvalidArgs)?;
        self.ccc_direct_read(device.dat_index, ccc, defining_byte, out)
    }

    pub fn bus_reset(&mut self) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let command = HciCommandV1::internal(tid, 0x4, 0)?.with_toc();
        let mut rx = [];
        self.pio.submit_v1(PioTransfer {
            command,
            tx: &[],
            rx: &mut rx,
        })
    }

    fn write_dat_entry(&self, dat_index: u8, entry: DatEntryV1) -> Result<()> {
        if self.sections.dat.offset == 0
            || self.sections.dat.entry_size != DAT_V1_ENTRY_SIZE
            || u16::from(dat_index) >= self.sections.dat.entries
        {
            return Err(I3cHciError::InvalidArgs);
        }
        self.regs
            .write_table_word(
                self.sections.dat.offset,
                self.sections.dat.entry_size,
                dat_index,
                0,
                entry.word0,
            )
            .ok_or(I3cHciError::InvalidArgs)?;
        self.regs
            .write_table_word(
                self.sections.dat.offset,
                self.sections.dat.entry_size,
                dat_index,
                1,
                entry.word1,
            )
            .ok_or(I3cHciError::InvalidArgs)
    }

    fn next_tid(&mut self) -> u8 {
        let tid = self.next_tid & 0x0f;
        self.next_tid = self.next_tid.wrapping_add(1) & 0x0f;
        tid
    }
}
