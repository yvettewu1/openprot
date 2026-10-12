// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Minimal OpenProt-facing MIPI I3C HCI master API.

use super::ccc::{
    I3C_CCC_DISEC_DIRECT, I3C_CCC_ENEC_BROADCAST, I3C_CCC_ENEC_DIRECT, I3C_CCC_EVENT_HJ,
    I3C_CCC_EVENT_SIR, I3C_CCC_GETBCR, I3C_CCC_GETDCR, I3C_CCC_GETMXDS, I3C_CCC_GETPID,
    I3C_CCC_GETSTATUS, I3C_CCC_RSTDAA_BROADCAST, I3C_CCC_RSTDAA_DIRECT, I3C_CCC_SETDASA,
    I3C_CCC_SETNEWDA,
};
use super::cmd::{
    i2c_mode, i2c_v2_rate, i3c_sdr_mode, i3c_sdr_v2_rate, HciCommand, HciCommandV1, HciCommandV2,
};
use super::core::{CommandDescriptor, HciSections};
use super::daa::{
    classify_entdaa_response, prepare_entdaa_step, prepare_next_entdaa_step, AddressSlots,
    DaaResponse,
};
use super::dat::{
    DatAllocator, DatEntryV1, DAT_0_I2C_DEVICE, DAT_0_IBI_PAYLOAD, DAT_0_SIR_REJECT,
    DAT_V1_ENTRY_SIZE,
};
use super::dct::{read_dct_entry_v1, DctEntryV1};
use super::error::{I3cHciError, Result};
use super::pio::{
    I3cHciPio, IbiStatus, PioTransfer, PioTransferResult, STAT_ALL_ERRORS, STAT_IBI_STATUS_THLD,
};
use super::registers::I3cHciRegisters;

pub const I3C_BROADCAST_ADDR: u8 = 0x7e;
pub const I3C_HOT_JOIN_ADDR: u8 = 0x02;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HciIbiEvent {
    TargetInterrupt {
        device: HciI3cDevice,
        status: IbiStatus,
        payload_len: usize,
    },
    HotJoin {
        status: IbiStatus,
    },
    Unknown {
        status: IbiStatus,
        payload_len: usize,
    },
}

pub struct I3cHciMaster<'a, Y: FnMut(u32)> {
    regs: &'a I3cHciRegisters,
    sections: HciSections,
    pio: I3cHciPio<'a, Y>,
    next_tid: u8,
    command_descriptor: CommandDescriptor,
    i3c_mode: u8,
    i2c_mode: u8,
    i3c_v2_rate: u8,
    i2c_v2_rate: u8,
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
            command_descriptor: CommandDescriptor::V1,
            i3c_mode: i3c_sdr_mode(i3c_scl_hz),
            i2c_mode: i2c_mode(400_000),
            i3c_v2_rate: i3c_sdr_v2_rate(i3c_scl_hz),
            i2c_v2_rate: i2c_v2_rate(400_000),
        }
    }

    #[must_use]
    pub fn with_command_descriptor(mut self, descriptor: CommandDescriptor) -> Self {
        self.command_descriptor = descriptor;
        self
    }

    pub fn command_descriptor(&self) -> CommandDescriptor {
        self.command_descriptor
    }

    pub fn with_timeout_us(mut self, timeout_us: u32) -> Self {
        self.pio = self.pio.with_timeout_us(timeout_us);
        self
    }

    #[must_use]
    pub fn with_i2c_scl_hz(mut self, i2c_scl_hz: u32) -> Self {
        self.i2c_mode = i2c_mode(i2c_scl_hz);
        self.i2c_v2_rate = i2c_v2_rate(i2c_scl_hz);
        self
    }

    #[must_use]
    pub fn with_irq_events(mut self, bus: usize) -> Self {
        self.pio = self.pio.with_irq_events(bus);
        self
    }

    pub fn attach_i3c_device(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<()> {
        self.write_dat_entry(
            dat_index,
            DatEntryV1::rejected().with_dynamic_addr(dynamic_addr),
        )
    }

    pub fn attach_static_i3c_device(&mut self, dat_index: u8, static_addr: u8) -> Result<()> {
        if !valid_i3c_addr(static_addr) {
            return Err(I3cHciError::InvalidArgs);
        }
        self.write_dat_entry(
            dat_index,
            DatEntryV1::rejected().with_static_addr(static_addr),
        )
    }

    pub fn attach_i2c_device(&mut self, dat_index: u8, static_addr: u8) -> Result<()> {
        if !valid_i3c_addr(static_addr) {
            return Err(I3cHciError::InvalidArgs);
        }
        self.write_dat_entry(
            dat_index,
            DatEntryV1::rejected().with_i2c_device(static_addr),
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
        self.write_dat_entry(step.dat_index, step.dat_entry)?;
        self.regs.reset_dct_index();
        let mut rx = [];
        let result = self.pio.submit_raw(PioTransfer {
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
                Ok(None)
            }
        }
    }

    fn entdaa_once_v2(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<Option<HciI3cDevice>> {
        if dat_index > 0x7f || !valid_i3c_addr(dynamic_addr) {
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
            self.write_dat_entry(step.dat_index, step.dat_entry)?;
            self.regs.reset_dct_index();
            let mut rx = [];
            let result = match self.pio.submit_raw(PioTransfer {
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
                }
                Ok(DaaResponse::NoMoreDevices) => {
                    self.write_dat_entry(step.dat_index, DatEntryV1::rejected())?;
                    dat_allocator.free(step.dat_index)?;
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
        let result = self.pio.submit_pair_raw(read, assign, rx)?;

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

    pub fn private_write(&mut self, dat_index: u8, data: &[u8]) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let command = self
            .i3c_private_write_regular_command(dat_index, tid, data.len())?
            .with_roc()
            .with_toc();
        let mut rx = [];
        self.pio.submit(PioTransfer {
            command,
            tx: data,
            rx: &mut rx,
        })
    }

    pub fn private_read(&mut self, dat_index: u8, out: &mut [u8]) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let command = self
            .i3c_private_read_command(dat_index, tid, out.len())?
            .with_roc()
            .with_toc();
        self.pio.submit(PioTransfer {
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

    pub fn i2c_write(&mut self, dat_index: u8, data: &[u8]) -> Result<PioTransferResult> {
        let entry = self.read_dat_entry(dat_index)?;
        if (entry.word0 & DAT_0_I2C_DEVICE) == 0 {
            return Err(I3cHciError::InvalidArgs);
        }

        let tid = self.next_tid();
        let command = self
            .i2c_write_regular_command(dat_index, tid, data.len())?
            .with_roc()
            .with_toc();
        let mut rx = [];
        self.pio.submit(PioTransfer {
            command,
            tx: data,
            rx: &mut rx,
        })
    }

    pub fn i2c_read(&mut self, dat_index: u8, out: &mut [u8]) -> Result<PioTransferResult> {
        let entry = self.read_dat_entry(dat_index)?;
        if (entry.word0 & DAT_0_I2C_DEVICE) == 0 {
            return Err(I3cHciError::InvalidArgs);
        }

        let tid = self.next_tid();
        let command = self
            .i2c_read_command(dat_index, tid, out.len())?
            .with_roc()
            .with_toc();
        self.pio.submit(PioTransfer {
            command,
            tx: &[],
            rx: out,
        })
    }

    pub fn ccc_broadcast_write(&mut self, ccc: u8, data: &[u8]) -> Result<()> {
        let tid = self.next_tid();
        let command = self.ccc_command(0, tid, false, ccc, data, None)?.with_toc();
        let mut rx = [];
        if command.expects_response() {
            self.pio
                .submit(PioTransfer {
                    command,
                    tx: data,
                    rx: &mut rx,
                })
                .map(|_| ())
        } else {
            let tx = if data.len() <= 4 { &[] } else { data };
            self.pio.submit_no_response(PioTransfer {
                command,
                tx,
                rx: &mut rx,
            })
        }
    }

    pub fn rstdaa_broadcast(&mut self) -> Result<()> {
        self.ccc_broadcast_write(I3C_CCC_RSTDAA_BROADCAST, &[])
    }

    pub fn rstdaa_direct(&mut self, dat_index: u8) -> Result<PioTransferResult> {
        self.ccc_direct_write(dat_index, I3C_CCC_RSTDAA_DIRECT, &[])
    }

    pub fn setdasa(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<PioTransferResult> {
        self.ccc_direct_write(
            dat_index,
            I3C_CCC_SETDASA,
            &[dynamic_address_payload(dynamic_addr)?],
        )
    }

    pub fn setnewda(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<PioTransferResult> {
        self.ccc_direct_write(
            dat_index,
            I3C_CCC_SETNEWDA,
            &[dynamic_address_payload(dynamic_addr)?],
        )
    }

    pub fn getpid(&mut self, dat_index: u8, out: &mut [u8; 6]) -> Result<PioTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETPID, None, out)
    }

    pub fn getbcr(&mut self, dat_index: u8, out: &mut [u8; 1]) -> Result<PioTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETBCR, None, out)
    }

    pub fn getdcr(&mut self, dat_index: u8, out: &mut [u8; 1]) -> Result<PioTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETDCR, None, out)
    }

    pub fn getstatus(&mut self, dat_index: u8, out: &mut [u8; 2]) -> Result<PioTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETSTATUS, None, out)
    }

    pub fn getmxds(&mut self, dat_index: u8, out: &mut [u8]) -> Result<PioTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETMXDS, None, out)
    }

    pub fn ccc_direct_write(
        &mut self,
        dat_index: u8,
        ccc: u8,
        data: &[u8],
    ) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let command = self
            .ccc_regular_command(dat_index, tid, false, ccc, data.len(), None)?
            .with_roc()
            .with_toc();
        let mut rx = [];
        self.pio.submit(PioTransfer {
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
        let command = self
            .ccc_regular_command(dat_index, tid, true, ccc, out.len(), defining_byte)?
            .with_roc()
            .with_toc();
        self.pio.submit(PioTransfer {
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

    pub fn enable_ibi(&mut self, device: HciI3cDevice, payload: bool) -> Result<PioTransferResult> {
        let mut entry = self.read_dat_entry(device.dat_index)?;
        entry = entry.clear_flags(DAT_0_SIR_REJECT, 0);
        if payload {
            entry = entry.set_flags(DAT_0_IBI_PAYLOAD, 0);
        } else {
            entry = entry.clear_flags(DAT_0_IBI_PAYLOAD, 0);
        }
        self.write_dat_entry(device.dat_index, entry)?;
        self.ccc_direct_write(device.dat_index, I3C_CCC_ENEC_DIRECT, &[I3C_CCC_EVENT_SIR])
    }

    pub fn disable_ibi(&mut self, device: HciI3cDevice) -> Result<()> {
        let mut entry = self.read_dat_entry(device.dat_index)?;
        entry = entry.set_flags(DAT_0_SIR_REJECT, 0);
        self.write_dat_entry(device.dat_index, entry)?;

        /*
         * Match Zephyr/Linux behavior: after the DAT entry rejects SIR, DISEC
         * failures do not leave the controller accepting unexpected IBIs.
         */
        let _ = self.ccc_direct_write(device.dat_index, I3C_CCC_DISEC_DIRECT, &[I3C_CCC_EVENT_SIR]);
        Ok(())
    }

    pub fn enable_ibi_addr<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        dynamic_addr: u8,
        payload: bool,
    ) -> Result<PioTransferResult> {
        let device = table
            .by_dynamic_addr(dynamic_addr)
            .ok_or(I3cHciError::InvalidArgs)?;
        self.enable_ibi(device, payload)
    }

    pub fn disable_ibi_addr<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        dynamic_addr: u8,
    ) -> Result<()> {
        let device = table
            .by_dynamic_addr(dynamic_addr)
            .ok_or(I3cHciError::InvalidArgs)?;
        self.disable_ibi(device)
    }

    pub fn enable_hotjoin(&mut self) -> Result<()> {
        self.regs
            .clear_hc_control(super::constants::HC_CONTROL_HOT_JOIN_CTRL);
        if let Err(error) = self.ccc_broadcast_write(I3C_CCC_ENEC_BROADCAST, &[I3C_CCC_EVENT_HJ]) {
            self.regs
                .set_hc_control(super::constants::HC_CONTROL_HOT_JOIN_CTRL);
            return Err(error);
        }
        Ok(())
    }

    pub fn disable_hotjoin(&mut self) {
        self.regs
            .set_hc_control(super::constants::HC_CONTROL_HOT_JOIN_CTRL);
    }

    pub fn enable_ibi_irq_events(&self) {
        self.regs
            .write_pio_intr_status_enable(STAT_IBI_STATUS_THLD | STAT_ALL_ERRORS);
        self.rearm_ibi_irq_events();
    }

    pub fn rearm_ibi_irq_events(&self) {
        self.regs.write_pio_intr_signal_enable(STAT_IBI_STATUS_THLD);
    }

    pub fn accept_hotjoin<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
    ) -> Result<usize> {
        self.entdaa_new_devices(table, dat_allocator)
    }

    pub fn next_ibi_event<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        payload: &mut [u8],
    ) -> Result<HciIbiEvent> {
        let status = self.pio.next_ibi_status(payload)?;
        let payload_len = status.data_len;

        let event = if status.target_addr == I3C_HOT_JOIN_ADDR {
            HciIbiEvent::HotJoin { status }
        } else if let Some(device) = table.by_dynamic_addr(status.target_addr) {
            HciIbiEvent::TargetInterrupt {
                device,
                status,
                payload_len,
            }
        } else {
            HciIbiEvent::Unknown {
                status,
                payload_len,
            }
        };

        self.rearm_ibi_irq_events();
        Ok(event)
    }

    pub fn bus_reset(&mut self) -> Result<PioTransferResult> {
        let tid = self.next_tid();
        let command = self.internal_command(tid, 0x4, 0)?.with_toc();
        let mut rx = [];
        self.pio.submit(PioTransfer {
            command,
            tx: &[],
            rx: &mut rx,
        })
    }

    fn i3c_private_write_regular_command(
        &self,
        dat_index: u8,
        tid: u8,
        data_len: usize,
    ) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => Ok(HciCommand::V1(HciCommandV1::i3c_private_write_regular(
                dat_index,
                tid,
                data_len,
                self.i3c_mode,
            )?)),
            CommandDescriptor::V2 => Ok(HciCommand::V2(HciCommandV2::i3c_private_write_regular(
                self.i3c_addr_from_dat(dat_index)?,
                tid,
                data_len,
                self.i3c_v2_rate,
            )?)),
        }
    }

    fn i3c_private_read_command(&self, dat_index: u8, tid: u8, len: usize) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => Ok(HciCommand::V1(HciCommandV1::i3c_private_read(
                dat_index,
                tid,
                len,
                self.i3c_mode,
            )?)),
            CommandDescriptor::V2 => Ok(HciCommand::V2(HciCommandV2::i3c_private_read(
                self.i3c_addr_from_dat(dat_index)?,
                tid,
                len,
                self.i3c_v2_rate,
            )?)),
        }
    }

    fn i2c_write_regular_command(
        &self,
        dat_index: u8,
        tid: u8,
        data_len: usize,
    ) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => Ok(HciCommand::V1(HciCommandV1::i2c_write_regular(
                dat_index,
                tid,
                data_len,
                self.i2c_mode,
            )?)),
            CommandDescriptor::V2 => Ok(HciCommand::V2(HciCommandV2::i2c_write_regular(
                self.i2c_addr_from_dat(dat_index)?,
                tid,
                data_len,
                self.i2c_v2_rate,
            )?)),
        }
    }

    fn i2c_read_command(&self, dat_index: u8, tid: u8, len: usize) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => Ok(HciCommand::V1(HciCommandV1::i2c_read(
                dat_index,
                tid,
                len,
                self.i2c_mode,
            )?)),
            CommandDescriptor::V2 => Ok(HciCommand::V2(HciCommandV2::i2c_read(
                self.i2c_addr_from_dat(dat_index)?,
                tid,
                len,
                self.i2c_v2_rate,
            )?)),
        }
    }

    fn ccc_command(
        &self,
        dat_index: u8,
        tid: u8,
        read: bool,
        ccc: u8,
        data: &[u8],
        defining_byte: Option<u8>,
    ) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => Ok(HciCommand::V1(HciCommandV1::ccc(
                dat_index,
                tid,
                read,
                ccc,
                data,
                self.i3c_mode,
                defining_byte,
            )?)),
            CommandDescriptor::V2 => Ok(HciCommand::V2(HciCommandV2::ccc(
                if dat_index == 0 {
                    I3C_BROADCAST_ADDR
                } else {
                    self.i3c_addr_from_dat(dat_index)?
                },
                tid,
                read,
                ccc,
                data,
                self.i3c_v2_rate,
                defining_byte,
            )?)),
        }
    }

    fn ccc_regular_command(
        &self,
        dat_index: u8,
        tid: u8,
        read: bool,
        ccc: u8,
        data_len: usize,
        defining_byte: Option<u8>,
    ) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => Ok(HciCommand::V1(HciCommandV1::ccc_regular(
                dat_index,
                tid,
                read,
                ccc,
                data_len,
                self.i3c_mode,
                defining_byte,
            )?)),
            CommandDescriptor::V2 => Ok(HciCommand::V2(HciCommandV2::ccc_regular(
                self.i3c_addr_from_dat(dat_index)?,
                tid,
                read,
                ccc,
                data_len,
                self.i3c_v2_rate,
                defining_byte,
            )?)),
        }
    }

    fn internal_command(&self, tid: u8, sub_cmd: u8, param: u32) -> Result<HciCommand> {
        match self.command_descriptor {
            CommandDescriptor::V1 => {
                Ok(HciCommand::V1(HciCommandV1::internal(tid, sub_cmd, param)?))
            }
            CommandDescriptor::V2 => {
                Ok(HciCommand::V2(HciCommandV2::internal(tid, sub_cmd, param)?))
            }
        }
    }

    fn i3c_addr_from_dat(&self, dat_index: u8) -> Result<u8> {
        let entry = self.read_dat_entry(dat_index)?;
        let dynamic_addr = entry.dynamic_addr();
        if valid_i3c_addr(dynamic_addr) {
            return Ok(dynamic_addr);
        }

        let static_addr = entry.static_addr();
        if valid_i3c_addr(static_addr) {
            return Ok(static_addr);
        }

        Err(I3cHciError::InvalidArgs)
    }

    fn i2c_addr_from_dat(&self, dat_index: u8) -> Result<u8> {
        let entry = self.read_dat_entry(dat_index)?;
        if (entry.word0 & DAT_0_I2C_DEVICE) == 0 {
            return Err(I3cHciError::InvalidArgs);
        }

        let addr = entry.static_addr();
        if !valid_i3c_addr(addr) {
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(addr)
    }

    fn read_dat_entry(&self, dat_index: u8) -> Result<DatEntryV1> {
        if self.sections.dat.offset == 0
            || self.sections.dat.entry_size != DAT_V1_ENTRY_SIZE
            || u16::from(dat_index) >= self.sections.dat.entries
        {
            return Err(I3cHciError::InvalidArgs);
        }

        let word0 = self
            .regs
            .table_word(
                self.sections.dat.offset,
                self.sections.dat.entry_size,
                dat_index,
                0,
            )
            .ok_or(I3cHciError::InvalidArgs)?;
        let word1 = self
            .regs
            .table_word(
                self.sections.dat.offset,
                self.sections.dat.entry_size,
                dat_index,
                1,
            )
            .ok_or(I3cHciError::InvalidArgs)?;

        Ok(DatEntryV1 { word0, word1 })
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

#[must_use]
const fn valid_i3c_addr(address: u8) -> bool {
    address > 0x07 && address < I3C_BROADCAST_ADDR
}

fn decode_v2_daa_identity(
    dat_index: u8,
    dynamic_addr: u8,
    identity: &[u8; 8],
) -> Result<HciI3cDevice> {
    if dat_index > 0x7f || !valid_i3c_addr(dynamic_addr) {
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

#[must_use]
pub fn dynamic_address_payload(address: u8) -> Result<u8> {
    if !valid_i3c_addr(address) {
        return Err(I3cHciError::InvalidArgs);
    }

    let parity = if super::dat::odd_parity7(address) {
        0
    } else {
        1
    };
    Ok((address << 1) | parity)
}
