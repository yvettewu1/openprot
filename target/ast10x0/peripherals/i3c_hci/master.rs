// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Minimal OpenProt-facing MIPI I3C HCI master API.

use super::api::{ControllerEvent, I3cBusManagement, I3cController, I3cDevice};
use super::ccc::{
    I3C_CCC_DISEC_DIRECT, I3C_CCC_ENEC_BROADCAST, I3C_CCC_ENEC_DIRECT, I3C_CCC_EVENT_HJ,
    I3C_CCC_EVENT_SIR, I3C_CCC_GETBCR, I3C_CCC_GETDCR, I3C_CCC_GETMXDS, I3C_CCC_GETPID,
    I3C_CCC_GETSTATUS, I3C_CCC_RSTDAA_BROADCAST, I3C_CCC_RSTDAA_DIRECT, I3C_CCC_SETDASA,
    I3C_CCC_SETNEWDA,
};
use super::cmd::{HciCommand, HciCommandKind, HciResponse};
use super::core::{CommandDescriptor, HciSections};
use super::core::{I3cHciCore, IoMode};

use super::cmd::IbiStatus;
use super::dat::{
    DAT_0_I2C_DEVICE, DAT_0_IBI_PAYLOAD, DAT_0_SIR_REJECT, DAT_V1_ENTRY_SIZE, DatAllocator,
    DatEntryV1,
};
use super::dct::{DctEntryV1, read_dct_entry_v1};
use super::dma::Ring;
use super::error::{I3cHciError, Result};
use super::pio::{STAT_ALL_ERRORS, STAT_IBI_STATUS_THLD};
use super::registers::I3cHciRegisters;
use super::transport::Transport;

/// A command and borrowed data buffers shared by PIO and DMA transports.
pub struct HciTransfer<'a> {
    pub command: HciCommand,
    pub tx: &'a [u8],
    pub rx: &'a mut [u8],
}

/// Completion response and received byte count for either transport.
pub struct HciTransferResult {
    pub response: HciResponse,
    pub rx_len: usize,
}

/// Check command/buffer consistency without accessing controller hardware.
pub fn validate_transfer(transfer: &HciTransfer<'_>) -> Result<()> {
    if transfer.command.read() {
        if !transfer.tx.is_empty() || transfer.rx.len() < transfer.command.data_len() {
            return Err(I3cHciError::InvalidArgs);
        }
    } else if transfer.command.kind() == HciCommandKind::Immediate {
        if !transfer.tx.is_empty() || !transfer.rx.is_empty() {
            return Err(I3cHciError::InvalidArgs);
        }
    } else if transfer.tx.len() != transfer.command.data_len() || !transfer.rx.is_empty() {
        return Err(I3cHciError::InvalidArgs);
    }
    Ok(())
}

pub const I3C_BROADCAST_ADDR: u8 = 0x7e;
pub const I3C_HOT_JOIN_ADDR: u8 = 0x02;

pub use super::dat::{HciDeviceTable, HciDeviceTableIter, HciI3cDevice};

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
    pub(super) regs: &'a I3cHciRegisters,
    pub(super) sections: HciSections,
    pub(super) transport: Transport<'a, Y>,
    next_tid: u8,
    pub(super) command_descriptor: CommandDescriptor,
    encoding: super::cmd::CommandEncoding,
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
            transport: Transport::new(regs, yield_us),
            next_tid: 0,
            command_descriptor: CommandDescriptor::V1,
            encoding: super::cmd::CommandEncoding::new(i3c_scl_hz, 400_000),
        }
    }

    #[must_use]
    pub fn with_command_descriptor(mut self, descriptor: CommandDescriptor) -> Self {
        self.command_descriptor = descriptor;
        self.encoding.v2 = descriptor == CommandDescriptor::V2;
        self
    }

    /// Attach an initialized controller ring for this bus and command format.
    pub fn with_dma(mut self, ring: super::dma::Ring) -> Result<Self> {
        self.transport = self.transport.attach_dma(
            ring,
            self.regs.bus(),
            super::dma::DmaMode::Controller,
            Some(self.command_descriptor),
        )?;
        Ok(self)
    }

    pub fn dma_ring_mut(&mut self) -> Option<&mut super::dma::Ring> {
        self.transport.dma_mut()
    }

    pub(super) fn submit_raw(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        self.transport.submit_raw(transfer)
    }

    fn submit(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        let result = self.submit_raw(transfer)?;
        if !result.response.success() {
            return Err(I3cHciError::Transfer(result.response.status));
        }
        Ok(result)
    }

    fn submit_write(&mut self, command: HciCommand, data: &[u8]) -> Result<HciTransferResult> {
        // Immediate command bytes already live in the descriptor.
        let tx = if command.kind() == HciCommandKind::Immediate {
            &[]
        } else {
            data
        };
        self.submit(HciTransfer {
            command,
            tx,
            rx: &mut [],
        })
    }

    fn submit_read(&mut self, command: HciCommand, out: &mut [u8]) -> Result<HciTransferResult> {
        self.submit(HciTransfer {
            command,
            tx: &[],
            rx: out,
        })
    }

    pub(super) fn recover_after_daa_nack(&mut self) -> Result<()> {
        self.transport.recover_after_daa_nack(self.regs)
    }

    pub fn command_descriptor(&self) -> CommandDescriptor {
        self.command_descriptor
    }

    pub fn with_poll_budget(mut self, poll_budget: u32) -> Self {
        self.transport = self.transport.with_poll_budget(poll_budget);
        self
    }

    /// Forward optional diagnostics to the selected backend without a logger dependency.
    #[must_use]
    pub fn with_trace(mut self, trace: fn(core::fmt::Arguments<'_>)) -> Self {
        self.transport = self.transport.with_trace(trace);
        self
    }

    #[must_use]
    pub fn with_i2c_scl_hz(mut self, i2c_scl_hz: u32) -> Self {
        self.encoding.set_i2c_rate(i2c_scl_hz);
        self
    }

    #[must_use]
    pub fn with_irq_events(mut self, bus: usize) -> Self {
        self.transport = self.transport.with_irq_events(bus);
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

    pub fn private_write(&mut self, dat_index: u8, data: &[u8]) -> Result<HciTransferResult> {
        let tid = self.next_tid();
        let command = self
            .i3c_private_write_regular_command(dat_index, tid, data.len())?
            .with_roc()
            .with_toc();
        self.submit_write(command, data)
    }

    pub fn private_read(&mut self, dat_index: u8, out: &mut [u8]) -> Result<HciTransferResult> {
        let tid = self.next_tid();
        let command = self
            .i3c_private_read_command(dat_index, tid, out.len())?
            .with_roc()
            .with_toc();
        self.submit_read(command, out)
    }

    pub fn i2c_write(&mut self, dat_index: u8, data: &[u8]) -> Result<HciTransferResult> {
        let entry = self.read_dat_entry(dat_index)?;
        if (entry.word0 & DAT_0_I2C_DEVICE) == 0 {
            return Err(I3cHciError::InvalidArgs);
        }

        let tid = self.next_tid();
        let command = self
            .i2c_write_regular_command(dat_index, tid, data.len())?
            .with_roc()
            .with_toc();
        self.submit_write(command, data)
    }

    pub fn i2c_read(&mut self, dat_index: u8, out: &mut [u8]) -> Result<HciTransferResult> {
        let entry = self.read_dat_entry(dat_index)?;
        if (entry.word0 & DAT_0_I2C_DEVICE) == 0 {
            return Err(I3cHciError::InvalidArgs);
        }

        let tid = self.next_tid();
        let command = self
            .i2c_read_command(dat_index, tid, out.len())?
            .with_roc()
            .with_toc();
        self.submit_read(command, out)
    }

    pub fn ccc_broadcast_write(&mut self, ccc: u8, data: &[u8]) -> Result<()> {
        let tid = self.next_tid();
        // Match Zephyr do_ccc: broadcast CCCs also request a completion
        // response. Returning after enqueue alone hides reset/CCC failures.
        let command = self
            .ccc_command(0, tid, false, ccc, data, None)?
            .with_roc()
            .with_toc();
        self.submit_write(command, data).map(|_| ())
    }

    pub fn rstdaa_broadcast(&mut self) -> Result<()> {
        self.ccc_broadcast_write(I3C_CCC_RSTDAA_BROADCAST, &[])
    }

    pub fn rstdaa_direct(&mut self, dat_index: u8) -> Result<HciTransferResult> {
        self.ccc_direct_write(dat_index, I3C_CCC_RSTDAA_DIRECT, &[])
    }

    pub fn setdasa(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<HciTransferResult> {
        self.ccc_direct_write(
            dat_index,
            I3C_CCC_SETDASA,
            &[dynamic_address_payload(dynamic_addr)?],
        )
    }

    pub fn setnewda(&mut self, dat_index: u8, dynamic_addr: u8) -> Result<HciTransferResult> {
        self.ccc_direct_write(
            dat_index,
            I3C_CCC_SETNEWDA,
            &[dynamic_address_payload(dynamic_addr)?],
        )
    }

    pub fn getpid(&mut self, dat_index: u8, out: &mut [u8; 6]) -> Result<HciTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETPID, None, out)
    }

    pub fn getbcr(&mut self, dat_index: u8, out: &mut [u8; 1]) -> Result<HciTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETBCR, None, out)
    }

    pub fn getdcr(&mut self, dat_index: u8, out: &mut [u8; 1]) -> Result<HciTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETDCR, None, out)
    }

    pub fn getstatus(&mut self, dat_index: u8, out: &mut [u8; 2]) -> Result<HciTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETSTATUS, None, out)
    }

    pub fn getmxds(&mut self, dat_index: u8, out: &mut [u8]) -> Result<HciTransferResult> {
        self.ccc_direct_read(dat_index, I3C_CCC_GETMXDS, None, out)
    }

    pub fn ccc_direct_write(
        &mut self,
        dat_index: u8,
        ccc: u8,
        data: &[u8],
    ) -> Result<HciTransferResult> {
        let tid = self.next_tid();
        let command = self
            .ccc_regular_command(dat_index, tid, false, ccc, data.len(), None)?
            .with_roc()
            .with_toc();
        self.submit_write(command, data)
    }

    pub fn ccc_direct_read(
        &mut self,
        dat_index: u8,
        ccc: u8,
        defining_byte: Option<u8>,
        out: &mut [u8],
    ) -> Result<HciTransferResult> {
        let tid = self.next_tid();
        let command = self
            .ccc_regular_command(dat_index, tid, true, ccc, out.len(), defining_byte)?
            .with_roc()
            .with_toc();
        self.submit_read(command, out)
    }

    /// Enable IBI with an explicit maximum notification payload length.
    /// The ASPEED hardware limit is shared across targets and only grows.
    /// A zero length enables notifications without payload bytes.
    /// ENEC uses the selected command transport (PIO by default, DMA with `with_dma`).
    pub fn enable_ibi_with_max_payload(
        &mut self,
        device: HciI3cDevice,
        max_len: u16,
    ) -> Result<HciTransferResult> {
        // Validate the DAT slot before changing the shared hardware limit.
        self.read_dat_entry(device.dat_index)?;
        if max_len != 0 {
            self.regs.aspeed_ibi_terminate(max_len);
        }
        self.enable_ibi(device, max_len != 0)
    }
    pub fn enable_ibi(&mut self, device: HciI3cDevice, payload: bool) -> Result<HciTransferResult> {
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
        if self.transport.is_dma() {
            return;
        }
        self.regs.write_pio_intr_status_enable(
            self.regs.pio_intr_status_enable() | STAT_IBI_STATUS_THLD | STAT_ALL_ERRORS,
        );
        self.rearm_ibi_irq_events();
    }

    pub fn rearm_ibi_irq_events(&self) {
        self.transport
            .enable_pio_irq_signals(STAT_IBI_STATUS_THLD | STAT_ALL_ERRORS);
    }

    pub fn accept_hotjoin<const N: usize>(
        &mut self,
        table: &mut HciDeviceTable<N>,
        dat_allocator: &mut DatAllocator,
    ) -> Result<usize> {
        self.entdaa_new_devices(table, dat_allocator)
    }

    /// Poll one notification through the selected transport. This method does
    /// not enable CPU interrupt signaling; scheduling belongs to the caller.
    pub fn poll_controller_event(&mut self, payload: &mut [u8]) -> Result<Option<ControllerEvent>> {
        let Some((status, payload_len)) = self.transport.poll_ibi(payload)? else {
            return Ok(None);
        };
        if status.error {
            return Err(I3cHciError::TransferStatus(status.raw));
        }
        Ok(Some(
            if status.target_addr == I3C_HOT_JOIN_ADDR && !status.read {
                ControllerEvent::HotJoin
            } else if status.read {
                ControllerEvent::TargetInterrupt {
                    dynamic_addr: status.target_addr,
                    payload_len,
                }
            } else {
                ControllerEvent::Unknown {
                    address: status.target_addr,
                    payload_len,
                }
            },
        ))
    }

    pub fn discard_partial_ibi(&mut self) -> Result<bool> {
        self.transport.discard_partial_ibi()
    }

    pub fn next_ibi_event<const N: usize>(
        &mut self,
        table: &HciDeviceTable<N>,
        payload: &mut [u8],
    ) -> Result<HciIbiEvent> {
        if self.transport.is_dma() {
            return Err(I3cHciError::Unsupported);
        }
        let (status, payload_len) = self.transport.pio_mut()?.next_ibi(payload)?;

        let event = if status.target_addr == I3C_HOT_JOIN_ADDR && !status.read {
            HciIbiEvent::HotJoin { status }
        } else if let Some(device) = table
            .by_dynamic_addr(status.target_addr)
            .filter(|_| status.read)
        {
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
        // The PIO transport rearms only when IRQ event delivery was selected.
        Ok(event)
    }

    pub fn bus_reset(&mut self) -> Result<HciTransferResult> {
        let tid = self.next_tid();
        let command = self.internal_command(tid, 0x4, 0)?.with_toc();
        let mut rx = [];
        self.submit(HciTransfer {
            command,
            tx: &[],
            rx: &mut rx,
        })
    }

    fn command_address(&self, dat_index: u8, i2c: bool, broadcast: bool) -> Result<u8> {
        if self.command_descriptor == CommandDescriptor::V1 {
            return Ok(dat_index);
        }
        if broadcast && dat_index == 0 {
            return Ok(I3C_BROADCAST_ADDR);
        }
        if i2c {
            self.i2c_addr_from_dat(dat_index)
        } else {
            self.i3c_addr_from_dat(dat_index)
        }
    }
    fn i3c_private_write_regular_command(
        &self,
        dat_index: u8,
        tid: u8,
        len: usize,
    ) -> Result<HciCommand> {
        self.encoding.i3c_private_write_regular_command(
            self.command_address(dat_index, false, false)?,
            tid,
            len,
        )
    }
    fn i3c_private_read_command(&self, dat_index: u8, tid: u8, len: usize) -> Result<HciCommand> {
        self.encoding.i3c_private_read_command(
            self.command_address(dat_index, false, false)?,
            tid,
            len,
        )
    }
    fn i2c_write_regular_command(&self, dat_index: u8, tid: u8, len: usize) -> Result<HciCommand> {
        self.encoding.i2c_write_regular_command(
            self.command_address(dat_index, true, false)?,
            tid,
            len,
        )
    }
    fn i2c_read_command(&self, dat_index: u8, tid: u8, len: usize) -> Result<HciCommand> {
        self.encoding
            .i2c_read_command(self.command_address(dat_index, true, false)?, tid, len)
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
        self.encoding.ccc_command(
            self.command_address(dat_index, false, true)?,
            tid,
            read,
            ccc,
            data,
            defining_byte,
        )
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
        self.encoding.ccc_regular_command(
            self.command_address(dat_index, false, false)?,
            tid,
            read,
            ccc,
            data_len,
            defining_byte,
        )
    }
    fn internal_command(&self, tid: u8, sub_cmd: u8, param: u32) -> Result<HciCommand> {
        self.encoding.internal_command(tid, sub_cmd, param)
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

    pub(super) fn read_dat_entry(&self, dat_index: u8) -> Result<DatEntryV1> {
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

    pub(super) fn write_dat_entry(&self, dat_index: u8, entry: DatEntryV1) -> Result<()> {
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

    pub(super) fn next_tid(&mut self) -> u8 {
        let tid = self.next_tid & 0x0f;
        self.next_tid = self.next_tid.wrapping_add(1) & 0x0f;
        tid
    }
}

#[must_use]
const fn valid_i3c_addr(address: u8) -> bool {
    address > 0x07 && address < I3C_BROADCAST_ADDR
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

impl From<HciI3cDevice> for I3cDevice {
    fn from(device: HciI3cDevice) -> Self {
        Self {
            pid: device.pid,
            dynamic_addr: device.dynamic_addr,
            bcr: device.bcr,
            dcr: device.dcr,
        }
    }
}

/// Owns a fresh controller's discovery table, allocator and selected transport.
/// Keep one session for the bus lifetime; do not independently reset/readdress
/// devices through the low-level API while a session owns their assignments.
#[must_use = "controller sessions require explicit shutdown"]
pub struct Controller<'a, Y: FnMut(u32), const N: usize> {
    master: I3cHciMaster<'a, Y>,
    devices: HciDeviceTable<N>,
    allocator: DatAllocator,
    topology: super::daa::TopologyState,
}

impl<'a, Y: FnMut(u32), const N: usize> Controller<'a, Y, N> {
    /// Attach using the firmware-selected command rate and timeout.
    pub fn from_configured_core<F: FnMut(u32)>(
        core: &'a mut I3cHciCore<F>,
        yield_us: Y,
    ) -> Result<Self> {
        let config = core.board_config().ok_or(I3cHciError::InvalidArgs)?;
        if !matches!(config.role, super::config::BusRole::Controller { .. })
            || core.init_mode() != Some(config.role.init_mode())
            || core.selected_io() != Some(config.io_mode)
        {
            return Err(I3cHciError::InvalidArgs);
        }
        Self::new(core, config.scl_hz, yield_us).map(|bus| bus.with_poll_budget(config.poll_budget))
    }

    /// Attach to an initialized, idle primary before discovery. The core's
    /// command capability and selected transport are applied automatically.
    /// scl_hz selects command timing; board setup must configure matching PHY.
    pub fn new<F: FnMut(u32)>(
        core: &'a mut I3cHciCore<F>,
        scl_hz: u32,
        yield_us: Y,
    ) -> Result<Self> {
        let format = core.command_descriptor().ok_or(I3cHciError::InvalidArgs)?;
        let mode = core.selected_io().ok_or(I3cHciError::InvalidArgs)?;
        if core.init_mode() != Some(super::types::InitMode::Primary)
            || N == 0
            || scl_hz == 0
            || core.sections().dat.entries == 0
            || core.regs().controller_device_addr() & super::constants::MASTER_DYNAMIC_ADDR_VALID
                == 0
        {
            return Err(I3cHciError::InvalidArgs);
        }
        let mut master = I3cHciMaster::new(core.regs(), core.sections(), scl_hz, yield_us)
            .with_command_descriptor(format);
        if mode == IoMode::Dma {
            master = master.with_dma(Ring::init(core).map_err(I3cHciError::from)?)?;
        }
        Self::from_master(master)
    }

    /// Wrap a freshly initialized, undiscovered master, taking ownership of its
    /// transport. Board lifecycle code must not create another session on it.
    pub fn from_master(master: I3cHciMaster<'a, Y>) -> Result<Self> {
        let entries = master.sections.dat.entries.min(128) as u8;
        if N == 0 || entries == 0 {
            let mut master = master;
            master.transport.shutdown()?;
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(Self {
            master,
            devices: HciDeviceTable::new(),
            allocator: DatAllocator::new(entries, true),
            topology: super::daa::TopologyState::Known,
        })
    }

    pub fn with_poll_budget(mut self, poll_budget: u32) -> Self {
        self.master = self.master.with_poll_budget(poll_budget);
        self
    }

    /// Connect the pre-registered IRQ event source. Board setup owns NVIC.
    pub fn with_irq_events(mut self, bus: usize) -> Self {
        self.master = self.master.with_irq_events(bus);
        self
    }

    pub fn devices(&self) -> impl Iterator<Item = I3cDevice> + '_ {
        self.devices
            .iter()
            .filter(|_| self.topology == super::daa::TopologyState::Known)
            .map(I3cDevice::from)
    }

    fn resolve(&self, device: I3cDevice) -> Result<HciI3cDevice> {
        self.topology.require_known()?;
        self.devices
            .by_dynamic_addr(device.dynamic_addr)
            .filter(|entry| I3cDevice::from(*entry) == device)
            .ok_or(I3cHciError::InvalidArgs)
    }

    /// Explicit shutdown, since stopping DMA can fail and cannot be hidden in Drop.
    pub fn shutdown(mut self) -> Result<()> {
        self.master.transport.shutdown()
    }
}

impl<Y: FnMut(u32), const N: usize> I3cController for Controller<'_, Y, N> {
    fn discover(&mut self) -> Result<usize> {
        self.topology.require_known()?;
        if self.devices.is_full() {
            return Err(I3cHciError::NoSpace);
        }
        self.topology = super::daa::TopologyState::RecoveryRequired;
        let count = self
            .master
            .entdaa_new_devices(&mut self.devices, &mut self.allocator)?;
        self.topology = super::daa::TopologyState::Known;
        Ok(count)
    }
    fn device_by_pid(&self, pid: u64) -> Option<I3cDevice> {
        self.topology.require_known().ok()?;
        self.devices.by_pid(pid).map(I3cDevice::from)
    }
    fn device_by_address(&self, address: u8) -> Option<I3cDevice> {
        self.topology.require_known().ok()?;
        self.devices.by_dynamic_addr(address).map(I3cDevice::from)
    }
    fn write(&mut self, device: I3cDevice, data: &[u8]) -> Result<()> {
        let device = self.resolve(device)?;
        self.master
            .private_write(device.dat_index, data)
            .map(|_| ())
    }
    fn read(&mut self, device: I3cDevice, data: &mut [u8]) -> Result<usize> {
        let device = self.resolve(device)?;
        self.master
            .private_read(device.dat_index, data)
            .and_then(checked_read_length)
    }
    fn ccc_write(&mut self, device: I3cDevice, ccc: u8, data: &[u8]) -> Result<()> {
        // Address-changing CCCs must go through I3cBusManagement so the
        // session updates its discovery table along with the hardware.
        if ccc < 0x80 || matches!(ccc, 0x86 | 0x87 | 0x88) {
            return Err(I3cHciError::Unsupported);
        }
        let device = self.resolve(device)?;
        self.master
            .ccc_direct_write(device.dat_index, ccc, data)
            .map(|_| ())
    }
    fn ccc_read(&mut self, device: I3cDevice, ccc: u8, data: &mut [u8]) -> Result<usize> {
        if ccc < 0x80 {
            return Err(I3cHciError::InvalidArgs);
        }
        let device = self.resolve(device)?;
        self.master
            .ccc_direct_read(device.dat_index, ccc, None, data)
            .and_then(checked_read_length)
    }
    fn enable_ibi(&mut self, device: I3cDevice, max_payload: u16) -> Result<()> {
        let device = self.resolve(device)?;
        self.master
            .enable_ibi_with_max_payload(device, max_payload)
            .map(|_| ())
    }
    fn disable_ibi(&mut self, device: I3cDevice) -> Result<()> {
        let device = self.resolve(device)?;
        self.master.disable_ibi(device)
    }
    fn discard_partial_ibi(&mut self) -> Result<bool> {
        self.master.discard_partial_ibi()
    }
    fn poll_event(&mut self, payload: &mut [u8]) -> Result<Option<ControllerEvent>> {
        self.topology.require_known()?;
        self.master.poll_controller_event(payload)
    }
}

impl<Y: FnMut(u32), const N: usize> I3cBusManagement for Controller<'_, Y, N> {
    fn broadcast_ccc(&mut self, ccc: u8, data: &[u8]) -> Result<()> {
        self.topology.require_known()?;
        // Do not permit raw broadcasts to silently invalidate discovery state.
        if ccc >= 0x80 || matches!(ccc, I3C_CCC_RSTDAA_BROADCAST | super::ccc::I3C_CCC_ENTDAA) {
            return Err(I3cHciError::Unsupported);
        }
        self.master.ccc_broadcast_write(ccc, data)
    }

    fn reset_addresses(&mut self, device: Option<I3cDevice>) -> Result<()> {
        if let Some(device) = device {
            if self.devices.len() != 1 {
                return Err(I3cHciError::Unsupported);
            }
            let device = self.resolve(device)?;
            self.topology = super::daa::TopologyState::RecoveryRequired;
            self.master.rstdaa_direct(device.dat_index)?;
        } else {
            self.topology = super::daa::TopologyState::RecoveryRequired;
            self.master.rstdaa_broadcast()?;
        }
        self.devices.clear();
        self.allocator = DatAllocator::new(self.master.sections.dat.entries.min(128) as u8, true);
        self.topology = super::daa::TopologyState::Known;
        Ok(())
    }

    fn set_address(&mut self, device: I3cDevice, address: u8) -> Result<I3cDevice> {
        let old = self.resolve(device)?;
        self.check_new_address(address)?;
        let entry = self.master.read_dat_entry(old.dat_index)?;
        // Discovery uses address-indexed DAT allocation. Reserve the new slot
        // before changing the bus address, then release the old slot only after
        // the hardware entry and cached identity agree. Otherwise discovery can
        // choose the freed old address but fail to allocate its still-used slot.
        let slot = self.allocator.allocate(Some(address))?;
        // A failed transfer may already have changed the target's address. Keep
        // both reservations on error; the caller must reset/re-enumerate.
        self.topology = super::daa::TopologyState::RecoveryRequired;
        self.master.setnewda(old.dat_index, address)?;
        self.master
            .write_dat_entry(slot, entry.with_dynamic_addr(address))?;
        self.master
            .write_dat_entry(old.dat_index, DatEntryV1::rejected())?;
        let moved = HciI3cDevice {
            dat_index: slot,
            dynamic_addr: address,
            ..old
        };
        // Replace by the old slot, including targets whose PID happens to be 0.
        // Every existing entry is copied exactly once, so capacity cannot grow.
        let mut updated = HciDeviceTable::new();
        for device in self.devices.iter() {
            updated.push(if device.dat_index == old.dat_index {
                moved
            } else {
                device
            })?;
        }
        self.devices = updated;
        self.allocator.free(old.dat_index)?;
        self.topology = super::daa::TopologyState::Known;
        Ok(moved.into())
    }

    fn assign_static_address(&mut self, static_addr: u8, identity: I3cDevice) -> Result<I3cDevice> {
        self.check_new_address(static_addr)?;
        self.check_new_address(identity.dynamic_addr)?;
        if self.devices.is_full()
            || identity.pid >= (1u64 << 48)
            || self.devices.by_pid(identity.pid).is_some()
        {
            return Err(I3cHciError::InvalidArgs);
        }
        let slot = self.allocator.allocate(Some(identity.dynamic_addr))?;
        self.topology = super::daa::TopologyState::RecoveryRequired;
        let result = (|| {
            self.master.attach_static_i3c_device(slot, static_addr)?;
            self.master.setdasa(slot, identity.dynamic_addr)?;
            self.master.attach_i3c_device(slot, identity.dynamic_addr)?;
            self.devices.push(HciI3cDevice {
                dat_index: slot,
                dynamic_addr: identity.dynamic_addr,
                pid: identity.pid,
                bcr: identity.bcr,
                dcr: identity.dcr,
            })?;
            self.topology = super::daa::TopologyState::Known;
            Ok(identity)
        })();
        // On an uncertain bus error keep the slot reserved; callers should stop
        // traffic and reset/re-enumerate, rather than reuse possibly live state.
        result
    }
}

impl<Y: FnMut(u32), const N: usize> Controller<'_, Y, N> {
    fn check_new_address(&self, address: u8) -> Result<()> {
        self.topology.require_known()?;
        let controller = self.master.regs.controller_device_addr();
        let controller_address = ((controller & super::constants::MASTER_DYNAMIC_ADDR_MASK)
            >> super::constants::MASTER_DYNAMIC_ADDR_SHIFT) as u8;
        if !valid_i3c_addr(address)
            || address == controller_address
            || self.devices.by_dynamic_addr(address).is_some()
        {
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(())
    }
}

/// Do not report a clipped PIO read as a complete application transfer.
/// DMA already rejects overflow; both transports expose the same contract.
fn checked_read_length(result: HciTransferResult) -> Result<usize> {
    if result.rx_len != result.response.data_len {
        return Err(I3cHciError::NoSpace);
    }
    Ok(result.rx_len)
}
