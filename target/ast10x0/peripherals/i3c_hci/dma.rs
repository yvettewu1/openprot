// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI DMA ring support including IBI reception.
use super::HciResponse;
use super::cmd::HciCommand;
use super::constants::{HC_CONTROL_BUS_ENABLE, HC_CONTROL_PIO_MODE, HC_CONTROL_RESUME};
use super::core::{CommandDescriptor, HciSections, I3cHciCore};
use super::error::{DmaError, I3cHciError, Result};
use super::master::{HciTransfer, HciTransferResult, validate_transfer};
use super::pio::TargetResponse;
use super::registers::I3cHciRegisters;
use core::arch::asm;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

#[path = "dma_memory.rs"]
mod memory;
use memory::*;

pub const RHS_CONTROL_MAX_HEADER_COUNT_SHIFT: u32 = 0;
pub const RHS_CONTROL_MAX_HEADER_COUNT_MASK: u32 = 0x0f << RHS_CONTROL_MAX_HEADER_COUNT_SHIFT;
pub const RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT: u32 = 4;
pub const RHS_CONTROL_MAX_HEADER_COUNT_CAP_MASK: u32 =
    0x0f << RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT;
pub const RHS_CONTROL_HEADER_SIZE_SHIFT: u32 = 16;
pub const RHS_CONTROL_HEADER_SIZE_MASK: u32 = 0xff << RHS_CONTROL_HEADER_SIZE_SHIFT;
pub const RHS_CONTROL_PREAMBLE_SIZE_SHIFT: u32 = 24;
pub const RHS_CONTROL_PREAMBLE_SIZE_MASK: u32 = 0xff << RHS_CONTROL_PREAMBLE_SIZE_SHIFT;

const IBI_STATUS_RING_ENTRIES: usize = 32;
const IBI_CHUNK_SIZE: usize = 4;
const IBI_CHUNK_POOL_SIZE: usize = 128;

const INTR_IBI_READY: u32 = 1 << 12;
const INTR_IBI_RING_FULL: u32 = 1 << 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RhsInfo {
    pub preamble_size: u8,
    pub header_size: u8,
    pub max_header_count_cap: u8,
    pub driver_header_count: u8,
    pub ring_headers_offset: u16,
}

#[must_use]
pub const fn parse_rhs_control(control: u32, ring_headers_offset: u16) -> RhsInfo {
    RhsInfo {
        preamble_size: ((control & RHS_CONTROL_PREAMBLE_SIZE_MASK)
            >> RHS_CONTROL_PREAMBLE_SIZE_SHIFT) as u8,
        header_size: ((control & RHS_CONTROL_HEADER_SIZE_MASK) >> RHS_CONTROL_HEADER_SIZE_SHIFT)
            as u8,
        max_header_count_cap: ((control & RHS_CONTROL_MAX_HEADER_COUNT_CAP_MASK)
            >> RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT) as u8,
        driver_header_count: ((control & RHS_CONTROL_MAX_HEADER_COUNT_MASK)
            >> RHS_CONTROL_MAX_HEADER_COUNT_SHIFT) as u8,
        ring_headers_offset,
    }
}

pub fn rhs_info(regs: &I3cHciRegisters, sections: HciSections) -> Result<RhsInfo> {
    if sections.ring_headers_offset == 0 {
        return Err(I3cHciError::Unsupported);
    }

    let info = parse_rhs_control(regs.rhs_control(), sections.ring_headers_offset);

    if info.max_header_count_cap == 0 || info.header_size == 0 {
        return Err(I3cHciError::Unsupported);
    }

    Ok(info)
}

pub fn init_rhs(regs: &I3cHciRegisters, sections: HciSections, ring_count: u8) -> Result<RhsInfo> {
    let info = rhs_info(regs, sections)?;

    if ring_count == 0 || ring_count > info.max_header_count_cap || ring_count > 8 {
        return Err(I3cHciError::InvalidArgs);
    }

    let control = (regs.rhs_control() & !RHS_CONTROL_MAX_HEADER_COUNT_MASK)
        | (u32::from(ring_count) << RHS_CONTROL_MAX_HEADER_COUNT_SHIFT);

    regs.write_rhs_control(control);

    Ok(parse_rhs_control(
        regs.rhs_control(),
        sections.ring_headers_offset,
    ))
}

type DmaResult<T> = core::result::Result<T, DmaError>;

const DEFAULT_POLL_BUDGET: u32 = 1_000_000; // Iterations, not microseconds.
const ENTRIES: usize = 4;
// Controller v1: two command words plus three data-buffer words.
// A 16-byte hardware stride indicates target/secondary command format.
const MIN_XFER_STRUCT_SIZE: usize = 20;
const MIN_TARGET_XFER_STRUCT_SIZE: usize = 16;

const ENABLE_RUN: u32 = 3;
const ABORT: u32 = 4;
const RUNNING: u32 = 2;
const DMA_ERRORS: u32 = (1 << 9) | (1 << 5);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaMode {
    Controller,
    Target,
}

impl DmaMode {
    fn from_init_mode(mode: Option<super::types::InitMode>) -> DmaResult<Self> {
        use super::types::InitMode;
        match mode {
            Some(InitMode::Primary) => Ok(Self::Controller),
            Some(InitMode::Secondary | InitMode::Target) => Ok(Self::Target),
            None => Err(DmaError::Fault("DMA requires an initialized logical role")),
        }
    }
}

struct PendingDaa {
    first: HciCommand,
    second: HciCommand,
    start: usize,
    assign: usize,
    end: usize,
    buffer: *mut u8,
}

struct DaaCompletion {
    intr: u32,
    first_raw: u32,
    second_raw: Option<u32>,
}

#[derive(Clone, Copy)]
struct PendingCommand {
    command: HciCommand,
    tid: u8,
    slot: usize,
    next: usize,
}

struct CommandRingState {
    next: usize,
    target_resp_deq: usize,
    target_queued: [Option<u8>; ENTRIES],
    payloads: [Option<PayloadAllocation>; ENTRIES],
    command_faulted: core::cell::Cell<bool>,
}
struct IbiRingState {
    ibi_deq: usize,
    ibi_chunk_ptr: usize,
}

#[must_use = "DMA rings require explicit stop before releasing hardware ownership"]
pub struct Ring {
    commands: CommandRingState,
    ibi: IbiRingState,
    memory: DmaMemory,
    active: core::cell::Cell<bool>,
    poll_budget: u32,
    irq_events: Option<&'static super::irq::IsrEvents>,
    pending_irq: core::cell::Cell<u32>,
    regs: I3cHciRegisters,
    mode: DmaMode,
    descriptors: *mut DescriptorMemory,
    ibi_data: *mut u8,
    payload: *mut u8,
    payload_capacity: usize,
    command_format: CommandDescriptor,
    command_stride: usize,
    response_stride: usize,
    ibi_stride: usize,
    trace: Option<fn(core::fmt::Arguments<'_>)>,
}

#[derive(Debug, Clone, Copy)]
pub struct DmaIbiEvent {
    pub status: u32,
    pub target_addr: u8,
    pub payload_len: usize,
}

/// Raw target response, including the target-specific response flags.
#[derive(Debug, Clone, Copy)]
pub struct DmaTargetResponse {
    pub raw: u32,
    pub tid: u8,
    pub status: u8,
    pub data_len: usize,
}

/// Target notification received through the DMA IBI status/data ring.
#[derive(Debug, Clone, Copy)]
pub struct DmaTargetRx {
    /// Bytes copied into the caller's RX buffer; zero for target-transmit status.
    pub len: usize,
    pub ccc: bool,
    pub ccc_hdr: u8,
    pub raw_status: u32,
}

impl Ring {
    pub fn with_trace(mut self, trace: fn(core::fmt::Arguments<'_>)) -> Self {
        self.trace = Some(trace);
        self
    }
    pub(super) fn diagnostic_context(
        &self,
    ) -> Option<(&I3cHciRegisters, u32, fn(core::fmt::Arguments<'_>))> {
        self.trace
            .map(|trace| (&self.regs, self.event_status(), trace))
    }

    pub fn init(core: &I3cHciCore<impl FnMut(u32)>) -> DmaResult<Self> {
        Self::init_internal(core, None)
    }

    pub fn init_with_trace(
        core: &I3cHciCore<impl FnMut(u32)>,
        trace: fn(core::fmt::Arguments<'_>),
    ) -> DmaResult<Self> {
        Self::init_internal(core, Some(trace))
    }

    fn init_internal<F>(
        core: &I3cHciCore<F>,
        trace: Option<fn(core::fmt::Arguments<'_>)>,
    ) -> DmaResult<Self>
    where
        F: FnMut(u32),
    {
        let regs = core.regs();
        let mode = DmaMode::from_init_mode(core.init_mode())?;
        // Claim before any MMIO changes, so a second init cannot disrupt a live ring.
        let memory = DmaMemory::claim(regs.bus() as usize)?;
        let descriptors = memory.descriptors;
        let ibi_data = memory.ibi_data;
        let payload = memory.payload;
        let payload_capacity = memory.payload_capacity;
        let sections = core.sections();
        let rhs = sections.ring_headers_offset as usize;
        if rhs != 0x800 {
            return Err(DmaError::Fault(
                "RHS section does not match AST1080 PAC layout",
            ));
        }
        if regs.dma_ring_status() & 3 != 0 {
            return Err(DmaError::Fault("ring already active"));
        }
        // No transfers or IBIs may be pending at this PIO-to-DMA transition.
        regs.clear_hc_control(HC_CONTROL_BUS_ENABLE);
        init_rhs(regs, sections, 1).map_err(|_| DmaError::Fault("RHS setup failed"))?;
        let offset = regs.rhs_ring0_offset() as usize;
        if offset != 0x830 {
            return Err(DmaError::Fault(
                "ring-0 offset does not match AST1080 PAC layout",
            ));
        }
        let mut ring = Self {
            memory,
            active: core::cell::Cell::new(false),
            irq_events: None,
            pending_irq: core::cell::Cell::new(0),
            // SAFETY: this facade is used synchronously with the core/master
            // under their existing unique, serialized controller ownership.
            regs: unsafe { I3cHciRegisters::new(regs.bus()) }
                .ok_or(DmaError::Fault("invalid DMA bus"))?,
            mode,
            descriptors,
            ibi_data,
            payload,
            payload_capacity,
            command_format: core
                .command_descriptor()
                .ok_or(DmaError::Fault("command format not detected"))?,
            command_stride: 0,
            response_stride: 0,
            ibi_stride: 0,
            commands: CommandRingState {
                next: 0,
                target_resp_deq: 0,
                target_queued: [None; ENTRIES],
                payloads: [None; ENTRIES],
                command_faulted: core::cell::Cell::new(false),
            },
            ibi: IbiRingState {
                ibi_deq: 0,
                ibi_chunk_ptr: 0,
            },
            trace,
            poll_budget: core
                .board_config()
                .map_or(DEFAULT_POLL_BUDGET, |config| config.poll_budget),
        };
        let (command_stride, response_stride) = ring.regs.dma_descriptor_sizes();
        ring.command_stride = command_stride as usize;
        ring.response_stride = response_stride as usize;
        let min_stride = match mode {
            DmaMode::Controller => match ring.command_format {
                CommandDescriptor::V1 => MIN_XFER_STRUCT_SIZE,
                CommandDescriptor::V2 => 28, // Four command words + three buffer words.
            },
            DmaMode::Target => MIN_TARGET_XFER_STRUCT_SIZE,
        };
        ring.ibi_stride = ring.regs.dma_ibi_status_stride() as usize;
        validate_descriptor_stride(
            ring.command_stride,
            min_stride,
            ENTRIES,
            COMMAND_STORAGE_BYTES,
        )?;
        validate_descriptor_stride(ring.response_stride, 4, ENTRIES, RESPONSE_STORAGE_BYTES)?;
        validate_descriptor_stride(
            ring.ibi_stride,
            4,
            IBI_STATUS_RING_ENTRIES,
            IBI_STATUS_STORAGE_BYTES,
        )?;
        // Validate every complete allocation before clearing memory or publishing
        // bases; an enlarged user pool must remain inside DMA-addressable SRAM.
        dma_address(
            ring.descriptors.cast(),
            core::mem::size_of::<DescriptorMemory>(),
        )?;
        dma_address(ring.ibi_data, core::mem::size_of::<IbiDataMemory>())?;
        dma_address(ring.payload, ring.payload_capacity)?;
        // NOLOAD SRAM is not zeroed by the shared startup. Explicitly initialize
        // it before the DMA engine owns any descriptor or buffer.
        unsafe {
            core::ptr::write_bytes(ring.descriptors, 0, 1);
            core::ptr::write_bytes(ring.ibi_data, 0, IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE);
        }
        // Poison all response storage, including padding, so a misplaced DMA
        // write is distinguishable from untouched SRAM in the failure dump.
        let responses = unsafe { addr_of_mut!((*ring.descriptors).responses).cast::<u32>() };
        for i in 0..RESPONSE_STORAGE_BYTES / 4 {
            unsafe {
                write_volatile(responses.add(i), 0xffff_ffff);
            }
        }
        let cmd_addr = dma_address(
            unsafe { addr_of_mut!((*ring.descriptors).commands).cast() },
            ENTRIES * ring.command_stride,
        )?;
        let resp_addr = dma_address(
            unsafe { addr_of_mut!((*ring.descriptors).responses).cast() },
            ENTRIES * ring.response_stride,
        )?;
        ring.regs.write_dma_ring_cmd_base_lo(cmd_addr);
        ring.regs.write_dma_ring_cmd_base_hi(0);
        ring.regs.write_dma_ring_resp_base_lo(resp_addr);
        ring.regs.write_dma_ring_resp_base_hi(0);
        ring.regs.write_dma_ring_cr_setup(ENTRIES as u32);
        // Strides were all validated before programming any DMA base.
        let status_addr = dma_address(
            unsafe { addr_of_mut!((*ring.descriptors).ibi_status).cast() },
            IBI_STATUS_RING_ENTRIES * ring.ibi_stride,
        )?;
        let data_addr = dma_address(ring.ibi_data, IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE)?;
        ring.regs.write_dma_ring_ibi_status_base_lo(status_addr);
        ring.regs.write_dma_ring_ibi_status_base_hi(0);
        ring.regs.write_dma_ring_ibi_data_base_lo(data_addr);
        ring.regs.write_dma_ring_ibi_data_base_hi(0);
        // ASPEED uses 4-byte chunks: log2(4) - 2 = 0.
        ring.regs.write_dma_ring_ibi_setup(
            ((IBI_STATUS_RING_ENTRIES as u32) << 16) | IBI_CHUNK_POOL_SIZE as u32,
        );

        if let Some(trace) = ring.trace {
            trace(format_args!(
                "IBI DMA status={:08x} data={:08x} stride={}",
                status_addr, data_addr, ring.ibi_stride
            ));
        }
        ring.regs.write_dma_ring_intr_signal_enable(u32::MAX);
        ring.regs.write_dma_ring_intr_status_enable(u32::MAX);
        ring.regs.write_dma_ring_intr_status(u32::MAX);
        ring.regs.write_dma_ring_operation1(0);
        if ring.regs.dma_ring_operation2() & 0xff != 0 {
            return Err(DmaError::Fault("hardware dequeue pointer is not reset"));
        }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        ring.memory.arm();
        ring.active.set(true);
        regs.clear_hc_control(HC_CONTROL_PIO_MODE);
        regs.set_hc_control(HC_CONTROL_BUS_ENABLE | HC_CONTROL_RESUME);
        ring.regs.write_dma_ring_control(ENABLE_RUN);
        if let Err(error) = ring.wait(I3cHciRegisters::dma_ring_status, 3, 3) {
            // Initialization can time out after hardware has started. We still
            // own the ring here; stop it before returning and losing that handle.
            if let Err(stop_error) = ring.stop() {
                if let Some(trace) = ring.trace {
                    trace(format_args!(
                        "DMA startup failed: {}; shutdown also failed: {}",
                        error, stop_error
                    ));
                }
                // Failed stop keeps the DMA memory claim armed. Never make
                // those descriptors or payloads available to another session.
                return Err(DmaError::Fault(
                    "DMA startup failed; shutdown failed and DMA memory remains reserved",
                ));
            }
            return Err(error);
        }
        if let Some(trace) = ring.trace {
            trace(format_args!(
                "DMA {:?} ring0 offset={:04x} cmd_stride={} resp_stride={} cmd={:08x} resp={:08x}",
                mode, offset, ring.command_stride, ring.response_stride, cmd_addr, resp_addr
            ));
        }
        Ok(ring)
    }
    pub(crate) fn bus(&self) -> u8 {
        self.regs.bus()
    }

    /// Mode selected from the core logical initialization role.
    pub const fn mode(&self) -> DmaMode {
        self.mode
    }

    /// Target DMA command encoding and event dispatch are not interchangeable
    /// with controller DMA. Reject controller transfers on a target ring.
    fn require_controller(&self) -> DmaResult<()> {
        self.require_active()?;
        if self.mode != DmaMode::Controller {
            return Err(DmaError::Fault("controller DMA operation on target ring"));
        }
        Ok(())
    }

    /// Report raw target DMA ring pointers for target bring-up.
    pub fn target_ring_pointers(&self) -> DmaResult<(u32, u32, u32, u32)> {
        if self.mode != DmaMode::Target {
            return Err(DmaError::Fault("not a target DMA ring"));
        }
        Ok((
            self.regs.dma_ring_operation1(),
            self.regs.dma_ring_operation2(),
            self.event_status(),
            self.regs.dma_ring_status(),
        ))
    }

    pub(crate) fn command_format(&self) -> CommandDescriptor {
        self.command_format
    }

    fn allocate_payload(&mut self, slot: usize, len: usize) -> DmaResult<*mut u8> {
        if self.commands.payloads[slot].is_some() {
            return Err(DmaError::Fault(
                "DMA slot still owns a payload buffer; reinitialize after failed transfer",
            ));
        }
        let aligned = len
            .checked_add(3)
            .ok_or(DmaError::Fault("DMA payload length overflow"))?
            & !3;
        let offset = payload_offset(&self.commands.payloads, aligned, self.payload_capacity)
            .ok_or(DmaError::Fault("DMA payload SRAM exhausted"))?;
        let pointer = unsafe { self.payload.add(offset) };
        if len != 0 {
            dma_address(pointer, len)?;
        }
        self.commands.payloads[slot] = Some(PayloadAllocation {
            offset,
            len: aligned,
        });
        Ok(pointer)
    }

    // CPU pointers only: bus address translation remains in dma_address().
    // Callers validate ring slots before accessing these descriptors.
    fn command_ptr(&self, slot: usize) -> *mut u32 {
        debug_assert!(slot < ENTRIES);
        unsafe {
            addr_of_mut!((*self.descriptors).commands)
                .cast::<u8>()
                .add(slot * self.command_stride)
                .cast::<u32>()
        }
    }

    fn response_ptr(&self, slot: usize) -> *mut u32 {
        debug_assert!(slot < ENTRIES);
        unsafe {
            addr_of_mut!((*self.descriptors).responses)
                .cast::<u8>()
                .add(slot * self.response_stride)
                .cast::<u32>()
        }
    }

    /// Fill padding as well as command/data words before publishing the slot.
    fn write_command(&self, slot: usize, words: &[u32]) {
        let descriptor = self.command_ptr(slot);
        for i in 0..self.command_stride / 4 {
            unsafe { write_volatile(descriptor.add(i), words.get(i).copied().unwrap_or(0)) };
        }
    }

    fn accepts_command(&self, command: HciCommand) -> bool {
        matches!(
            (self.command_format, command),
            (CommandDescriptor::V1, HciCommand::V1(_)) | (CommandDescriptor::V2, HciCommand::V2(_))
        )
    }

    /// Submit without interpreting response status: DAA's final NACK is expected.
    pub(crate) fn submit_raw(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        self.require_controller()
            .map_err(|_| I3cHciError::Unsupported)?;
        validate_transfer(&transfer)?;
        let command = transfer.command;
        if !self.accepts_command(command) {
            return Err(I3cHciError::Unsupported);
        }
        if !command.expects_response() {
            return Err(I3cHciError::Unsupported);
        }
        // DAA uses the response length as an assignment count, not RX bytes.
        let read =
            command.read() && command.kind() != super::cmd::HciCommandKind::AddressAssignment;
        let len = if read {
            command.data_len()
        } else {
            transfer.tx.len()
        };
        if (read && !transfer.tx.is_empty()) || (!read && !transfer.rx.is_empty()) {
            return Err(I3cHciError::InvalidArgs);
        }
        let data = if read {
            &transfer.rx[..len]
        } else {
            transfer.tx
        };
        let (response, buffer) = self
            .transfer_buffered(command, command.tid(), read, data)
            .map_err(|message| {
                if let Some(trace) = self.trace {
                    trace(format_args!("DMA submit failed: {}", message));
                }
                I3cHciError::from(message)
            })?;
        let rx_len = if read && response.success() {
            response.data_len
        } else {
            0
        };
        if rx_len > transfer.rx.len() {
            return Err(I3cHciError::NoSpace);
        }
        for (i, byte) in transfer.rx[..rx_len].iter_mut().enumerate() {
            *byte = unsafe { read_volatile(buffer.add(i)) };
        }
        Ok(HciTransferResult { response, rx_len })
    }

    /// Publish both V2 DAA phases together: the PID read holds the bus until
    /// the address-assignment command runs. Never submit them synchronously.
    /// A failed PID read may leave the assignment unexecuted; no response is
    /// fabricated for that slot. The caller classifies the terminal DAA NACK.
    pub(crate) fn submit_daa_pair(
        &mut self,
        first: HciCommand,
        second: HciCommand,
        rx: &mut [u8; 8],
    ) -> Result<(HciResponse, Option<HciResponse>)> {
        self.require_controller().map_err(I3cHciError::from)?;
        self.require_command_ready().map_err(I3cHciError::from)?;
        if self.mode != DmaMode::Controller
            || self.command_format != CommandDescriptor::V2
            || !self.accepts_command(first)
            || !self.accepts_command(second)
            || first.kind() != super::cmd::HciCommandKind::AddressAssignment
            || second.kind() != super::cmd::HciCommandKind::AddressAssignment
            || !first.read()
            || first.data_len() != rx.len()
            || second.read()
            || second.data_len() != 0
            || !first.expects_response()
            || !second.expects_response()
            || first.tid() == second.tid()
        {
            return Err(I3cHciError::InvalidArgs);
        }
        let start = self.commands.next;
        let assign = (start + 1) % ENTRIES;
        let end = (assign + 1) % ENTRIES;
        if self.regs.dma_ring_operation2() & 0xff != start as u32 {
            return Err(I3cHciError::Busy);
        }
        let buffer = self
            .allocate_payload(start, rx.len())
            .map_err(I3cHciError::Dma)?;
        let address = dma_address(buffer, rx.len()).map_err(|_| I3cHciError::InvalidArgs)?;
        let mut responses = [core::ptr::null_mut::<u32>(); 2];
        for (index, (slot, command, data)) in
            [(start, first, &[0u8; 8][..]), (assign, second, &[][..])]
                .into_iter()
                .enumerate()
        {
            responses[index] = self.response_ptr(slot);
            let words = Self::prepare_dma_command_words(command, command.read(), data, address);
            self.write_command(slot, &words);
            unsafe { write_volatile(responses[index], u32::MAX) };
        }
        const COMPLETE: u32 = 1 << 11;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA V2 DAA pair slots={}/{} TIDs={}/{} enqueue={}",
                start,
                assign,
                first.tid(),
                second.tid(),
                end
            ));
        }
        // Clear only sampled stale command events; preserve IBI notifications.
        let stale = self.event_status() & (COMPLETE | DMA_ERRORS);
        self.ack_events(stale);
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        self.publish_commands(end);

        let pending = PendingDaa {
            first,
            second,
            start,
            assign,
            end,
            buffer,
        };
        let result = self
            .wait_daa_completion(&pending)
            .and_then(|completion| self.complete_daa_pair(pending, completion, rx));
        if result.is_err() {
            self.commands.command_faulted.set(true);
        }
        result
    }

    /// Inspect both DAA phases once. No polling, acknowledgement or cancellation.
    fn try_daa_completion(&self, pending: &PendingDaa) -> Result<Option<DaaCompletion>> {
        const COMPLETE: u32 = 1 << 11;
        const ERROR: u32 = 1 << 9;
        let intr = self.event_status();
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        let deq = (self.regs.dma_ring_operation2() & 0xff) as usize;
        if deq >= ENTRIES {
            return Err(I3cHciError::InvalidArgs);
        }
        let first_raw = unsafe { read_volatile(self.response_ptr(pending.start)) };
        if intr & (COMPLETE | ERROR) != 0 && first_raw != u32::MAX && deq != pending.start {
            let first_response = HciResponse::parse(first_raw);
            if first_response.tid != pending.first.tid() {
                return Err(I3cHciError::InvalidArgs);
            }
            if !first_response.success() {
                return Ok(Some(DaaCompletion {
                    intr,
                    first_raw,
                    second_raw: None,
                }));
            }
            let second_raw = unsafe { read_volatile(self.response_ptr(pending.assign)) };
            if deq == pending.end && second_raw != u32::MAX {
                return Ok(Some(DaaCompletion {
                    intr,
                    first_raw,
                    second_raw: Some(second_raw),
                }));
            }
        }
        if intr & (1 << 5) != 0 {
            return Err(I3cHciError::TransferStatus(intr));
        }
        Ok(None)
    }

    fn complete_daa_pair(
        &mut self,
        pending: PendingDaa,
        completion: DaaCompletion,
        rx: &mut [u8; 8],
    ) -> Result<(HciResponse, Option<HciResponse>)> {
        const COMPLETE: u32 = 1 << 11;
        let PendingDaa {
            first: _,
            second,
            start,
            assign,
            end,
            buffer,
        } = pending;
        let DaaCompletion {
            intr,
            first_raw: raw,
            second_raw,
        } = completion;
        let first_response = HciResponse::parse(raw);
        let Some(second_raw) = second_raw else {
            // Stop fetching before cancelling the suffix; preserve OP1 IBI fields.
            self.regs
                .write_dma_ring_control(self.regs.dma_ring_control() & !RUNNING);
            self.wait(I3cHciRegisters::dma_ring_status, RUNNING, 0)
                .map_err(I3cHciError::from)?;
            self.wait_aspeed_dma_idle().map_err(I3cHciError::from)?;
            let stopped = (self.regs.dma_ring_operation2() & 0xff) as usize;
            if stopped != assign && stopped != end {
                return Err(I3cHciError::InvalidArgs);
            }
            self.ack_events(self.event_status() & (COMPLETE | DMA_ERRORS));
            unsafe {
                asm!("dsb sy", options(nostack, preserves_flags));
            }
            self.regs.write_dma_ring_operation1(
                (self.regs.dma_ring_operation1() & !0xffff)
                    | stopped as u32
                    | ((stopped as u32) << 8),
            );
            self.commands.next = stopped;
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA V2 DAA PID response={:08x}; cancelled suffix, next={}",
                    raw, stopped
                ));
            }
            self.commands.payloads[start] = None;
            return Ok((first_response, None));
        };
        self.ack_events(intr & (COMPLETE | DMA_ERRORS));
        self.wait_aspeed_dma_idle().map_err(I3cHciError::from)?;
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        let second_response = HciResponse::parse(second_raw);
        if second_response.tid != second.tid() || first_response.data_len != rx.len() {
            return Err(I3cHciError::InvalidArgs);
        }
        for (i, byte) in rx.iter_mut().enumerate() {
            *byte = unsafe { read_volatile(buffer.add(i)) };
        }
        self.return_responses(end);
        self.commands.next = end;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA V2 DAA responses={:08x}/{:08x} PID bytes={:02x?}",
                raw, second_raw, rx
            ));
        }
        self.commands.payloads[start] = None;
        Ok((first_response, Some(second_response)))
    }

    fn prepare_dma_command_words(
        command: HciCommand,
        read: bool,
        data: &[u8],
        address: u32,
    ) -> [u32; 7] {
        let command = match command {
            HciCommand::V2(command) => {
                // V2 command bits must not pass through the v1 CCC rewrite.
                return [
                    command.words[0],
                    command.words[1],
                    command.words[2],
                    command.words[3],
                    (1 << 30) | data.len() as u32,
                    if data.is_empty() { 0 } else { address },
                    0,
                ];
            }
            HciCommand::V1(command) => command,
        };
        const ATTR_MASK: u32 = 0x7;
        const ATTR_IMMEDIATE: u32 = 0x1;
        const CP: u32 = 1 << 15;
        const RNW: u32 = 1 << 29;
        const DBP: u32 = 1 << 25;
        const DTT_SHIFT: u32 = 23;

        let len = data.len();

        // Convert only Regular CCC writes with up to 4 bytes.
        // Exclude CCCs with a defining byte.
        let immediate_ccc = !read
            && len <= 4
            && (command.words[0] & CP) != 0
            && (command.words[0] & RNW) == 0
            && (command.words[0] & DBP) == 0
            && (command.words[0] & ATTR_MASK) == 0;

        if immediate_ccc {
            let mut immediate_data = 0u32;

            for (i, &byte) in data.iter().enumerate() {
                immediate_data |= (byte as u32) << (i * 8);
            }

            let word0 =
                (command.words[0] & !ATTR_MASK) | ATTR_IMMEDIATE | ((len as u32) << DTT_SHIFT);

            return [word0, immediate_data, 1 << 30, 0, 0, 0, 0];
        }

        // Existing Regular DMA descriptor.
        [
            command.words[0],
            command.words[1],
            (1 << 30) | len as u32,
            if len == 0 { 0 } else { address },
            0,
            0,
            0,
        ]
    }

    /// Allocate the requested payload length in reserved SRAM. Keep allocation
    /// ownership after submission errors; hardware may still reference it.
    /// Reinitialization after stopping the ring reclaims quarantined buffers.
    pub(super) fn transfer_buffered(
        &mut self,
        command: HciCommand,
        tid: u8,
        read: bool,
        data: &[u8],
    ) -> DmaResult<(HciResponse, *mut u8)> {
        self.require_controller()?;
        self.require_command_ready()?;
        if !self.accepts_command(command) {
            return Err(DmaError::Fault(
                "DMA command format does not match hardware",
            ));
        }
        if self.regs.dma_ring_operation2() & 0xff != self.commands.next as u32 {
            return Err(DmaError::Fault("DMA ring has outstanding work"));
        }
        let slot = self.commands.next;
        let buffer = self.allocate_payload(slot, data.len())?;
        // RX is written by hardware. TX copies only the actual payload:
        // neither direction needs diagnostic fill or padding initialization.
        if !read {
            for (i, &byte) in data.iter().enumerate() {
                unsafe { write_volatile(buffer.add(i), byte) };
            }
        }
        let address = if data.is_empty() {
            0
        } else {
            dma_address(buffer, data.len())?
        };
        let words = Self::prepare_dma_command_words(command, read, data, address);
        let result = self.transfer_raw(command, tid, &words[..command.words().len() + 3])?;
        if read && result.success() && result.data_len > data.len() {
            return Err(DmaError::Fault("DMA read exceeds requested buffer"));
        }
        // Completion made this allocation reusable. The returned pointer is
        // consumed synchronously before another &mut self submission can occur.
        self.commands.payloads[slot] = None;
        Ok((result, buffer))
    }

    /// Submit a prepared descriptor and consume its response. Buffer ownership,
    /// DMA mapping and lifetime are established by the caller before entry.
    /// No payload allocation, initialization or content validation belongs here.
    fn transfer_raw(
        &mut self,
        command: HciCommand,
        tid: u8,
        words: &[u32],
    ) -> DmaResult<HciResponse> {
        let budget = self.poll_budget;
        let operation = self.begin_command(command, tid, words)?;
        let result = super::completion::Polling { budget }.run(
            operation,
            DmaError::Timeout("DMA command completion timeout"),
        );
        if result.is_err() {
            self.dump_dma_state("COMMAND FAILURE");
            self.snapshot();
        }
        result
    }

    pub(crate) fn begin_command(
        &mut self,
        command: HciCommand,
        tid: u8,
        words: &[u32],
    ) -> DmaResult<DmaCommandOperation<'_>> {
        let pending = self.submit_descriptor(command, tid, words)?;
        Ok(DmaCommandOperation {
            ring: self,
            pending,
            phase: CommandPhase::Event,
            completed: false,
        })
    }

    pub(crate) fn set_poll_budget(&mut self, budget: u32) {
        self.poll_budget = budget.max(1);
    }

    /// One nonblocking step. Polling and future kernel wakeups drive this same
    /// state; the ISR never owns response consumption or buffer reclamation.
    fn progress_command(
        &mut self,
        pending: PendingCommand,
        phase: &mut CommandPhase,
    ) -> DmaResult<super::completion::Progress<HciResponse>> {
        use super::completion::Progress;
        match phase {
            CommandPhase::Event => {
                let Some(status) = self.try_command_event(&pending)? else {
                    return Ok(Progress::Pending);
                };
                self.acknowledge_command_event(status);
                *phase = CommandPhase::DmaIdle;
                Ok(Progress::Advanced)
            }
            CommandPhase::DmaIdle => {
                if self.regs.aspeed_wdma_debug_low() & 2 != 0
                    || self.regs.aspeed_rdma_debug_low() & 2 != 0
                {
                    return Ok(Progress::Pending);
                }
                *phase = CommandPhase::Response;
                Ok(Progress::Advanced)
            }
            CommandPhase::Response => {
                let Some(raw) = self.try_response_visible(&pending) else {
                    return Ok(Progress::Pending);
                };
                self.complete_descriptor(pending, raw)
                    .map(Progress::Complete)
            }
        }
    }

    fn submit_descriptor(
        &mut self,
        command: HciCommand,
        tid: u8,
        words: &[u32],
    ) -> DmaResult<PendingCommand> {
        self.require_controller()?;
        self.require_command_ready()?;
        if !self.accepts_command(command) {
            return Err(DmaError::Fault(
                "DMA command format does not match hardware",
            ));
        }
        if words.len() > self.command_stride / 4 {
            return Err(DmaError::Fault("DMA descriptor exceeds command stride"));
        }
        let slot = self.commands.next;
        if self.regs.dma_ring_operation2() & 0xff != slot as u32 {
            return Err(DmaError::Fault("DMA ring has outstanding work"));
        }
        let descriptor = self.command_ptr(slot);

        let response = self.response_ptr(slot);

        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA CMD descriptor address={:08x}",
                descriptor as usize
            ));

            trace(format_args!(
                "DMA RESP descriptor address={:08x}",
                response as usize
            ));
        }

        // Write the complete descriptor.
        self.write_command(slot, words);

        // Read back the actual descriptor from SRAM.
        if let Some(trace) = self.trace {
            for i in 0..command.words().len() + 3 {
                let value = unsafe { read_volatile(descriptor.add(i)) };

                trace(format_args!("DMA DESC[{}]={:08x}", i, value));
            }
        }

        unsafe {
            write_volatile(response, 0xffff_ffff);
        }

        // Only stale command events belong to submission; preserve IBI evidence.
        let stale = self.event_status() & (DMA_ERRORS | (1 << 11));
        self.ack_events(stale);
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        self.dump_dma_state("BEFORE SUBMIT");
        let next = (slot + 1) % ENTRIES;
        self.publish_commands(next);
        self.dump_dma_state("AFTER SUBMIT");

        Ok(PendingCommand {
            command,
            tid,
            slot,
            next,
        })
    }

    /// Inspect one hardware snapshot without waiting or consuming a response.
    fn try_command_event(&self, pending: &PendingCommand) -> DmaResult<Option<u32>> {
        const COMPLETE: u32 = 1 << 11;
        const ERROR: u32 = 1 << 9;
        const ABORTED: u32 = 1 << 5;
        let status = self.event_status();
        if status & (COMPLETE | ERROR) != 0
            && self.regs.dma_ring_operation2() & 0xff == pending.next as u32
        {
            return Ok(Some(status));
        }
        if status & ABORTED != 0 {
            self.snapshot();
            self.ack_events(status & (COMPLETE | DMA_ERRORS));
            return Err(DmaError::Fault("DMA transfer aborted"));
        }
        Ok(None)
    }

    fn try_response_visible(&self, pending: &PendingCommand) -> Option<u32> {
        // Dequeue advancement does not guarantee SRAM writeback is visible.
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        let raw = unsafe { read_volatile(self.response_ptr(pending.slot)) };
        (raw != u32::MAX).then_some(raw)
    }

    fn acknowledge_command_event(&self, intr: u32) {
        const TRANSFER_COMPLETION: u32 = 1 << 11;
        const TRANSFER_ERROR: u32 = 1 << 9;
        let ack = intr & (TRANSFER_COMPLETION | DMA_ERRORS);
        self.ack_events(ack);
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA completion ACK: captured={:08x} ack={:08x} remaining={:08x}",
                intr,
                ack,
                self.event_status()
            ));
        }

        // Retain the existing ASPEED DMA-idle check before reading SRAM.
        // Use the saved status: error bits have already been acknowledged.
        if intr & TRANSFER_ERROR != 0 {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA TRANSFER_ERR detected: captured INTR={:08x}",
                    intr
                ));
            }
        }
    }

    fn complete_descriptor(
        &mut self,
        pending: PendingCommand,
        raw_response: u32,
    ) -> DmaResult<HciResponse> {
        let PendingCommand {
            command, tid, next, ..
        } = pending;
        // Parse valid response.
        let result = HciResponse::parse(raw_response);

        // Update response dequeue pointer.
        self.return_responses(next);

        self.commands.next = next;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA {:?} {} response={:08x} tid={} status={} length={}",
                command.kind(),
                if command.read() { "READ" } else { "WRITE" },
                result.raw,
                result.tid,
                result.status,
                result.data_len
            ));
        }

        if result.tid != tid {
            return Err(DmaError::Fault("DMA response TID mismatch"));
        }
        Ok(result)
    }
    /// Poll and consume one complete IBI. Returns None if no complete IBI is ready.
    /// The caller supplies storage; all ring management remains inside this driver.
    pub fn poll_ibi(&mut self, output: &mut [u8]) -> DmaResult<Option<DmaIbiEvent>> {
        if self.mode != DmaMode::Controller {
            return Err(DmaError::Fault(
                "controller IBI polling called on target DMA ring",
            ));
        }
        self.require_controller()?;
        // Match Linux: acknowledge sampled IBI flags before processing. A
        // notification arriving during processing must not be cleared later.
        let interrupt = self.event_status();
        if interrupt & DMA_ERRORS != 0 {
            // Preserve error evidence. Do not reset shared FIFOs with possible
            // outstanding commands; let the caller stop/reinitialize the ring.
            self.dump_ibi_state();
            return Err(DmaError::Fault(
                "DMA transfer error/abort while waiting for IBI",
            ));
        }
        let ack = interrupt & (INTR_IBI_READY | INTR_IBI_RING_FULL);
        if ack != 0 {
            if let Some(trace) = self.trace {
                trace(format_args!("got ibi interrupt: {}", interrupt));
            }
            self.ack_events(ack);
        }
        if interrupt & INTR_IBI_RING_FULL != 0 {
            self.dump_ibi_state();
            // Drain complete events below to free space, even when full.
        }
        let enq = ((self.regs.dma_ring_operation2() >> 16) & 0xff) as usize;
        if enq >= IBI_STATUS_RING_ENTRIES {
            return Err(DmaError::Fault("invalid IBI enqueue pointer"));
        }
        if enq == self.ibi.ibi_deq {
            return Ok(None);
        }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        // Collect a complete sequence, as in Linux hci_dma_process_ibi().
        let mut idx = self.ibi.ibi_deq;
        let mut total_chunks = 0usize;
        let mut size = 0usize;
        let mut addr = None;
        let mut bad = false;
        let mut rnw = None;
        let mut complete = false;
        let mut final_status = 0;
        while idx != enq {
            let ptr = self.ibi_status_ptr(idx);
            let status = unsafe { read_volatile(ptr) };
            final_status = status;

            if let Some(trace) = self.trace {
                trace(format_args!("IBI DMA status[{}]={:08x}", idx, status));
            }

            let decoded = super::cmd::IbiStatus::parse(status);
            let a = decoded.target_addr;
            let read = decoded.read;
            if rnw.is_some_and(|previous| previous != read) {
                bad = true;
            }
            rnw = Some(read);
            let chunks = decoded.chunks as usize;
            let last = decoded.last;
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "IBI decoded: addr={:02x} rnw={} last={} error={} chunks={} tail={}",
                    a,
                    status & (1 << 8) != 0,
                    last,
                    status & (1 << 30) != 0,
                    chunks,
                    status & 0xff,
                ));
            }
            if decoded.error {
                bad = true;
            }
            if let Some(prev) = addr {
                if prev != a {
                    bad = true;
                }
            } else {
                addr = Some(a);
            }
            if chunks > IBI_CHUNK_POOL_SIZE || total_chunks + chunks > IBI_CHUNK_POOL_SIZE {
                return Err(DmaError::Fault("invalid IBI chunk count"));
            }
            total_chunks += chunks;
            if last {
                let tail = decoded.data_len;
                if chunks > 0 && (tail == 0 || tail > IBI_CHUNK_SIZE) {
                    // Chunk ownership is still known: reclaim the completed
                    // event without copying its malformed payload.
                    bad = true;
                }
                size += if chunks == 0 {
                    0
                } else {
                    (chunks - 1) * IBI_CHUNK_SIZE + tail
                };
                complete = true;
            } else {
                size += chunks * IBI_CHUNK_SIZE;
            }
            idx = (idx + 1) % IBI_STATUS_RING_ENTRIES;
            if complete {
                break;
            }
        }
        if !complete {
            return Ok(None);
        }
        let oversized = size > output.len();
        let copy_len = if bad || oversized { 0 } else { size };
        self.copy_ibi_data(&mut output[..copy_len]);
        // Complete payload reads before returning ownership to hardware.
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        // Reclaim even discarded complete events, preserving command pointers.
        self.advance_ibi_status(idx, total_chunks);
        if bad {
            return Err(DmaError::Fault(
                "IBI controller reported an error or malformed status",
            ));
        }
        if oversized {
            return Err(DmaError::Fault(
                "IBI output buffer too small (event reclaimed)",
            ));
        }
        if let Some(trace) = self.trace {
            trace(format_args!(
                "IBI DMA received addr={:02x} len={} data={:02x?}",
                addr.unwrap_or(0),
                size,
                &output[..size]
            ));
        }
        Ok(Some(DmaIbiEvent {
            status: final_status,
            target_addr: addr.unwrap_or(0),
            payload_len: size,
        }))
    }

    /// Queue a v1 target DMA descriptor without waiting for a bus transaction.
    /// `command_word` is the target-format command (not a controller v1 command).
    /// Each outstanding slot owns a payload allocation sized to its command.
    pub fn target_queue_command(&mut self, command_word: u32, data: &[u8]) -> DmaResult<usize> {
        self.require_active()?;
        if self.mode != DmaMode::Target {
            return Err(DmaError::Fault("target DMA command requires a target ring"));
        }
        if data.len() > 0xffff || (command_word >> 16) as usize != data.len() {
            return Err(DmaError::Fault(
                "target DMA command/payload length mismatch",
            ));
        }
        let slot = self.commands.next;
        // Preserve one empty entry: equal producer/consumer pointers mean empty.
        let next = (slot + 1) % ENTRIES;
        if self.commands.target_queued[slot].is_some() || next == self.commands.target_resp_deq {
            return Err(DmaError::Fault("target DMA command ring full"));
        }
        let tx = self.allocate_payload(slot, data.len())?;
        for (i, &byte) in data.iter().enumerate() {
            unsafe { write_volatile(tx.add(i), byte) };
        }
        let addr = if data.is_empty() {
            0
        } else {
            dma_address(tx, data.len())?
        };
        let resp = self.response_ptr(slot);
        // HCI v1 target: one command word and three data-buffer words.
        // Data buffer descriptor: IOC (bit 30) and length (bits 21:0).
        let words = [command_word, (1u32 << 30) | data.len() as u32, addr, 0];
        self.write_command(slot, &words);
        unsafe {
            write_volatile(resp, 0xffff_ffff);
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        self.publish_commands(next);
        self.commands.target_queued[slot] = Some(((command_word >> 3) & 0x03) as u8);
        self.commands.next = next;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "target DMA queued slot={} cmd={:08x} len={} addr={:08x}",
                slot,
                command_word,
                data.len(),
                addr
            ));
        }
        Ok(slot)
    }
    fn ibi_status_ptr(&self, index: usize) -> *mut u32 {
        debug_assert!(index < IBI_STATUS_RING_ENTRIES);
        unsafe {
            addr_of_mut!((*self.descriptors).ibi_status)
                .cast::<u8>()
                .add(index * self.ibi_stride)
                .cast::<u32>()
        }
    }

    /// Copy from the circular chunk pool before returning ownership to hardware.
    fn copy_ibi_data(&self, output: &mut [u8]) {
        let base = self.ibi_data;
        for (i, byte) in output.iter_mut().enumerate() {
            let offset = (self.ibi.ibi_chunk_ptr * IBI_CHUNK_SIZE + i)
                % (IBI_CHUNK_POOL_SIZE * IBI_CHUNK_SIZE);
            *byte = unsafe { read_volatile(base.add(offset)) };
        }
    }

    fn advance_ibi_status(&mut self, next_deq: usize, chunks: usize) {
        self.ibi.ibi_deq = next_deq;

        self.regs.write_dma_ring_operation1(replace_pointer(
            self.regs.dma_ring_operation1(),
            16,
            next_deq,
        ));

        if chunks > 0 {
            self.ibi.ibi_chunk_ptr = (self.ibi.ibi_chunk_ptr + chunks) % IBI_CHUNK_POOL_SIZE;

            self.regs
                .write_dma_ring_chunk_control(self.regs.dma_ring_chunk_control() + chunks as u32);
        }
    }

    pub fn target_poll_rx(&mut self, output: &mut [u8]) -> DmaResult<Option<DmaTargetRx>> {
        self.require_active()?;
        if self.mode != DmaMode::Target {
            return Err(DmaError::Fault("target RX polling requires a target ring"));
        }

        let intr = self.event_status();
        if intr & DMA_ERRORS != 0 {
            self.snapshot();
            return Err(DmaError::Fault("target DMA RX error"));
        }

        // OP2[23:16] is the hardware status enqueue pointer.
        let enq = ((self.regs.dma_ring_operation2() >> 16) & 0xff) as usize;

        if enq >= IBI_STATUS_RING_ENTRIES {
            return Err(DmaError::Fault("invalid target RX status enqueue pointer"));
        }

        if enq == self.ibi.ibi_deq {
            return Ok(None);
        }

        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Process one target response descriptor per call.
        let idx = self.ibi.ibi_deq;
        let ptr = self.ibi_status_ptr(idx);

        let raw = unsafe { read_volatile(ptr) };

        let response = TargetResponse::parse(raw);
        let status = response.status;
        let received = response.target_received;
        let ccc = response.ccc;
        let ccc_hdr = response.ccc_hdr;
        let len = response.received_len();

        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI{} target RX[{}] raw={:08x} status={} received={} ccc={} hdr={:02x} len={}",
                self.regs.bus(),
                idx,
                raw,
                status,
                received,
                ccc,
                ccc_hdr,
                response.data_len
            ));
        }

        if status != 0 {
            return Err(DmaError::Fault("target RX response reports error"));
        }

        // Like Linux/Zephyr, consume target-transmit notifications (e.g. CCC
        // reads) without reading or releasing RX chunks. Only target-received
        // bytes own chunks; bits [23:16] are the CCC header, not a chunk count.
        let chunks = target_rx_chunks(response);

        if chunks > IBI_CHUNK_POOL_SIZE {
            return Err(DmaError::Fault("target RX chunk count invalid"));
        }

        if len > output.len() {
            return Err(DmaError::Fault("target RX output buffer too small"));
        }

        self.copy_ibi_data(&mut output[..len]);

        // Finish reading DMA memory before returning ownership.
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }

        let next = (idx + 1) % IBI_STATUS_RING_ENTRIES;

        self.advance_ibi_status(next, chunks);

        self.ack_events(intr & (INTR_IBI_READY | INTR_IBI_RING_FULL));

        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI{} RX complete: raw={:08x} ccc={} hdr={:02x} len={} chunks={}",
                self.regs.bus(),
                raw,
                ccc,
                ccc_hdr,
                len,
                chunks
            ));
        }

        Ok(Some(DmaTargetRx {
            len,
            ccc,
            ccc_hdr,
            raw_status: raw,
        }))
    }

    /// Poll one target command completion, independently of incoming private writes.    
    pub fn target_poll_response(&mut self) -> DmaResult<Option<DmaTargetResponse>> {
        self.require_active()?;
        if self.mode != DmaMode::Target {
            return Err(DmaError::Fault(
                "target response polling requires a target ring",
            ));
        }

        let intr = self.event_status();

        if intr & DMA_ERRORS != 0 {
            self.snapshot();
            return Err(DmaError::Fault("target DMA transfer error or abort"));
        }

        let slot = self.commands.target_resp_deq;

        let Some(expected_tid) = self.commands.target_queued[slot] else {
            return Ok(None);
        };

        // Zephyr: RING_OP2_CR_DEQ_PTR.
        // OP2[7:0] is the hardware command dequeue pointer.
        let hw_deq = (self.regs.dma_ring_operation2() & 0xff) as usize;

        if hw_deq >= ENTRIES {
            return Err(DmaError::Fault(
                "invalid target DMA command dequeue pointer",
            ));
        }

        if slot == hw_deq {
            return Ok(None);
        }

        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }

        let ptr = self.response_ptr(slot);

        let raw = unsafe { read_volatile(ptr) };

        if raw == 0xffff_ffff {
            return Ok(None);
        }

        // Target response format from Zephyr cmd.h.
        let result = DmaTargetResponse {
            raw,
            tid: ((raw >> 24) & 0x03) as u8,
            status: ((raw >> 28) & 0x0f) as u8,
            data_len: (raw & 0xffff) as usize,
        };
        if result.tid != expected_tid {
            // Keep the response and its payload owned until explicit shutdown.
            return Err(DmaError::Fault("target DMA response TID mismatch"));
        }

        let next = (slot + 1) % ENTRIES;

        // OP1[15:8] = software command/response dequeue pointer.
        self.return_responses(next);

        self.commands.target_queued[slot] = None;
        self.commands.payloads[slot] = None;
        self.commands.target_resp_deq = next;

        self.ack_events(intr & (1 << 11));

        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI{} target DMA response slot={} raw={:08x} tid={} status={} len={} hw_deq={}",
                self.regs.bus(),
                slot,
                result.raw,
                result.tid,
                result.status,
                result.data_len,
                hw_deq,
            ));
        }

        Ok(Some(result))
    }

    /// Receive one target-side write/CCC through the IBI status/data DMA ring.
    /// In target mode these ring entries are not controller-originated IBIs.
    //github.com/AspeedTech-BMC/zephyr/blob/aspeed-main-v3.7.0/drivers/i3c/mipi_hci/target.c

    /// Called only after consuming the final DAA NACK, with no queued work.
    pub(crate) fn recover_after_daa_nack(&mut self, regs: &I3cHciRegisters) -> Result<()> {
        use super::constants;
        self.require_controller().map_err(I3cHciError::from)?;
        if regs.bus() != self.regs.bus() {
            return Err(I3cHciError::InvalidArgs);
        }
        // Leave ENABLE set so command/response pointer positions are retained.
        self.regs
            .write_dma_ring_control(self.regs.dma_ring_control() & !2);
        self.wait(I3cHciRegisters::dma_ring_status, RUNNING, 0)
            .map_err(I3cHciError::from)?;
        // ASPEED DMA reuses the PIO FIFOs. Wait for both DMA engines before
        // resetting them, as in Linux aspeed_pio_fifo_reset().
        self.wait_aspeed_dma_idle().map_err(I3cHciError::from)?;
        let mask = constants::CMD_QUEUE_RST
            | constants::RESP_QUEUE_RST
            | constants::TX_FIFO_RST
            | constants::RX_FIFO_RST;
        regs.write_reset_control(mask);
        self.wait_reset_clear(regs, mask)?;
        self.ack_events(DMA_ERRORS);
        regs.set_hc_control(HC_CONTROL_RESUME);
        self.regs.write_dma_ring_control(ENABLE_RUN);
        self.wait(I3cHciRegisters::dma_ring_status, 3, 3)
            .map_err(I3cHciError::from)?;
        self.dump_dma_state("DMA DAA end recovery");
        Ok(())
    }

    pub fn stop(&self) -> DmaResult<()> {
        self.regs
            .write_dma_ring_control(self.regs.dma_ring_control() & !2);
        if self
            .wait(I3cHciRegisters::dma_ring_status, RUNNING, 0)
            .is_err()
        {
            self.regs
                .write_dma_ring_control(self.regs.dma_ring_control() | ABORT);
            self.wait(I3cHciRegisters::dma_ring_status, RUNNING, 0)?;
        }
        self.regs.write_dma_ring_control(0);
        self.wait(I3cHciRegisters::dma_ring_status, 3, 0)?;
        self.wait_aspeed_dma_idle()?;
        self.active.set(false);
        self.memory.stopped();
        Ok(())
    }
}

#[cfg(test)]
mod command_layout_tests {
    use super::super::cmd::{HciCommandKind, HciCommandV1, HciCommandV2};
    use super::*;

    const TEST_POOL_SIZE: usize = 8192;

    #[test]
    fn dma_role_comes_from_initialization_mode() {
        use super::super::types::InitMode;
        assert_eq!(
            DmaMode::from_init_mode(Some(InitMode::Primary)),
            Ok(DmaMode::Controller)
        );
        assert_eq!(
            DmaMode::from_init_mode(Some(InitMode::Secondary)),
            Ok(DmaMode::Target)
        );
        assert_eq!(
            DmaMode::from_init_mode(Some(InitMode::Target)),
            Ok(DmaMode::Target)
        );
        assert!(DmaMode::from_init_mode(None).is_err());
    }

    #[test]
    fn payloads_larger_than_64_bytes_use_available_pool() {
        let live = [None; ENTRIES];
        assert_eq!(payload_offset(&live, 4096, TEST_POOL_SIZE), Some(0));
        assert_eq!(
            payload_offset(&live, TEST_POOL_SIZE, TEST_POOL_SIZE),
            Some(0)
        );
        assert_eq!(
            payload_offset(&live, TEST_POOL_SIZE + 1, TEST_POOL_SIZE),
            None
        );
        assert_eq!(payload_offset(&live, usize::MAX, TEST_POOL_SIZE), None);
    }

    #[test]
    fn live_target_payloads_do_not_overlap_and_freed_holes_are_reused() {
        // Slot order differs from address order after ring wrap.
        let mut live = [
            Some(PayloadAllocation {
                offset: 1024,
                len: 2048,
            }),
            Some(PayloadAllocation {
                offset: 0,
                len: 1024,
            }),
            None,
            None,
        ];
        assert_eq!(payload_offset(&live, 4096, TEST_POOL_SIZE), Some(3072));
        live[1] = None;
        assert_eq!(payload_offset(&live, 512, TEST_POOL_SIZE), Some(0));
        assert_eq!(payload_offset(&live, 2048, TEST_POOL_SIZE), Some(3072));
    }

    #[test]
    fn payload_pool_exhaustion_keeps_outstanding_storage_reserved() {
        let live = [
            Some(PayloadAllocation {
                offset: 0,
                len: TEST_POOL_SIZE,
            }),
            None,
            None,
            None,
        ];
        assert_eq!(payload_offset(&live, 4, TEST_POOL_SIZE), None);
        assert_eq!(payload_offset(&live, 0, TEST_POOL_SIZE), Some(0));
    }

    #[test]
    fn v2_buffer_descriptor_follows_all_four_command_words() {
        // Include v1-looking CP bits: v2 must never undergo the v1 CCC rewrite.
        let command = HciCommandV2 {
            words: [0x0000_8000, 0x1234_5678, 0x9abc_def0, 0x1122_3344],
            tid: 1,
            kind: HciCommandKind::Regular,
            read: false,
            data_len: 1,
        };
        let words =
            Ring::prepare_dma_command_words(HciCommand::V2(command), false, &[1], 0x000a_0800);
        assert_eq!(
            words,
            [
                0x0000_8000,
                0x1234_5678,
                0x9abc_def0,
                0x1122_3344,
                0x4000_0001,
                0x000a_0800,
                0
            ]
        );
    }

    #[test]
    fn v2_daa_pair_has_one_rx_buffer_and_only_assignment_terminates() {
        let read = HciCommandV2::daa_read_pid(2, 8).unwrap();
        let assign = HciCommandV2::daa_assign_address(3, 9).unwrap();
        let read_words =
            Ring::prepare_dma_command_words(HciCommand::V2(read), true, &[0; 8], 0x000a_0840);
        let assign_words = Ring::prepare_dma_command_words(HciCommand::V2(assign), false, &[], 0);
        assert_eq!(&read_words[..4], &read.words);
        assert_eq!(&read_words[4..], &[0x4000_0008, 0x000a_0840, 0]);
        assert_eq!(&assign_words[..4], &assign.words);
        assert_eq!(&assign_words[4..], &[0x4000_0000, 0, 0]);
        assert_eq!(read.words[0] & (1 << 31), 0);
        assert_ne!(assign.words[0] & (1 << 31), 0);
    }

    #[test]
    fn v1_buffer_descriptor_stays_after_two_command_words() {
        let command = HciCommandV1::i3c_private_read(9, 3, 8, 4).unwrap();
        let words =
            Ring::prepare_dma_command_words(HciCommand::V1(command), true, &[0; 8], 0x000a_0840);
        assert_eq!(&words[..2], &command.words);
        assert_eq!(&words[2..], &[0x4000_0008, 0x000a_0840, 0, 0, 0]);
    }
}

impl Ring {
    fn wait(&self, read: fn(&I3cHciRegisters) -> u32, mask: u32, expected: u32) -> DmaResult<()> {
        let result = super::completion::poll(
            self.poll_budget,
            || Ok((read(&self.regs) & mask == expected).then_some(())),
            DmaError::Timeout("DMA ring control timeout"),
        );
        if result.is_err() {
            self.snapshot();
        }
        result
    }
    fn wait_aspeed_dma_idle(&self) -> DmaResult<()> {
        for read in [
            I3cHciRegisters::aspeed_wdma_debug_low,
            I3cHciRegisters::aspeed_rdma_debug_low,
        ] {
            super::completion::poll(
                self.poll_budget,
                || Ok((read(&self.regs) & 2 == 0).then_some(())),
                DmaError::Timeout("ASPEED DMA engine busy timeout"),
            )?;
        }
        Ok(())
    }

    fn wait_daa_completion(&self, pending: &PendingDaa) -> Result<DaaCompletion> {
        super::completion::poll(
            self.poll_budget,
            || self.try_daa_completion(pending),
            I3cHciError::Dma(DmaError::Timeout("DMA DAA completion timeout")),
        )
    }
    fn wait_reset_clear(&self, regs: &I3cHciRegisters, mask: u32) -> Result<()> {
        super::completion::poll(
            self.poll_budget,
            || Ok((regs.reset_control() & mask == 0).then_some(())),
            I3cHciError::Dma(DmaError::Timeout("DMA queue reset timeout")),
        )
    }
}

impl Ring {
    pub(crate) fn with_irq_events(mut self, bus: usize) -> Self {
        if bus == self.regs.bus() as usize {
            self.irq_events = super::irq::isr_events(bus);
        }
        self
    }
    fn event_status(&self) -> u32 {
        let (live, latched) = match self.irq_events {
            Some(events) => events.snapshot_io_status(|| self.regs.dma_ring_intr_status()),
            None => (self.regs.dma_ring_intr_status(), 0),
        };
        let status = self.pending_irq.get() | latched | live;
        self.pending_irq.set(status);
        status
    }
    fn ack_events(&self, mask: u32) {
        if let Some(events) = self.irq_events {
            events.ack_io_status(mask, || self.regs.write_dma_ring_intr_status(mask));
            self.pending_irq.set(self.pending_irq.get() & !mask);
            events.rearm_io_signals(|saved| {
                self.regs.write_dma_ring_intr_signal_enable(
                    self.regs.dma_ring_intr_signal_enable() | saved,
                );
            });
        } else {
            self.regs.write_dma_ring_intr_status(mask);
            self.pending_irq.set(self.pending_irq.get() & !mask);
        }
    }
}

#[derive(Clone, Copy)]
enum CommandPhase {
    Event,
    DmaIdle,
    Response,
}
impl Ring {
    fn require_active(&self) -> DmaResult<()> {
        if self.active.get() {
            Ok(())
        } else {
            Err(DmaError::Fault("DMA ring is stopped"))
        }
    }
}

/// The exclusive ring borrow prevents another submission while this is pending.
pub(crate) struct DmaCommandOperation<'a> {
    ring: &'a mut Ring,
    pending: PendingCommand,
    phase: CommandPhase,
    completed: bool,
}
impl super::completion::Operation for DmaCommandOperation<'_> {
    type Output = HciResponse;
    type Error = DmaError;
    fn advance(&mut self) -> DmaResult<super::completion::Progress<HciResponse>> {
        let progress = self.ring.progress_command(self.pending, &mut self.phase)?;
        if matches!(&progress, super::completion::Progress::Complete(_)) {
            self.completed = true;
        }
        Ok(progress)
    }
}

impl Drop for DmaCommandOperation<'_> {
    fn drop(&mut self) {
        // Cancellation/timeout must not permit reuse of in-flight SRAM.
        // IBI draining remains available for the diagnostic response bypass.
        if !self.completed {
            self.ring.commands.command_faulted.set(true);
        }
    }
}
impl Ring {
    fn require_command_ready(&self) -> DmaResult<()> {
        if self.commands.command_faulted.get() {
            Err(DmaError::Fault(
                "DMA command recovery required; stop and reinitialize",
            ))
        } else {
            Ok(())
        }
    }
}

/// OP1 contains independent command and IBI ownership fields.
fn replace_pointer(op: u32, shift: u32, pointer: usize) -> u32 {
    (op & !(0xff << shift)) | (((pointer as u32) & 0xff) << shift)
}
impl Ring {
    fn publish_commands(&self, next: usize) {
        self.regs.write_dma_ring_operation1(replace_pointer(
            self.regs.dma_ring_operation1(),
            0,
            next,
        ));
    }
    fn return_responses(&self, next: usize) {
        self.regs.write_dma_ring_operation1(replace_pointer(
            self.regs.dma_ring_operation1(),
            8,
            next,
        ));
    }
}

fn target_rx_chunks(response: TargetResponse) -> usize {
    (response.received_len() + IBI_CHUNK_SIZE - 1) / IBI_CHUNK_SIZE
}

#[cfg(test)]
mod ownership_tests {
    use super::*;
    #[test]
    fn ccc_reads_between_writes_do_not_consume_rx_chunks() {
        // ENEC (one received byte), GETMWL/GETMRL/GETPID automatic replies,
        // then an eight-byte private write. Queries must not shift that write.
        let records = [
            (0x0c80_0001, 1, 1),
            (0x048b_0002, 0, 0),
            (0x048c_0003, 0, 0),
            (0x048d_0006, 0, 0),
            (0x0800_0008, 8, 2),
        ];
        for (raw, len, chunks) in records {
            let response = TargetResponse::parse(raw);
            assert!(response.success());
            assert_eq!(response.received_len(), len);
            assert_eq!(target_rx_chunks(response), chunks);
        }
    }

    #[test]
    fn command_wrap_preserves_response_and_ibi_ownership() {
        let op = 0xa57f0303;
        let wrapped = replace_pointer(op, 0, 0);
        assert_eq!(wrapped, 0xa57f0300);
        assert_eq!(replace_pointer(wrapped, 8, 0), 0xa57f0000);
        assert_eq!(replace_pointer(wrapped, 16, 0), 0xa5000300);
    }
}
