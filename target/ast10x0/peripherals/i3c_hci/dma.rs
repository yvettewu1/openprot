// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI DMA ring support including IBI reception.
use super::cmd::HciCommand;
use super::constants::{HC_CONTROL_BUS_ENABLE, HC_CONTROL_PIO_MODE, HC_CONTROL_RESUME};
use super::core::{CommandDescriptor, HciSections, I3cHciCore};
use super::error::{I3cHciError, Result};
use super::master::{HciTransfer, HciTransferResult, validate_transfer};
use super::registers::I3cHciRegisters;
use super::{HciCommandV1, HciResponse};
use core::arch::asm;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

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
const IBI_STATUS_MAX_STRIDE: usize = 256;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DmaRingEntry {
    pub words: [u32; 4],
}

impl DmaRingEntry {
    #[must_use]
    pub const fn empty() -> Self {
        Self { words: [0; 4] }
    }

    #[must_use]
    pub const fn from_words(words: [u32; 4]) -> Self {
        Self { words }
    }
}

pub struct DmaRing<'a> {
    entries: &'a mut [DmaRingEntry],
    producer: usize,
    consumer: usize,
    len: usize,
}

impl<'a> DmaRing<'a> {
    pub fn new(entries: &'a mut [DmaRingEntry]) -> Result<Self> {
        if entries.is_empty() {
            return Err(I3cHciError::InvalidArgs);
        }

        Ok(Self {
            entries,
            producer: 0,
            consumer: 0,
            len: 0,
        })
    }

    #[must_use]
    pub fn capacity(&self) -> usize {
        self.entries.len()
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
    pub fn is_full(&self) -> bool {
        self.len == self.entries.len()
    }

    #[must_use]
    pub const fn producer_index(&self) -> usize {
        self.producer
    }

    #[must_use]
    pub const fn consumer_index(&self) -> usize {
        self.consumer
    }

    pub fn push(&mut self, entry: DmaRingEntry) -> Result<usize> {
        if self.is_full() {
            return Err(I3cHciError::NoSpace);
        }

        let index = self.producer;
        self.entries[index] = entry;
        self.producer = (self.producer + 1) % self.entries.len();
        self.len += 1;
        Ok(index)
    }

    pub fn pop(&mut self) -> Option<DmaRingEntry> {
        if self.is_empty() {
            return None;
        }

        let index = self.consumer;
        let entry = self.entries[index];
        self.entries[index] = DmaRingEntry::empty();
        self.consumer = (self.consumer + 1) % self.entries.len();
        self.len -= 1;
        Some(entry)
    }

    pub fn clear(&mut self) {
        for entry in self.entries.iter_mut() {
            *entry = DmaRingEntry::empty();
        }
        self.producer = 0;
        self.consumer = 0;
        self.len = 0;
    }
}

type DmaResult<T> = core::result::Result<T, &'static str>;

const POLL_BUDGET: u32 = 1_000_000; // Iterations, not microseconds.
const ENTRIES: usize = 4;
const MAX_STRIDE: usize = 256;
// Controller v1: two command words plus three data-buffer words.
// A 16-byte hardware stride indicates target/secondary command format.
const MIN_XFER_STRUCT_SIZE: usize = 20;
const MIN_TARGET_XFER_STRUCT_SIZE: usize = 16;

// Match the Zephyr ASPEED I3C vendor DMA hook for non-AST27XX:
// pass the buffer address unchanged. The generic G2 CM4 address translator
// is not called by that I3C hook. Keep cacheability separate from addressing.
const DMA_BUS_OFFSET: u32 = 0;

// Per-ring allocation pool. Two pools plus descriptor/IBI storage fit the
// board's 128 KiB DMA_NC region, leaving space for alignment and other users.
// This is a total SRAM budget, not a per-transfer or per-slot size.
const DMA_PAYLOAD_POOL_SIZE: usize = 48 * 1024;

#[derive(Clone, Copy)]
struct PayloadAllocation {
    offset: usize,
    len: usize,
}

/// First-fit allocation among the at most ENTRIES live command buffers.
fn payload_offset(live: &[Option<PayloadAllocation>; ENTRIES], len: usize) -> Option<usize> {
    let mut offset = 0usize;
    loop {
        let end = offset.checked_add(len)?;
        if end > DMA_PAYLOAD_POOL_SIZE {
            return None;
        }
        let overlap = live
            .iter()
            .flatten()
            .find(|a| a.len != 0 && offset < a.offset + a.len && a.offset < end);
        match overlap {
            Some(a) => offset = a.offset + a.len,
            None => return Some(offset),
        }
    }
}

const ENABLE_RUN: u32 = 3;
const ABORT: u32 = 4;
const RUNNING: u32 = 2;
const DMA_ERRORS: u32 = (1 << 9) | (1 << 5);

#[repr(C, align(64))]
struct DmaMemory {
    commands: [u32; ENTRIES * MAX_STRIDE / 4],
    responses: [u32; ENTRIES * MAX_STRIDE / 4],
    payload: [u8; DMA_PAYLOAD_POOL_SIZE],

    ibi_status: [u32; IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE / 4],
    ibi_data: [u8; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE],
}

// Keep both HCI instances within the board's reserved DMA region.
const _: () = assert!(2 * core::mem::size_of::<DmaMemory>() <= 128 * 1024);

#[unsafe(link_section = ".dma_nc")]
static mut DMA_MEMORY: DmaMemory = DmaMemory {
    commands: [0; ENTRIES * MAX_STRIDE / 4],
    responses: [0; ENTRIES * MAX_STRIDE / 4],
    payload: [0; DMA_PAYLOAD_POOL_SIZE],
    ibi_status: [0; IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE / 4],
    ibi_data: [0; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE],
};

// Independent backing storage: HCI0 and HCI1 must never share DMA descriptors.
#[unsafe(link_section = ".dma_nc")]
static mut DMA_MEMORY_HCI1: DmaMemory = DmaMemory {
    commands: [0; ENTRIES * MAX_STRIDE / 4],
    responses: [0; ENTRIES * MAX_STRIDE / 4],
    payload: [0; DMA_PAYLOAD_POOL_SIZE],
    ibi_status: [0; IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE / 4],
    ibi_data: [0; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE],
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaMode {
    Controller,
    Target,
}

pub struct Ring {
    regs: I3cHciRegisters,
    mode: DmaMode,
    memory: *mut DmaMemory,
    command_format: CommandDescriptor,
    command_stride: usize,
    response_stride: usize,
    next: usize,
    ibi_stride: usize,
    ibi_deq: usize,
    ibi_chunk_ptr: usize,
    target_resp_deq: usize,
    target_queued: [bool; ENTRIES],
    payloads: [Option<PayloadAllocation>; ENTRIES],
    trace: Option<fn(core::fmt::Arguments<'_>)>,
}

#[derive(Debug, Clone, Copy)]
pub struct DmaIbiEvent {
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

/// Target RX record received through the DMA IBI status/data ring.
#[derive(Debug, Clone, Copy)]
pub struct DmaTargetRx {
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
    fn dump_wdma_debug(&self, stage: &str) {
        if let Some(trace) = self.trace {
            trace(format_args!(
                "{}: WDMA_DBG={:08x} RDMA_DBG={:08x}",
                stage,
                self.regs.aspeed_wdma_debug_low(),
                self.regs.aspeed_rdma_debug_low(),
            ));

            trace(format_args!(
                "{}: OP1={:08x} OP2={:08x} INTR={:08x}",
                stage,
                self.regs.dma_ring_operation1(),
                self.regs.dma_ring_operation2(),
                self.regs.dma_ring_intr_status(),
            ));

            trace(format_args!(
                "{}: CMD_BASE={:08x} RESP_BASE={:08x}",
                stage,
                self.regs.dma_ring_cmd_base_lo(),
                self.regs.dma_ring_resp_base_lo(),
            ));
        }
    }
    fn dump_dma_state(&self, stage: &str) {
        if let Some(trace) = self.trace {
            trace(format_args!(
                "{}: DMA:: RING_STATUS={:08x} INTR={:08x} OP1={:08x} OP2={:08x}",
                stage,
                self.regs.dma_ring_status(),
                self.regs.dma_ring_intr_status(),
                self.regs.dma_ring_operation1(),
                self.regs.dma_ring_operation2(),
            ));
        }
    }
    pub fn dump_ibi_state(&self) {
        if let Some(trace) = self.trace {
            trace(format_args!(
                "IBI controller: HC_CONTROL={:08x} HC_INTR_STATUS={:08x} ASPEED_STATUS={:08x} IBI_TERMINATE={:08x}",
                self.regs.hc_control(),
                self.regs.intr_status(),
                self.regs.aspeed_status(),
                self.regs.aspeed_ibi_terminate_control(),
            ));
            trace(format_args!(
                "DMA: RING_STATUS={:08x} RING_CONTROL={:08x} CR_SETUP={:08x} IBI_SETUP={:08x}",
                self.regs.dma_ring_status(),
                self.regs.dma_ring_control(),
                self.regs.dma_ring_cr_setup(),
                self.regs.dma_ring_ibi_setup(),
            ));

            trace(format_args!(
                "DMA: INTR={:08x} INTR_EN={:08x} OP1={:08x} OP2={:08x} CHUNK={:08x}",
                self.regs.dma_ring_intr_status(),
                self.regs.dma_ring_intr_status_enable(),
                self.regs.dma_ring_operation1(),
                self.regs.dma_ring_operation2(),
                self.regs.dma_ring_chunk_control(),
            ));

            trace(format_args!(
                "DMA: IBI_STATUS_BASE={:08x}:{:08x} IBI_DATA_BASE={:08x}:{:08x}",
                self.regs.dma_ring_ibi_status_base_hi(),
                self.regs.dma_ring_ibi_status_base_lo(),
                self.regs.dma_ring_ibi_data_base_hi(),
                self.regs.dma_ring_ibi_data_base_lo(),
            ));
        }
    }
    fn snapshot(&self) {
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA ring status={:08x} intr={:08x} op1={:08x} op2={:08x}",
                self.regs.dma_ring_status(),
                self.regs.dma_ring_intr_status(),
                self.regs.dma_ring_operation1(),
                self.regs.dma_ring_operation2()
            ));
        }
    }
    fn wait(&self, read: fn(&I3cHciRegisters) -> u32, mask: u32, expected: u32) -> DmaResult<()> {
        for _ in 0..POLL_BUDGET {
            if read(&self.regs) & mask == expected {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        self.snapshot();
        Err("DMA ring control timeout")
    }
    fn wait_aspeed_dma_idle(&self) -> DmaResult<()> {
        for (name, read) in [
            (
                "WDMA",
                I3cHciRegisters::aspeed_wdma_debug_low as fn(&I3cHciRegisters) -> u32,
            ),
            (
                "RDMA",
                I3cHciRegisters::aspeed_rdma_debug_low as fn(&I3cHciRegisters) -> u32,
            ),
        ] {
            let mut idle = false;
            for _ in 0..POLL_BUDGET {
                if read(&self.regs) & 2 == 0 {
                    idle = true;
                    break;
                }
                core::hint::spin_loop();
            }
            if !idle {
                if let Some(trace) = self.trace {
                    trace(format_args!("ASPEED {} engine busy timeout", name));
                }
                return Err("ASPEED DMA engine busy timeout");
            }
        }
        Ok(())
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
        // TODO:: need to use the InitMode instead of DmaMode
        let mode = match regs.bus() {
            0 => DmaMode::Controller,
            1 => DmaMode::Target,
            _ => return Err("unsupported DMA bus"),
        };
        let sections = core.sections();
        let rhs = sections.ring_headers_offset as usize;
        if rhs != 0x800 {
            return Err("RHS section does not match AST1080 PAC layout");
        }
        // No transfers or IBIs may be pending at this PIO-to-DMA transition.
        regs.clear_hc_control(HC_CONTROL_BUS_ENABLE);
        init_rhs(regs, sections, 1).map_err(|_| "RHS setup failed")?;
        let offset = regs.rhs_ring0_offset() as usize;
        if offset != 0x830 {
            return Err("ring-0 offset does not match AST1080 PAC layout");
        }
        let mut ring = Self {
            // SAFETY: this facade is used synchronously with the core/master
            // under their existing unique, serialized controller ownership.
            regs: unsafe { I3cHciRegisters::new(regs.bus()) }.ok_or("invalid DMA bus")?,
            mode,
            memory: if mode == DmaMode::Controller {
                addr_of_mut!(DMA_MEMORY)
            } else {
                addr_of_mut!(DMA_MEMORY_HCI1)
            },
            command_format: core
                .command_descriptor()
                .ok_or("command format not detected")?,
            command_stride: 0,
            response_stride: 0,
            next: 0,
            ibi_stride: 0,
            ibi_deq: 0,
            ibi_chunk_ptr: 0,
            target_resp_deq: 0,
            target_queued: [false; ENTRIES],
            payloads: [None; ENTRIES],
            trace,
        };
        if ring.regs.dma_ring_status() & 3 != 0 {
            return Err("ring already active");
        }
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
        if ring.command_stride < min_stride
            || ring.command_stride > MAX_STRIDE
            || ring.command_stride % 4 != 0
            || ring.response_stride < 4
            || ring.response_stride > MAX_STRIDE
            || ring.response_stride % 4 != 0
        {
            if let Some(trace) = trace {
                trace(format_args!(
                    "command_stride=0x{:08x} response_stride=0x{:08x}",
                    command_stride, response_stride
                ));
            }
            return Err("unsupported DMA descriptor sizes");
        }
        // NOLOAD SRAM is not zeroed by the shared startup. Explicitly initialize
        // it before the DMA engine owns any descriptor or buffer.
        unsafe {
            core::ptr::write_bytes(ring.memory, 0, 1);
        }
        // Poison all response storage, including padding, so a misplaced DMA
        // write is distinguishable from untouched SRAM in the failure dump.
        let responses = unsafe { addr_of_mut!((*ring.memory).responses).cast::<u32>() };
        for i in 0..ENTRIES * MAX_STRIDE / 4 {
            unsafe {
                write_volatile(responses.add(i), 0xffff_ffff);
            }
        }
        let cmd_addr = dma_address(
            unsafe { addr_of_mut!((*ring.memory).commands).cast() },
            ENTRIES * MAX_STRIDE,
        )?;
        let resp_addr = dma_address(
            unsafe { addr_of_mut!((*ring.memory).responses).cast() },
            ENTRIES * MAX_STRIDE,
        )?;
        ring.regs.write_dma_ring_cmd_base_lo(cmd_addr);
        ring.regs.write_dma_ring_cmd_base_hi(0);
        ring.regs.write_dma_ring_resp_base_lo(resp_addr);
        ring.regs.write_dma_ring_resp_base_hi(0);
        ring.regs.write_dma_ring_cr_setup(ENTRIES as u32);
        // Configure the independent IBI status and data DMA rings.
        ring.ibi_stride = ring.regs.dma_ibi_status_stride() as usize;
        if ring.ibi_stride < 4
            || ring.ibi_stride > IBI_STATUS_MAX_STRIDE
            || ring.ibi_stride % 4 != 0
        {
            return Err("invalid IBI status descriptor stride");
        }
        let status_addr = dma_address(
            unsafe { addr_of_mut!((*ring.memory).ibi_status).cast() },
            IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE,
        )?;
        let data_addr = dma_address(
            unsafe { addr_of_mut!((*ring.memory).ibi_data).cast() },
            IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE,
        )?;
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
            return Err("hardware dequeue pointer is not reset");
        }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        regs.clear_hc_control(HC_CONTROL_PIO_MODE);
        regs.set_hc_control(HC_CONTROL_BUS_ENABLE | HC_CONTROL_RESUME);
        ring.regs.write_dma_ring_control(ENABLE_RUN);
        ring.wait(I3cHciRegisters::dma_ring_status, 3, 3)?;
        if let Some(trace) = ring.trace {
            trace(format_args!(
                "DMA {:?} ring0 offset={:04x} cmd_stride={} resp_stride={} cmd={:08x} resp={:08x}",
                mode, offset, ring.command_stride, ring.response_stride, cmd_addr, resp_addr
            ));
        }
        Ok(ring)
    }
    /// Mode selected from the HCI bus instance at initialization.
    pub const fn mode(&self) -> DmaMode {
        self.mode
    }

    /// Target DMA command encoding and event dispatch are not interchangeable
    /// with controller DMA. Reject controller transfers on a target ring.
    fn require_controller(&self) -> DmaResult<()> {
        if self.mode != DmaMode::Controller {
            return Err("controller DMA operation on target ring");
        }
        Ok(())
    }

    /// Report raw target DMA ring pointers for initial HCI1 bring-up.
    pub fn target_ring_pointers(&self) -> DmaResult<(u32, u32, u32, u32)> {
        if self.mode != DmaMode::Target {
            return Err("not a target DMA ring");
        }
        Ok((
            self.regs.dma_ring_operation1(),
            self.regs.dma_ring_operation2(),
            self.regs.dma_ring_intr_status(),
            self.regs.dma_ring_status(),
        ))
    }

    pub(crate) fn command_format(&self) -> CommandDescriptor {
        self.command_format
    }

    fn allocate_payload(&mut self, slot: usize, len: usize) -> DmaResult<*mut u8> {
        if self.payloads[slot].is_some() {
            return Err("DMA slot still owns a payload buffer; reinitialize after failed transfer");
        }
        let aligned = len.checked_add(3).ok_or("DMA payload length overflow")? & !3;
        let offset = payload_offset(&self.payloads, aligned).ok_or("DMA payload SRAM exhausted")?;
        let pointer = unsafe {
            addr_of_mut!((*self.memory).payload)
                .cast::<u8>()
                .add(offset)
        };
        if len != 0 {
            dma_address(pointer, len)?;
        }
        self.payloads[slot] = Some(PayloadAllocation {
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
            addr_of_mut!((*self.memory).commands)
                .cast::<u8>()
                .add(slot * self.command_stride)
                .cast::<u32>()
        }
    }

    fn response_ptr(&self, slot: usize) -> *mut u32 {
        debug_assert!(slot < ENTRIES);
        unsafe {
            addr_of_mut!((*self.memory).responses)
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
                if message == "DMA transfer completion timeout"
                    || message == "DMA response visibility timeout"
                {
                    I3cHciError::Timeout
                } else {
                    I3cHciError::Dma(message)
                }
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
        let start = self.next;
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
        const ERROR: u32 = 1 << 9;
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
        let stale = self.regs.dma_ring_intr_status() & (COMPLETE | DMA_ERRORS);
        self.regs.write_dma_ring_intr_status(stale);
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        self.regs
            .write_dma_ring_operation1((self.regs.dma_ring_operation1() & !0xff) | end as u32);

        for _ in 0..POLL_BUDGET {
            let intr = self.regs.dma_ring_intr_status();
            unsafe {
                asm!("dsb sy", options(nostack, preserves_flags));
            }
            let deq = (self.regs.dma_ring_operation2() & 0xff) as usize;
            if deq >= ENTRIES {
                return Err(I3cHciError::InvalidArgs);
            }
            let raw = unsafe { read_volatile(responses[0]) };
            if intr & (COMPLETE | ERROR) != 0 && raw != u32::MAX && deq != start {
                let first_response = HciResponse::parse(raw);
                if first_response.tid != first.tid() {
                    return Err(I3cHciError::InvalidArgs);
                }
                if !first_response.success() {
                    // Stop fetching before cancelling the unexecuted suffix.
                    // Rebase both software pointers to the stopped hardware
                    // dequeue, retaining unrelated OP1 IBI fields.
                    self.regs
                        .write_dma_ring_control(self.regs.dma_ring_control() & !RUNNING);
                    self.wait(I3cHciRegisters::dma_ring_status, RUNNING, 0)
                        .map_err(|_| I3cHciError::Timeout)?;
                    self.wait_aspeed_dma_idle()
                        .map_err(|_| I3cHciError::Timeout)?;
                    let stopped = (self.regs.dma_ring_operation2() & 0xff) as usize;
                    if stopped != assign && stopped != end {
                        return Err(I3cHciError::InvalidArgs);
                    }
                    self.regs.write_dma_ring_intr_status(
                        self.regs.dma_ring_intr_status() & (COMPLETE | DMA_ERRORS),
                    );
                    unsafe {
                        asm!("dsb sy", options(nostack, preserves_flags));
                    }
                    self.regs.write_dma_ring_operation1(
                        (self.regs.dma_ring_operation1() & !0xffff)
                            | stopped as u32
                            | ((stopped as u32) << 8),
                    );
                    self.next = stopped;
                    if let Some(trace) = self.trace {
                        trace(format_args!(
                            "DMA V2 DAA PID response={:08x}; cancelled suffix, next={}",
                            raw, stopped
                        ));
                    }
                    self.payloads[start] = None;
                    return Ok((first_response, None));
                }
                let second_raw = unsafe { read_volatile(responses[1]) };
                if deq == end && second_raw != u32::MAX {
                    self.regs
                        .write_dma_ring_intr_status(intr & (COMPLETE | DMA_ERRORS));
                    self.wait_aspeed_dma_idle()
                        .map_err(|_| I3cHciError::Timeout)?;
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
                    self.regs.write_dma_ring_operation1(
                        (self.regs.dma_ring_operation1() & !0xff00) | ((end as u32) << 8),
                    );
                    self.next = end;
                    if let Some(trace) = self.trace {
                        trace(format_args!(
                            "DMA V2 DAA responses={:08x}/{:08x} PID bytes={:02x?}",
                            raw, second_raw, rx
                        ));
                    }
                    self.payloads[start] = None;
                    return Ok((first_response, Some(second_response)));
                }
            }
            if intr & (1 << 5) != 0 {
                return Err(I3cHciError::TransferStatus(intr));
            }
            core::hint::spin_loop();
        }
        Err(I3cHciError::Timeout)
    }

    pub fn transfer(
        &mut self,
        command: HciCommandV1,
        tid: u8,
        read: bool,
        xfer_data: &[u8],
    ) -> DmaResult<HciResponse> {
        self.require_controller()?;
        let (result, buffer) =
            self.transfer_buffered(HciCommand::V1(command), tid, read, xfer_data)?;
        if !result.success() {
            return Err("DMA response reports transfer failure");
        }
        if read {
            if result.data_len != xfer_data.len() {
                return Err("DMA read length mismatch");
            }
            for (i, expected) in xfer_data.iter().enumerate() {
                let actual = unsafe { read_volatile(buffer.add(i)) };
                if actual != *expected {
                    return Err("DMA read payload mismatch");
                }
            }
        } else if result.data_len != 0 {
            return Err("DMA write reports untransferred bytes");
        }
        Ok(result)
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
    fn transfer_buffered(
        &mut self,
        command: HciCommand,
        tid: u8,
        read: bool,
        data: &[u8],
    ) -> DmaResult<(HciResponse, *mut u8)> {
        self.require_controller()?;
        if !self.accepts_command(command) {
            return Err("DMA command format does not match hardware");
        }
        if self.regs.dma_ring_operation2() & 0xff != self.next as u32 {
            return Err("DMA ring has outstanding work");
        }
        let slot = self.next;
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
            return Err("DMA read exceeds requested buffer");
        }
        // Completion made this allocation reusable. The returned pointer is
        // consumed synchronously before another &mut self submission can occur.
        self.payloads[slot] = None;
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
        self.require_controller()?;
        if !self.accepts_command(command) {
            return Err("DMA command format does not match hardware");
        }
        if words.len() > self.command_stride / 4 {
            return Err("DMA descriptor exceeds command stride");
        }
        let slot = self.next;
        if self.regs.dma_ring_operation2() & 0xff != slot as u32 {
            return Err("DMA ring has outstanding work");
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
        // Two v1 or four v2 command words, then length/IOC and a 64-bit
        // DMA address. Hardware stride may include additional padding.
        self.dump_wdma_debug("BEFORE DMA SUBMIT");

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

        //self.regs.write_dma_ring_intr_status(DMA_ERRORS | (1 << 11));
        self.regs.write_dma_ring_intr_status(0x0fff);
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        self.dump_dma_state("BEFORE SUBMIT");
        let next = (slot + 1) % ENTRIES;
        let op = self.regs.dma_ring_operation1();
        self.regs
            .write_dma_ring_operation1((op & !0xff) | next as u32);
        self.dump_dma_state("AFTER SUBMIT");
        self.dump_wdma_debug("AFTER DMA SUBMIT");

        const TRANSFER_COMPLETION: u32 = 1 << 11;
        const TRANSFER_ERROR: u32 = 1 << 9;
        const TRANSFER_ABORT: u32 = 1 << 5;
        let mut completion_status = None;
        for _ in 0..POLL_BUDGET {
            let status = self.regs.dma_ring_intr_status();
            // Like Linux's IRQ path, require a completion/error event before
            // consuming the response. A terminal DAA NACK is still returned
            // as a raw response for the DAA classifier.
            if status & (TRANSFER_COMPLETION | TRANSFER_ERROR) != 0
                && self.regs.dma_ring_operation2() & 0xff == next as u32
            {
                completion_status = Some(status);
                break;
            }
            if status & TRANSFER_ABORT != 0 {
                self.snapshot();
                self.regs
                    .write_dma_ring_intr_status(status & (TRANSFER_COMPLETION | DMA_ERRORS));
                return Err("DMA transfer aborted");
            }
            core::hint::spin_loop();
        }
        let Some(intr) = completion_status else {
            self.dump_wdma_debug("RESPONSE TIMEOUT");
            self.snapshot();
            return Err("DMA transfer completion timeout");
        };

        let ack = intr & (TRANSFER_COMPLETION | DMA_ERRORS);
        self.regs.write_dma_ring_intr_status(ack);
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA completion ACK: captured={:08x} ack={:08x} remaining={:08x}",
                intr,
                ack,
                self.regs.dma_ring_intr_status()
            ));
        }
        //self.dump_wdma_debug("AFTER COMPLETION ACK");

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
        self.wait_aspeed_dma_idle()?;

        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }

        // CR_DEQ advancement alone does not prove that the response is
        // visible in SRAM. Poll the poisoned slot with a bounded budget. This
        // diagnoses delayed writeback without accepting an unwritten response.
        // A barrier orders CPU accesses; it does not complete a DMA write.
        let mut raw_response = 0xffff_ffff;
        let mut response_polls = 0;
        for poll in 0..POLL_BUDGET {
            unsafe {
                asm!("dsb sy", options(nostack, preserves_flags));
            }
            raw_response = unsafe { read_volatile(response) };
            if raw_response != 0xffff_ffff {
                response_polls = poll;
                break;
            }
            core::hint::spin_loop();
        }
        if raw_response == 0xffff_ffff {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA response timeout: slot={} expected_tid={} address={:08x} stride={}",
                    slot, tid, response as usize, self.response_stride
                ));
                trace(format_args!(
                    "DMA RESP_BASE={:08x}:{:08x} CR_SETUP={:08x} WDMA_DBG={:08x} RDMA_DBG={:08x}",
                    self.regs.dma_ring_resp_base_hi(),
                    self.regs.dma_ring_resp_base_lo(),
                    self.regs.dma_ring_cr_setup(),
                    self.regs.aspeed_wdma_debug_low(),
                    self.regs.aspeed_rdma_debug_low()
                ));
                // Include nearby padding to expose writes at an unexpected
                // stride. Never consume a response from a different address.
                let base = unsafe { addr_of_mut!((*self.memory).responses).cast::<u32>() };
                for i in 0..16 {
                    let ptr = unsafe { base.add(i) };
                    trace(format_args!(
                        "DMA RESP word[{}] addr={:08x} value={:08x}",
                        i,
                        ptr as usize,
                        unsafe { read_volatile(ptr) }
                    ));
                }
            }
            self.snapshot();
            return Err("DMA response visibility timeout");
        }
        if response_polls != 0 {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA response visible after {} extra polls",
                    response_polls
                ));
            }
        }

        // Parse valid response.
        let result = HciResponse::parse(raw_response);

        // Update response dequeue pointer.
        self.regs.write_dma_ring_operation1(
            (self.regs.dma_ring_operation1() & !0xff00) | ((next as u32) << 8),
        );

        self.next = next;
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
            return Err("DMA response TID mismatch");
        }
        Ok(result)
    }
    /// Poll and consume one complete IBI. Returns None if no complete IBI is ready.
    /// The caller supplies storage; all ring management remains inside this driver.
    pub fn poll_ibi(&mut self, output: &mut [u8]) -> DmaResult<Option<DmaIbiEvent>> {
        if self.mode != DmaMode::Controller {
            return Err("controller IBI polling called on target DMA ring");
        }
        self.require_controller()?;
        // Match Linux: acknowledge sampled IBI flags before processing. A
        // notification arriving during processing must not be cleared later.
        let interrupt = self.regs.dma_ring_intr_status();
        if interrupt & DMA_ERRORS != 0 {
            // Preserve error evidence. Do not reset shared FIFOs with possible
            // outstanding commands; let the caller stop/reinitialize the ring.
            self.dump_ibi_state();
            return Err("DMA transfer error/abort while waiting for IBI");
        }
        let ack = interrupt & (INTR_IBI_READY | INTR_IBI_RING_FULL);
        if ack != 0 {
            if let Some(trace) = self.trace {
                trace(format_args!("got ibi interrupt: {}", interrupt));
            }
            self.regs.write_dma_ring_intr_status(ack);
        }
        if interrupt & INTR_IBI_RING_FULL != 0 {
            self.dump_ibi_state();
            // Drain complete events below to free space, even when full.
        }
        let enq = ((self.regs.dma_ring_operation2() >> 16) & 0xff) as usize;
        if enq >= IBI_STATUS_RING_ENTRIES {
            return Err("invalid IBI enqueue pointer");
        }
        if enq == self.ibi_deq {
            return Ok(None);
        }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        // Collect a complete sequence, as in Linux hci_dma_process_ibi().
        let mut idx = self.ibi_deq;
        let mut total_chunks = 0usize;
        let mut size = 0usize;
        let mut addr = None;
        let mut bad = false;
        let mut rnw = None;
        let mut complete = false;
        while idx != enq {
            let ptr = self.ibi_status_ptr(idx);
            let status = unsafe { read_volatile(ptr) };

            if let Some(trace) = self.trace {
                trace(format_args!("IBI DMA status[{}]={:08x}", idx, status));
            }

            let a = ((status >> 9) & 0x7f) as u8;
            let read = status & (1 << 8) != 0;
            if rnw.is_some_and(|previous| previous != read) {
                bad = true;
            }
            rnw = Some(read);
            let chunks = ((status >> 16) & 0xff) as usize;
            let last = status & (1 << 24) != 0;
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
            if status & (1 << 30) != 0 {
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
                return Err("invalid IBI chunk count");
            }
            total_chunks += chunks;
            if last {
                let tail = (status & 0xff) as usize;
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
        // This API delivers target interrupts only. Consume unsupported
        // hot-join/controller-role requests without passing them to MCTP.
        let target_interrupt = rnw == Some(true);
        let oversized = size > output.len();
        let copy_len = if bad || oversized || !target_interrupt {
            0
        } else {
            size
        };
        self.copy_ibi_data(&mut output[..copy_len]);
        // Complete payload reads before returning ownership to hardware.
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        // Reclaim even discarded complete events, preserving command pointers.
        self.advance_ibi_status(idx, total_chunks);
        if bad {
            return Err("IBI controller reported an error or malformed status");
        }
        if !target_interrupt {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "Discarded unsupported IBI event: addr={:02x} rnw=0",
                    addr.unwrap_or(0)
                ));
            }
            return Ok(None);
        }
        if oversized {
            return Err("IBI output buffer too small (event reclaimed)");
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
            target_addr: addr.unwrap_or(0),
            payload_len: size,
        }))
    }

    /// Queue a v1 target DMA descriptor without waiting for a bus transaction.
    /// `command_word` is the target-format command (not a controller v1 command).
    /// Each outstanding slot owns a payload allocation sized to its command.
    pub fn target_queue_command(&mut self, command_word: u32, data: &[u8]) -> DmaResult<usize> {
        if self.mode != DmaMode::Target {
            return Err("target DMA command requires HCI1 target ring");
        }
        if data.len() > 0xffff || (command_word >> 16) as usize != data.len() {
            return Err("target DMA command/payload length mismatch");
        }
        let slot = self.next;
        if self.target_queued[slot] {
            return Err("target DMA command ring full");
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
        let next = (slot + 1) % ENTRIES;
        let op = self.regs.dma_ring_operation1();
        self.regs
            .write_dma_ring_operation1((op & !0xff) | next as u32);
        self.target_queued[slot] = true;
        self.next = next;
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
            addr_of_mut!((*self.memory).ibi_status)
                .cast::<u8>()
                .add(index * self.ibi_stride)
                .cast::<u32>()
        }
    }

    /// Copy from the circular chunk pool before returning ownership to hardware.
    fn copy_ibi_data(&self, output: &mut [u8]) {
        let base = unsafe { addr_of_mut!((*self.memory).ibi_data).cast::<u8>() };
        for (i, byte) in output.iter_mut().enumerate() {
            let offset =
                (self.ibi_chunk_ptr * IBI_CHUNK_SIZE + i) % (IBI_CHUNK_POOL_SIZE * IBI_CHUNK_SIZE);
            *byte = unsafe { read_volatile(base.add(offset)) };
        }
    }

    fn advance_ibi_status(&mut self, next_deq: usize, chunks: usize) {
        self.ibi_deq = next_deq;

        self.regs.write_dma_ring_operation1(
            (self.regs.dma_ring_operation1() & !(0xff << 16)) | ((next_deq as u32) << 16),
        );

        if chunks > 0 {
            self.ibi_chunk_ptr = (self.ibi_chunk_ptr + chunks) % IBI_CHUNK_POOL_SIZE;

            self.regs
                .write_dma_ring_chunk_control(self.regs.dma_ring_chunk_control() + chunks as u32);
        }
    }

    pub fn target_poll_rx(&mut self, output: &mut [u8]) -> DmaResult<Option<DmaTargetRx>> {
        if self.mode != DmaMode::Target {
            return Err("target RX polling requires HCI1 target ring");
        }

        let intr = self.regs.dma_ring_intr_status();
        if intr & DMA_ERRORS != 0 {
            self.snapshot();
            return Err("target DMA RX error");
        }

        // OP2[23:16] is the hardware status enqueue pointer.
        let enq = ((self.regs.dma_ring_operation2() >> 16) & 0xff) as usize;

        if enq >= IBI_STATUS_RING_ENTRIES {
            return Err("invalid target RX status enqueue pointer");
        }

        if enq == self.ibi_deq {
            return Ok(None);
        }

        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Process one target response descriptor per call.
        let idx = self.ibi_deq;
        let ptr = self.ibi_status_ptr(idx);

        let raw = unsafe { read_volatile(ptr) };

        // MIPI I3C HCI target response descriptor.
        let status = ((raw >> 28) & 0x0f) as u8;
        let received = (raw & (1 << 27)) != 0;
        let ccc = (raw & (1 << 26)) != 0;
        let ccc_hdr = ((raw >> 16) & 0xff) as u8;
        let len = (raw & 0xffff) as usize;

        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI1 target RX[{}] raw={:08x} status={} received={} ccc={} hdr={:02x} len={}",
                idx, raw, status, received, ccc, ccc_hdr, len
            ));
        }

        if status != 0 {
            return Err("target RX response reports error");
        }

        if !received {
            return Err("unexpected target RX transfer direction");
        }

        // Target-mode chunk count comes from data length,
        // not from bits [23:16].
        let chunks = (len + IBI_CHUNK_SIZE - 1) / IBI_CHUNK_SIZE;

        if chunks > IBI_CHUNK_POOL_SIZE {
            return Err("target RX chunk count invalid");
        }

        if len > output.len() {
            return Err("target RX output buffer too small");
        }

        self.copy_ibi_data(&mut output[..len]);

        // Finish reading DMA memory before returning ownership.
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }

        let next = (idx + 1) % IBI_STATUS_RING_ENTRIES;

        self.advance_ibi_status(next, chunks);

        self.regs
            .write_dma_ring_intr_status(intr & (INTR_IBI_READY | INTR_IBI_RING_FULL));

        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI1 RX complete: raw={:08x} ccc={} hdr={:02x} len={} chunks={}",
                raw, ccc, ccc_hdr, len, chunks
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
        if self.mode != DmaMode::Target {
            return Err("target response polling requires HCI1 target ring");
        }

        let intr = self.regs.dma_ring_intr_status();

        if intr & DMA_ERRORS != 0 {
            self.snapshot();
            return Err("target DMA transfer error or abort");
        }

        let slot = self.target_resp_deq;

        if !self.target_queued[slot] {
            return Ok(None);
        }

        // Zephyr: RING_OP2_CR_DEQ_PTR.
        // OP2[7:0] is the hardware command dequeue pointer.
        let hw_deq = (self.regs.dma_ring_operation2() & 0xff) as usize;

        if hw_deq >= ENTRIES {
            return Err("invalid target DMA command dequeue pointer");
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

        let next = (slot + 1) % ENTRIES;

        // OP1[15:8] = software command/response dequeue pointer.
        self.regs.write_dma_ring_operation1(
            (self.regs.dma_ring_operation1() & !0xff00) | ((next as u32) << 8),
        );

        self.target_queued[slot] = false;
        self.payloads[slot] = None;
        self.target_resp_deq = next;

        self.regs.write_dma_ring_intr_status(intr & (1 << 11));

        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI1 target DMA response slot={} raw={:08x} tid={} status={} len={} hw_deq={}",
                slot, result.raw, result.tid, result.status, result.data_len, hw_deq,
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
        // Leave ENABLE set so command/response pointer positions are retained.
        self.regs
            .write_dma_ring_control(self.regs.dma_ring_control() & !2);
        self.wait(I3cHciRegisters::dma_ring_status, RUNNING, 0)
            .map_err(|_| I3cHciError::Timeout)?;
        // ASPEED DMA reuses the PIO FIFOs. Wait for both DMA engines before
        // resetting them, as in Linux aspeed_pio_fifo_reset().
        self.wait_aspeed_dma_idle()
            .map_err(|_| I3cHciError::Timeout)?;
        let mask = constants::CMD_QUEUE_RST
            | constants::RESP_QUEUE_RST
            | constants::TX_FIFO_RST
            | constants::RX_FIFO_RST;
        regs.write_reset_control(mask);
        let mut reset = false;
        for _ in 0..POLL_BUDGET {
            if regs.reset_control() & mask == 0 {
                reset = true;
                break;
            }
            core::hint::spin_loop();
        }
        if !reset {
            return Err(I3cHciError::Timeout);
        }
        self.regs.write_dma_ring_intr_status(DMA_ERRORS);
        regs.set_hc_control(HC_CONTROL_RESUME);
        self.regs.write_dma_ring_control(ENABLE_RUN);
        self.wait(I3cHciRegisters::dma_ring_status, 3, 3)
            .map_err(|_| I3cHciError::Timeout)?;
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
        self.wait(I3cHciRegisters::dma_ring_status, 3, 0)
    }
}

fn dma_address(ptr: *const u8, len: usize) -> DmaResult<u32> {
    let start = ptr as usize;
    if start < 0x000a0000 || start.checked_add(len).is_none_or(|end| end > 0x000c0000) {
        return Err("DMA allocation is outside reserved SRAM");
    }
    (start as u32)
        .checked_add(DMA_BUS_OFFSET)
        .ok_or("DMA address overflow")
}

#[cfg(test)]
mod command_layout_tests {
    use super::super::cmd::{HciCommandKind, HciCommandV2};
    use super::*;

    #[test]
    fn payloads_larger_than_64_bytes_use_available_pool() {
        let live = [None; ENTRIES];
        assert_eq!(payload_offset(&live, 4096), Some(0));
        assert_eq!(payload_offset(&live, DMA_PAYLOAD_POOL_SIZE), Some(0));
        assert_eq!(payload_offset(&live, DMA_PAYLOAD_POOL_SIZE + 1), None);
        assert_eq!(payload_offset(&live, usize::MAX), None);
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
        assert_eq!(payload_offset(&live, 4096), Some(3072));
        live[1] = None;
        assert_eq!(payload_offset(&live, 512), Some(0));
        assert_eq!(payload_offset(&live, 2048), Some(3072));
    }

    #[test]
    fn payload_pool_exhaustion_keeps_outstanding_storage_reserved() {
        let live = [
            Some(PayloadAllocation {
                offset: 0,
                len: DMA_PAYLOAD_POOL_SIZE,
            }),
            None,
            None,
            None,
        ];
        assert_eq!(payload_offset(&live, 4), None);
        assert_eq!(payload_offset(&live, 0), Some(0));
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
