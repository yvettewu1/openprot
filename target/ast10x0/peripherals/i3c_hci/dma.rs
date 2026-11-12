// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI DMA ring support including IBI reception.
use super::constants::{HC_CONTROL_BUS_ENABLE, HC_CONTROL_PIO_MODE, HC_CONTROL_RESUME};
use super::core::{HciSections, I3cHciCore};
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

// Match the Zephyr ASPEED I3C vendor DMA hook for non-AST27XX:
// pass the buffer address unchanged. The generic G2 CM4 address translator
// is not called by that I3C hook. Keep cacheability separate from addressing.
const DMA_BUS_OFFSET: u32 = 0;

const ENABLE_RUN: u32 = 3;
const ABORT: u32 = 4;
const RUNNING: u32 = 2;
const DMA_ERRORS: u32 = (1 << 9) | (1 << 5);

#[repr(C, align(64))]
struct DmaMemory {
    commands: [u32; ENTRIES * MAX_STRIDE / 4],
    responses: [u32; ENTRIES * MAX_STRIDE / 4],
    tx: [u8; 64],
    rx: [u8; 64],

    ibi_status: [u32; IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE / 4],
    ibi_data: [u8; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE],
}

#[unsafe(link_section = ".dma_nc")]
static mut DMA_MEMORY: DmaMemory = DmaMemory {
    commands: [0; ENTRIES * MAX_STRIDE / 4],
    responses: [0; ENTRIES * MAX_STRIDE / 4],
    tx: [0; 64],
    rx: [0; 64],
    ibi_status: [0; IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE / 4],
    ibi_data: [0; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE],
};

pub struct Ring {
    regs: I3cHciRegisters,
    memory: *mut DmaMemory,
    command_stride: usize,
    response_stride: usize,
    next: usize,
    ibi_stride: usize,
    ibi_deq: usize,
    ibi_chunk_ptr: usize,
    trace: Option<fn(core::fmt::Arguments<'_>)>,
}

#[derive(Debug, Clone, Copy)]
pub struct DmaIbiEvent {
    pub target_addr: u8,
    pub payload_len: usize,
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
        if regs.bus() != 0 {
            return Err("DMA ring currently supports HCI0 only");
        }
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
            memory: addr_of_mut!(DMA_MEMORY),
            command_stride: 0,
            response_stride: 0,
            next: 0,
            ibi_stride: 0,
            ibi_deq: 0,
            ibi_chunk_ptr: 0,
            trace,
        };
        if ring.regs.dma_ring_status() & 3 != 0 {
            return Err("ring already active");
        }
        let (command_stride, response_stride) = ring.regs.dma_descriptor_sizes();
        ring.command_stride = command_stride as usize;
        ring.response_stride = response_stride as usize;
        if ring.command_stride < MIN_XFER_STRUCT_SIZE
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
                "DMA ring0 offset={:04x} cmd_stride={} resp_stride={} cmd={:08x} resp={:08x}",
                offset, ring.command_stride, ring.response_stride, cmd_addr, resp_addr
            ));
        }
        Ok(ring)
    }
    /// Submit without interpreting response status: DAA's final NACK is expected.
    pub(crate) fn submit_raw(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        validate_transfer(&transfer)?;
        let super::cmd::HciCommand::V1(command) = transfer.command else {
            return Err(I3cHciError::Unsupported);
        };
        if !command.expects_response() {
            return Err(I3cHciError::Unsupported);
        }
        // DAA uses the response length as an assignment count, not RX bytes.
        let read = command.read && command.kind != super::cmd::HciCommandKind::AddressAssignment;
        let len = if read {
            command.data_len
        } else {
            transfer.tx.len()
        };
        if len > 64 || (read && !transfer.tx.is_empty()) || (!read && !transfer.rx.is_empty()) {
            return Err(I3cHciError::InvalidArgs);
        }
        let placeholder = [0u8; 64];
        let data = if read {
            &placeholder[..len]
        } else {
            transfer.tx
        };
        let response = self
            .transfer_raw(command, command.tid, read, data)
            .map_err(|message| {
                if let Some(trace) = self.trace {
                    trace(format_args!("DMA submit failed: {}", message));
                }
                if message == "DMA transfer completion timeout"
                    || message == "DMA response visibility timeout"
                {
                    I3cHciError::Timeout
                } else {
                    I3cHciError::TransferStatus(self.regs.dma_ring_intr_status())
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
            *byte = unsafe { read_volatile(addr_of_mut!((*self.memory).rx).cast::<u8>().add(i)) };
        }
        Ok(HciTransferResult { response, rx_len })
    }

    pub fn transfer(
        &mut self,
        command: HciCommandV1,
        tid: u8,
        read: bool,
        xfer_data: &[u8],
    ) -> DmaResult<HciResponse> {
        let result = self.transfer_raw(command, tid, read, xfer_data)?;
        if !result.success() {
            return Err("DMA response reports transfer failure");
        }
        if read {
            if result.data_len != xfer_data.len() {
                return Err("DMA read length mismatch");
            }
            for (i, expected) in xfer_data.iter().enumerate() {
                let actual =
                    unsafe { read_volatile(addr_of_mut!((*self.memory).rx).cast::<u8>().add(i)) };
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
        command: HciCommandV1,
        read: bool,
        data: &[u8],
        address: u32,
    ) -> [u32; 5] {
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

            return [word0, immediate_data, 1 << 30, 0, 0];
        }

        // Existing Regular DMA descriptor.
        [
            command.words[0],
            command.words[1],
            (1 << 30) | len as u32,
            if len == 0 { 0 } else { address },
            0,
        ]
    }

    fn transfer_raw(
        &mut self,
        command: HciCommandV1,
        tid: u8,
        read: bool,
        xfer_data: &[u8],
    ) -> DmaResult<HciResponse> {
        let len = xfer_data.len();
        if len > 64 {
            return Err("DMA transfer exceeds 64-byte buffer");
        }
        let slot = self.next;
        if self.regs.dma_ring_operation2() & 0xff != slot as u32 {
            return Err("DMA ring has outstanding work");
        }
        let buffer = unsafe {
            if read {
                addr_of_mut!((*self.memory).rx).cast::<u8>()
            } else {
                addr_of_mut!((*self.memory).tx).cast::<u8>()
            }
        };
        for i in 0..64 {
            let value = if read {
                0xa5
            } else {
                xfer_data.get(i).copied().unwrap_or(0)
            };
            unsafe {
                write_volatile(buffer.add(i), value);
            }
        }
        let address = dma_address(buffer, 64)?;

        // Print the actual DMA buffer contents.
        if !read {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA TX SRAM address={:08x} len={}",
                    address, len
                ));

                for i in 0..len {
                    let value = unsafe { read_volatile(buffer.add(i)) };

                    trace(format_args!("DMA TX[{}]={:02x}", i, value));
                }
            }
        }

        let descriptor = unsafe {
            addr_of_mut!((*self.memory).commands)
                .cast::<u8>()
                .add(slot * self.command_stride)
                .cast::<u32>()
        };

        let response = unsafe {
            addr_of_mut!((*self.memory).responses)
                .cast::<u8>()
                .add(slot * self.response_stride)
                .cast::<u32>()
        };

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
        // HCI v1 master command:
        // Word 0-1: command descriptor
        // Word 2: Data Buffer Descriptor (length + IOC)
        // Word 3: DMA address low
        // Word 4: DMA address high
        self.dump_wdma_debug("BEFORE DMA SUBMIT");
        let words = Self::prepare_dma_command_words(command, read, xfer_data, address);

        // Write the complete descriptor.
        for i in 0..self.command_stride / 4 {
            unsafe {
                write_volatile(descriptor.add(i), words.get(i).copied().unwrap_or(0));
            }
        }

        // Read back the actual descriptor from SRAM.
        if let Some(trace) = self.trace {
            for i in 0..5 {
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
            //return Err("DMA response visibility timeout");
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
                command.kind,
                if command.read { "READ" } else { "WRITE" },
                result.raw,
                result.tid,
                result.status,
                result.data_len
            ));
        }
        
        if result.tid != tid {
            return Err("DMA response TID mismatch");
        }
        if read {
            if result.success() && result.data_len > len {
                return Err("DMA read exceeds requested buffer");
            }
            for i in len..64 {
                if unsafe { read_volatile(buffer.add(i)) } != 0xa5 {
                    return Err("DMA RX buffer guard overwritten");
                }
            }
        } 
        Ok(result)
    }
    /// Poll and consume one complete IBI. Returns None if no complete IBI is ready.
    /// The caller supplies storage; all ring management remains inside this driver.
    pub fn poll_ibi(&mut self, output: &mut [u8]) -> DmaResult<Option<DmaIbiEvent>> {
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
            let ptr = unsafe {
                addr_of_mut!((*self.memory).ibi_status)
                    .cast::<u8>()
                    .add(idx * self.ibi_stride)
                    .cast::<u32>()
            };
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
        let ptr = unsafe { addr_of_mut!((*self.memory).ibi_data).cast::<u8>() };
        for (i, byte) in output[..copy_len].iter_mut().enumerate() {
            let off =
                (self.ibi_chunk_ptr * IBI_CHUNK_SIZE + i) % (IBI_CHUNK_POOL_SIZE * IBI_CHUNK_SIZE);
            *byte = unsafe { read_volatile(ptr.add(off)) };
        }
        // Complete payload reads before returning ownership to hardware.
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        // Reclaim even discarded complete events, preserving command pointers.
        self.ibi_deq = idx;
        self.regs.write_dma_ring_operation1(
            (self.regs.dma_ring_operation1() & !(0xff << 16)) | ((idx as u32) << 16),
        );
        self.ibi_chunk_ptr = (self.ibi_chunk_ptr + total_chunks) % IBI_CHUNK_POOL_SIZE;
        if total_chunks != 0 {
            let available = self.regs.dma_ring_chunk_control();
            self.regs
                .write_dma_ring_chunk_control(available + total_chunks as u32);
        }
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
