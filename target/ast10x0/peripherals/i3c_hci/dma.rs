// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI DMA ring support including IBI reception.
use core::arch::asm;
use super::core::{HciSections, I3cHciCore};
use super::{HciCommandV1, HciResponse};
use super::constants::{HC_CONTROL_BUS_ENABLE, HC_CONTROL_PIO_MODE, HC_CONTROL_RESUME};
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use super::error::{I3cHciError, Result};
use super::registers::I3cHciRegisters;

pub const RHS_CONTROL_MAX_HEADER_COUNT_SHIFT: u32 = 0;
pub const RHS_CONTROL_MAX_HEADER_COUNT_MASK: u32 = 0x0f << RHS_CONTROL_MAX_HEADER_COUNT_SHIFT;
pub const RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT: u32 = 4;
pub const RHS_CONTROL_MAX_HEADER_COUNT_CAP_MASK: u32 =
    0x0f << RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT;
pub const RHS_CONTROL_HEADER_SIZE_SHIFT: u32 = 16;
pub const RHS_CONTROL_HEADER_SIZE_MASK: u32 = 0xff << RHS_CONTROL_HEADER_SIZE_SHIFT;
pub const RHS_CONTROL_PREAMBLE_SIZE_SHIFT: u32 = 24;
pub const RHS_CONTROL_PREAMBLE_SIZE_MASK: u32 = 0xff << RHS_CONTROL_PREAMBLE_SIZE_SHIFT;

pub const RHS_INTR_RING0: u32 = 1 << 0;
pub const RHS_INTR_ALL_RINGS: u32 = 0xff;

const RH_CHUNK_CONTROL: usize = 0x08;

const RH_IBI_STATUS_RING_BASE_LO: usize = 0x40;
const RH_IBI_STATUS_RING_BASE_HI: usize = 0x44;
const RH_IBI_DATA_RING_BASE_LO: usize = 0x48;
const RH_IBI_DATA_RING_BASE_HI: usize = 0x4c;

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
// Board-specific: current AST10x0 DMA drivers pass non-cacheable SRAM pointers
// directly. Confirm this mapping for the actual AST1040 revision before running.
const DMA_BUS_OFFSET: u32 = 0;

const CR_SETUP: usize = 0x00;
const IBI_SETUP: usize = 0x04;
const INTR_STATUS: usize = 0x10;
const INTR_STATUS_ENABLE: usize = 0x14;
const INTR_SIGNAL_ENABLE: usize = 0x18;
const RING_STATUS: usize = 0x20;
const RING_CONTROL: usize = 0x24;
const OP1: usize = 0x28;
const OP2: usize = 0x2c;
const CMD_BASE_LO: usize = 0x30;
const CMD_BASE_HI: usize = 0x34;
const RESP_BASE_LO: usize = 0x38;
const RESP_BASE_HI: usize = 0x3c;
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
    responses: [0; ENTRIES * MAX_STRIDE / 4], tx: [0; 64], rx: [0; 64],
    ibi_status: [0; IBI_STATUS_RING_ENTRIES * IBI_STATUS_MAX_STRIDE / 4],
    ibi_data: [0; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE],
};

pub struct Ring {
    base: usize,
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
    pub fn with_trace(
        mut self,
        trace: fn(core::fmt::Arguments<'_>),
    ) -> Self {
        self.trace = Some(trace);
        self
    }
    fn dump_dma_state(&self, stage: &str) {
        if let Some(trace) = self.trace {
            trace(format_args!(
                "{}: RING_STATUS={:08x} INTR={:08x} OP1={:08x} OP2={:08x}",
                stage,
                self.read(RING_STATUS),
                self.read(INTR_STATUS),
                self.read(OP1),
                self.read(OP2),
            ));
        }
    }
    fn read(&self, offset: usize) -> u32 {
        // SAFETY: init validates the discovered ring header inside HCI0 MMIO.
        unsafe { read_volatile((self.base + offset) as *const u32) }
    }
    fn write(&self, offset: usize, value: u32) {
        unsafe { write_volatile((self.base + offset) as *mut u32, value) }
    }
    fn snapshot(&self) {
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA ring status={:08x} intr={:08x} op1={:08x} op2={:08x}",
                self.read(RING_STATUS),
                self.read(INTR_STATUS),
                self.read(OP1),
                self.read(OP2)
            ));
        }
    }
    fn wait(&self, offset: usize, mask: u32, expected: u32) -> DmaResult<()> {
        for _ in 0..POLL_BUDGET {
            if self.read(offset) & mask == expected { return Ok(()); }
            core::hint::spin_loop();
        }
        self.snapshot();
        Err("DMA ring control timeout")
    }

    pub fn init(
        core: &I3cHciCore<impl FnMut(u32)>,
    ) -> DmaResult<Self> {
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
        let sections = core.sections();
        let hci_base = ast1080_pac::I3c::ptr() as usize;
        let rhs = sections.ring_headers_offset as usize;
        if rhs == 0 || rhs > 0xff8 { return Err("invalid RHS section"); }
        // No transfers or IBIs may be pending at this PIO-to-DMA transition.
        regs.clear_hc_control(HC_CONTROL_BUS_ENABLE);
        init_rhs(regs, sections, 1).map_err(|_| "RHS setup failed")?;
        let offset = unsafe { read_volatile((hci_base + rhs + 4) as *const u32) } as usize;
        if offset == 0 || offset & 3 != 0 || offset > 0xfa0 {
            return Err("invalid ring-0 register offset");
        }
        let mut ring = Self { base: hci_base + offset, memory: addr_of_mut!(DMA_MEMORY),
            command_stride: 0, response_stride: 0, next: 0,
            ibi_stride: 0, ibi_deq: 0, ibi_chunk_ptr: 0, trace, };
        if ring.read(RING_STATUS) & 3 != 0 { return Err("ring already active"); }
        let setup = ring.read(CR_SETUP);
        ring.command_stride = ((setup >> 24) & 0xff) as usize;
        ring.response_stride = ((setup >> 16) & 0xff) as usize;
        if ring.command_stride < 20 || ring.command_stride > MAX_STRIDE || ring.command_stride % 4 != 0
            || ring.response_stride < 4 || ring.response_stride > MAX_STRIDE || ring.response_stride % 4 != 0 {
            return Err("unsupported DMA descriptor sizes");
        }
        // NOLOAD SRAM is not zeroed by the shared startup. Explicitly initialize
        // it before the DMA engine owns any descriptor or buffer.
        unsafe { core::ptr::write_bytes(ring.memory, 0, 1); }
        let cmd_addr = dma_address(unsafe { addr_of_mut!((*ring.memory).commands).cast() }, ENTRIES * MAX_STRIDE)?;
        let resp_addr = dma_address(unsafe { addr_of_mut!((*ring.memory).responses).cast() }, ENTRIES * MAX_STRIDE)?;
        ring.write(CMD_BASE_LO, cmd_addr);
        ring.write(CMD_BASE_HI, 0);
        ring.write(RESP_BASE_LO, resp_addr);
        ring.write(RESP_BASE_HI, 0);
        ring.write(CR_SETUP, ENTRIES as u32);
        // Configure the independent IBI status and data DMA rings.
        let ibi_hw_setup = ring.read(IBI_SETUP);
        ring.ibi_stride = ((ibi_hw_setup >> 24) & 0xff) as usize;
        if ring.ibi_stride < 4 || ring.ibi_stride > IBI_STATUS_MAX_STRIDE
            || ring.ibi_stride % 4 != 0 {
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
        ring.write(RH_IBI_STATUS_RING_BASE_LO, status_addr);
        ring.write(RH_IBI_STATUS_RING_BASE_HI, 0);
        ring.write(RH_IBI_DATA_RING_BASE_LO, data_addr);
        ring.write(RH_IBI_DATA_RING_BASE_HI, 0);
        // ASPEED uses 4-byte chunks: log2(4) - 2 = 0.
        ring.write(IBI_SETUP,
            ((IBI_STATUS_RING_ENTRIES as u32) << 16) | IBI_CHUNK_POOL_SIZE as u32);

        if let Some(trace) = ring.trace {
            trace(format_args!(
                "IBI DMA status={:08x} data={:08x} stride={}",
                status_addr,
                data_addr,
                ring.ibi_stride
            ));
        }
        ring.write(INTR_SIGNAL_ENABLE, 0);
        ring.write(INTR_STATUS_ENABLE, u32::MAX);
        ring.write(INTR_STATUS, u32::MAX);
        ring.write(OP1, 0);
        if ring.read(OP2) & 0xff != 0 { return Err("hardware dequeue pointer is not reset"); }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        regs.clear_hc_control(HC_CONTROL_PIO_MODE);
        regs.set_hc_control(HC_CONTROL_BUS_ENABLE | HC_CONTROL_RESUME);
        ring.write(RING_CONTROL, ENABLE_RUN);
        ring.wait(RING_STATUS, 3, 3)?;
        if let Some(trace) = ring.trace {
            trace(format_args!(
                "DMA ring0 offset={:04x} cmd_stride={} resp_stride={} cmd={:08x} resp={:08x}",
                offset,
                ring.command_stride,
                ring.response_stride,
                cmd_addr,
                resp_addr
            ));
        }
        Ok(ring)
    }
    pub fn transfer(&mut self, command: HciCommandV1, tid: u8, read: bool,  xfer_data: &[u8] ) -> DmaResult<HciResponse> {
        let len = xfer_data.len();
        if len > 64 { return Err("DMA transfer exceeds 64-byte buffer"); }
        let slot = self.next;
        if self.read(OP2) & 0xff != slot as u32 { return Err("DMA ring has outstanding work"); }
        let buffer = unsafe {
            if read { addr_of_mut!((*self.memory).rx).cast::<u8>() }
            else { addr_of_mut!((*self.memory).tx).cast::<u8>() }
        };
        for i in 0..64 {
            let value = if read { 0xa5 } else { xfer_data.get(i).copied().unwrap_or(0) };
            unsafe { write_volatile(buffer.add(i), value); }
        }
       let address = dma_address(buffer, 64)?;

        // Print the actual DMA buffer contents.
        if !read {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA TX SRAM address={:08x} len={}",
                    address,
                    len
                ));

                for i in 0..len {
                    let value = unsafe {
                        read_volatile(buffer.add(i))
                    };

                    trace(format_args!(
                        "DMA TX[{}]={:02x}",
                        i,
                        value
                    ));
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
        // FIX 3: Print descriptor and response SRAM addresses.
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
        let words = [
            command.words[0],
            command.words[1],
            (1 << 30) | len as u32,
            address,
            0,
        ];

        // Write the complete descriptor.
        for i in 0..self.command_stride / 4 {
            unsafe {
                write_volatile(
                    descriptor.add(i),
                    words.get(i).copied().unwrap_or(0),
                );
            }
        }

        // Read back the actual descriptor from SRAM.
        if let Some(trace) = self.trace {
            for i in 0..5 {
                let value = unsafe {
                    read_volatile(descriptor.add(i))
                };

                trace(format_args!(
                    "DMA DESC[{}]={:08x}",
                    i,
                    value
                ));
            }
        }

        unsafe { write_volatile(response, 0xffff_ffff); }
        self.write(INTR_STATUS, DMA_ERRORS | (1 << 0));
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        self.dump_dma_state("BEFORE SUBMIT");
        let next = (slot + 1) % ENTRIES;
        let op = self.read(OP1);
        self.write(OP1, (op & !0xff) | next as u32);
        self.dump_dma_state("AFTER SUBMIT");

        let mut completed = false;
        for _ in 0..POLL_BUDGET {
            if self.read(INTR_STATUS) & DMA_ERRORS != 0 {
                self.snapshot();
                return Err("DMA transfer error or abort");
            }
            if self.read(OP2) & 0xff == next as u32 { completed = true; break; }
            core::hint::spin_loop();
        }
        if !completed { self.snapshot(); return Err("DMA transfer completion timeout"); }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        let result = HciResponse::parse(unsafe { read_volatile(response) });
        self.dump_dma_state("AFTER RESPONSE");
        
        self.write(OP1, (self.read(OP1) & !0xff00) | ((next as u32) << 8));
        self.next = next;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DMA {} response={:08x} tid={} status={} length={}",
                if read { "READ" } else { "WRITE" },
                result.raw,
                result.tid,
                result.status,
                result.data_len
            ));
        }
        if result.tid != tid { return Err("DMA response TID mismatch"); }
        if !result.success() { return Err("DMA response reports transfer failure"); }
        if read {
            if result.data_len != len { return Err("DMA read length mismatch"); }
            for i in 0..len {
                if unsafe { read_volatile(buffer.add(i)) } != xfer_data[i] {
                    return Err("DMA read payload mismatch");
                }
            }
            for i in len..64 {
                if unsafe { read_volatile(buffer.add(i)) } != 0xa5 { return Err("DMA RX buffer guard overwritten"); }
            }
        } else if result.data_len != 0 { return Err("DMA write reports untransferred bytes"); }
        Ok(result)
    }
    /// Poll and consume one complete IBI. Returns None if no complete IBI is ready.
    /// The caller supplies storage; all ring management remains inside this driver.
    pub fn poll_ibi(&mut self, output: &mut [u8]) -> DmaResult<Option<DmaIbiEvent>> {
        if self.read(INTR_STATUS) & INTR_IBI_RING_FULL != 0 {
            return Err("IBI status ring full");
        }
        let enq = ((self.read(OP2) >> 16) & 0xff) as usize;
        if enq >= IBI_STATUS_RING_ENTRIES { return Err("invalid IBI enqueue pointer"); }
        if enq == self.ibi_deq { return Ok(None); }
        unsafe {
            asm!("dsb sy", options(nostack, preserves_flags));
        }
        // Collect a complete sequence, as in Linux hci_dma_process_ibi().
        let mut idx = self.ibi_deq;
        let mut total_chunks = 0usize;
        let mut size = 0usize;
        let mut addr = None;
        let mut bad = false;
        let mut complete = false;
        while idx != enq {
            let ptr = unsafe { addr_of_mut!((*self.memory).ibi_status).cast::<u8>()
                .add(idx * self.ibi_stride).cast::<u32>() };
            let status = unsafe { read_volatile(ptr) };
            let a = ((status >> 9) & 0x7f) as u8;
            let chunks = ((status >> 16) & 0xff) as usize;
            let last = status & (1 << 24) != 0;
            if status & (1 << 30) != 0 { bad = true; }
            if let Some(prev) = addr { if prev != a { bad = true; } }
            else { addr = Some(a); }
            if chunks > IBI_CHUNK_POOL_SIZE || total_chunks + chunks > IBI_CHUNK_POOL_SIZE {
                return Err("invalid IBI chunk count");
            }
            total_chunks += chunks;
            if last {
                let tail = (status & 0xff) as usize;
                if chunks > 0 && (tail == 0 || tail > IBI_CHUNK_SIZE) {
                    return Err("invalid final IBI chunk length");
                }
                size += if chunks == 0 { 0 } else { (chunks - 1) * IBI_CHUNK_SIZE + tail };
                complete = true;
            } else {
                size += chunks * IBI_CHUNK_SIZE;
            }
            idx = (idx + 1) % IBI_STATUS_RING_ENTRIES;
            if complete { break; }
        }
        if !complete { return Ok(None); }
        if size > output.len() { return Err("IBI output buffer too small"); }
        let ptr = unsafe { addr_of_mut!((*self.memory).ibi_data).cast::<u8>() };
        for (i, byte) in output[..size].iter_mut().enumerate() {
            let off = (self.ibi_chunk_ptr * IBI_CHUNK_SIZE + i)
                % (IBI_CHUNK_POOL_SIZE * IBI_CHUNK_SIZE);
            *byte = unsafe { read_volatile(ptr.add(off)) };
        }
        // Reclaim descriptors and data chunks, preserving command ring pointers.
        self.ibi_deq = idx;
        self.write(OP1, (self.read(OP1) & !(0xff << 16)) | ((idx as u32) << 16));
        self.ibi_chunk_ptr = (self.ibi_chunk_ptr + total_chunks) % IBI_CHUNK_POOL_SIZE;
        if total_chunks != 0 {
            let available = self.read(RH_CHUNK_CONTROL);
            self.write(RH_CHUNK_CONTROL, available + total_chunks as u32);
        }
        self.write(INTR_STATUS, INTR_IBI_READY);
        if bad { return Err("IBI controller reported an error or address mismatch"); }
        Ok(Some(DmaIbiEvent { target_addr: addr.unwrap_or(0), payload_len: size }))
    }

    pub fn stop(&self) -> DmaResult<()> {
        self.write(RING_CONTROL, self.read(RING_CONTROL) & !2);
        if self.wait(RING_STATUS, RUNNING, 0).is_err() {
            self.write(RING_CONTROL, self.read(RING_CONTROL) | ABORT);
            self.wait(RING_STATUS, RUNNING, 0)?;
        }
        self.write(RING_CONTROL, 0);
        self.wait(RING_STATUS, 3, 0)
    }
}

fn dma_address(ptr: *const u8, len: usize) -> DmaResult<u32> {
    let start = ptr as usize;
    if start < 0x000a0000 || start.checked_add(len).is_none_or(|end| end > 0x000c0000) {
        return Err("DMA allocation is outside non-cacheable SRAM");
    }
    (start as u32).checked_add(DMA_BUS_OFFSET).ok_or("DMA address overflow")
}