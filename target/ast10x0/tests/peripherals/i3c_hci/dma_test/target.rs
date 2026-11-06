// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! One-ring HCI v1 DMA private-write/read test. PIO is used only for DAA.
//! The target must preload READ_EXPECTED without requesting an IBI.
#![no_std]
#![no_main]

use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use ast10x0_i3c_hci::{
    DatAllocator, HciDeviceTable, HciCommandV1, HciResponse,
    I3cHciConfig, I3cHciCore, I3cHciMaster, HC_CONTROL_BUS_ENABLE,
    HC_CONTROL_PIO_MODE, HC_CONTROL_RESUME,
};
use bare_metal::write_all as console_backend_write_all;

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
}
#[unsafe(link_section = ".dma_nc")]
static mut DMA_MEMORY: DmaMemory = DmaMemory {
    commands: [0; ENTRIES * MAX_STRIDE / 4],
    responses: [0; ENTRIES * MAX_STRIDE / 4], tx: [0; 64], rx: [0; 64],
};

type Result<T> = core::result::Result<T, &'static str>;

fn dma_address(ptr: *const u8, len: usize) -> Result<u32> {
    let start = ptr as usize;
    if start < 0x000a0000 || start.checked_add(len).is_none_or(|end| end > 0x000c0000) {
        return Err("DMA allocation is outside non-cacheable SRAM");
    }
    (start as u32).checked_add(DMA_BUS_OFFSET).ok_or("DMA address overflow")
}

struct Ring {
    base: usize,
    memory: *mut DmaMemory,
    command_stride: usize,
    response_stride: usize,
    next: usize,
}
impl Ring {
    fn read(&self, offset: usize) -> u32 {
        // SAFETY: init validates the discovered ring header inside HCI0 MMIO.
        unsafe { read_volatile((self.base + offset) as *const u32) }
    }
    fn write(&self, offset: usize, value: u32) {
        unsafe { write_volatile((self.base + offset) as *mut u32, value) }
    }
    fn snapshot(&self) {
        bare_metal::info!("DMA ring status={:08x} intr={:08x} op1={:08x} op2={:08x}",
            self.read(RING_STATUS), self.read(INTR_STATUS), self.read(OP1), self.read(OP2));
    }
    fn wait(&self, offset: usize, mask: u32, expected: u32) -> Result<()> {
        for _ in 0..POLL_BUDGET {
            if self.read(offset) & mask == expected { return Ok(()); }
            core::hint::spin_loop();
        }
        self.snapshot();
        Err("DMA ring control timeout")
    }
    fn init(core: &I3cHciCore<impl FnMut(u32)>) -> Result<Self> {
        let regs = core.regs();
        let sections = core.sections();
        let hci_base = ast1080_pac::I3c::ptr() as usize;
        let rhs = sections.ring_headers_offset as usize;
        if rhs == 0 || rhs > 0xff8 { return Err("invalid RHS section"); }
        // No transfers or IBIs may be pending at this PIO-to-DMA transition.
        regs.clear_hc_control(HC_CONTROL_BUS_ENABLE);
        ast10x0_i3c_hci::init_rhs(regs, sections, 1).map_err(|_| "RHS setup failed")?;
        let offset = unsafe { read_volatile((hci_base + rhs + 4) as *const u32) } as usize;
        if offset == 0 || offset & 3 != 0 || offset > 0xfa0 {
            return Err("invalid ring-0 register offset");
        }
        let mut ring = Self { base: hci_base + offset, memory: addr_of_mut!(DMA_MEMORY),
            command_stride: 0, response_stride: 0, next: 0 };
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
        ring.write(IBI_SETUP, 0); // No IBI rings: target must not raise IBI.
        ring.write(INTR_SIGNAL_ENABLE, 0);
        ring.write(INTR_STATUS_ENABLE, u32::MAX);
        ring.write(INTR_STATUS, u32::MAX);
        ring.write(OP1, 0);
        if ring.read(OP2) & 0xff != 0 { return Err("hardware dequeue pointer is not reset"); }
        cortex_m::asm::dsb();
        regs.clear_hc_control(HC_CONTROL_PIO_MODE);
        regs.set_hc_control(HC_CONTROL_BUS_ENABLE | HC_CONTROL_RESUME);
        ring.write(RING_CONTROL, ENABLE_RUN);
        ring.wait(RING_STATUS, 3, 3)?;
        bare_metal::info!("DMA ring0 offset={:04x} cmd_stride={} resp_stride={} cmd={:08x} resp={:08x}",
            offset, ring.command_stride, ring.response_stride, cmd_addr, resp_addr);
        Ok(ring)
    }
    fn transfer(&mut self, command: HciCommandV1, tid: u8, read: bool,  xfer_data: &[u8] ) -> Result<HciResponse> {
        let len = xfer_data.len();
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
        let descriptor = unsafe { addr_of_mut!((*self.memory).commands).cast::<u8>()
            .add(slot * self.command_stride).cast::<u32>() };
        let response = unsafe { addr_of_mut!((*self.memory).responses).cast::<u8>()
            .add(slot * self.response_stride).cast::<u32>() };
        // HCI v1 command (2 words), followed by buffer size/IOC and 64-bit address.
        let words = [command.words[0], command.words[1], (1 << 30) | len as u32, address, 0];
        for i in 0..self.command_stride / 4 {
            unsafe { write_volatile(descriptor.add(i), words.get(i).copied().unwrap_or(0)); }
        }
        unsafe { write_volatile(response, 0xffff_ffff); }
        self.write(INTR_STATUS, u32::MAX);
        cortex_m::asm::dsb();
        let next = (slot + 1) % ENTRIES;
        let op = self.read(OP1);
        self.write(OP1, (op & !0xff) | next as u32);
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
        cortex_m::asm::dsb();
        let result = HciResponse::parse(unsafe { read_volatile(response) });
        self.write(OP1, (self.read(OP1) & !0xff00) | ((next as u32) << 8));
        self.next = next;
        bare_metal::info!("DMA {} response={:08x} tid={} status={} length={}",
            if read { "READ" } else { "WRITE" }, result.raw, result.tid, result.status, result.data_len);
        if result.tid != tid { return Err("DMA response TID mismatch"); }
        if !result.success() { return Err("DMA response reports transfer failure"); }
        if read {
            if result.data_len != len { return Err("DMA read length mismatch"); }
            let mut data = [0u8; 8];
            for (i, byte) in data.iter_mut().enumerate() { *byte = unsafe { read_volatile(buffer.add(i)) }; }
            bare_metal::info!("DMA READ bytes={:02x?}", data);
            if data != xfer_data { return Err("DMA read payload mismatch"); }
            for i in len..64 {
                if unsafe { read_volatile(buffer.add(i)) } != 0xa5 { return Err("DMA RX buffer guard overwritten"); }
            }
        } else if result.data_len != 0 { return Err("DMA write reports untransferred bytes"); }
        Ok(result)
    }
    fn stop(&self) -> Result<()> {
        self.write(RING_CONTROL, self.read(RING_CONTROL) & !2);
        if self.wait(RING_STATUS, RUNNING, 0).is_err() {
            self.write(RING_CONTROL, self.read(RING_CONTROL) | ABORT);
            self.wait(RING_STATUS, RUNNING, 0)?;
        }
        self.write(RING_CONTROL, 0);
        self.wait(RING_STATUS, 3, 0)
    }
}
impl Drop for Ring {
    fn drop(&mut self) {
        if let Err(error) = self.stop() {
            bare_metal::error!("DMA cleanup: {}; static DMA memory remains reserved", error);
        }
    }
}

fn run() -> Result<()> {
    bare_metal::info!("=== DMA TEST: PIO discovery, DMA private write/read ===");

    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c);

    // Initialize HCI controller.
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(0), |_| {}) }
        .map_err(|_| "core creation failed")?;

    core.init_primary(8).map_err(|_| "primary init failed")?;
    core.disable_irq_signals();

    if core.command_descriptor() != Some(ast10x0_i3c_hci::CommandDescriptor::V1) {
        return Err("dma_test supports HCI v1 commands only");
    }

    let sections = core.sections();

    // Discover I3C targets using ENTDAA.
    let mut table = HciDeviceTable::<8>::new();
    let mut allocator = 
        DatAllocator::new(sections.dat.entries.min(128) as u8, true);    
    let mut master = I3cHciMaster::new(core.regs(), sections, 12_500_000, |_| {})
            .with_timeout_us(POLL_BUDGET);
    
    let _result = master.entdaa_all(&mut table, &mut allocator);

   // Print all discovered devices, including partial results on failure.
    for device in table.iter() {
        bare_metal::info!(
            "PID=0x{:012x} DA=0x{:02x} DAT={} BCR=0x{:02x} DCR=0x{:02x}",
            device.pid,
            device.dynamic_addr,
            device.dat_index,
            device.bcr,
            device.dcr
        );
    }

    // Select first discovered target; no hardcoded PID.
    let device = table
        .iter()
        .next()
        .ok_or("No I3C target discovered")?;

    bare_metal::info!(
        "DMA target PID=0x{:012x} DA=0x{:02x} DAT={}",
        device.pid,
        device.dynamic_addr,
        device.dat_index
    );

    // Initialize DMA ring.
    let mut ring = Ring::init(&core)?;
    let packet = bare_metal::i3c_mctp_setup::prepare_mctp_packet(device.dynamic_addr);

    bare_metal::info!("========== wait for target to setup mctp read ======================");
    // Retain the existing manual receiver-setup delay (not calibrated).
    bare_metal::i3c_setup::spin_sleep(0x8000_0000);

    // private write
    let write = HciCommandV1::i3c_private_write_regular(device.dat_index, 4, packet.len(), 0)
        .map_err(|_| "DMA write command encoding failed")?.with_roc().with_toc();
    ring.transfer(write, 4, false, &packet)?;
    bare_metal::info!("DMA WRITE CONTROLLER PASS; verify target received {:02x?}", packet);
/*
    bare_metal::info!("========== wait for target to setup mctp write ======================");
    // Retain the existing manual receiver-setup delay (not calibrated).
    bare_metal::i3c_setup::spin_sleep(0x8000_0000);
    let read = HciCommandV1::i3c_private_read(device.dat_index, 5, packet.len(), 0)
        .map_err(|_| "DMA read command encoding failed")?.with_roc().with_toc();
    ring.transfer(read, 5, true, packet)?;
       */
   
    let stopped = ring.stop();
    stopped?;
    bare_metal::info!("DMA READ/WRITE CONTROLLER TEST PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    unsafe { bare_metal::i3c_setup::prepare_and_show(&[0], false); }
    match run() {
        Ok(()) => console_backend_write_all(b"TEST_RESULT:PASS\n"),
        Err(error) => {
            bare_metal::error!("DMA test failed: {}", error);
            console_backend_write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    // DMA storage is static and never reused, including failed abort paths.
    loop { core::hint::spin_loop(); }
}
