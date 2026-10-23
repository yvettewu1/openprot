// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 I3C HCI IBI self-test testing both polling and irq.
//!
//! Topology:
//!     HCI0 = master/controller side: IRQ-latched receive path
//!     HCI1 = target side: polling/request path
//! target-side generation is polling-style, while master-side receive is IRQ-latched.
//!
//! The two I3C ports must be physically connected.

#![no_std]
#![no_main]

use ast10x0_i3c_hci::pio::STAT_IBI_STATUS_THLD;
use ast10x0_i3c_hci::{
    DatAllocator, HciDeviceTable, HciIbiEvent, HciTargetConfig, I3cHciConfig, I3cHciCore,
    I3cHciMaster, I3cHciTarget, IoMode, IsrCtx, IsrEvents, dispatch_i3c_hci_irq, isr_events,
    register_i3c_hci_irq_handler,
};
use ast10x0_scu_pinctrl::{ScuRegisters, pinctrl};

const MASTER_BUS: u8 = 0;
const TARGET_BUS: u8 = 1;
const I3C_SCL_HZ: u32 = 12_500_000;
const TIMEOUT_US: u32 = 1_000_000;
const ADDRESS_INDEXED_DAT: bool = true;
const MAX_I3C_DEVICES: usize = 4;
const TARGET_PID: u64 = 0x1234_5678_9abd;
const TARGET_BCR: u8 = 0x02;
const TARGET_DCR: u8 = 0x00;

#[unsafe(no_mangle)]
pub extern "C" fn interrupt_handler_96() {
    dispatch_i3c_hci_irq(MASTER_BUS as usize);
}

#[unsafe(no_mangle)]
pub extern "C" fn interrupt_handler_97() {
    dispatch_i3c_hci_irq(TARGET_BUS as usize);
}

use bare_metal::write_all as console_backend_write_all;

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(message) }
}
fn expect_target_ibi(
    event: HciIbiEvent,
    assigned_addr: u8,
    expected_payload_len: usize,
) -> Result<(), &'static str> {
    match event {
        HciIbiEvent::TargetInterrupt {
            device,
            status,
            payload_len,
        } => {
            bare_metal::info!(
                "IBI received: DA={:02x} len={} raw={:08x}",
                status.target_addr as u32,
                payload_len as u32,
                status.raw as u32,
            );

            expect_true(device.dynamic_addr == assigned_addr, "IBI source mismatch")?;
            expect_true(
                status.target_addr == assigned_addr,
                "IBI status address mismatch",
            )?;
            expect_true(
                payload_len == expected_payload_len,
                "unexpected IBI payload",
            )?;
        }
        HciIbiEvent::HotJoin { .. } => return Err("unexpected hot-join IBI"),
        HciIbiEvent::Unknown { .. } => return Err("unknown IBI source"),
    }

    Ok(())
}

fn wait_for_master_ibi_irq(events: &IsrEvents) -> Result<(), &'static str> {
    let mut remaining = TIMEOUT_US;

    loop {
        let status = events.take_io_status();
        if (status & STAT_IBI_STATUS_THLD) != 0 {
            return Ok(());
        }

        if remaining == 0 {
            return Err("timed out waiting for IBI IRQ latch");
        }

        remaining -= 1;
    }
}

fn run_ibi_test() -> Result<(), &'static str> {
    bare_metal::info!("=== AST1080 I3C HCI IBI self-test ===");

    let scu = unsafe { ScuRegisters::new_global_unlocked() };
    scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);
    scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI1);

    let mut target_core = unsafe { I3cHciCore::new(I3cHciConfig::pio(TARGET_BUS), |_us| {}) }
        .map_err(|_| "target HCI core create failed")?;
    target_core
        .init()
        .map_err(|_| "target HCI core init failed")?;

    let target_config = HciTargetConfig {
        pid: TARGET_PID,
        bcr: TARGET_BCR,
        dcr: TARGET_DCR,
        static_addr: None,
    };

    let mut target = I3cHciTarget::new(target_core.regs(), |_us| {}).with_timeout_us(TIMEOUT_US);
    target
        .configure(&target_config)
        .map_err(|_| "target identity configure failed")?;
    target.enable().map_err(|_| "target mode enable failed")?;

    expect_true(target.role().target, "HCI1 did not enter target mode")?;
    expect_true(
        target.dynamic_addr().is_none(),
        "target unexpectedly has dynamic address before ENTDAA",
    )?;

    let mut master_core = unsafe { I3cHciCore::new(I3cHciConfig::pio(MASTER_BUS), |_us| {}) }
        .map_err(|_| "master HCI core create failed")?;
    master_core
        .init()
        .map_err(|_| "master HCI core init failed")?;

    let sections = master_core.sections();
    let dat_entries = if sections.dat.entries > 128 {
        128
    } else {
        sections.dat.entries as u8
    };

    let mut dat_allocator = DatAllocator::new(dat_entries, ADDRESS_INDEXED_DAT);
    let mut device_table = HciDeviceTable::<MAX_I3C_DEVICES>::new();
    let mut master = I3cHciMaster::new(master_core.regs(), sections, I3C_SCL_HZ, |_us| {})
        .with_timeout_us(TIMEOUT_US);

    let count = master
        .entdaa_all(&mut device_table, &mut dat_allocator)
        .map_err(|_| "ENTDAA failed")?;
    expect_true(count != 0, "ENTDAA discovered no targets")?;

    let target_device = device_table
        .by_pid(TARGET_PID)
        .ok_or("target PID not found after ENTDAA")?;
    let assigned_addr = target_device.dynamic_addr;

    expect_true(
        target.dynamic_addr() == Some(assigned_addr),
        "master/target dynamic address mismatch",
    )?;

    bare_metal::info!(
        "target discovered: DAT={} DA={:02x}",
        target_device.dat_index as u32,
        assigned_addr as u32,
    );

    master
        .enable_ibi(target_device, false)
        .map_err(|_| "master enable IBI failed")?;

    bare_metal::info!("master enabled IBI for DA={:02x}", assigned_addr as u32);

    /*
     * First prove the direct PIO polling datapath.
     */
    target
        .request_ibi()
        .map_err(|_| "target polling IBI request failed")?;

    bare_metal::info!("target requested polling IBI");

    let mut ibi_payload = [0u8; 8];
    let event = master
        .next_ibi_event(&device_table, &mut ibi_payload)
        .map_err(|_| "master failed to receive polling IBI")?;

    expect_target_ibi(event, assigned_addr, 0)?;

    bare_metal::info!("IBI polling PASS");

    /*
     * Then prove the ISR latch path. The board vector can call
     * dispatch_i3c_hci_irq() directly; this self-test enables the hardware
     * interrupt after registering its context so the HCI ISR service and IsrEvents latch
     * are covered without adding another IBI queue.
     */
    let irq_ctx = unsafe { IsrCtx::new(MASTER_BUS, IoMode::Pio) }
        .ok_or("master IRQ context create failed")?;

    expect_true(
        register_i3c_hci_irq_handler(MASTER_BUS as usize, irq_ctx),
        "master IRQ handler registration failed",
    )?;

    let master_irq_events = isr_events(MASTER_BUS as usize).ok_or("master IRQ events missing")?;

    master.enable_ibi_irq_events();
    // The context is registered and the bare-metal vector routes IRQ 96 to bus 0.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(ast1080_pac::Interrupt::i3c);
    }

    target
        .request_ibi()
        .map_err(|_| "target IRQ IBI request failed")?;

    bare_metal::info!("target requested IRQ IBI");

    wait_for_master_ibi_irq(master_irq_events)?;

    let event = master
        .next_ibi_event(&device_table, &mut ibi_payload)
        .map_err(|_| "master failed to receive IRQ IBI")?;

    expect_target_ibi(event, assigned_addr, 0)?;

    bare_metal::info!("IBI IRQ PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    let sentinel: &[u8] = match run_ibi_test() {
        Ok(()) => b"TEST_RESULT:PASS\n",
        Err(error) => {
            bare_metal::error!("I3C HCI IBI test failed: {}", error as &str);
            b"TEST_RESULT:FAIL\n"
        }
    };

    console_backend_write_all(sentinel);

    #[expect(clippy::empty_loop)]
    loop {}
}
