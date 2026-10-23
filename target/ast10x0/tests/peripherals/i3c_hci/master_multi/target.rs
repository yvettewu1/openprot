// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_i3c_hci::dispatch_i3c_hci_irq;
use ast10x0_i3c_hci::{
    DatAllocator, HciDeviceTable, I3C_CCC_GETPID, I3cHciConfig, I3cHciCore, I3cHciMaster,
};
use ast10x0_scu_pinctrl::{ScuRegisters, pinctrl};
use cortex_m::peripheral::NVIC;

const I3C_BUS: u8 = 0;
const I3C_SCL_HZ: u32 = 12_500_000;
const DAA_TIMEOUT_US: u32 = 1_000_000;
const MAX_I3C_DEVICES: usize = 8;
const EXPECTED_MIN_DEVICES: usize = 2;
const ADDRESS_INDEXED_DAT: bool = true;

#[unsafe(no_mangle)]
pub extern "C" fn interrupt_handler_96() {
    dispatch_i3c_hci_irq(I3C_BUS as usize);
}

use bare_metal::write_all as console_backend_write_all;

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(message) }
}

fn run_master_multi_test() -> Result<(), &'static str> {
    bare_metal::info!("=== AST10x0 I3C HCI multi-device master test ===");

    // SAFETY: Test target runs once at boot with exclusive access to SCU.
    let scu = unsafe { ScuRegisters::new_global_unlocked() };
    scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);

    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(I3C_BUS), |_us| {}) }
        .map_err(|_| "HCI core create failed")?;
    core.init().map_err(|_| "HCI core init failed")?;
    let sections = core.sections();
    expect_true(sections.dat.entries > 0, "HCI DAT table missing")?;
    expect_true(sections.dct.entries > 0, "HCI DCT table missing")?;

    // enable irq
    let ctx = core
        .isr_ctx()
        .map_err(|_| "HCI ISR context create failed")?;

    if !ast10x0_i3c_hci::register_i3c_hci_irq_handler(I3C_BUS as usize, ctx) {
        return Err("HCI IRQ registration failed");
    }

    core.enable_irq_signals();
    // The bare-metal vector (IRQ 96 -> `interrupt_handler_96`) is in place and
    // the handler is registered; this integration layer owns the NVIC line
    // for the bus it selected (`I3C_BUS` = 0 -> `Interrupt::i3c0`), so unmask
    // it now.
    // SAFETY: handler registered and hardware initialized (Ready state);
    // unmasking cannot deliver an IRQ into partially-initialized state.
    unsafe {
        NVIC::unmask(ast1080_pac::Interrupt::i3c);
    }
    let dat_entries = if sections.dat.entries > 128 {
        128
    } else {
        sections.dat.entries as u8
    };
    let mut dat_allocator = DatAllocator::new(dat_entries, ADDRESS_INDEXED_DAT);
    let mut table = HciDeviceTable::<MAX_I3C_DEVICES>::new();
    let mut master = I3cHciMaster::new(core.regs(), sections, I3C_SCL_HZ, |_us| {})
        .with_timeout_us(DAA_TIMEOUT_US);

    let count = master
        .entdaa_all(&mut table, &mut dat_allocator)
        .map_err(|_| "ENTDAA all failed")?;
    bare_metal::info!("ENTDAA discovered {} I3C devices", count as u32);
    expect_true(
        count >= EXPECTED_MIN_DEVICES,
        "too few I3C devices discovered",
    )?;

    for device in table.iter() {
        bare_metal::info!(
            "I3C device da={:02x} dat={} pid={:04x}_{:08x} bcr={:02x} dcr={:02x}",
            device.dynamic_addr as u32,
            device.dat_index as u32,
            ((device.pid >> 32) & 0xffff) as u32,
            (device.pid & 0xffff_ffff) as u32,
            device.bcr as u32,
            device.dcr as u32
        );

        let mut pid = [0u8; 6];
        let result = master
            .ccc_direct_read_addr(&table, device.dynamic_addr, I3C_CCC_GETPID, None, &mut pid)
            .map_err(|_| "GETPID by dynamic address failed")?;
        expect_true(result.rx_len == pid.len(), "GETPID short read")?;

        let irq_status = ast10x0_i3c_hci::isr_events(I3C_BUS as usize)
            .ok_or("missing HCI ISR events")?
            .take_io_status();

        expect_true(
            irq_status & ast10x0_i3c_hci::pio::STAT_RESP_READY != 0,
            "PIO response IRQ not observed",
        )?;

        bare_metal::info!(
            "GETPID da={:02x} bytes={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            device.dynamic_addr as u32,
            pid[0] as u32,
            pid[1] as u32,
            pid[2] as u32,
            pid[3] as u32,
            pid[4] as u32,
            pid[5] as u32
        );
    }

    bare_metal::info!("=== AST10x0 I3C HCI multi-device master test complete ===");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    let sentinel: &[u8] = match run_master_multi_test() {
        Ok(()) => b"TEST_RESULT:PASS\n",
        Err(error) => {
            bare_metal::error!("I3C HCI master multi-device test failed: {}", error as &str);
            b"TEST_RESULT:FAIL\n"
        }
    };

    console_backend_write_all(sentinel);
    #[expect(clippy::empty_loop)]
    loop {}
}
