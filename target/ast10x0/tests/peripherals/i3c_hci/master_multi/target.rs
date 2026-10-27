// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

#[cfg(feature = "irq")]
use ast10x0_i3c_hci::dispatch_i3c_hci_irq;
use ast10x0_i3c_hci::{
    DatAllocator, HciDeviceTable, I3C_CCC_GETPID, I3cHciConfig, I3cHciCore, I3cHciMaster,
};

use cortex_m::peripheral::NVIC;

const I3C_BUS: u8 = 0;
const I3C_SCL_HZ: u32 = 12_500_000;
const DAA_TIMEOUT_US: u32 = 1_000_000;
const MAX_I3C_DEVICES: usize = 8;
// Accept one external AST2700 target; still enumerate up to MAX_I3C_DEVICES.
const EXPECTED_MIN_DEVICES: usize = 1;
const ADDRESS_INDEXED_DAT: bool = true;

#[cfg(feature = "irq")]
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

    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(I3C_BUS), |_us| {}) }
        .map_err(|_| "HCI core create failed")?;
    core.init_primary(0x08).map_err(|error| {
        bare_metal::error!("HCI core init failed: {:?}", error);
        let _ = bare_metal::i3c_setup::show_registers(core.regs(), "core init failure");
        "HCI core init failed"
    })?;
    bare_metal::info!(
        "After core.init: HC_CONTROL=0x{:08x} BUS_ENABLE={}",
        core.regs().hc_control(),
        (core.regs().hc_control() >> 31) & 1
    );
    let sections = core.sections();
    expect_true(sections.dat.entries > 0, "HCI DAT table missing")?;
    expect_true(sections.dct.entries > 0, "HCI DCT table missing")?;

    // Start with the controller IRQ masked in both build modes.
    NVIC::mask(ast1080_pac::Interrupt::i3c);
    core.disable_irq_signals();
    #[cfg(feature = "irq")]
    {
        let ctx = core
            .isr_ctx()
            .map_err(|_| "HCI ISR context create failed")?;
        if !ast10x0_i3c_hci::register_i3c_hci_irq_handler(I3C_BUS as usize, ctx) {
            return Err("HCI IRQ registration failed");
        }
        NVIC::unpend(ast1080_pac::Interrupt::i3c);
        core.enable_irq_signals();
        // SAFETY: The hardware and ISR context are initialized before unmasking.
        unsafe {
            NVIC::unmask(ast1080_pac::Interrupt::i3c);
        }
        bare_metal::info!("PIO mode: IRQ enabled");
    }
    #[cfg(not(feature = "irq"))]
    bare_metal::info!("PIO mode: polling (IRQ disabled)");

    let dat_entries = if sections.dat.entries > 128 {
        128
    } else {
        sections.dat.entries as u8
    };
    let mut dat_allocator = DatAllocator::new(dat_entries, ADDRESS_INDEXED_DAT);
    let mut table = HciDeviceTable::<MAX_I3C_DEVICES>::new();
    let master = I3cHciMaster::new(core.regs(), sections, I3C_SCL_HZ, |_us| {})
        .with_timeout_us(DAA_TIMEOUT_US)
        .with_trace(|args| bare_metal::log("PIO", args));
    #[cfg(feature = "irq")]
    let master = master.with_irq_events(I3C_BUS as usize);
    let mut master = master;

    bare_metal::info!(
        "AST2700 target config supplied: PID=0x07ec06010000 DCR=0xcc (informational)"
    );
    bare_metal::info!(
        "ENTDAA start: bus={} SCL={} timeout_budget={} DAT slots={} address_indexed={} descriptor={:?}",
        I3C_BUS,
        I3C_SCL_HZ,
        DAA_TIMEOUT_US,
        dat_entries,
        ADDRESS_INDEXED_DAT,
        master.command_descriptor()
    );
    bare_metal::info!(
        "Timeout callback is a no-op: budget counts polling iterations, not calibrated microseconds"
    );
    let _ = bare_metal::i3c_setup::show_registers(core.regs(), "before ENTDAA");
    bare_metal::info!("ENTDAA: entering command submission / response wait");
    let count = match master.entdaa_all(&mut table, &mut dat_allocator) {
        Ok(count) => count,
        Err(error) => {
            bare_metal::error!(
                "ENTDAA failed: {:?}; partial devices={}",
                error,
                table.len()
            );
            for device in table.iter() {
                bare_metal::info!(
                    "Partial device DA=0x{:02x} DAT={} PID=0x{:012x} BCR=0x{:02x} DCR=0x{:02x}",
                    device.dynamic_addr,
                    device.dat_index,
                    device.pid,
                    device.bcr,
                    device.dcr
                );
            }
            bare_metal::info!(
                "Snapshot is after driver cleanup; error/status bits may already be cleared"
            );
            let _ = bare_metal::i3c_setup::show_registers(core.regs(), "ENTDAA failure");
            return Err("ENTDAA all failed");
        }
    };
    let _ = bare_metal::i3c_setup::show_registers(core.regs(), "after ENTDAA");
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

        // Completion is checked above. In IRQ mode the PIO driver consumes
        // latched events itself, so do not drain/assert that event latch here.

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
    // SAFETY: Test startup owns SCU and the I3C controllers; no transfers are active.
    unsafe {
        bare_metal::i3c_setup::prepare_and_show(&[0], false);
    }
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
