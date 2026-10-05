// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{DatAllocator, HciDeviceTable, I3cHciConfig, I3cHciCore, I3cHciMaster};
use ast10x0_scu_pinctrl::{pinctrl, ScuRegisters};
use codegen as _;
use entry as _;
use target_common::{declare_target, TargetInterface};

const UART5_BASE: *const ast1080_pac::uart::RegisterBlock = 0x7e78_4000 as *const _;
const I3C_BUS: u8 = 0;
const I3C_SCL_HZ: u32 = 12_500_000;
const DAA_TIMEOUT_US: u32 = 1_000_000;
const MAX_I3C_DEVICES: usize = 8;
const EXPECTED_MIN_DEVICES: usize = 2;
const ADDRESS_INDEXED_DAT: bool = true;
const I3C_CCC_GETPID: u8 = 0x8d;

#[unsafe(no_mangle)]
pub fn console_backend_write_all(buf: &[u8]) -> pw_status::Result<()> {
    let uart = unsafe { &*UART5_BASE };
    for byte in buf {
        while !uart.uartlsr().read().thre().bit() {}
        uart.uartthr().write(|w| unsafe { w.bits(*byte as u32) });
    }
    Ok(())
}

pub struct Target {}

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition {
        Ok(())
    } else {
        Err(message)
    }
}

fn run_master_multi_test() -> Result<(), &'static str> {
    pw_log::info!("=== AST10x0 I3C HCI multi-device master test ===");

    // SAFETY: Test target runs once at boot with exclusive access to SCU.
    let scu = unsafe { ScuRegisters::new_global_unlocked() };
    scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);

    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(I3C_BUS), |_us| {}) }
        .map_err(|_| "HCI core create failed")?;
    core.init().map_err(|_| "HCI core init failed")?;
    let sections = core.sections();
    expect_true(sections.dat.entries > 0, "HCI DAT table missing")?;
    expect_true(sections.dct.entries > 0, "HCI DCT table missing")?;

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
    pw_log::info!("ENTDAA discovered {} I3C devices", count as u32);
    expect_true(
        count >= EXPECTED_MIN_DEVICES,
        "too few I3C devices discovered",
    )?;

    for device in table.iter() {
        pw_log::info!(
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
        pw_log::info!(
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

    pw_log::info!("=== AST10x0 I3C HCI multi-device master test complete ===");
    Ok(())
}

impl TargetInterface for Target {
    const NAME: &'static str = "AST10x0 I3C HCI master multi-device";

    fn main() -> ! {
        let sentinel: &[u8] = match run_master_multi_test() {
            Ok(()) => b"TEST_RESULT:PASS\n",
            Err(error) => {
                pw_log::error!("I3C HCI master multi-device test failed: {}", error);
                b"TEST_RESULT:FAIL\n"
            }
        };

        let _ = console_backend_write_all(sentinel);
        #[expect(clippy::empty_loop)]
        loop {}
    }
}

declare_target!(Target);
