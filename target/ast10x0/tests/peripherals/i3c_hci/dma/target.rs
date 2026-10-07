// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 I3C HCI DMA/RHS smoke test.
//!
//! This validates the DMA mode bring-up foundation only:
//! - RHS control parsing
//! - RHS section presence
//! - HCI core selection of DMA mode
//! - RHS interrupt status register access
//!
//! It intentionally does not submit DMA transfers yet; ring descriptor memory
//! ownership and cache/NC placement come in the follow-up DMA transfer phase.

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    parse_hci_version, parse_rhs_control, rhs_info, I3cHciConfig, I3cHciCore,
    IoMode,
};
use ast10x0_scu_pinctrl::{pinctrl, ScuRegisters};
use codegen as _;
use entry as _;
use target_common::{declare_target, TargetInterface};

const UART5_BASE: *const ast1080_pac::uart::RegisterBlock = 0x7e78_4000 as *const _;
const BUS: u8 = 0;

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

fn run_rhs_logic_tests() -> Result<(), &'static str> {
    let control = (4u32 << 24) | (0x20u32 << 16) | (8u32 << 4) | 1u32;
    let info = parse_rhs_control(control, 0x600);

    expect_true(info.preamble_size == 4, "bad RHS preamble size")?;
    expect_true(info.header_size == 0x20, "bad RHS header size")?;
    expect_true(info.max_header_count_cap == 8, "bad RHS max header cap")?;
    expect_true(info.driver_header_count == 1, "bad RHS driver header count")?;
    expect_true(info.ring_headers_offset == 0x600, "bad RHS offset")?;

    let cfg = I3cHciConfig::dma(BUS);
    expect_true(cfg.bus == BUS, "bad DMA config bus")?;
    expect_true(cfg.io_mode == IoMode::Dma, "bad DMA config mode")?;
    expect_true(!cfg.data_big_endian, "bad DMA config endian")?;

    Ok(())
}

fn run_rhs_hardware_smoke_test() -> Result<(), &'static str> {
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::dma(BUS), |_us| {}) }
        .map_err(|_| "DMA HCI core create failed")?;

    core.init().map_err(|_| "DMA HCI core init failed")?;

    expect_true(
        core.selected_io() == Some(IoMode::Dma),
        "DMA mode not selected",
    )?;

    let regs = core.regs();
    let sections = core.sections();
    let rhs = rhs_info(regs, sections).map_err(|_| "RHS info unavailable")?;

    pw_log::info!(
        "HCI{} DMA RHS off={:04x} preamble={} header={} max={} driver={}",
        regs.bus() as u32,
        rhs.ring_headers_offset as u32,
        rhs.preamble_size as u32,
        rhs.header_size as u32,
        rhs.max_header_count_cap as u32,
        rhs.driver_header_count as u32,
    );

    pw_log::info!(
        "HCI DMA version raw={:08x} rhs_status={:08x}",
        regs.hci_version() as u32,
        regs.rhs_intr_status() as u32,
    );

    let version = parse_hci_version(regs.hci_version()).map_err(|_| "unsupported HCI version")?;
    expect_true(version.major != 0, "bad HCI version")?;

    Ok(())
}

fn run_dma_test() -> Result<(), &'static str> {
    pw_log::info!("=== AST1080 I3C HCI DMA/RHS smoke test ===");

    let scu = unsafe { ScuRegisters::new_global_unlocked() };
    scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);

    run_rhs_logic_tests()?;
    pw_log::info!("RHS logic checks passed");

    run_rhs_hardware_smoke_test()?;
    pw_log::info!("DMA/RHS hardware smoke test passed");

    Ok(())
}

impl TargetInterface for Target {
    const NAME: &'static str = "AST1080 I3C HCI DMA/RHS smoke test";

    fn main() -> ! {
        let sentinel: &[u8] = match run_dma_test() {
            Ok(()) => b"TEST_RESULT:PASS\n",
            Err(error) => {
                pw_log::error!("I3C HCI DMA/RHS test failed: {}", error as &str);
                b"TEST_RESULT:FAIL\n"
            }
        };

        let _ = console_backend_write_all(sentinel);

        #[expect(clippy::empty_loop)]
        loop {}
    }
}

declare_target!(Target);
