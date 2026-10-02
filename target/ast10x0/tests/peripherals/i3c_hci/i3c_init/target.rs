// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_board::{Ast10x0Board, Ast10x0BoardDescriptor};
use ast10x0_i3c_hci::{
    constants, discover_sections_from_regs, parse_hci_version, select_command_descriptor,
    CommandDescriptor, I3cHciConfig, I3cHciError, IoMode,
};
use ast10x0_peripherals::scu::pinctrl;
use codegen as _;
use console_backend::console_backend_write_all;
use entry as _;
use target_common::{declare_target, TargetInterface};

pub struct Target {}

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition {
        Ok(())
    } else {
        Err(message)
    }
}

fn run_core_logic_tests() -> Result<(), &'static str> {
    let version = parse_hci_version(0x0112).map_err(|_| "HCI v1.1 parse failed")?;
    expect_true(version.major == 1, "bad HCI major")?;
    expect_true(version.minor == 1, "bad HCI minor")?;
    expect_true(version.revision == 2, "bad HCI revision")?;
    expect_true(version.raw == 0x0112, "bad HCI raw version")?;
    expect_true(parse_hci_version(0x0100).is_ok(), "HCI v1.0 rejected")?;
    expect_true(parse_hci_version(0x0200).is_ok(), "HCI v2.0 rejected")?;
    expect_true(
        parse_hci_version(0x0120) == Err(I3cHciError::Unsupported),
        "unsupported HCI version accepted",
    )?;

    expect_true(
        select_command_descriptor(0).map_err(|_| "CMD v1 select failed")? == CommandDescriptor::V1,
        "CMD v1 mismatch",
    )?;
    expect_true(
        select_command_descriptor(1 << constants::HC_CAP_CMD_SIZE_SHIFT)
            .map_err(|_| "CMD v2 select failed")?
            == CommandDescriptor::V2,
        "CMD v2 mismatch",
    )?;
    expect_true(
        select_command_descriptor(2 << constants::HC_CAP_CMD_SIZE_SHIFT)
            == Err(I3cHciError::InvalidArgs),
        "invalid CMD size accepted",
    )?;

    let dat = (5 << constants::DAT_TABLE_SIZE_SHIFT) | 0x120;
    let dct = (3 << constants::DCT_TABLE_SIZE_SHIFT) | 0x400;
    let sections = discover_sections_from_regs(version, dat, dct, 0x600, 0x0d0, 0xf00)
        .map_err(|_| "HCI v1.1 section discovery failed")?;
    expect_true(sections.dat.offset == 0x120, "bad DAT offset")?;
    expect_true(sections.dat.entries == 5, "bad DAT entries")?;
    expect_true(sections.dat.entry_size == 8, "bad DAT entry size")?;
    expect_true(sections.dct.offset == 0x400, "bad DCT offset")?;
    expect_true(sections.dct.entries == 3, "bad DCT entries")?;
    expect_true(sections.dct.entry_size == 16, "bad DCT entry size")?;
    expect_true(sections.ring_headers_offset == 0x600, "bad RHS offset")?;
    expect_true(sections.pio_offset == 0x0d0, "bad PIO offset")?;
    expect_true(sections.ext_caps_offset == 0xf00, "bad ext-cap offset")?;

    let v10 = parse_hci_version(0x0100).map_err(|_| "HCI v1.0 parse failed")?;
    let dat_dwords = (16 << constants::DAT_TABLE_SIZE_SHIFT) | 0x120;
    let dct_dwords = (16 << constants::DCT_TABLE_SIZE_SHIFT) | 0x400;
    let sections_v10 = discover_sections_from_regs(v10, dat_dwords, dct_dwords, 0, 0x0d0, 0)
        .map_err(|_| "HCI v1.0 section discovery failed")?;
    expect_true(
        sections_v10.dat.entries == 8,
        "bad v1.0 DAT dword conversion",
    )?;
    expect_true(
        sections_v10.dct.entries == 4,
        "bad v1.0 DCT dword conversion",
    )?;

    let bad_dat = (1 << constants::DAT_ENTRY_SIZE_SHIFT) | 0x120;
    expect_true(
        discover_sections_from_regs(version, bad_dat, 0x400, 0, 0, 0)
            == Err(I3cHciError::InvalidArgs),
        "bad DAT entry-size encoding accepted",
    )?;
    let bad_dct = (1 << constants::DCT_ENTRY_SIZE_SHIFT) | 0x400;
    expect_true(
        discover_sections_from_regs(version, 0x120, bad_dct, 0, 0, 0)
            == Err(I3cHciError::InvalidArgs),
        "bad DCT entry-size encoding accepted",
    )?;

    let cfg = I3cHciConfig::pio(2);
    expect_true(cfg.bus == 2, "bad PIO config bus")?;
    expect_true(cfg.io_mode == IoMode::Pio, "bad PIO config mode")?;
    expect_true(!cfg.data_big_endian, "bad PIO config endian")?;

    Ok(())
}

fn run_i3c_hci_init_smoke_test() -> Result<(), &'static str> {
    pw_log::info!("=== AST10x0 I3C HCI init smoke test ===");

    let board = Ast10x0Board::new(Ast10x0BoardDescriptor {
        pinctrl_groups: &[pinctrl::PINCTRL_I3C_HCI0],
        i2c_buses: &[],
    });
    // SAFETY: Test target runs once at boot with exclusive access to the board.
    // TODO: board init sequence and SCU programming may need HCI-specific updates.
    unsafe { board.init() }.map_err(|_| "board init failed")?;
    pw_log::info!("Board-level pinctrl applied for I3C HCI0");

    run_core_logic_tests()?;
    pw_log::info!("I3C HCI core logic checks passed");

    pw_log::info!("=== AST10x0 I3C HCI init smoke test complete ===");
    Ok(())
}

impl TargetInterface for Target {
    const NAME: &'static str = "AST10x0 Kernel I3C HCI";

    fn main() -> ! {
        let sentinel: &[u8] = match run_i3c_hci_init_smoke_test() {
            Ok(()) => b"TEST_RESULT:PASS\n",
            Err(error) => {
                pw_log::error!("I3C HCI init smoke test failed: {}", error);
                b"TEST_RESULT:FAIL\n"
            }
        };

        let _ = console_backend_write_all(sentinel);
        #[expect(clippy::empty_loop)]
        loop {}
    }
}

declare_target!(Target);
