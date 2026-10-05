// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    bytes_from_word, constants, discover_sections_from_regs, i3c_sdr_mode, parse_hci_version,
    select_command_descriptor, word_from_bytes, CommandDescriptor, HciCommandKind, HciCommandV1,
    HciResponse, I3cHciConfig, I3cHciError, I3cHciRegisters, IoMode,
};
use ast10x0_scu_pinctrl::{pinctrl, ScuRegisters};
use codegen as _;
use entry as _;
use target_common::{declare_target, TargetInterface};

const UART5_BASE: *const ast1080_pac::uart::RegisterBlock = 0x7e78_4000 as *const _;

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

fn run_phase2_command_logic_tests() -> Result<(), &'static str> {
    expect_true(i3c_sdr_mode(12_500_000) == 0, "bad SDR0 mode")?;
    expect_true(i3c_sdr_mode(8_000_000) == 1, "bad SDR1 boundary")?;
    expect_true(i3c_sdr_mode(2_000_000) == 4, "bad SDR4 boundary")?;

    expect_true(
        word_from_bytes(&[0x11, 0x22, 0x33, 0x44]) == 0x4433_2211,
        "bad TX word pack",
    )?;
    let mut unpacked = [0u8; 3];
    bytes_from_word(0x00cc_bbaa, &mut unpacked);
    expect_true(unpacked == [0xaa, 0xbb, 0xcc], "bad RX word unpack")?;

    let immediate = HciCommandV1::i3c_private_write(3, 5, &[0xaa, 0xbb], 0)
        .map_err(|_| "private immediate encode failed")?;
    expect_true(
        immediate.kind == HciCommandKind::Immediate,
        "private immediate kind",
    )?;
    expect_true(immediate.tid == 5, "private immediate tid")?;
    expect_true(immediate.words[0] == 0x0103_0029, "private immediate word0")?;
    expect_true(immediate.words[1] == 0x0000_bbaa, "private immediate word1")?;

    let read =
        HciCommandV1::i3c_private_read(4, 6, 8, 0).map_err(|_| "private read encode failed")?;
    expect_true(read.kind == HciCommandKind::Regular, "private read kind")?;
    expect_true(read.words[0] == 0x2004_0030, "private read word0")?;
    expect_true(read.words[1] == 0x0008_0000, "private read word1")?;

    let ccc = HciCommandV1::ccc(0, 7, false, 0x06, &[0x5a], 0, None)
        .map_err(|_| "CCC immediate encode failed")?;
    expect_true(ccc.words[0] == 0x0080_8339, "CCC immediate word0")?;
    expect_true(ccc.words[1] == 0x0000_005a, "CCC immediate word1")?;

    let internal =
        HciCommandV1::internal(8, 0x6, 0x12345).map_err(|_| "internal command encode failed")?;
    expect_true(internal.kind == HciCommandKind::Internal, "internal kind")?;
    expect_true(internal.expects_response(), "internal response expectation")?;
    expect_true(internal.words[0] == 0x1234_5647, "internal word0")?;

    let response = HciResponse::parse(0x0500_0010);
    expect_true(response.status == 0, "response status")?;
    expect_true(response.tid == 5, "response tid")?;
    expect_true(response.data_len == 16, "response data length")?;
    expect_true(response.success(), "response success")?;

    Ok(())
}

fn run_hardware_register_smoke_test() -> Result<(), &'static str> {
    // SAFETY: This test is the only code touching HCI bus 0.
    let regs = unsafe { I3cHciRegisters::new(0) }.ok_or("invalid I3C HCI bus")?;

    let raw_version = regs.hci_version();
    let version = parse_hci_version(raw_version).map_err(|_| "unsupported HCI version")?;
    let caps = regs.hc_capabilities();
    let dat = regs.dat_section();
    let dct = regs.dct_section();
    let rhs = regs.ring_headers_section();
    let pio = regs.pio_section();
    let ext = regs.ext_caps_section();
    let sections = discover_sections_from_regs(version, dat, dct, rhs, pio, ext)
        .map_err(|_| "invalid HCI section registers")?;

    pw_log::info!(
        "HCI{} version raw={:08x} major={} minor={} rev={} caps={:08x}",
        regs.bus() as u32,
        raw_version as u32,
        version.major as u32,
        version.minor as u32,
        version.revision as u32,
        caps as u32
    );
    pw_log::info!(
        "HCI sections DAT off={:04x} entries={} size={} DCT off={:04x} entries={} size={}",
        sections.dat.offset as u32,
        sections.dat.entries as u32,
        sections.dat.entry_size as u32,
        sections.dct.offset as u32,
        sections.dct.entries as u32,
        sections.dct.entry_size as u32
    );
    pw_log::info!(
        "HCI sections RHS={:04x} PIO={:04x} EXT={:04x} present={:08x} pio_q={:08x}",
        sections.ring_headers_offset as u32,
        sections.pio_offset as u32,
        sections.ext_caps_offset as u32,
        regs.present_state() as u32,
        regs.pio_queue_size() as u32
    );

    Ok(())
}

fn run_i3c_hci_init_smoke_test() -> Result<(), &'static str> {
    pw_log::info!("=== AST10x0 I3C HCI init smoke test ===");

    // SAFETY: Test target runs once at boot with exclusive access to SCU.
    let scu = unsafe { ScuRegisters::new_global_unlocked() };
    scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);
    pw_log::info!("SCU pinctrl applied for I3C HCI0");

    run_core_logic_tests()?;
    pw_log::info!("I3C HCI core logic checks passed");

    run_phase2_command_logic_tests()?;
    pw_log::info!("I3C HCI phase 2 command logic checks passed");

    run_hardware_register_smoke_test()?;
    pw_log::info!("I3C HCI hardware register smoke checks passed");

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
