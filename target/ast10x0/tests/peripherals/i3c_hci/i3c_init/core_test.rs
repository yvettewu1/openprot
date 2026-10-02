// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

use ast10x0_i3c_hci::{
    constants, discover_sections_from_regs, parse_hci_version, select_command_descriptor,
    CommandDescriptor, I3cHciConfig, I3cHciError, IoMode,
};

#[test]
fn parse_supported_hci_versions() {
    let v = parse_hci_version(0x0112).unwrap();
    assert_eq!(v.major, 1);
    assert_eq!(v.minor, 1);
    assert_eq!(v.revision, 2);
    assert_eq!(v.raw, 0x0112);

    assert_eq!(parse_hci_version(0x0100).unwrap().major, 1);
    assert_eq!(parse_hci_version(0x0200).unwrap().major, 2);
}

#[test]
fn reject_unsupported_hci_version() {
    assert_eq!(parse_hci_version(0x0120), Err(I3cHciError::Unsupported));
}

#[test]
fn select_command_descriptor_from_capabilities() {
    assert_eq!(select_command_descriptor(0).unwrap(), CommandDescriptor::V1);
    assert_eq!(
        select_command_descriptor(1 << constants::HC_CAP_CMD_SIZE_SHIFT).unwrap(),
        CommandDescriptor::V2
    );
    assert_eq!(
        select_command_descriptor(2 << constants::HC_CAP_CMD_SIZE_SHIFT),
        Err(I3cHciError::InvalidArgs)
    );
}

#[test]
fn discover_sections_for_hci_1_1_uses_entry_counts_directly() {
    let version = parse_hci_version(0x0110).unwrap();
    let dat = (5 << constants::DAT_TABLE_SIZE_SHIFT) | 0x120;
    let dct = (3 << constants::DCT_TABLE_SIZE_SHIFT) | 0x400;

    let sections = discover_sections_from_regs(version, dat, dct, 0x600, 0x0d0, 0xf00).unwrap();

    assert_eq!(sections.dat.offset, 0x120);
    assert_eq!(sections.dat.entries, 5);
    assert_eq!(sections.dat.entry_size, 8);
    assert_eq!(sections.dct.offset, 0x400);
    assert_eq!(sections.dct.entries, 3);
    assert_eq!(sections.dct.entry_size, 16);
    assert_eq!(sections.ring_headers_offset, 0x600);
    assert_eq!(sections.pio_offset, 0x0d0);
    assert_eq!(sections.ext_caps_offset, 0xf00);
}

#[test]
fn discover_sections_for_hci_1_0_converts_dword_counts_to_entries() {
    let version = parse_hci_version(0x0100).unwrap();
    let dat = (16 << constants::DAT_TABLE_SIZE_SHIFT) | 0x120;
    let dct = (16 << constants::DCT_TABLE_SIZE_SHIFT) | 0x400;

    let sections = discover_sections_from_regs(version, dat, dct, 0, 0x0d0, 0).unwrap();

    assert_eq!(sections.dat.entries, 8);
    assert_eq!(sections.dct.entries, 4);
}

#[test]
fn reject_unknown_dat_or_dct_entry_size_encoding() {
    let version = parse_hci_version(0x0110).unwrap();
    let bad_dat = (1 << constants::DAT_ENTRY_SIZE_SHIFT) | 0x120;
    let good_dct = 0x400;
    assert_eq!(
        discover_sections_from_regs(version, bad_dat, good_dct, 0, 0, 0),
        Err(I3cHciError::InvalidArgs)
    );

    let good_dat = 0x120;
    let bad_dct = (1 << constants::DCT_ENTRY_SIZE_SHIFT) | 0x400;
    assert_eq!(
        discover_sections_from_regs(version, good_dat, bad_dct, 0, 0, 0),
        Err(I3cHciError::InvalidArgs)
    );
}

#[test]
fn pio_config_defaults_match_core_mvp() {
    let cfg = I3cHciConfig::pio(2);
    assert_eq!(cfg.bus, 2);
    assert_eq!(cfg.io_mode, IoMode::Pio);
    assert!(!cfg.data_big_endian);
}
