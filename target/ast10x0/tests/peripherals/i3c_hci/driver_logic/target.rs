// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    AddressSlots, CommandDescriptor, DAT_0_I2C_DEVICE, DaaResponse, DatAllocator, DatEntryV1,
    DctEntryV1, HciCommand, HciCommandKind, HciCommandV1, HciCommandV2, HciDeviceTable,
    HciI3cDevice, HciResponse, HciTargetCcc, HciTransfer, I3C_CCC_GETBCR, I3C_CCC_GETDCR,
    I3C_CCC_GETMXDS, I3C_CCC_GETPID, I3C_CCC_GETSTATUS, I3C_CCC_RSTDAA_BROADCAST, I3C_CCC_SETDASA,
    I3C_CCC_SETNEWDA, I3cHciConfig, I3cHciError, IoMode, bytes_from_word, classify_entdaa_response,
    classify_target_ccc, constants, discover_sections_from_regs, dynamic_address_payload, i2c_mode,
    i3c_sdr_mode, i3c_sdr_v2_rate, odd_parity7, parse_hci_version, prepare_entdaa_step,
    prepare_next_entdaa_step, select_command_descriptor, valid_i3c_address, validate_transfer,
    word_from_bytes,
};

use bare_metal::write_all as console_backend_write_all;

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(message) }
}

fn run_command_and_table_tests() -> Result<(), &'static str> {
    expect_true(i3c_sdr_mode(12_500_000) == 0, "bad SDR0 mode")?;
    expect_true(i3c_sdr_mode(8_000_000) == 1, "bad SDR1 boundary")?;
    expect_true(i3c_sdr_mode(2_000_000) == 4, "bad SDR4 boundary")?;
    expect_true(i2c_mode(1_000_000) == 1, "bad I2C FMP mode")?;
    expect_true(i2c_mode(400_000) == 0, "bad I2C FM mode")?;

    expect_true(
        word_from_bytes(&[0x11, 0x22, 0x33, 0x44]) == 0x4433_2211,
        "bad TX word pack",
    )?;
    expect_true(word_from_bytes(&[0x5a]) == 0x5a, "bad short TX pack")?;
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
    expect_true(
        !immediate.expects_response(),
        "private immediate unexpected response",
    )?;
    expect_true(immediate.words[0] == 0x0103_0029, "private immediate word0")?;
    expect_true(immediate.words[1] == 0x0000_bbaa, "private immediate word1")?;

    let regular_write = HciCommandV1::i3c_private_write(3, 9, &[0; 5], 0)
        .map_err(|_| "private regular write encode failed")?
        .with_roc()
        .with_toc();
    expect_true(
        regular_write.kind == HciCommandKind::Regular,
        "regular write kind",
    )?;
    expect_true(
        regular_write.expects_response(),
        "regular write response expectation",
    )?;
    expect_true(regular_write.words[0] == 0xc003_0048, "regular write word0")?;
    expect_true(regular_write.words[1] == 0x0005_0000, "regular write word1")?;

    let read =
        HciCommandV1::i3c_private_read(4, 6, 8, 0).map_err(|_| "private read encode failed")?;
    expect_true(read.words[0] == 0x2004_0030, "private read base word0")?;
    let read = read.with_roc().with_toc();
    expect_true(read.kind == HciCommandKind::Regular, "private read kind")?;
    expect_true(read.read, "private read flag")?;
    expect_true(read.expects_response(), "private read response expectation")?;
    expect_true(read.words[0] == 0xe004_0030, "private read word0")?;
    expect_true(read.words[1] == 0x0008_0000, "private read word1")?;

    let regular_write_for_status = HciCommandV1::i3c_private_write_regular(3, 10, 2, 0)
        .map_err(|_| "private regular status write encode failed")?
        .with_roc()
        .with_toc();
    expect_true(
        regular_write_for_status.kind == HciCommandKind::Regular,
        "private status write kind",
    )?;
    expect_true(
        regular_write_for_status.words[0] == 0xc003_0050,
        "private status write word0",
    )?;
    expect_true(
        regular_write_for_status.words[1] == 0x0002_0000,
        "private status write word1",
    )?;

    let ccc = HciCommandV1::ccc(0, 7, false, 0x06, &[0x5a], 0, None)
        .map_err(|_| "CCC immediate encode failed")?;
    expect_true(ccc.words[0] == 0x0080_8339, "CCC immediate word0")?;
    expect_true(ccc.words[1] == 0x0000_005a, "CCC immediate word1")?;

    let ccc_with_db = HciCommandV1::ccc(0, 2, true, 0x8d, &[], 0, Some(0x7e))
        .map_err(|_| "CCC read encode failed")?
        .with_roc();
    expect_true(
        ccc_with_db.words[0] == 0x6200_c690,
        "CCC defining-byte word0",
    )?;
    expect_true(
        ccc_with_db.words[1] == 0x0000_007e,
        "CCC defining-byte word1",
    )?;

    let ccc_regular = HciCommandV1::ccc_regular(3, 11, false, 0x89, 5, 0, None)
        .map_err(|_| "CCC regular write encode failed")?
        .with_roc()
        .with_toc();
    expect_true(ccc_regular.words[0] == 0xc003_c458, "CCC regular word0")?;
    expect_true(ccc_regular.words[1] == 0x0005_0000, "CCC regular word1")?;

    let internal =
        HciCommandV1::internal(8, 0x6, 0x12345).map_err(|_| "internal command encode failed")?;
    expect_true(internal.kind == HciCommandKind::Internal, "internal kind")?;
    expect_true(internal.expects_response(), "internal response expectation")?;
    expect_true(internal.words[0] == 0x1234_5647, "internal word0")?;

    expect_true(valid_i3c_address(0x09), "valid dynamic address rejected")?;
    expect_true(!valid_i3c_address(0x02), "reserved address accepted")?;
    let mut slots = AddressSlots::new();
    slots.mark_used(0x09).map_err(|_| "mark used failed")?;
    expect_true(
        slots.next_free(0x09).map_err(|_| "next free failed")? == 0x0a,
        "bad next free address",
    )?;

    let dat = DatEntryV1::rejected().with_dynamic_addr(0x0a);
    expect_true(dat.dynamic_addr() == 0x0a, "bad DAT dynamic address")?;
    expect_true(dat.word0 == 0x000a_6000, "bad DAT dynamic address word")?;
    expect_true(odd_parity7(0x0a), "bad odd parity helper")?;

    let dct = DctEntryV1::from_words([0x1234_5678, 0x0000_9abc, 0x0000_abcd, 0]);
    expect_true(dct.pid == 0x1234_5678_9abc, "bad DCT PID decode")?;
    expect_true(dct.dcr == 0xcd, "bad DCT DCR decode")?;
    expect_true(dct.bcr == 0xab, "bad DCT BCR decode")?;

    let mut devices = HciDeviceTable::<2>::new();
    let first_device = HciI3cDevice {
        dat_index: 0x0a,
        dynamic_addr: 0x0a,
        pid: dct.pid,
        bcr: dct.bcr,
        dcr: dct.dcr,
    };
    devices
        .push(first_device)
        .map_err(|_| "device table push failed")?;
    expect_true(devices.len() == 1, "bad device table len")?;
    expect_true(
        devices.by_dynamic_addr(0x0a) == Some(first_device),
        "dynamic-address lookup failed",
    )?;
    expect_true(
        devices.by_pid(dct.pid) == Some(first_device),
        "PID lookup failed",
    )?;

    let moved_device = HciI3cDevice {
        dat_index: first_device.dat_index,
        dynamic_addr: 0x0b,
        pid: first_device.pid,
        bcr: first_device.bcr,
        dcr: first_device.dcr,
    };
    devices
        .push_or_replace(moved_device)
        .map_err(|_| "device table replace failed")?;
    expect_true(devices.len() == 1, "device table replace changed len")?;
    expect_true(
        devices.by_dynamic_addr(0x0b) == Some(moved_device),
        "dynamic-address replace failed",
    )?;

    let static_device = HciI3cDevice {
        dat_index: 0x0c,
        dynamic_addr: 0x0c,
        pid: 0,
        bcr: 0,
        dcr: 0,
    };
    devices
        .push_or_replace(static_device)
        .map_err(|_| "unknown-PID table push failed")?;
    expect_true(devices.len() == 2, "unknown-PID push changed wrong len")?;
    let other_static_device = HciI3cDevice {
        dat_index: 0x0d,
        dynamic_addr: 0x0d,
        pid: 0,
        bcr: 0,
        dcr: 0,
    };
    expect_true(
        devices.push_or_replace(other_static_device) == Err(I3cHciError::NoSpace),
        "unknown-PID device replaced unrelated entry",
    )?;

    let daa = prepare_entdaa_step(0x0a, 0x0a, 3).map_err(|_| "ENTDAA prepare failed")?;
    expect_true(daa.dynamic_addr == 0x0a, "bad DAA dynamic address")?;
    expect_true(daa.dat_index == 0x0a, "bad DAA DAT index")?;
    expect_true(
        daa.dat_entry == dat,
        "DAA step did not carry matching DAT entry",
    )?;
    expect_true(
        daa.command.kind == HciCommandKind::AddressAssignment,
        "bad DAA command kind",
    )?;
    expect_true(
        daa.command.expects_response(),
        "DAA command must expect response",
    )?;
    expect_true(
        daa.command.words[0] == 0xc50a_039a,
        "bad ENTDAA command word",
    )?;

    let mut address_indexed_dat = DatAllocator::new(128, true);
    let planned_daa = prepare_next_entdaa_step(&slots, &mut address_indexed_dat, None, 4)
        .map_err(|_| "next ENTDAA prepare failed")?;
    expect_true(planned_daa.dynamic_addr == 0x0a, "bad planned DAA address")?;
    expect_true(
        planned_daa.dat_index == 0x0a,
        "bad address-indexed DAT slot",
    )?;
    expect_true(
        address_indexed_dat.is_allocated(0x0a),
        "planned DAT slot was not allocated",
    )?;
    expect_true(
        planned_daa.dat_entry.word0 == 0x000a_6000,
        "bad planned DAT word",
    )?;

    let mut too_small_address_indexed_dat = DatAllocator::new(2, true);
    expect_true(
        prepare_next_entdaa_step(
            &AddressSlots::new(),
            &mut too_small_address_indexed_dat,
            None,
            5,
        ) == Err(I3cHciError::NoSpace),
        "address-indexed DAT fallback accepted",
    )?;

    let assigned = HciResponse::parse(0x0400_0000);
    expect_true(
        classify_entdaa_response(assigned).map_err(|_| "assigned DAA response rejected")?
            == DaaResponse::Assigned,
        "bad assigned DAA response",
    )?;
    let no_more = HciResponse::parse(0x5500_0001);
    expect_true(
        classify_entdaa_response(no_more).map_err(|_| "no-more DAA response rejected")?
            == DaaResponse::NoMoreDevices,
        "bad no-more DAA response",
    )?;

    let response = HciResponse::parse(0x0500_0010);
    expect_true(response.status == 0, "response status")?;
    expect_true(response.tid == 5, "response tid")?;
    expect_true(response.data_len == 16, "response data length")?;
    expect_true(response.success(), "response success")?;

    let response_error = HciResponse::parse(0x5500_0000);
    expect_true(response_error.status == 5, "response error status")?;
    expect_true(!response_error.success(), "response error success")?;

    let mut rx = [0u8; 8];
    let read_transfer = HciTransfer {
        command: HciCommand::V1(read),
        tx: &[],
        rx: &mut rx,
    };
    validate_transfer(&read_transfer).map_err(|_| "valid read transfer rejected")?;

    let tx = [1u8, 2, 3, 4, 5];
    let mut empty_rx = [];
    let write_transfer = HciTransfer {
        command: HciCommand::V1(regular_write),
        tx: &tx,
        rx: &mut empty_rx,
    };
    validate_transfer(&write_transfer).map_err(|_| "valid write transfer rejected")?;

    let mut immediate_rx = [];
    let immediate_transfer = HciTransfer {
        command: HciCommand::V1(immediate),
        tx: &[],
        rx: &mut immediate_rx,
    };
    validate_transfer(&immediate_transfer).map_err(|_| "valid immediate transfer rejected")?;

    let bad_tx = [0u8; 1];
    let mut bad_rx = [0u8; 8];
    let bad_read_transfer = HciTransfer {
        command: HciCommand::V1(read),
        tx: &bad_tx,
        rx: &mut bad_rx,
    };
    expect_true(
        validate_transfer(&bad_read_transfer) == Err(I3cHciError::InvalidArgs),
        "bad read transfer accepted",
    )?;

    Ok(())
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

fn run_additional_encoding_tests() -> Result<(), &'static str> {
    let i2c_entry = DatEntryV1::rejected().with_i2c_device(0x50);
    expect_true(
        (i2c_entry.word0 & DAT_0_I2C_DEVICE) != 0,
        "I2C DAT flag missing",
    )?;
    expect_true(i2c_entry.static_addr() == 0x50, "I2C static addr mismatch")?;

    let i2c_write = HciCommandV1::i2c_write(2, 9, &[0xde, 0xad], i2c_mode(400_000))
        .map_err(|_| "I2C immediate encode failed")?;
    expect_true(
        i2c_write.kind == HciCommandKind::Immediate,
        "I2C immediate kind",
    )?;
    expect_true(i2c_write.words[0] == 0x0102_0049, "I2C immediate word0")?;
    expect_true(i2c_write.words[1] == 0x0000_adde, "I2C immediate word1")?;

    let i2c_read = HciCommandV1::i2c_read(2, 10, 3, i2c_mode(1_000_000))
        .map_err(|_| "I2C read encode failed")?;
    expect_true(i2c_read.kind == HciCommandKind::Regular, "I2C read kind")?;
    expect_true(i2c_read.words[0] == 0x2402_0050, "I2C read word0")?;
    expect_true(i2c_read.words[1] == 0x0003_0000, "I2C read word1")?;

    expect_true(I3C_CCC_RSTDAA_BROADCAST == 0x06, "bad RSTDAA broadcast CCC")?;
    expect_true(I3C_CCC_SETDASA == 0x87, "bad SETDASA CCC")?;
    expect_true(I3C_CCC_SETNEWDA == 0x88, "bad SETNEWDA CCC")?;
    expect_true(I3C_CCC_GETPID == 0x8d, "bad GETPID CCC")?;
    expect_true(I3C_CCC_GETBCR == 0x8e, "bad GETBCR CCC")?;
    expect_true(I3C_CCC_GETDCR == 0x8f, "bad GETDCR CCC")?;
    expect_true(I3C_CCC_GETSTATUS == 0x90, "bad GETSTATUS CCC")?;
    expect_true(I3C_CCC_GETMXDS == 0x94, "bad GETMXDS CCC")?;
    expect_true(
        dynamic_address_payload(0x09).map_err(|_| "dynamic address payload failed")? == 0x13,
        "bad dynamic-address payload",
    )?;

    expect_true(
        classify_target_ccc(I3C_CCC_GETPID) == HciTargetCcc::GetPid,
        "bad target GETPID classification",
    )?;
    expect_true(
        classify_target_ccc(I3C_CCC_SETDASA) == HciTargetCcc::SetDynamicAddressFromStatic,
        "bad target SETDASA classification",
    )?;
    expect_true(
        classify_target_ccc(0xff) == HciTargetCcc::Unknown(0xff),
        "bad target unknown CCC classification",
    )?;

    expect_true(i3c_sdr_v2_rate(12_500_000) == 0, "bad V2 SDR0 rate")?;
    expect_true(i3c_sdr_v2_rate(8_000_000) == 2, "bad V2 SDR2 boundary")?;
    expect_true(i3c_sdr_v2_rate(2_000_000) == 5, "bad V2 FM/FMP rate")?;

    let v2_read = HciCommandV2::i3c_private_read(4, 6, 8, i3c_sdr_v2_rate(12_500_000))
        .map_err(|_| "V2 private read encode failed")?;
    expect_true(
        v2_read.kind == HciCommandKind::Regular,
        "V2 private read kind",
    )?;
    expect_true(v2_read.words[0] == 0x0000_04b4, "V2 private read word0")?;
    expect_true(v2_read.words[1] == 0x0000_0008, "V2 private read word1")?;

    let v2_ccc = HciCommandV2::ccc(
        0,
        7,
        false,
        0x06,
        &[0x5a],
        i3c_sdr_v2_rate(12_500_000),
        None,
    )
    .map_err(|_| "V2 CCC immediate encode failed")?;
    expect_true(
        v2_ccc.kind == HciCommandKind::Immediate,
        "V2 CCC immediate kind",
    )?;
    expect_true(v2_ccc.words[0] == 0x0200_003c, "V2 CCC immediate word0")?;
    expect_true(v2_ccc.words[2] == 0x0000_5a06, "V2 CCC immediate word2")?;

    let v2_daa_read = HciCommandV2::daa_read_pid(3, 8).map_err(|_| "V2 DAA read encode failed")?;
    expect_true(
        v2_daa_read.kind == HciCommandKind::AddressAssignment,
        "V2 DAA read kind",
    )?;
    expect_true(v2_daa_read.words[0] == 0x4000_001a, "V2 DAA read word0")?;
    expect_true(v2_daa_read.words[1] == 0x0000_0008, "V2 DAA read word1")?;

    let v2_daa_assign =
        HciCommandV2::daa_assign_address(4, 0x0a).map_err(|_| "V2 DAA assign encode failed")?;
    expect_true(
        v2_daa_assign.kind == HciCommandKind::AddressAssignment,
        "V2 DAA assign kind",
    )?;
    expect_true(v2_daa_assign.words[0] == 0xc000_0a22, "V2 DAA assign word0")?;

    let v2_internal =
        HciCommandV2::internal(8, 0x6, 0x12345).map_err(|_| "V2 internal encode failed")?;
    expect_true(
        v2_internal.kind == HciCommandKind::Internal,
        "V2 internal kind",
    )?;
    expect_true(
        v2_internal.expects_response(),
        "V2 internal response expectation",
    )?;
    expect_true(v2_internal.words[0] == 0x1234_5647, "V2 internal word0")?;

    Ok(())
}

fn run() -> Result<(), &'static str> {
    run_core_logic_tests()?;
    run_command_and_table_tests()?;
    run_additional_encoding_tests()
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // Pure driver logic: no HCI reset, register access or pin setup.
    let sentinel: &[u8] = match run() {
        Ok(()) => b"TEST_RESULT:PASS\n",
        Err(error) => {
            bare_metal::error!("I3C driver logic test failed: {}", error as &str);
            b"TEST_RESULT:FAIL\n"
        }
    };

    console_backend_write_all(sentinel);
    #[expect(clippy::empty_loop)]
    loop {}
}
