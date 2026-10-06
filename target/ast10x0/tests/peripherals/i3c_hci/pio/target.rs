// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    bytes_from_word, classify_entdaa_response, i2c_mode, i3c_sdr_mode, odd_parity7,
    prepare_entdaa_step, prepare_next_entdaa_step, valid_i3c_address, validate_pio_transfer,
    word_from_bytes, AddressSlots, DaaResponse, DatAllocator, DatEntryV1, DctEntryV1,
    HciCommandKind, HciCommandV1, HciDeviceTable, HciI3cDevice, HciResponse, I3cHciError,
    PioTransfer,
};
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

fn run_pio_tests() -> Result<(), &'static str> {
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

    let read = HciCommandV1::i3c_private_read(4, 6, 8, 0)
        .map_err(|_| "private read encode failed")?
        .with_roc()
        .with_toc();
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
    let read_transfer = PioTransfer {
        command: read,
        tx: &[],
        rx: &mut rx,
    };
    validate_pio_transfer(&read_transfer).map_err(|_| "valid read transfer rejected")?;

    let tx = [1u8, 2, 3, 4, 5];
    let mut empty_rx = [];
    let write_transfer = PioTransfer {
        command: regular_write,
        tx: &tx,
        rx: &mut empty_rx,
    };
    validate_pio_transfer(&write_transfer).map_err(|_| "valid write transfer rejected")?;

    let mut immediate_rx = [];
    let immediate_transfer = PioTransfer {
        command: immediate,
        tx: &[],
        rx: &mut immediate_rx,
    };
    validate_pio_transfer(&immediate_transfer).map_err(|_| "valid immediate transfer rejected")?;

    let bad_tx = [0u8; 1];
    let mut bad_rx = [0u8; 8];
    let bad_read_transfer = PioTransfer {
        command: read,
        tx: &bad_tx,
        rx: &mut bad_rx,
    };
    expect_true(
        validate_pio_transfer(&bad_read_transfer) == Err(I3cHciError::InvalidArgs),
        "bad read transfer accepted",
    )?;

    Ok(())
}

impl TargetInterface for Target {
    const NAME: &'static str = "AST10x0 I3C HCI PIO";

    fn main() -> ! {
        let sentinel: &[u8] = match run_pio_tests() {
            Ok(()) => b"TEST_RESULT:PASS\n",
            Err(error) => {
                pw_log::error!("I3C HCI PIO test failed: {}", error);
                b"TEST_RESULT:FAIL\n"
            }
        };

        let _ = console_backend_write_all(sentinel);
        #[expect(clippy::empty_loop)]
        loop {}
    }
}

declare_target!(Target);
