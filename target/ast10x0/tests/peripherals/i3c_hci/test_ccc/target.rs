// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

#[cfg(feature = "irq")]
use ast10x0_i3c_hci::dispatch_i3c_hci_irq;
use ast10x0_i3c_hci::{ccc::*, dynamic_address_payload, valid_i3c_address, DatAllocator, HciDeviceTable, HciI3cDevice, I3cHciConfig, I3cHciCore, I3cHciMaster};

use cortex_m::peripheral::NVIC;

const I3C_BUS: u8 = 0;
const I3C_SCL_HZ: u32 = 12_500_000;
const DAA_TIMEOUT_US: u32 = 1_000_000;
const MAX_I3C_DEVICES: usize = 8;
// Accept one external AST2700 target; still enumerate up to MAX_I3C_DEVICES.
const EXPECTED_MIN_DEVICES: usize = 1;
const ADDRESS_INDEXED_DAT: bool = true;
// SETDASA requires a real static address configured on a single target.
const TARGET_STATIC_ADDR: Option<u8> = None;
// Direct RSTDAA is a legacy command; enable only for a target supporting it.
const TEST_LEGACY_DIRECT_RSTDAA: bool = false;
const BCR_IBI_PAYLOAD: u8 = 1 << 2;
const BCR_IBI_CAPABLE: u8 = 1 << 1;
const BCR_MAX_DATA_SPEED: u8 = 1;

fn read_ccc<Y: FnMut(u32), const N: usize>(
    master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice,
    name: &'static str, command: u8,
) -> Result<[u8; N], &'static str> {
    let mut data = [0u8; N];
    let result = master.ccc_direct_read(device.dat_index, command, None, &mut data)
        .map_err(|error| {
            bare_metal::error!("CCC {} da=0x{:02x}: {:?}", name, device.dynamic_addr, error);
            "CCC read failed"
        })?;
    bare_metal::info!("CCC {} da=0x{:02x} len={} response=0x{:08x} bytes={:02x?}",
        name, device.dynamic_addr, result.rx_len, result.response.raw, data);
    expect_true(result.rx_len == N && result.response.data_len == N, "CCC read length mismatch")?;
    Ok(data)
}

fn write_ccc<Y: FnMut(u32)>(
    master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice,
    name: &'static str, command: u8, data: &[u8],
) -> Result<(), &'static str> {
    master.ccc_direct_write(device.dat_index, command, data).map_err(|error| {
        bare_metal::error!("CCC {} da=0x{:02x}: {:?}", name, device.dynamic_addr, error);
        "CCC direct write failed"
    })?;
    bare_metal::info!("CCC {} direct BUS PASS da=0x{:02x}", name, device.dynamic_addr);
    Ok(())
}

fn broadcast_ccc<Y: FnMut(u32)>(
    master: &mut I3cHciMaster<'_, Y>, name: &'static str, command: u8, data: &[u8],
) -> Result<(), &'static str> {
    master.ccc_broadcast_write(command, data).map_err(|error| {
        bare_metal::error!("CCC {} broadcast: {:?}", name, error);
        "CCC broadcast failed"
    })?;
    // Broadcast descriptors do not necessarily produce a response entry.
    bare_metal::info!("CCC {} broadcast SENT; target effect needs readback", name);
    Ok(())
}

fn check_identity<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice)
    -> Result<(), &'static str> {
    let pid: [u8; 6] = read_ccc(master, device, "GETPID", I3C_CCC_GETPID)?;
    let expected = device.pid.to_be_bytes();
    expect_true(pid.as_slice() == &expected[2..], "GETPID differs from ENTDAA PID")?;
    let bcr: [u8; 1] = read_ccc(master, device, "GETBCR", I3C_CCC_GETBCR)?;
    expect_true(bcr[0] == device.bcr, "GETBCR differs from ENTDAA BCR")?;
    let dcr: [u8; 1] = read_ccc(master, device, "GETDCR", I3C_CCC_GETDCR)?;
    expect_true(dcr[0] == device.dcr, "GETDCR differs from ENTDAA DCR")?;
    bare_metal::info!("CCC identity PASS da=0x{:02x}", device.dynamic_addr);
    Ok(())
}

fn read_mrl<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice)
    -> Result<([u8; 3], usize), &'static str> {
    if device.bcr & BCR_IBI_PAYLOAD != 0 {
        Ok((read_ccc(master, device, "GETMRL", I3C_CCC_GETMRL)?, 3))
    } else {
        let data: [u8; 2] = read_ccc(master, device, "GETMRL", I3C_CCC_GETMRL)?;
        Ok(([data[0], data[1], 0], 2))
    }
}

fn test_lengths<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice)
    -> Result<(), &'static str> {
    let mwl: [u8; 2] = read_ccc(master, device, "GETMWL", I3C_CCC_GETMWL)?;
    let (mrl, mrl_len) = read_mrl(master, device)?;
    // Exercise the setters without changing the working Linux target's limits.
    // Readback checks encoding/length and target responsiveness, not a change of value.
    write_ccc(master, device, "SETMWL", I3C_CCC_SETMWL_DIRECT, &mwl)?;
    let after: [u8; 2] = read_ccc(master, device, "GETMWL", I3C_CCC_GETMWL)?;
    expect_true(after == mwl, "SETMWL readback mismatch")?;
    write_ccc(master, device, "SETMRL", I3C_CCC_SETMRL_DIRECT, &mrl[..mrl_len])?;
    expect_true(read_mrl(master, device)?.0 == mrl, "SETMRL readback mismatch")?;
    bare_metal::info!("CCC direct length round-trip PASS da=0x{:02x}", device.dynamic_addr);
    Ok(())
}

fn test_mxds<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice)
    -> Result<(), &'static str> {
    if device.bcr & BCR_MAX_DATA_SPEED == 0 {
        bare_metal::info!("CCC GETMXDS SKIP: BCR does not advertise speed limits");
        return Ok(());
    }
    let mut data = [0u8; 5];
    let result = master.getmxds(device.dat_index, &mut data).map_err(|error| {
        bare_metal::error!("CCC GETMXDS: {:?}", error);
        "GETMXDS failed"
    })?;
    expect_true((result.rx_len == 2 || result.rx_len == 5)
        && result.response.data_len == result.rx_len, "GETMXDS invalid response length")?;
    bare_metal::info!("CCC GETMXDS PASS bytes={:02x?}", &data[..result.rx_len]);
    Ok(())
}

fn test_activity<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice)
    -> Result<(), &'static str> {
    for command in [I3C_CCC_ENTAS0, I3C_CCC_ENTAS1, I3C_CCC_ENTAS2, I3C_CCC_ENTAS3] {
        bare_metal::info!("CCC ENTAS{} direct", command - I3C_CCC_ENTAS0);
        write_ccc(master, device, "ENTAS", command | 0x80, &[])?;
        // Return to AS0 before the next test; AS1-3 are activity hints.
        write_ccc(master, device, "ENTAS0 restore", I3C_CCC_ENTAS0 | 0x80, &[])?;
        let _: [u8; 2] = read_ccc(master, device, "GETSTATUS", I3C_CCC_GETSTATUS)?;
    }
    Ok(())
}

fn test_new_address<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>, device: HciI3cDevice,
    spare: u8) -> Result<(), &'static str> {
    let payload = dynamic_address_payload(spare).map_err(|_| "invalid spare address")?;
    write_ccc(master, device, "SETNEWDA", I3C_CCC_SETNEWDA, &[payload])?;
    // DAT index is a descriptor slot; update its address before the next command.
    master.attach_i3c_device(device.dat_index, spare).map_err(|_| "SETNEWDA DAT update failed")?;
    let moved = HciI3cDevice { dynamic_addr: spare, ..device };
    let verification = check_identity(master, moved);
    // Restore the original address even if the identity comparison failed.
    let restore = dynamic_address_payload(device.dynamic_addr).map_err(|_| "invalid original address")?;
    write_ccc(master, moved, "SETNEWDA restore", I3C_CCC_SETNEWDA, &[restore])?;
    master.attach_i3c_device(device.dat_index, device.dynamic_addr)
        .map_err(|_| "SETNEWDA DAT restore failed")?;
    verification?;
    check_identity(master, device)?;
    bare_metal::info!("CCC SETNEWDA change/restore PASS");
    Ok(())
}

fn discover<Y: FnMut(u32)>(master: &mut I3cHciMaster<'_, Y>,
    table: &mut HciDeviceTable<MAX_I3C_DEVICES>, allocator: &mut DatAllocator)
    -> Result<(), &'static str> {
    let count = master.entdaa_all(table, allocator).map_err(|error| {
        bare_metal::error!("CCC ENTDAA failed: {:?}; partial devices={}", error, table.len());
        "ENTDAA failed"
    })?;
    expect_true(count >= EXPECTED_MIN_DEVICES, "too few I3C devices discovered")?;
    bare_metal::info!("CCC ENTDAA PASS devices={}", count);
    Ok(())
}

#[cfg(feature = "irq")]
#[unsafe(no_mangle)]
pub extern "C" fn interrupt_handler_96() {
    dispatch_i3c_hci_irq(I3C_BUS as usize);
}

use bare_metal::write_all as console_backend_write_all;

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(message) }
}

fn run_ccc_test() -> Result<(), &'static str> {
    bare_metal::info!("=== AST10x0 I3C HCI CCC test ===");

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

    // Reset addresses so the suite can be rerun without rebooting AST2700.
    broadcast_ccc(&mut master, "RSTDAA", I3C_CCC_RSTDAA_BROADCAST, &[])?;
    discover(&mut master, &mut table, &mut dat_allocator)?;
    for device in table.iter() {
        bare_metal::info!("CCC target DA=0x{:02x} PID=0x{:012x} BCR=0x{:02x} DCR=0x{:02x}",
            device.dynamic_addr, device.pid, device.bcr, device.dcr);
        check_identity(&mut master, device)?;
        let _: [u8; 2] = read_ccc(&mut master, device, "GETSTATUS", I3C_CCC_GETSTATUS)?;
        test_lengths(&mut master, device)?;
        test_mxds(&mut master, device)?;
        test_activity(&mut master, device)?;
        if device.bcr & BCR_IBI_CAPABLE != 0 {
            // Keep the host DAT rejecting unsolicited IBIs: this tests CCC
            // delivery only. No target userspace transmitter should be running.
            write_ccc(&mut master, device, "ENEC(SIR)", I3C_CCC_ENEC_DIRECT, &[I3C_CCC_EVENT_SIR])?;
            write_ccc(&mut master, device, "DISEC(SIR)", I3C_CCC_DISEC_DIRECT, &[I3C_CCC_EVENT_SIR])?;
        } else {
            bare_metal::info!("CCC direct ENEC/DISEC(SIR) SKIP: target has no IBI capability");
        }
    }
    broadcast_ccc(&mut master, "ENEC(SIR)", I3C_CCC_ENEC_BROADCAST, &[I3C_CCC_EVENT_SIR])?;
    broadcast_ccc(&mut master, "DISEC(SIR)", I3C_CCC_DISEC_BROADCAST, &[I3C_CCC_EVENT_SIR])?;
    for command in [I3C_CCC_ENTAS0, I3C_CCC_ENTAS1, I3C_CCC_ENTAS2, I3C_CCC_ENTAS3] {
        bare_metal::info!("CCC ENTAS{} broadcast", command - I3C_CCC_ENTAS0);
        broadcast_ccc(&mut master, "ENTAS", command, &[])?;
        broadcast_ccc(&mut master, "ENTAS0 restore", I3C_CCC_ENTAS0, &[])?;
        for device in table.iter() {
            let _: [u8; 2] = read_ccc(&mut master, device, "GETSTATUS", I3C_CCC_GETSTATUS)?;
        }
    }
    if table.len() == 1 {
        let device = table.iter().next().ok_or("missing target")?;
        let mwl: [u8; 2] = read_ccc(&mut master, device, "GETMWL", I3C_CCC_GETMWL)?;
        let (mrl, mrl_len) = read_mrl(&mut master, device)?;
        broadcast_ccc(&mut master, "SETMWL", I3C_CCC_SETMWL_BROADCAST, &mwl)?;
        let after: [u8; 2] = read_ccc(&mut master, device, "GETMWL", I3C_CCC_GETMWL)?;
        expect_true(after == mwl, "broadcast SETMWL readback mismatch")?;
        broadcast_ccc(&mut master, "SETMRL", I3C_CCC_SETMRL_BROADCAST, &mrl[..mrl_len])?;
        expect_true(read_mrl(&mut master, device)?.0 == mrl, "broadcast SETMRL readback mismatch")?;
        bare_metal::info!("CCC broadcast length round-trip PASS");
    } else {
        bare_metal::info!("CCC broadcast SETMWL/SETMRL SKIP: preserve distinct multi-target limits");
    }
    let spare = (0x09u8..0x78).find(|address| valid_i3c_address(*address)
        && table.by_dynamic_addr(*address).is_none()).ok_or("no spare address for SETNEWDA")?;
    for device in table.iter() {
        test_new_address(&mut master, device, spare)?;
    }

    // Keep the original identities to verify reset/re-enumeration, not only count.
    let mut rediscovered = HciDeviceTable::<MAX_I3C_DEVICES>::new();
    let mut fresh_allocator = DatAllocator::new(dat_entries, ADDRESS_INDEXED_DAT);
    broadcast_ccc(&mut master, "RSTDAA final", I3C_CCC_RSTDAA_BROADCAST, &[])?;
    discover(&mut master, &mut rediscovered, &mut fresh_allocator)?;
    expect_true(rediscovered.len() == table.len(), "rediscovery device count mismatch")?;
    for original in table.iter() {
        let device = rediscovered.by_pid(original.pid).ok_or("PID missing after RSTDAA")?;
        expect_true(device.bcr == original.bcr && device.dcr == original.dcr,
            "identity changed after RSTDAA")?;
        check_identity(&mut master, device)?;
    }
    bare_metal::info!("CCC RSTDAA/ENTDAA rediscovery PASS");

    if TEST_LEGACY_DIRECT_RSTDAA {
        expect_true(rediscovered.len() == 1, "direct RSTDAA test requires one target")?;
        let device = rediscovered.iter().next().ok_or("missing target")?;
        write_ccc(&mut master, device, "legacy RSTDAA", I3C_CCC_RSTDAA_DIRECT, &[])?;
        fresh_allocator = DatAllocator::new(dat_entries, ADDRESS_INDEXED_DAT);
        discover(&mut master, &mut rediscovered, &mut fresh_allocator)?;
        let after = rediscovered.by_pid(device.pid).ok_or("PID missing after direct RSTDAA")?;
        check_identity(&mut master, after)?;
    } else {
        bare_metal::info!("CCC direct RSTDAA SKIP: legacy command disabled by configuration");
    }
    if let Some(static_addr) = TARGET_STATIC_ADDR {
        expect_true(rediscovered.len() == 1, "SETDASA test requires one target")?;
        let device = rediscovered.iter().next().ok_or("missing target")?;
        expect_true(valid_i3c_address(static_addr) && static_addr != 0x08,
            "invalid or conflicting static address")?;
        broadcast_ccc(&mut master, "RSTDAA before SETDASA", I3C_CCC_RSTDAA_BROADCAST, &[])?;
        master.attach_static_i3c_device(device.dat_index, static_addr)
            .map_err(|_| "SETDASA static DAT setup failed")?;
        let payload = dynamic_address_payload(device.dynamic_addr).map_err(|_| "invalid dynamic address")?;
        write_ccc(&mut master, device, "SETDASA", I3C_CCC_SETDASA, &[payload])?;
        master.attach_i3c_device(device.dat_index, device.dynamic_addr)
            .map_err(|_| "SETDASA dynamic DAT update failed")?;
        check_identity(&mut master, device)?;
        bare_metal::info!("CCC SETDASA PASS");
    } else {
        bare_metal::info!("CCC SETDASA SKIP: no target static address configured");
    }
    bare_metal::info!("=== CCC suite complete: enabled cases passed; review SKIP lines ===");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // SAFETY: Test startup owns SCU and the I3C controllers; no transfers are active.
    unsafe {
        bare_metal::i3c_setup::prepare_and_show(&[0], false);
    }
    let sentinel: &[u8] = match run_ccc_test() {
        Ok(()) => b"TEST_RESULT:PASS\n",
        Err(error) => {
            bare_metal::error!("I3C HCI CCC test failed: {}", error as &str);
            b"TEST_RESULT:FAIL\n"
        }
    };

    console_backend_write_all(sentinel);
    #[expect(clippy::empty_loop)]
    loop {}
}
