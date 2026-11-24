// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{I3cBusManagement, I3cDevice, IoMode, ccc::*, valid_i3c_address};

const MAX_I3C_DEVICES: usize = 8;
// Accept one external AST2700 target; still enumerate up to MAX_I3C_DEVICES.
const EXPECTED_MIN_DEVICES: usize = 1;
// SETDASA requires a real static address configured on a single target.
const TARGET_STATIC_ADDR: Option<u8> = None;
// Direct RSTDAA is a legacy command; enable only for a target supporting it.
const TEST_LEGACY_DIRECT_RSTDAA: bool = false;
const BCR_IBI_PAYLOAD: u8 = 1 << 2;
const BCR_IBI_CAPABLE: u8 = 1 << 1;
const BCR_MAX_DATA_SPEED: u8 = 1;

fn read_ccc<const N: usize>(
    master: &mut dyn I3cBusManagement,
    device: I3cDevice,
    name: &'static str,
    command: u8,
) -> Result<[u8; N], &'static str> {
    let mut data = [0u8; N];
    let result = master
        .ccc_read(device, command, &mut data)
        .map_err(|error| {
            bare_metal::error!("CCC {} da=0x{:02x}: {:?}", name, device.dynamic_addr, error);
            "CCC read failed"
        })?;
    bare_metal::info!(
        "CCC {} da=0x{:02x} len={} bytes={:02x?}",
        name,
        device.dynamic_addr,
        result,
        data
    );
    expect_true(result == N, "CCC read length mismatch")?;
    Ok(data)
}

fn write_ccc(
    master: &mut dyn I3cBusManagement,
    device: I3cDevice,
    name: &'static str,
    command: u8,
    data: &[u8],
) -> Result<(), &'static str> {
    master.ccc_write(device, command, data).map_err(|error| {
        bare_metal::error!("CCC {} da=0x{:02x}: {:?}", name, device.dynamic_addr, error);
        "CCC direct write failed"
    })?;
    bare_metal::info!(
        "CCC {} direct BUS PASS da=0x{:02x}",
        name,
        device.dynamic_addr
    );
    Ok(())
}

fn broadcast_ccc(
    master: &mut dyn I3cBusManagement,
    name: &'static str,
    command: u8,
    data: &[u8],
) -> Result<(), &'static str> {
    (if command == I3C_CCC_RSTDAA_BROADCAST {
        master.reset_addresses(None)
    } else {
        master.broadcast_ccc(command, data)
    })
    .map_err(|error| {
        bare_metal::error!("CCC {} broadcast: {:?}", name, error);
        "CCC broadcast failed"
    })?;
    // The driver now requests and checks the broadcast completion response.
    bare_metal::info!(
        "CCC {} broadcast BUS PASS; target effect needs readback",
        name
    );
    Ok(())
}

fn check_identity(
    master: &mut dyn I3cBusManagement,
    device: I3cDevice,
) -> Result<(), &'static str> {
    let pid: [u8; 6] = read_ccc(master, device, "GETPID", I3C_CCC_GETPID)?;
    let expected = device.pid.to_be_bytes();
    expect_true(
        pid.as_slice() == &expected[2..],
        "GETPID differs from ENTDAA PID",
    )?;
    let bcr: [u8; 1] = read_ccc(master, device, "GETBCR", I3C_CCC_GETBCR)?;
    expect_true(bcr[0] == device.bcr, "GETBCR differs from ENTDAA BCR")?;
    let dcr: [u8; 1] = read_ccc(master, device, "GETDCR", I3C_CCC_GETDCR)?;
    expect_true(dcr[0] == device.dcr, "GETDCR differs from ENTDAA DCR")?;
    bare_metal::info!("CCC identity PASS da=0x{:02x}", device.dynamic_addr);
    Ok(())
}

fn read_mrl(
    master: &mut dyn I3cBusManagement,
    device: I3cDevice,
) -> Result<([u8; 3], usize), &'static str> {
    if device.bcr & BCR_IBI_PAYLOAD != 0 {
        Ok((read_ccc(master, device, "GETMRL", I3C_CCC_GETMRL)?, 3))
    } else {
        let data: [u8; 2] = read_ccc(master, device, "GETMRL", I3C_CCC_GETMRL)?;
        Ok(([data[0], data[1], 0], 2))
    }
}

fn test_lengths(master: &mut dyn I3cBusManagement, device: I3cDevice) -> Result<(), &'static str> {
    let mwl: [u8; 2] = read_ccc(master, device, "GETMWL", I3C_CCC_GETMWL)?;
    let (mrl, mrl_len) = read_mrl(master, device)?;
    // Exercise the setters without changing the working Linux target's limits.
    // Readback checks encoding/length and target responsiveness, not a change of value.
    write_ccc(master, device, "SETMWL", I3C_CCC_SETMWL_DIRECT, &mwl)?;
    let after: [u8; 2] = read_ccc(master, device, "GETMWL", I3C_CCC_GETMWL)?;
    expect_true(after == mwl, "SETMWL readback mismatch")?;
    write_ccc(
        master,
        device,
        "SETMRL",
        I3C_CCC_SETMRL_DIRECT,
        &mrl[..mrl_len],
    )?;
    expect_true(
        read_mrl(master, device)?.0 == mrl,
        "SETMRL readback mismatch",
    )?;
    bare_metal::info!(
        "CCC direct length round-trip PASS da=0x{:02x}",
        device.dynamic_addr
    );
    Ok(())
}

fn test_mxds(master: &mut dyn I3cBusManagement, device: I3cDevice) -> Result<(), &'static str> {
    if device.bcr & BCR_MAX_DATA_SPEED == 0 {
        bare_metal::info!("CCC GETMXDS SKIP: BCR does not advertise speed limits");
        return Ok(());
    }
    let mut data = [0u8; 5];
    let result = master
        .ccc_read(device, I3C_CCC_GETMXDS, &mut data)
        .map_err(|error| {
            bare_metal::error!("CCC GETMXDS: {:?}", error);
            "GETMXDS failed"
        })?;
    expect_true(
        result == 2 || result == 5,
        "GETMXDS invalid response length",
    )?;
    bare_metal::info!("CCC GETMXDS PASS bytes={:02x?}", &data[..result]);
    Ok(())
}

fn test_activity(master: &mut dyn I3cBusManagement, device: I3cDevice) -> Result<(), &'static str> {
    for command in [
        I3C_CCC_ENTAS0,
        I3C_CCC_ENTAS1,
        I3C_CCC_ENTAS2,
        I3C_CCC_ENTAS3,
    ] {
        bare_metal::info!("CCC ENTAS{} direct", command - I3C_CCC_ENTAS0);
        write_ccc(master, device, "ENTAS", command | 0x80, &[])?;
        // Return to AS0 before the next test; AS1-3 are activity hints.
        write_ccc(master, device, "ENTAS0 restore", I3C_CCC_ENTAS0 | 0x80, &[])?;
        let _: [u8; 2] = read_ccc(master, device, "GETSTATUS", I3C_CCC_GETSTATUS)?;
    }
    Ok(())
}

fn test_new_address(
    master: &mut dyn I3cBusManagement,
    device: I3cDevice,
    spare: u8,
) -> Result<(), &'static str> {
    let moved = master
        .set_address(device, spare)
        .map_err(|_| "SETNEWDA failed")?;
    let verification = check_identity(master, moved);
    // Restore the original address even if the identity comparison failed.
    let restored = master
        .set_address(moved, device.dynamic_addr)
        .map_err(|_| "SETNEWDA restore failed")?;
    verification?;
    check_identity(master, restored)?;
    bare_metal::info!("CCC SETNEWDA change/restore PASS");
    Ok(())
}

fn discover(
    master: &mut dyn I3cBusManagement,
) -> Result<[Option<I3cDevice>; MAX_I3C_DEVICES], &'static str> {
    let count = master.discover().map_err(|error| {
        bare_metal::error!("CCC ENTDAA failed: {:?}", error);
        "ENTDAA failed"
    })?;
    expect_true(
        count >= EXPECTED_MIN_DEVICES,
        "too few I3C devices discovered",
    )?;
    let mut devices = [None; MAX_I3C_DEVICES];
    let mut index = 0;
    for address in 8..0x7f {
        if let Some(device) = master.device_by_address(address) {
            let slot = devices.get_mut(index).ok_or("too many targets")?;
            *slot = Some(device);
            index += 1;
        }
    }
    bare_metal::info!("CCC ENTDAA PASS devices={}", count);
    Ok(devices)
}

use bare_metal::write_all as console_backend_write_all;

fn expect_true(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(message) }
}

// Re-read through the same configured PHY/transport after an identity mismatch.
// Do not sweep descriptor rates independently of the board's PHY counters.
fn diagnose_identity_reads(master: &mut dyn I3cBusManagement, device: I3cDevice) {
    let _ = check_identity(master, device);
    bare_metal::info!("CCC identity diagnostic complete; original failure preserved");
}

fn controller_address() -> u8 {
    use ast10x0_i3c_hci::{BusRole, config};
    match config::BUSES[config::CONTROLLER_BUS as usize].role {
        BusRole::Controller { dynamic_addr } => dynamic_addr,
        _ => unreachable!("board controller validated by runner"),
    }
}

fn run_ccc_test(master: &mut dyn I3cBusManagement) -> Result<(), &'static str> {
    bare_metal::info!("=== I3C controller CCC test ===");
    // Reset addresses so the suite can be rerun without rebooting AST2700.
    broadcast_ccc(master, "RSTDAA", I3C_CCC_RSTDAA_BROADCAST, &[])?;
    let table = discover(master)?;
    for device in table.iter().flatten().copied() {
        bare_metal::info!(
            "CCC target DA=0x{:02x} PID=0x{:012x} BCR=0x{:02x} DCR=0x{:02x}",
            device.dynamic_addr,
            device.pid,
            device.bcr,
            device.dcr
        );
        if let Err(error) = check_identity(master, device) {
            if error == "GETPID differs from ENTDAA PID" {
                diagnose_identity_reads(master, device);
            }
            return Err(error);
        }
        let _: [u8; 2] = read_ccc(master, device, "GETSTATUS", I3C_CCC_GETSTATUS)?;
        test_lengths(master, device)?;
        test_mxds(master, device)?;
        test_activity(master, device)?;
        if device.bcr & BCR_IBI_CAPABLE != 0 {
            // Keep the host DAT rejecting unsolicited IBIs: this tests CCC
            // delivery only. No target userspace transmitter should be running.
            write_ccc(
                master,
                device,
                "ENEC(SIR)",
                I3C_CCC_ENEC_DIRECT,
                &[I3C_CCC_EVENT_SIR],
            )?;
            write_ccc(
                master,
                device,
                "DISEC(SIR)",
                I3C_CCC_DISEC_DIRECT,
                &[I3C_CCC_EVENT_SIR],
            )?;
        } else {
            bare_metal::info!("CCC direct ENEC/DISEC(SIR) SKIP: target has no IBI capability");
        }
    }
    broadcast_ccc(
        master,
        "ENEC(SIR)",
        I3C_CCC_ENEC_BROADCAST,
        &[I3C_CCC_EVENT_SIR],
    )?;
    broadcast_ccc(
        master,
        "DISEC(SIR)",
        I3C_CCC_DISEC_BROADCAST,
        &[I3C_CCC_EVENT_SIR],
    )?;
    for command in [
        I3C_CCC_ENTAS0,
        I3C_CCC_ENTAS1,
        I3C_CCC_ENTAS2,
        I3C_CCC_ENTAS3,
    ] {
        bare_metal::info!("CCC ENTAS{} broadcast", command - I3C_CCC_ENTAS0);
        broadcast_ccc(master, "ENTAS", command, &[])?;
        broadcast_ccc(master, "ENTAS0 restore", I3C_CCC_ENTAS0, &[])?;
        for device in table.iter().flatten().copied() {
            let _: [u8; 2] = read_ccc(master, device, "GETSTATUS", I3C_CCC_GETSTATUS)?;
        }
    }
    if table.iter().flatten().count() == 1 {
        let device = table
            .iter()
            .flatten()
            .copied()
            .next()
            .ok_or("missing target")?;
        let mwl: [u8; 2] = read_ccc(master, device, "GETMWL", I3C_CCC_GETMWL)?;
        let (mrl, mrl_len) = read_mrl(master, device)?;
        broadcast_ccc(master, "SETMWL", I3C_CCC_SETMWL_BROADCAST, &mwl)?;
        let after: [u8; 2] = read_ccc(master, device, "GETMWL", I3C_CCC_GETMWL)?;
        expect_true(after == mwl, "broadcast SETMWL readback mismatch")?;
        broadcast_ccc(master, "SETMRL", I3C_CCC_SETMRL_BROADCAST, &mrl[..mrl_len])?;
        expect_true(
            read_mrl(master, device)?.0 == mrl,
            "broadcast SETMRL readback mismatch",
        )?;
        bare_metal::info!("CCC broadcast length round-trip PASS");
    } else {
        bare_metal::info!(
            "CCC broadcast SETMWL/SETMRL SKIP: preserve distinct multi-target limits"
        );
    }
    let spare = (0x09u8..0x78)
        .find(|address| {
            valid_i3c_address(*address)
                && *address != controller_address()
                && master.device_by_address(*address).is_none()
        })
        .ok_or("no spare address for SETNEWDA")?;
    for device in table.iter().flatten().copied() {
        test_new_address(master, device, spare)?;
    }

    // Keep the original identities to verify reset/re-enumeration, not only count.
    broadcast_ccc(master, "RSTDAA final", I3C_CCC_RSTDAA_BROADCAST, &[])?;
    let mut rediscovered = discover(master)?;
    expect_true(
        rediscovered.iter().flatten().count() == table.iter().flatten().count(),
        "rediscovery device count mismatch",
    )?;
    for original in table.iter().flatten().copied() {
        let device = master
            .device_by_pid(original.pid)
            .ok_or("PID missing after RSTDAA")?;
        expect_true(
            device.bcr == original.bcr && device.dcr == original.dcr,
            "identity changed after RSTDAA",
        )?;
        check_identity(master, device)?;
    }
    bare_metal::info!("CCC RSTDAA/ENTDAA rediscovery PASS");

    if TEST_LEGACY_DIRECT_RSTDAA {
        expect_true(
            rediscovered.iter().flatten().count() == 1,
            "direct RSTDAA test requires one target",
        )?;
        let device = rediscovered
            .iter()
            .flatten()
            .copied()
            .next()
            .ok_or("missing target")?;
        master
            .reset_addresses(Some(device))
            .map_err(|_| "legacy RSTDAA failed")?;
        rediscovered = discover(master)?;
        let after = master
            .device_by_pid(device.pid)
            .ok_or("PID missing after direct RSTDAA")?;
        check_identity(master, after)?;
    } else {
        bare_metal::info!("CCC direct RSTDAA SKIP: legacy command disabled by configuration");
    }
    if let Some(static_addr) = TARGET_STATIC_ADDR {
        expect_true(
            rediscovered.iter().flatten().count() == 1,
            "SETDASA test requires one target",
        )?;
        let device = rediscovered
            .iter()
            .flatten()
            .copied()
            .next()
            .ok_or("missing target")?;
        expect_true(
            valid_i3c_address(static_addr) && static_addr != controller_address(),
            "invalid or conflicting static address",
        )?;
        broadcast_ccc(
            master,
            "RSTDAA before SETDASA",
            I3C_CCC_RSTDAA_BROADCAST,
            &[],
        )?;
        let device = master
            .assign_static_address(static_addr, device)
            .map_err(|_| "SETDASA failed")?;
        check_identity(master, device)?;
        bare_metal::info!("CCC SETDASA PASS");
    } else {
        bare_metal::info!("CCC SETDASA SKIP: no target static address configured");
    }
    bare_metal::info!("=== CCC suite complete: enabled cases passed; review SKIP lines ===");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // SAFETY: standalone boot exclusively owns SCU and all HCI ports.
    let outcome = unsafe {
        bare_metal::i3c_controller::run(IoMode::Pio, cfg!(feature = "irq"), None, run_ccc_test)
    };
    let sentinel: &[u8] = match outcome {
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
