// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
#![no_std]
#![no_main]

use ast10x0_i3c_hci::pio::{STAT_ALL_ERRORS, STAT_IBI_STATUS_THLD};
use ast10x0_i3c_hci::{
    DatAllocator, HciDeviceTable, HciIbiEvent, I3cHciConfig, I3cHciCore, I3cHciMaster,
};
use bare_metal::write_all as console_backend_write_all;
use cortex_m::peripheral::NVIC;

const I3C_BUS: u8 = 0;
//const TARGET_PID: u64 = 0x07ec06014000;
const I3C_SCL_HZ: u32 = 12_500_000;
const TRANSFER_POLL_BUDGET: u32 = 1_000_000;
const MAX_I3C_DEVICES: usize = 8;
const EXPECTED_MIN_DEVICES: usize = 1;
const ADDRESS_INDEXED_DAT: bool = true;
const EXPECTED_DATA: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

fn expect(condition: bool, message: &'static str) -> Result<(), &'static str> {
    if condition { Ok(()) } else { Err(message) }
}
#[allow(dead_code)]
fn ibi_pec(address: u8, mdb: u8) -> u8 {
    let mut crc = 0u8;
    for byte in [(address << 1) | 1, mdb] {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}



fn run_ibi_read_test() -> Result<(), &'static str> {
    bare_metal::info!("=== AST10x0 external AST2700 IBI/private-read test ===");
    // SAFETY: Test target runs once at boot with exclusive access to SCU.

    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(I3C_BUS), |_us| {}) }
        .map_err(|_| "HCI core create failed")?;
    core.regs().aspeed_copy_sdr4_to_sdr0();
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
    expect(sections.dat.entries > 0, "HCI DAT table missing")?;
    expect(sections.dct.entries > 0, "HCI DCT table missing")?;

    // Use the same initialization and discovery as mctp_write polling mode.
    NVIC::mask(ast1080_pac::Interrupt::i3c);
    core.disable_irq_signals();
    bare_metal::info!("PIO mode: polling (IRQ disabled)");

    let dat_entries = if sections.dat.entries > 128 {
        128
    } else {
        sections.dat.entries as u8
    };
    let mut dat_allocator = DatAllocator::new(dat_entries, ADDRESS_INDEXED_DAT);
    let mut table = HciDeviceTable::<MAX_I3C_DEVICES>::new();
    let master = I3cHciMaster::new(core.regs(), sections, I3C_SCL_HZ, |_us| {})
        .with_timeout_us(TRANSFER_POLL_BUDGET)
        .with_trace(|args| bare_metal::log("PIO", args));
    let mut master = master;

    bare_metal::info!(
        "AST2700 target config supplied: PID=0x07ec06014000 DCR=0xcc (informational)"
    );
    bare_metal::info!(
        "ENTDAA start: bus={} SCL={} timeout_budget={} DAT slots={} address_indexed={} descriptor={:?}",
        I3C_BUS,
        I3C_SCL_HZ,
        TRANSFER_POLL_BUDGET,
        dat_entries,
        ADDRESS_INDEXED_DAT,
        master.command_descriptor()
    );
    bare_metal::info!(
        "Timeout callback is a no-op: budget counts polling iterations, not calibrated microseconds"
    );
    //let _ = bare_metal::i3c_setup::show_registers(core.regs(), "before ENTDAA");
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
    //let _ = bare_metal::i3c_setup::show_registers(core.regs(), "after ENTDAA");
    bare_metal::info!("ENTDAA discovered {} I3C devices", count as u32);
    expect(
        count >= EXPECTED_MIN_DEVICES,
        "too few I3C devices discovered",
    )?;
/*
    let device = table.by_pid(TARGET_PID).ok_or_else(|| {
        for found in table.iter() {
            bare_metal::info!(
                "Discovered DA=0x{:02x} PID=0x{:012x}",
                found.dynamic_addr,
                found.pid
            );
        }
        "targets discovered but expected AST2700 PID is missing"
    })?;
    */
    let device = table.iter().next().ok_or("No I3C target discovered")?;

    bare_metal::info!(
        "Using discovered target: DA=0x{:02x} PID=0x{:012x}",
        device.dynamic_addr,
        device.pid
    );
    expect(device.dcr == 0xcc, "target is not MCTP class")?;
    expect(
        device.bcr & 0x06 == 0x06,
        "target lacks IBI request/payload capability",
    )?;
    bare_metal::info!(
        "Target DA=0x{:02x} DAT={} PID=0x{:012x}",
        device.dynamic_addr,
        device.dat_index,
        device.pid
    );
    // Linux pending-read notification: MDB plus optional PEC, not packet bytes.
    // Program the ASPEED receive limit before ENEC, as Linux/Zephyr do.
    master.enable_ibi_with_max_payload(device, 2)
        .map_err(|error| {
            bare_metal::error!("ENEC(SIR): {:?}", error);
            "enable IBI failed"
        })?; 
     //let _ =  master.enable_ibi(device, true);

    // G7 needs both IBI thresholds at one for short MDB notifications.
    // No CPU IRQ is needed for an IBI.
    let threshold = core.regs().pio_queue_thld_ctrl();
    core.regs()
        .write_pio_queue_thld_ctrl((threshold & 0x0000_ffff) | (1 << 24) | (16 << 16));
    bare_metal::info!(
        "IBI setup: QUEUE_THLD=0x{:08x} ASPEED_MST_MRL=0x{:08x}",
        core.regs().pio_queue_thld_ctrl(),
        core.regs().aspeed_ibi_terminate_control()
    );
    bare_metal::info!("READY: IBI enabled; send 11,22,33,44,55,66,77,88 from AST2700 now");
    bare_metal::info!("Waiting indefinitely for IBI; transfer timeouts count polling iterations");
    bare_metal::info!("=====================================================");
    core.regs().write_pio_intr_signal_enable(0x226);
  
    let received = (|| -> Result<(), &'static str> {
        // Wait without repeatedly consuming status/data or timing out before
        // the operator starts the Linux sender. Transfers themselves are bounded.
        loop {
            let status = core.regs().pio_intr_status();
            if status & STAT_ALL_ERRORS != 0 {
                bare_metal::error!("IBI wait PIO error=0x{:08x}", status);
                return Err("PIO error while waiting for IBI");
            }
            if status & STAT_IBI_STATUS_THLD != 0 {
                break;
            }
            core::hint::spin_loop();
        }
        let mut ibi_data = [0u8; 128];
        let event = master
            .next_ibi_event(&table, &mut ibi_data)
            .map_err(|error| {
                bare_metal::error!("Receive IBI: {:?}", error);
                "IBI receive failed"
            })?;
        // next_ibi_event rearms signal bits; keep this test polling-only.
        core.disable_irq_signals();

        let HciIbiEvent::TargetInterrupt {
            device: source,
            status,
            payload_len,
        } = event
        else {
            return Err("unexpected hot-join or unknown IBI source");
        };

        expect(
            source.dynamic_addr == device.dynamic_addr && status.target_addr == device.dynamic_addr,
            "IBI source mismatch",
        )?;
        expect(!status.error, "IBI status reports error")?;
        expect(status.last, "IBI transaction is incomplete")?;
        expect(
            payload_len == 1 || payload_len == 2,
            "unexpected IBI payload length",
        )?;
        //display_pio_reg();
        bare_metal::info!("IBI status raw=0x{:08x} addr=0x{:02x} len={} error={} last={}", status.raw, status.target_addr, payload_len, status.error, status.last);
        bare_metal::info!("IBI payload={:02x?}", &ibi_data[..payload_len]);
       
        expect(
            ibi_data[0] == 0xae,
            "IBI is not MCTP pending-read notification",
        )?; 
        
        if payload_len == 2 {
            expect(
                ibi_data[1] == ibi_pec(device.dynamic_addr, ibi_data[0]),
                "IBI PEC mismatch",
            )?;
        }
        // Read promptly: Linux's pending-read notification has a finite timeout.
        // Payload bytes are independent of the IBI MDB and optional MDB PEC.
        let mut rx = [0u8; EXPECTED_DATA.len()];
        let result = master
            .private_read_addr(&table, device.dynamic_addr, &mut rx)
            .map_err(|error| {
                bare_metal::error!("Private read after IBI: {:?}", error);
                "private read failed"
            })?;
        bare_metal::info!(
            "IBI PASS source=0x{:02x} raw=0x{:08x} payload={:02x?}",
            status.target_addr,
            status.raw,
            &ibi_data[..payload_len]
        );
        bare_metal::info!(
            "PRIVATE READ len={} response=0x{:08x} bytes={:02x?}",
            result.rx_len,
            result.response.raw,
            rx
        );
        expect(
            result.rx_len == rx.len() && result.response.data_len == rx.len(),
            "private read length mismatch",
        )?;
        expect(rx == EXPECTED_DATA, "private read data mismatch")?;
        bare_metal::info!("IBI + PRIVATE READ DATA PASS");
        Ok(())
    })();
    /*
    // Best-effort cleanup, retaining any original data/transport failure.
    let _ = bare_metal::i3c_setup::show_registers(core.regs(), "before disable IBI");
    let cleanup = master.disable_ibi(device).map_err(|_| "disable IBI failed");
    core.disable_irq_signals();
    cleanup?;
    */
    if received.is_err() {
        let _ = bare_metal::i3c_setup::show_registers(core.regs(), "IBI/read failure");
    }
    received?;
    
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // SAFETY: Test startup owns SCU and the controllers; no transfers are active.
    unsafe {
        bare_metal::i3c_setup::prepare_and_show(&[0], false);
    }
    let sentinel: &[u8] = match run_ibi_read_test() {
        Ok(()) => b"TEST_RESULT:PASS\n",
        Err(error) => {
            bare_metal::error!("IBI/private-read test failed: {}", error);
            b"TEST_RESULT:FAIL\n"
        }
    };
    console_backend_write_all(sentinel);
    loop {
        core::hint::spin_loop();
    }
}
