// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Application-level MCTP notification/private-read flow over PIO.
#![no_std]
#![no_main]
use ast10x0_i3c_hci::{
    Controller, ControllerEvent, I3cController, I3cDevice, I3cHciConfig, I3cHciCore,
};
use bare_metal::write_all as console_backend_write_all;
use cortex_m::peripheral::NVIC;
const EXPECTED_DATA: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
const I3C_SCL_PRESET: u32 = 1_000_000;
type Result<T> = core::result::Result<T, &'static str>;

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

// Protocol logic depends only on the trait, not DAT indices or PIO/DMA.
fn receive_packet(bus: &mut impl I3cController, device: I3cDevice) -> Result<()> {
    let mut notification = [0u8; 128];
    loop {
        match bus.poll_event(&mut notification).map_err(|e| {
            bare_metal::error!("IBI receive: {:?}", e);
            "IBI receive failed"
        })? {
            None => core::hint::spin_loop(),
            Some(ControllerEvent::TargetInterrupt {
                dynamic_addr,
                payload_len,
            }) => {
                if dynamic_addr != device.dynamic_addr {
                    return Err("IBI source mismatch");
                }
                if !(1..=2).contains(&payload_len) || notification[0] != 0xae {
                    return Err("IBI is not an MCTP pending-read notification");
                }
                if payload_len == 2 && notification[1] != ibi_pec(dynamic_addr, notification[0]) {
                    return Err("IBI PEC mismatch");
                }
                // Read before logging: the Linux target has a pending-read timeout.
                let mut packet = [0u8; EXPECTED_DATA.len()];
                let len = bus.read(device, &mut packet).map_err(|e| {
                    bare_metal::error!("Private read: {:?}", e);
                    "private read failed"
                })?;
                bare_metal::info!(
                    "IBI source={:02x} payload={:02x?}",
                    dynamic_addr,
                    &notification[..payload_len]
                );
                bare_metal::info!("PRIVATE READ len={} bytes={:02x?}", len, packet);
                if len != packet.len() || packet != EXPECTED_DATA {
                    return Err("private read data/length mismatch");
                }
                return Ok(());
            }
            Some(_) => return Err("unexpected controller event"),
        }
    }
}

fn run_ibi_read_test() -> Result<()> {
    NVIC::mask(ast1080_pac::Interrupt::i3c);
    // SAFETY: this test owns HCI0 and executes once at boot.
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(0), |_| {}) }
        .map_err(|_| "HCI core creation failed")?;
    core.init_primary(8).map_err(|_| "primary init failed")?;
    core.regs().aspeed_copy_sdr4_to_sdr0();
    core.disable_irq_signals();
    let mut bus = Controller::<_, 8>::new(&mut core, I3C_SCL_PRESET, |_| {})
        .map_err(|_| "controller session creation failed")?
        .with_timeout_us(1_000_000);
    bus.discover().map_err(|e| {
        bare_metal::error!("Discovery: {:?}", e);
        "discovery failed"
    })?;
    let device = (8..0x7f)
        .filter_map(|address| bus.device_by_address(address))
        .find(|device| device.dcr == 0xcc && device.bcr & 6 == 6)
        .ok_or("No MCTP target with IBI payload support discovered")?;
    bare_metal::info!(
        "Target DA={:02x} PID={:012x}",
        device.dynamic_addr,
        device.pid
    );
    bus.enable_ibi(device, 2).map_err(|_| "enable IBI failed")?;
    bare_metal::info!("READY: send 11,22,33,44,55,66,77,88 from AST2700");
    let received = receive_packet(&mut bus, device);
    let cleanup = bus.disable_ibi(device).map_err(|_| "disable IBI failed");
    let stopped = bus.shutdown().map_err(|_| "controller shutdown failed");
    received?;
    cleanup?;
    stopped?;
    bare_metal::info!("IBI + PRIVATE READ DATA PASS");
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
