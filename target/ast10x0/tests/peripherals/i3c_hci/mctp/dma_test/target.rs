// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! MCTP notification/private-read test using the controller trait over DMA.
#![no_std]
#![no_main]
use ast10x0_i3c_hci::{
    CommandDescriptor, Controller, ControllerEvent, I3cController, I3cHciConfig, I3cHciCore,
};
use bare_metal::write_all as console_backend_write_all;
type Result<T> = core::result::Result<T, &'static str>;

fn receive_packet(bus: &mut impl I3cController) -> Result<()> {
    bus.discover().map_err(|error| {
        bare_metal::error!("DMA discovery failed: {:?}", error);
        "DMA discovery failed"
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
    // Preserve the requested diagnostic bypass: log ENEC failure and continue.
    // The normal trait implementation still validates command responses.
    match bus.enable_ibi(device, 2) {
        Ok(()) => bare_metal::info!("ENEC completed (diagnostic bypass enabled)"),
        Err(error) => bare_metal::info!("ENEC error (diagnostic bypass): {:?}", error),
    }
    bare_metal::info!("READY: send aa,ee,33,44,55,66,77,88 from AST2700");
    let mut notification = [0u8; 128];
    let mut polls = 0u32;
    loop {
        match bus.poll_event(&mut notification).map_err(|error| {
            bare_metal::error!("DMA IBI receive failed: {:?}", error);
            "DMA IBI receive failed"
        })? {
            Some(ControllerEvent::TargetInterrupt {
                dynamic_addr,
                payload_len,
            }) => {
                if dynamic_addr != device.dynamic_addr {
                    bare_metal::error!("Ignoring IBI from unexpected source {:02x}", dynamic_addr);
                    continue;
                }
                if payload_len != 2 || notification[0] != 0xae {
                    return Err("IBI is not a two-byte MCTP pending-read notification");
                }
                break;
            }
            Some(_) => return Err("unexpected controller event"),
            None => {
                polls = polls.wrapping_add(1);
                if polls % 1_000_000 == 0 {
                    bare_metal::info!("Waiting for DMA IBI...");
                }
                core::hint::spin_loop();
            }
        }
    }
    let expected = [0xaa, 0xee, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    let mut packet = [0u8; 8];
    // Read promptly before printing the notification.
    let len = bus.read(device, &mut packet).map_err(|error| {
        bare_metal::error!("DMA private read failed: {:?}", error);
        "DMA private read failed"
    })?;
    bare_metal::info!("IBI payload={:02x?}", &notification[..2]);
    bare_metal::info!("DMA read length={} data={:02x?}", len, packet);
    if len != expected.len() || packet != expected {
        return Err("DMA private read length or payload mismatch");
    }
    Ok(())
}

fn run() -> Result<()> {
    bare_metal::info!("=== MCTP DMA discovery, IBI, private read ===");
    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c);
    // SAFETY: this test owns HCI0 once at boot.
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::dma(0), |_| {}) }
        .map_err(|_| "core creation failed")?;
    core.init_primary(8).map_err(|_| "primary init failed")?;
    core.enable_irq_signals(); // NVIC remains masked; this is a polling test.
    if core.command_descriptor() != Some(CommandDescriptor::V1) {
        return Err("dma_test supports HCI v1 commands only");
    }
    core.regs().aspeed_copy_sdr4_to_sdr0();
    let mut bus = Controller::<_, 8>::new(&mut core, 1_000_000, |_| {})
        .map_err(|_| "DMA controller session creation failed")?;
    let outcome = receive_packet(&mut bus);
    // Stop DMA on success and failure without exposing the ring to the protocol.
    let stopped = bus.shutdown().map_err(|error| {
        bare_metal::error!("DMA stop failed: {:?}", error);
        "DMA stop failed"
    });
    outcome?;
    stopped?;
    bare_metal::info!("DMA IBI/PRIVATE READ TEST PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    unsafe {
        bare_metal::i3c_setup::prepare_and_show(&[0], false);
    }
    match run() {
        Ok(()) => console_backend_write_all(b"TEST_RESULT:PASS\n"),
        Err(error) => {
            bare_metal::error!("DMA test failed: {}", error);
            console_backend_write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}
