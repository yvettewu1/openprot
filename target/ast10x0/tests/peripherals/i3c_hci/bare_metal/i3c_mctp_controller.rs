// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! MCTP controller echo over either transport: IBI, read, then write the same bytes.

use ast10x0_i3c_hci::{CommandDescriptor, ControllerEvent, I3cController, I3cDevice, IoMode};

pub type Result<T> = core::result::Result<T, &'static str>;
const ROUNDS: u8 = 6;
const RX_CAPACITY: usize = 128;

fn wait_notification(
    bus: &mut (impl I3cController + ?Sized),
    device: I3cDevice,
    io: IoMode,
    notification: &mut [u8],
) -> Result<usize> {
    loop {
        match bus.poll_event(notification).map_err(|error| {
            crate::error!("IBI receive failed: {:?}", error);
            "IBI receive failed"
        })? {
            None => core::hint::spin_loop(),
            Some(ControllerEvent::TargetInterrupt {
                dynamic_addr,
                payload_len,
            }) => {
                if dynamic_addr != device.dynamic_addr {
                    if io == IoMode::Dma {
                        // Preserve the DMA diagnostic's source-filter policy.
                        crate::error!("Ignoring IBI from unexpected source {:02x}", dynamic_addr);
                        continue;
                    }
                    return Err("IBI source mismatch");
                }
                let valid_len = match io {
                    IoMode::Dma => payload_len == 2,
                    IoMode::Pio => (1..=2).contains(&payload_len),
                };
                if !valid_len || notification[0] != 0xae {
                    return Err("IBI is not an MCTP pending-read notification");
                }
                // Keep the PIO optional-PEC check and the DMA diagnostic policy.
                if io == IoMode::Pio
                    && payload_len == 2
                    && notification[1]
                        != crate::i3c_mctp_setup::ibi_payload(dynamic_addr, notification[0])[1]
                {
                    return Err("IBI PEC mismatch");
                }
                return Ok(payload_len);
            }
            Some(_) => return Err("unexpected controller event"),
        }
    }
}

fn echo_packets(
    bus: &mut (impl I3cController + ?Sized),
    device: I3cDevice,
    io: IoMode,
) -> Result<()> {
    match (io, bus.enable_ibi(device, 2)) {
        (IoMode::Dma, Ok(())) => {
            crate::info!("ENEC completed (diagnostic bypass enabled)");
        }
        (IoMode::Dma, Err(error)) => {
            // Preserve the explicit ENEC diagnostic bypass. Driver response
            // validation and command-fault protection are not bypassed.
            crate::info!("ENEC error (diagnostic bypass): {:?}", error);
        }
        (IoMode::Pio, Err(error)) => {
            crate::error!("Enable IBI failed: {:?}", error);
            return Err("enable IBI failed");
        }
        (IoMode::Pio, Ok(())) => {}
    }
    crate::info!("READY: AST2700 target mctp-i3c -w <data> -r -c 6; echo up to 128 bytes");
    let mut notification = [0u8; 128];
    let mut packet = [0u8; RX_CAPACITY];
    for round in 0..ROUNDS {
        let payload_len = wait_notification(bus, device, io, &mut notification)?;
        // Read promptly: the Linux target has a pending-read timeout.
        let len = bus.read(device, &mut packet).map_err(|error| {
            crate::error!("Private read failed: {:?}", error);
            "private read failed"
        })?;
        let data = packet.get(..len).ok_or("read exceeds receive capacity")?;
        if data.is_empty() {
            return Err("empty packet cannot be echoed");
        }
        crate::info!(
            "IBI source={:02x} payload={:02x?}",
            device.dynamic_addr,
            &notification[..payload_len]
        );
        crate::info!("MCTP RX len={} data={:02x?}", len, data);
        // Echo precisely the actual bytes, without framing, padding or data PEC.
        // The synchronous driver owns/copies the buffer until completion.
        bus.write(device, data).map_err(|error| {
            crate::error!("Echo private write failed: {:?}", error);
            "echo private write failed"
        })?;
        crate::info!("MCTP TX len={} data={:02x?}", len, data);
        crate::info!("ROUND {} {} ECHO PASS", round, mode_name(io));
    }
    Ok(())
}

fn run_exchange(bus: &mut (impl I3cController + ?Sized), io: IoMode) -> Result<()> {
    bus.discover().map_err(|error| {
        crate::error!("Discovery failed: {:?}", error);
        "discovery failed"
    })?;
    let device = (8..0x7f)
        .filter_map(|address| bus.device_by_address(address))
        .find(|device| device.dcr == 0xcc && device.bcr & 6 == 6)
        .ok_or("No MCTP target with IBI payload support discovered")?;
    crate::info!(
        "Target DA={:02x} PID={:012x}",
        device.dynamic_addr,
        device.pid
    );

    let result = echo_packets(bus, device, io);
    if io == IoMode::Dma {
        // Keep the DMA diagnostic's existing stop-only cleanup and ENEC bypass.
        // The shared controller runner stops its ring on success or error.
        return result;
    }
    // PIO attempts IBI cleanup even after a partially successful ENEC or a
    // protocol failure. Preserve the original failure if cleanup also fails.
    let cleanup = bus.disable_ibi(device).map_err(|error| {
        crate::error!("Disable IBI cleanup failed: {:?}", error);
        "disable IBI failed"
    });
    result?;
    cleanup
}

/// # Safety
/// Call once at standalone boot with exclusive ownership of SCU and all HCI ports.
pub unsafe fn run(io: IoMode, irq: bool) -> Result<()> {
    let format = (io == IoMode::Dma).then_some(CommandDescriptor::V1);
    unsafe { crate::i3c_controller::run(io, irq, format, |bus| run_exchange(bus, io)) }
}

fn mode_name(io: IoMode) -> &'static str {
    match io {
        IoMode::Pio => "PIO",
        IoMode::Dma => "DMA",
    }
}

/// Report only after the shared runner has shut down the controller.
pub fn finish(result: Result<()>, io: IoMode) -> ! {
    match result {
        Ok(()) => crate::write_all(b"TEST_RESULT:PASS\n"),
        Err(error) => {
            crate::error!("MCTP {} controller echo failed: {}", mode_name(io), error);
            crate::write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}
