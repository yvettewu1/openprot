// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Board-configured AST1040 PIO controller/target loopback with physical jumpers.
#![no_std]
#![no_main]

use ast10x0_i3c_hci::{CommandDescriptor, ControllerEvent, IoMode, TargetEvent};
use bare_metal::i3c_loopback::Result;

const POLLS: u32 = 1_000_000;
const ROUNDS: u8 = 6; // More transfers than the four-entry command ring.
const IBI: [u8; 2] = [0xae, 0x2b];

fn run() -> Result<()> {
    // SAFETY: standalone startup owns SCU, all HCI instances and I3C DMA.
    unsafe {
        bare_metal::i3c_loopback::run(
            IoMode::Pio,
            IoMode::Pio,
            CommandDescriptor::V1,
            |core, peer_core, master, peer, pid| {
                let mut peer_rx = [0u8; 64];
                let device = bare_metal::i3c_loopback::discover_and_enable_ibi(
                    core,
                    peer_core,
                    master,
                    peer,
                    pid,
                    &mut peer_rx,
                    POLLS,
                )?;

                for round in 0..ROUNDS {
                    let written = [0x11, round, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
                    let expected = [0xaa, round, 0xcc, 0xdd, 0xee, 0x12, 0x34, 0x56];

                    master.write(device, &written).map_err(|e| {
                        bare_metal::error!("PIO write: {:?}", e);
                        "PIO write failed"
                    })?;
                    // DAA/ENEC notifications may precede the first private write.
                    let mut received = false;
                    for _ in 0..16 {
                        match bare_metal::i3c_loopback::next_target_event(
                            peer,
                            &mut peer_rx,
                            peer_core
                                .board_config()
                                .ok_or("target config missing")?
                                .poll_budget,
                        )
                        .map_err(|_| "target write receive failed")?
                        {
                            TargetEvent::CccReceived { .. } => {}
                            TargetEvent::WriteReceived { len } => {
                                if len != written.len() || peer_rx[..len] != written {
                                    return Err("target write payload mismatch");
                                }
                                received = true;
                                break;
                            }
                            _ => return Err("unexpected target event during write"),
                        }
                    }
                    if !received {
                        return Err("target write event not found");
                    }
                    let _ = core.clear_hc_irq();

                    bare_metal::info!("ROUND {} PIO WRITE PASS", round);

                    peer.queue_ibi(&IBI)
                        .map_err(|_| "target queue ibi payload failed")?;
                    // Service target completion and controller reception together;
                    // never block one port waiting for software on the other port.
                    peer.start_ibi().map_err(|e| {
                        bare_metal::error!("Target IBI request: {:?}", e);
                        "target IBI request failed"
                    })?;

                    bare_metal::i3c_setup::spin_sleep(0x10_0000);
                    let mut payload = [0u8; 8];
                    let event = bare_metal::i3c_loopback::next_controller_event(
                        master,
                        &mut payload,
                        core.board_config()
                            .ok_or("controller config missing")?
                            .poll_budget,
                    )?;
                    if !matches!(event, ControllerEvent::TargetInterrupt { dynamic_addr, payload_len }
                        if dynamic_addr == device.dynamic_addr && payload_len == IBI.len())
                        || payload[..IBI.len()] != IBI
                    {
                        return Err("PIO IBI address/payload mismatch");
                    }

                    let mut target_done = false;
                    for _ in 0..POLLS {
                        if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                            bare_metal::error!("Target IBI completion: {:?}", e);
                            "target IBI completion failed"
                        })? {
                            match event {
                                TargetEvent::IbiComplete => {
                                    target_done = true;
                                    break;
                                }
                                _ => return Err("unexpected target event during IBI"),
                            }
                        }
                        core::hint::spin_loop();
                    }

                    if !target_done {
                        return Err("target IBI completion timeout");
                    }

                    peer.queue_read(&expected)
                        .map_err(|_| "target queue read payload failed")?;

                    bare_metal::i3c_setup::spin_sleep(0x10_0000);
                    let mut read = [0u8; 8];
                    let result = master.read(device, &mut read).map_err(|e| {
                        bare_metal::error!("PIO read: {:?}", e);
                        "PIO read failed"
                    })?;

                    if result != expected.len() || read != expected {
                        return Err("PIO read payload/length mismatch");
                    }
                    bare_metal::info!("ROUND {} PIO READ PASS", round);
                    bare_metal::info!("ROUND {} PIO IBI PASS: ae 2b", round);

                    match bare_metal::i3c_loopback::next_target_event(
                        peer,
                        &mut peer_rx,
                        peer_core
                            .board_config()
                            .ok_or("target config missing")?
                            .poll_budget,
                    ) {
                        Ok(TargetEvent::ReadComplete { remaining: 0 }) => {}
                        Ok(TargetEvent::ReadComplete { .. }) => {
                            return Err("target read completion reports remaining data");
                        }
                        Ok(event) => {
                            bare_metal::error!("Unexpected target event: {:?}", event);
                            return Err("unexpected target read event");
                        }
                        Err(error) => {
                            bare_metal::error!("Target read completion: {:?}", error);
                            return Err("target read completion failed");
                        }
                    }
                } //rounds
                Ok(())
            },
        )
    }?;
    bare_metal::info!("PIO LOOPBACK WRITE/READ/IBI PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    bare_metal::i3c_loopback::finish(run(), "PIO")
}
