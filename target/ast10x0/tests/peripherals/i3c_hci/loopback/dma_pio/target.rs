// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Board-configured AST1040 DMA controller/PIO target loopback with physical jumpers.
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
            IoMode::Dma,
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
                        bare_metal::error!("DMA write: {:?}", e);
                        "DMA write failed"
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

                    bare_metal::info!("ROUND {} DMA WRITE PASS", round);

                    peer.queue_read(&expected)
                        .map_err(|_| "target queue read payload failed")?;
                    bare_metal::i3c_setup::spin_sleep(0x10_0000);
                    let mut read = [0u8; 8];
                    let result = master.read(device, &mut read).map_err(|e| {
                        bare_metal::error!("DMA read: {:?}", e);
                        "DMA read failed"
                    })?;

                    // Always collect the target response before interpreting the
                    // master's short read. This preserves the target's actual error.
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
                            bare_metal::error!("Unexpected target read event: {:?}", event);
                            return Err("unexpected target event after read");
                        }
                        Err(error) => {
                            bare_metal::error!("Target read completion: {:?}", error);
                            return Err("target read completion failed");
                        }
                    }
                    if result != expected.len() || read != expected {
                        return Err("DMA read payload/length mismatch");
                    }
                    bare_metal::info!("ROUND {} DMA READ PASS", round);

                    peer.queue_ibi(&IBI)
                        .map_err(|_| "target IBI queue failed")?;
                    // Service target completion and controller reception together;
                    // never block one port waiting for software on the other port.
                    peer.start_ibi().map_err(|e| {
                        bare_metal::error!("Target IBI request: {:?}", e);
                        "target IBI request failed"
                    })?;
                    let mut payload = [0u8; 8];
                    let mut got_ibi = false;
                    let mut ibi_matches = false;
                    let mut target_done = false;
                    for _ in 0..POLLS {
                        if !got_ibi {
                            if let Some(event) = master
                                .poll_event(&mut payload)
                                .map_err(|_| "DMA IBI receive failed")?
                            {
                                ibi_matches = matches!(event,
                                    ControllerEvent::TargetInterrupt { dynamic_addr, payload_len }
                                        if dynamic_addr == device.dynamic_addr && payload_len == IBI.len()
                                ) && payload[..IBI.len()] == IBI;
                                // Consume the target completion even on a malformed
                                // IBI so its raw error response is not hidden.
                                got_ibi = true;
                            }
                        }
                        if !target_done {
                            if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                                bare_metal::error!("Target IBI completion: {:?}", e);
                                "target IBI completion failed"
                            })? {
                                match event {
                                    TargetEvent::IbiComplete => {
                                        target_done = true;
                                    }
                                    _ => return Err("unexpected target event during IBI"),
                                }
                            }
                        }
                        if got_ibi && target_done {
                            break;
                        }
                        core::hint::spin_loop();
                    }
                    if !got_ibi {
                        return Err("DMA IBI timeout");
                    }
                    if !target_done {
                        return Err("target IBI completion timeout");
                    }
                    if !ibi_matches {
                        return Err("DMA IBI address/payload mismatch");
                    }
                    bare_metal::info!("ROUND {} DMA IBI PASS: ae 2b", round);
                } //rounds
                Ok(())
            },
        )
    }?;
    bare_metal::info!("DMA LOOPBACK WRITE/READ/IBI PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    bare_metal::i3c_loopback::finish(run(), "DMA")
}
