// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Shared hardware lifecycle for standalone loopback tests.
//! Transfer ordering and protocol assertions belong to the individual tests.

use ast10x0_i3c_hci::{
    CommandDescriptor, Controller, ControllerEvent, I3cController, I3cDevice, I3cHciCore,
    I3cHciMaster, I3cHciTarget, I3cTarget, IoMode, Ring, TargetEvent,
};

pub type Result<T> = core::result::Result<T, &'static str>;
pub type Core = I3cHciCore<fn(u32)>;
pub type Master<'a> = Controller<'a, fn(u32), 4>;
pub type Target<'a> = I3cHciTarget<'a, fn(u32)>;

const V2_UNSUPPORTED: &str = "SKIP: controller command V2 unsupported";

fn no_yield(_: u32) {}

/// Initialize the dedicated ports, run the test, then stop controller DMA
/// before shutting down the target. The original test error takes precedence
/// over cleanup errors, but both sides are always asked to stop after the body.
/// The borrowed sessions cannot escape this call or outlive their cores.
///
/// # Safety
/// Call once at standalone test startup with exclusive ownership of SCU, all
/// HCI instances and I3C DMA. Board preparation resets every HCI instance.
pub unsafe fn run(
    controller_io: IoMode,
    target_io: IoMode,
    format: CommandDescriptor,
    test: impl FnOnce(&Core, &Core, &mut dyn I3cController, &mut dyn I3cTarget, u64) -> Result<()>,
) -> Result<()> {
    let (controller_bus, controller_config) = super::i3c_setup::test_bus(true, controller_io)?;
    let (target_bus, target_config) = super::i3c_setup::test_bus(false, target_io)?;
    let identity = target_config
        .role
        .identity()
        .ok_or("target identity missing")?;
    cortex_m::peripheral::NVIC::mask(super::i3c_setup::bus_interrupt(controller_bus));
    cortex_m::peripheral::NVIC::mask(super::i3c_setup::bus_interrupt(target_bus));
    unsafe { super::i3c_setup::prepare_loopback(&[controller_bus, target_bus], false) };
    crate::info!(
        "Loopback: HCI{} {:?} -> HCI{} {:?}; wire corresponding SCL/SDA",
        controller_bus,
        controller_io,
        target_bus,
        target_io
    );

    // Construction does not touch hardware. V2 must skip before either ring
    // starts; all versions still initialize and enable the target first.
    let mut core =
        unsafe { Core::from_board_with_io(controller_bus, controller_io, no_yield as fn(u32)) }
            .map_err(|_| "controller core creation failed")?;
    if format == CommandDescriptor::V2 && ((core.regs().hc_capabilities() >> 20) & 3) != 1 {
        return Err(V2_UNSUPPORTED);
    }
    let mut peer_core =
        unsafe { Core::from_board_with_io(target_bus, target_io, no_yield as fn(u32)) }
            .map_err(|_| "target core creation failed")?;
    let mut diagnosed = false;
    let outcome = (|| -> Result<()> {
        peer_core
            .init_configured()
            .map_err(|_| "target core init failed")?;
        peer_core.trace_timing(|args| crate::info!("{}", args));
        let peer_ring = if target_io == IoMode::Dma {
            let ring = Ring::init(&peer_core).map_err(|error| {
                crate::error!("HCI{} target DMA init failed: {}", target_bus, error);
                error.message()
            })?;
            peer_core.enable_irq_signals();
            Some(ring)
        } else {
            peer_core.disable_irq_signals();
            None
        };
        let mut peer = Target::new(peer_core.regs(), no_yield as fn(u32))
            .with_poll_budget(target_config.poll_budget);
        if let Some(ring) = peer_ring {
            peer = peer.with_dma(ring).map_err(|error| {
                crate::error!(
                    "HCI{} target DMA attachment failed: {:?}",
                    target_bus,
                    error
                );
                "target DMA transport mismatch"
            })?;
        }

        let outcome = (|| -> Result<()> {
            peer.configure(&identity)
                .map_err(|_| "target identity failed")?;
            peer.enable_with_mode(target_config.role.init_mode())
                .map_err(|_| "target enable failed")?;
            if !peer.role().target {
                return Err("target HCI is not a target");
            }
            core.init_configured()
                .map_err(|_| "controller init failed")?;
            core.trace_timing(|args| crate::info!("{}", args));
            core.enable_irq_signals(); // NVIC masked: polling owns all completions.
            if core.command_descriptor() != Some(format) {
                return Err(match format {
                    CommandDescriptor::V1 => "loopback requires HCI v1",
                    CommandDescriptor::V2 => "loopback requires command v2",
                });
            }
            let ring = if controller_io == IoMode::Dma {
                Some(Ring::init(&core).map_err(|error| {
                    crate::error!(
                        "HCI{} controller DMA init failed: {}",
                        controller_bus,
                        error
                    );
                    error.message()
                })?)
            } else {
                None
            };
            let mut master = I3cHciMaster::new(
                core.regs(),
                core.sections(),
                controller_config.scl_hz,
                no_yield as fn(u32),
            )
            .with_poll_budget(controller_config.poll_budget)
            .with_command_descriptor(format);
            if let Some(ring) = ring {
                master = master.with_dma(ring).map_err(|error| {
                    crate::error!(
                        "HCI{} controller DMA attachment failed: {:?}",
                        controller_bus,
                        error
                    );
                    "DMA transport setup failed"
                })?;
            }

            let mut master =
                Master::from_master(master).map_err(|_| "controller session setup failed")?;
            let result = test(&core, &peer_core, &mut master, &mut peer, identity.pid);
            if result.is_err() {
                show_failure(&core, &peer_core, controller_io, target_io);
                diagnosed = true;
            }
            // Preserve cleanup order: diagnostics, controller ring stop, target
            // shutdown, controller signal disable. Never return early on stop error.
            let stopped = master.shutdown().map_err(|error| {
                crate::error!("HCI{} controller shutdown failed: {:?}; DMA memory remains reserved if not idle", controller_bus, error);
                "controller shutdown failed"
            });
            result?;
            stopped
        })();
        if outcome.is_err() && !diagnosed {
            show_failure(&core, &peer_core, controller_io, target_io);
            diagnosed = true;
        }
        // This also covers target configuration or controller initialization errors
        // after the target transport was attached, before the test body was entered.
        let target_stopped = peer.shutdown().map_err(|error| {
            crate::error!(
                "HCI{} target shutdown failed: {:?}; DMA memory stays reserved if not idle",
                target_bus,
                error
            );
            "target shutdown failed"
        });
        outcome?;
        target_stopped
    })();
    if outcome.is_err() && !diagnosed {
        show_failure(&core, &peer_core, controller_io, target_io);
    }
    // No setup/cleanup error may leave either port delivering interrupts.
    // NVIC has remained masked throughout. Masking signals does not stop DMA;
    // explicit stop attempts above and in init/attachment handle that separately.
    core.disable_irq_signals();
    peer_core.disable_irq_signals();
    outcome
}

/// Discover the configured local target and validate ENEC before private traffic.
/// Uses the already-selected controller transport/command format. V2 retains
/// its paired DMA discovery and terminal-NACK recovery through discover().
///
/// Discovery storage stays in the controller session. The caller supplies the
/// same target RX buffer used by later rounds and its existing polling budget.
/// Errors return through run()'s normal DMA/target cleanup; there is no bypass.
pub fn discover_and_enable_ibi(
    core: &Core,
    peer_core: &Core,
    master: &mut dyn I3cController,
    peer: &mut dyn I3cTarget,
    pid: u64,
    peer_rx: &mut [u8],
    polls: u32,
) -> Result<I3cDevice> {
    let dma = core.selected_io() == Some(IoMode::Dma);
    let (daa_label, daa_error) = if core.command_descriptor() == Some(CommandDescriptor::V2) {
        ("V2 DMA discovery", "V2 DMA discovery failed")
    } else if dma {
        ("DMA DAA", "DMA DAA failed")
    } else {
        ("PIO DAA", "PIO DAA failed")
    };
    let (enec_label, enec_error) = if dma {
        ("DMA ENEC", "DMA ENEC failed")
    } else {
        ("PIO ENEC", "PIO ENEC failed")
    };
    master.discover().map_err(|error| {
        crate::error!("{}: {:?}", daa_label, error);
        daa_error
    })?;
    let device = master
        .device_by_pid(pid)
        .ok_or("local target not discovered")?;
    if peer.address() != Some(device.dynamic_addr) {
        return Err("controller and target dynamic addresses differ");
    }
    let _ = core.clear_hc_irq();
    crate::info!("Local target DA={:02x}", device.dynamic_addr);

    // Loopback requires a validated ENEC response, unlike MCTP dma_test's
    // diagnostic bypass. MDB plus the second byte retains the two-byte limit.
    master.enable_ibi(device, 2).map_err(|error| {
        crate::error!("{}: {:?}", enec_label, error);
        enec_error
    })?;
    // Hardware executes DAA/ENEC. Drain exactly the existing notification
    // sequence before starting private transfers; do not add waits or retries.
    let mut enec_seen = false;
    let target_dma = peer_core.selected_io() == Some(IoMode::Dma);
    for _ in 0..polls {
        if let Some(event) = peer.poll_event(peer_rx).map_err(|error| {
            if target_dma {
                crate::error!("target HCI DMA poll_event: {:?}", error);
                crate::error!(
                    "target HCI DMA INTR={:08x} OP1={:08x} OP2={:08x} CHUNK={:08x}",
                    peer_core.regs().dma_ring_intr_status(),
                    peer_core.regs().dma_ring_operation1(),
                    peer_core.regs().dma_ring_operation2(),
                    peer_core.regs().dma_ring_chunk_control(),
                );
            }
            "target CCC receive failed"
        })? {
            match event {
                TargetEvent::CccReceived { ccc: 0x07, len: 0 } => {}
                TargetEvent::CccReceived { ccc: 0x80, len: 1 } if peer_rx.first() == Some(&1) => {
                    enec_seen = true;
                }
                _ => return Err("unexpected target CCC/event during setup"),
            }
        }
        if enec_seen && peer.ibi_enabled() {
            break;
        }
        core::hint::spin_loop();
    }
    if !enec_seen || !peer.ibi_enabled() {
        return Err("target ENEC notification or hardware SIR enable missing");
    }
    let _ = core.clear_hc_irq();
    Ok(device)
}

fn show_failure(core: &Core, peer_core: &Core, controller_io: IoMode, target_io: IoMode) {
    super::i3c_setup::show_loopback_failure(core.regs(), controller_io);
    super::i3c_setup::show_loopback_failure(peer_core.regs(), target_io);
}

/// Report only after the shared lifecycle has completed cleanup.
pub fn finish(outcome: Result<()>, label: &str) -> ! {
    match outcome {
        Ok(()) => crate::write_all(b"TEST_RESULT:PASS\n"),
        Err(V2_UNSUPPORTED) => {
            crate::info!("{}", V2_UNSUPPORTED);
            crate::write_all(b"TEST_RESULT:SKIP\n");
        }
        Err(error) => {
            crate::error!("{} loopback failed: {}", label, error);
            crate::write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Test-owned bounded scheduling; the target API itself remains nonblocking.
pub fn next_target_event(
    peer: &mut dyn I3cTarget,
    data: &mut [u8],
    polls: u32,
) -> Result<TargetEvent> {
    for _ in 0..polls.max(1) {
        if let Some(event) = peer.poll_event(data).map_err(|error| {
            crate::error!("Target event: {:?}", error);
            "target event failed"
        })? {
            return Ok(event);
        }
        core::hint::spin_loop();
    }
    Err("target event timeout")
}

pub fn next_controller_event(
    master: &mut dyn I3cController,
    data: &mut [u8],
    polls: u32,
) -> Result<ControllerEvent> {
    for _ in 0..polls.max(1) {
        if let Some(event) = master.poll_event(data).map_err(|error| {
            crate::error!("Controller event: {:?}", error);
            "controller event failed"
        })? {
            return Ok(event);
        }
        core::hint::spin_loop();
    }
    Err("controller event timeout")
}

/// Shared V1/V2 DMA/DMA assertions. Only the controller descriptor capability
/// differs; preserve the same six rounds, IBI-first sequence and read delay.
pub fn dma_rounds(
    core: &Core,
    peer_core: &Core,
    master: &mut dyn I3cController,
    peer: &mut dyn I3cTarget,
    pid: u64,
) -> Result<()> {
    const POLLS: u32 = 1_000_000;
    const ROUNDS: u8 = 6;
    const IBI: [u8; 2] = [0xae, 0x2b];
    let mut peer_rx = [0u8; 64];
    let device = discover_and_enable_ibi(core, peer_core, master, peer, pid, &mut peer_rx, POLLS)?;

    for round in 0..ROUNDS {
        let written = [0x11, round, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let expected = [0xaa, round, 0xcc, 0xdd, 0xee, 0x12, 0x34, 0x56];

        master.write(device, &written).map_err(|e| {
            crate::error!("DMA write: {:?}", e);
            "DMA write failed"
        })?;
        // DAA/ENEC notifications may precede the first private write.
        let mut received = false;

        for _ in 0..POLLS {
            if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                crate::error!("Target DMA write receive: {:?}", e);
                "target write receive failed"
            })? {
                match event {
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
            core::hint::spin_loop();
        }

        if !received {
            return Err("target DMA private write timeout");
        }
        let _ = core.clear_hc_irq();

        crate::info!("ROUND {} DMA WRITE PASS", round);

        peer.queue_ibi(&IBI)
            .map_err(|_| "target IBI queue failed")?;
        // Service target completion and controller reception together;
        // never block one port waiting for software on the other port.
        peer.start_ibi().map_err(|e| {
            crate::error!("Target IBI request: {:?}", e);
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
                    crate::error!("Target IBI completion: {:?}", e);
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
        crate::info!("ROUND {} DMA IBI PASS: ae 2b", round);

        peer.queue_read(&expected)
            .map_err(|_| "target queue read payload failed")?;
        super::i3c_setup::spin_sleep(0x10_0000);
        // Target read-data descriptor is queued before the private read.
        let mut read = [0u8; 8];

        let result = master.read(device, &mut read).map_err(|e| {
            crate::error!("DMA read: {:?}", e);
            "DMA private read failed"
        })?;

        if result != expected.len() {
            return Err("DMA private read length mismatch");
        }
        if read != expected {
            return Err("DMA private read payload mismatch");
        }

        let mut read_complete = false;

        for _ in 0..POLLS {
            if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                crate::error!("Target DMA read completion: {:?}", e);
                "target DMA read completion failed"
            })? {
                match event {
                    TargetEvent::ReadComplete { remaining: len } => {
                        // This is the target TX response field, not the
                        // controller's received-byte count. Successful
                        // target transmission reports zero here. Validate
                        // the eight received bytes on the controller above.
                        if len != 0 {
                            return Err("target DMA read completion reports nonzero length");
                        }

                        read_complete = true;
                        break;
                    }

                    TargetEvent::Other => {}

                    _ => return Err("unexpected target event during DMA read"),
                }
            }

            core::hint::spin_loop();
        }

        if !read_complete {
            return Err("target DMA read completion timeout");
        }

        crate::info!("ROUND {} DMA PRIVATE READ PASS", round);
    } //rounds
    Ok(())
}
