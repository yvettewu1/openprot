// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Capability-gated controller command-v2 DMA test; target command format is unchanged.
#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    CommandDescriptor, DatAllocator, HciDeviceTable, HciTargetConfig, HciTargetEvent, I3cHciConfig,
    I3cHciCore, I3cHciMaster, I3cHciTarget, Ring,
};
use ast10x0_scu_pinctrl::{ScuRegisters, pinctrl};

type Result<T> = core::result::Result<T, &'static str>;
const PID: u64 = 0x1234_5678_9abc;
const POLLS: u32 = 1_000_000;
const ROUNDS: u8 = 6; // More transfers than the four-entry command ring.
const IBI: [u8; 2] = [0xae, 0x2b];
//TODO: refactor this. put 0xae as MDB(mandatory data byte) which goes into command descriptor
//additional ibi data payload : 0x2b, goes into dma buffer

fn run() -> Result<()> {
    bare_metal::info!("=== V2 CONTROLLER DMA LOOPBACK: DMA discovery and transfers ===");
    bare_metal::info!("Wire SCL0-SCL1 and SDA0-SDA1; disconnect AST2700");
    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c);
    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c1);

    // Inspect capabilities before initializing either DMA ring. Never force
    // four-word controller commands onto hardware advertising v1.
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::dma(0), |_| {}) }
        .map_err(|_| "controller core creation failed")?;
    if ((core.regs().hc_capabilities() >> 20) & 3) != 1 {
        return Err("SKIP: controller command V2 unsupported");
    }
    let mut peer_core = unsafe { I3cHciCore::new(I3cHciConfig::dma(1), |_| {}) }
        .map_err(|_| "target core creation failed")?;

    peer_core
        .init_secondary()
        .map_err(|_| "target core init failed")?;
    peer_core.regs().aspeed_copy_sdr4_to_sdr0();
    let peer_ring = Ring::init_with_trace(&peer_core, |args| bare_metal::info!("{}", args))?;
    peer_core.enable_irq_signals();
    let mut peer = I3cHciTarget::new(peer_core.regs(), |_| {})
        .with_timeout_us(POLLS)
        .with_trace(|args| bare_metal::info!("{}", args))
        .with_dma(peer_ring);
    peer.configure(&HciTargetConfig {
        pid: PID,
        bcr: 0x06,
        dcr: 0xcc,
        static_addr: None,
    })
    .map_err(|_| "target identity failed")?;
    peer.enable().map_err(|_| "target enable failed")?;
    if !peer.role().target {
        return Err("HCI1 is not a target");
    }
    bare_metal::info!(
        "Target HCI1 ASPEED_CTRL=0x{:08x} ASPEED_STATUS=0x{:08x} INTR_SUMMARY=0x{:08x} ",
        peer_core.regs().aspeed_ctrl(),
        peer_core.regs().aspeed_status(),
        peer_core.regs().aspeed_intr_summary(),
    );
    core.init_primary(8).map_err(|_| "controller init failed")?;
    core.enable_irq_signals(); // NVIC masked: polling owns all completions.
    bare_metal::info!(
        "After core.init: HC_CONTROL=0x{:08x} BUS_ENABLE={}",
        core.regs().hc_control(),
        (core.regs().hc_control() >> 31) & 1
    );
    bare_metal::info!(
        "Master HCI0 ASPEED_CTRL=0x{:08x} ASPEED_STATUS=0x{:08x} INTR_SUMMARY=0x{:08x}",
        core.regs().aspeed_ctrl(),
        core.regs().aspeed_status(),
        core.regs().aspeed_intr_summary()
    );
    if core.command_descriptor() != Some(CommandDescriptor::V2) {
        return Err("loopback requires command v2");
    }

    // Keep the default timing bank consistent with the slow private-transfer
    // bank, as Zephyr does. In particular, IBI has no submitted command whose
    // mode field could select SDR4. Do this before DAA/ENEC can enable an IBI.
    core.regs().aspeed_copy_sdr4_to_sdr0();
    core.regs()
        .trace_transfer_state("SDR0 aligned with SDR4", |args| {
            bare_metal::info!("{}", args)
        });
    let sections = core.sections();
    let mut devices = HciDeviceTable::<4>::new();
    let mut allocator = DatAllocator::new(sections.dat.entries.min(128) as u8, true);
    let ring = Ring::init_with_trace(&core, |args| bare_metal::info!("{}", args))?;
    let mut master = I3cHciMaster::new(core.regs(), sections, 1_000_000, |_| {})
        .with_command_descriptor(CommandDescriptor::V2)
        .with_dma(ring)
        .map_err(|_| "V2 DMA transport setup failed")?;
    let outcome = (|| -> Result<()> {
        // Both PID-read and address assignment use the DMA ring. ENEC below
        // also verifies the ring remains usable after the terminal DAA NACK.
        master
            .entdaa_all(&mut devices, &mut allocator)
            .map_err(|e| {
                bare_metal::error!("V2 DMA discovery: {:?}", e);
                "V2 DMA discovery failed"
            })?;
        let device = devices.by_pid(PID).ok_or("local target not discovered")?;
        if peer.dynamic_addr() != Some(device.dynamic_addr) {
            return Err("controller and target dynamic addresses differ");
        }
        let status = core.clear_hc_irq();
        bare_metal::info!("core irq status:{:08x}", status);
        bare_metal::info!(
            "Local target DA={:02x} DAT={}",
            device.dynamic_addr,
            device.dat_index
        );
        bare_metal::info!(
            "HCI1 before ibi DMA: RING_STATUS={:08x} INTR={:08x} OP1={:08x} OP2={:08x}",
            peer_core.regs().dma_ring_status(),
            peer_core.regs().dma_ring_intr_status(),
            peer_core.regs().dma_ring_operation1(),
            peer_core.regs().dma_ring_operation2(),
        );
        // Unlike dma_test's diagnostic bypass, this test requires valid DMA
        // completions before reporting a phase PASS.
        master.enable_ibi_with_max_payload(device, 2).map_err(|e| {
            bare_metal::info!(
                "HCI1 DMA: RING_STATUS={:08x} INTR={:08x} OP1={:08x} OP2={:08x}",
                peer_core.regs().dma_ring_status(),
                peer_core.regs().dma_ring_intr_status(),
                peer_core.regs().dma_ring_operation1(),
                peer_core.regs().dma_ring_operation2(),
            );
            bare_metal::error!("DMA ENEC: {:?}", e);
            "DMA ENEC failed"
        })?;
        let mut peer_rx = [0u8; 64];
        // Hardware executes DAA/ENEC; software must drain and validate their
        // notifications before starting the private-transfer state machine.
        let mut enec_seen = false;
        bare_metal::info!(
            "HCI1 DMA: RING_STATUS={:08x} INTR={:08x} OP1={:08x} OP2={:08x}",
            peer_core.regs().dma_ring_status(),
            peer_core.regs().dma_ring_intr_status(),
            peer_core.regs().dma_ring_operation1(),
            peer_core.regs().dma_ring_operation2(),
        );

        bare_metal::info!(
            "HCI1 DMA: IBI_SETUP={:08x} CHUNK={:08x} STATUS_BASE={:08x} DATA_BASE={:08x}",
            peer_core.regs().dma_ring_ibi_setup(),
            peer_core.regs().dma_ring_chunk_control(),
            peer_core.regs().dma_ring_ibi_status_base_lo(),
            peer_core.regs().dma_ring_ibi_data_base_lo(),
        );

        for _ in 0..POLLS {
            if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                bare_metal::error!("HCI1 target DMA poll_event: {:?}", e);
                bare_metal::error!(
                    "HCI1 DMA INTR={:08x} OP1={:08x} OP2={:08x} CHUNK={:08x}",
                    peer_core.regs().dma_ring_intr_status(),
                    peer_core.regs().dma_ring_operation1(),
                    peer_core.regs().dma_ring_operation2(),
                    peer_core.regs().dma_ring_chunk_control(),
                );
                "target CCC receive failed"
            })? {
                match event {
                    HciTargetEvent::Ccc { ccc: 0x07, len: 0 } => {
                        bare_metal::info!("TARGET: ENTDAA completed");
                    }
                    HciTargetEvent::Ccc { ccc: 0x80, len: 1 } if peer_rx[0] == 1 => {
                        enec_seen = true;
                        bare_metal::info!("TARGET: ENEC SIR received");
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
        let status = core.clear_hc_irq();
        bare_metal::info!("core irq status:{:08x}", status);
        // todo: add the peer dma logic
        for round in 0..ROUNDS {
            let written = [0x11, round, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
            let expected = [0xaa, round, 0xcc, 0xdd, 0xee, 0x12, 0x34, 0x56];
            bare_metal::info!(
                "====================== test round{} =====================",
                round
            );
            bare_metal::info!("================master private write================");
            master
                .private_write(device.dat_index, &written)
                .map_err(|e| {
                    bare_metal::error!("DMA write: {:?}", e);
                    "DMA write failed"
                })?;
            // DAA/ENEC notifications may precede the first private write.
            let mut received = false;

            for _ in 0..POLLS {
                if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                    bare_metal::error!("Target DMA write receive: {:?}", e);
                    "target write receive failed"
                })? {
                    match event {
                        HciTargetEvent::Ccc { ccc, len } => {
                            bare_metal::info!("Target CCC={:02x} len={}", ccc, len);
                        }
                        HciTargetEvent::PrivateWrite { len } => {
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
            let status = core.clear_hc_irq();
            bare_metal::info!("core irq status:{:08x}", status);
            bare_metal::info!("ROUND {} DMA WRITE PASS", round);

            bare_metal::info!("================master private read================");
            core.regs()
                .trace_transfer_state("before private read", |args| bare_metal::info!("{}", args));

            peer.queue_ibi_payload(&IBI)
                .map_err(|_| "target IBI queue failed")?;
            // Service target completion and controller reception together;
            // never block one port waiting for software on the other port.
            peer.start_ibi().map_err(|e| {
                bare_metal::error!("Target IBI request: {:?}", e);
                "target IBI request failed"
            })?;
            let ring = master.dma_ring_mut().ok_or("DMA ring missing")?;
            let mut payload = [0u8; 8];
            let mut got_ibi = false;
            let mut ibi_matches = false;
            let mut target_done = false;
            for _ in 0..POLLS {
                if !got_ibi {
                    if let Some(event) = ring.poll_ibi(&mut payload)? {
                        ibi_matches = event.target_addr == device.dynamic_addr
                            && event.payload_len == IBI.len()
                            && payload[..IBI.len()] == IBI;
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
                            HciTargetEvent::IbiComplete { status } => {
                                if status != 0 {
                                    return Err("target IBI completion reports error");
                                }

                                bare_metal::info!("TARGET: DMA IBI completed");
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

            peer.queue_read_data(&expected)
                .map_err(|_| "target queue read payload failed")?;
            bare_metal::i3c_setup::spin_sleep(0x10_0000);
            // Target read-data descriptor is queued before the private read.
            let mut read = [0u8; 8];

            let result = master
                .private_read(device.dat_index, &mut read)
                .map_err(|e| {
                    bare_metal::error!("DMA read: {:?}", e);
                    "DMA private read failed"
                })?;

            bare_metal::info!("Master DMA read complete");
            bare_metal::info!("Master DMA read data: {:02x?}", read);

            if result.rx_len != expected.len() || result.response.data_len != expected.len() {
                return Err("DMA private read length mismatch");
            }
            if read != expected {
                return Err("DMA private read payload mismatch");
            }

            let mut read_complete = false;

            for _ in 0..POLLS {
                if let Some(event) = peer.poll_event(&mut peer_rx).map_err(|e| {
                    bare_metal::error!("Target DMA read completion: {:?}", e);
                    "target DMA read completion failed"
                })? {
                    match event {
                        HciTargetEvent::ReadComplete { len } => {
                            // This is the target TX response field, not the
                            // controller's received-byte count. Successful
                            // target transmission reports zero here. Validate
                            // the eight received bytes on the controller above.
                            if len != 0 {
                                return Err("target DMA read completion reports nonzero length");
                            }

                            bare_metal::info!("TARGET DMA READ COMPLETE len={}", len);

                            read_complete = true;
                            break;
                        }

                        HciTargetEvent::Other(response) => {
                            bare_metal::info!(
                                "Target response raw={:08x} tid={} status={}",
                                response.raw,
                                response.tid,
                                response.status
                            );
                        }

                        _ => return Err("unexpected target event during DMA read"),
                    }
                }

                core::hint::spin_loop();
            }

            if !read_complete {
                return Err("target DMA read completion timeout");
            }

            bare_metal::info!("ROUND {} DMA PRIVATE READ PASS", round);
        } //rounds
        Ok(())
    })();
    let ring = master.dma_ring_mut().ok_or("DMA ring missing")?;
    if outcome.is_err() {
        ring.dump_ibi_state();
        bare_metal::info!(
            "Target HC={:08x} DMA={:08x} STATUS1={:08x}",
            peer_core.regs().hc_control(),
            peer_core.regs().intr_status(),
            peer_core.regs().aspeed_slave_status1()
        );
    }
    let stopped = ring.stop();
    peer.disable();
    core.disable_irq_signals();
    outcome?;
    stopped?;
    bare_metal::info!("DMA LOOPBACK WRITE/READ/IBI PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    unsafe {
        bare_metal::i3c_setup::prepare_and_show(&[0, 1], false);
        let scu = ScuRegisters::new_global_unlocked();
        scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);
        scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI1);
    }
    match run() {
        Ok(()) => bare_metal::write_all(b"TEST_RESULT:PASS\n"),
        Err("SKIP: controller command V2 unsupported") => {
            bare_metal::info!("SKIP: controller command V2 unsupported");
            bare_metal::write_all(b"TEST_RESULT:SKIP\n");
        }
        Err(error) => {
            bare_metal::error!("DMA loopback failed: {}", error);
            bare_metal::write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}
