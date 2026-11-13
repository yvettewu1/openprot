// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! AST1040 HCI0 DMA controller <-> HCI1 PIO target, with physical jumpers.
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

fn run() -> Result<()> {
    bare_metal::info!("=== AST1040 DMA LOOPBACK: HCI0 DMA <-> HCI1 PIO ===");
    bare_metal::info!("Wire SCL0-SCL1 and SDA0-SDA1; disconnect AST2700");
    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c);
    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c1);

    let mut peer_core = unsafe { I3cHciCore::new(I3cHciConfig::pio(1), |_| {}) }
        .map_err(|_| "target core creation failed")?;
    peer_core.init().map_err(|_| "target core init failed")?;
    peer_core.disable_irq_signals();
    let mut peer = I3cHciTarget::new(peer_core.regs(), |_| {})
        .with_timeout_us(POLLS)
        .with_trace(|args| bare_metal::info!("{}", args));
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
        "Target HCI1 ASPEED_CTRL=0x{:08x} ASPEED_STATUS=0x{:08x} INTR_SUMMARY=0x{:08x}",
        peer_core.regs().aspeed_ctrl(),
        peer_core.regs().aspeed_status(),
        peer_core.regs().aspeed_intr_summary()
    );
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::dma(0), |_| {}) }
        .map_err(|_| "controller core creation failed")?;
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
    if core.command_descriptor() != Some(CommandDescriptor::V1) {
        return Err("loopback requires HCI v1");
    }

    // Keep the default timing bank consistent with the slow private-transfer
    // bank, as Zephyr does. In particular, IBI has no submitted command whose
    // mode field could select SDR4. Do this before DAA/ENEC can enable an IBI.
    core.regs().aspeed_copy_sdr4_to_sdr0();
    core.regs().trace_transfer_state("SDR0 aligned with SDR4", |args| {
        bare_metal::info!("{}", args)
    });
    let ring = Ring::init_with_trace(&core, |args| bare_metal::info!("{}", args))?;
    let sections = core.sections();
    // Diagnostic: select SDR4 instead of SDR0 for the jumper loopback.
    // This argument selects a timing bank; it does not program a 1 MHz clock.
    // Log the actual counters rather than claiming a measured bus frequency.
    bare_metal::info!("Loopback timing diagnostic: SDR4 selected; PHY counters determine SCL rate");
    let mut master = I3cHciMaster::new(core.regs(), sections, 1_000_000, |_| {})
        .with_dma(ring)
        .map_err(|_| "DMA transport setup failed")?;
    let outcome = (|| -> Result<()> {
        let mut devices = HciDeviceTable::<4>::new();
        let mut allocator = DatAllocator::new(sections.dat.entries.min(128) as u8, true);
        master
            .entdaa_all(&mut devices, &mut allocator)
            .map_err(|e| {
                bare_metal::error!("DMA DAA: {:?}", e);
                "DMA DAA failed"
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
        // Unlike dma_test's diagnostic bypass, this test requires valid DMA
        // completions before reporting a phase PASS.
        master.enable_ibi_with_max_payload(device, 2).map_err(|e| {
            bare_metal::error!("DMA ENEC: {:?}", e);
            "DMA ENEC failed"
        })?;
        let mut peer_rx = [0u8; 64];
        // Hardware executes DAA/ENEC; software must drain and validate their
        // notifications before starting the private-transfer state machine.
        let mut enec_seen = false;
        for _ in 0..POLLS {
            if let Some(event) = peer
                .poll_event(&mut peer_rx)
                .map_err(|_| "target CCC receive failed")?
            {
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
        for round in 0..ROUNDS {
            let written = [0x11, round, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
            let expected = [0xaa, round, 0xcc, 0xdd, 0xee, 0x12, 0x34, 0x56];
            bare_metal::info!("====================== test round{} =====================", round);
            bare_metal::info!("================master private write================");
            master.private_write(device.dat_index, &written)
                .map_err(|e| {
                    bare_metal::error!("DMA write: {:?}", e);
                    "DMA write failed"
                })?;
            // DAA/ENEC notifications may precede the first private write.
            let mut received = false;
            for _ in 0..16 {
                match peer
                    .next_event(&mut peer_rx)
                    .map_err(|_| "target write receive failed")?
                {
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
            if !received {
                return Err("target write event not found");
            }
            let status = core.clear_hc_irq();
            bare_metal::info!("core irq status:{:08x}", status);
            bare_metal::info!("ROUND {} DMA WRITE PASS", round);

            bare_metal::info!("================master private read================");
            bare_metal::i3c_setup::display_pio_reg(1);
            core.regs().trace_transfer_state("before private read", |args| {
                bare_metal::info!("{}", args)
            });
            peer.queue_read_data(&expected)
                .map_err(|_| "target read queue failed")?;    
            bare_metal::i3c_setup::spin_sleep(0x10_0000);
            let mut read = [0u8; 8];
            let result = master.private_read(device.dat_index, &mut read)
                .map_err(|e| {
                    bare_metal::error!("DMA read: {:?}", e);
                    "DMA read failed"
                })?;
            // Print received length and data
            bare_metal::info!("DMA read rx_len={}", result.rx_len);
            bare_metal::info!("DMA read data={:02x?}", read);
            bare_metal::info!("Expected data={:02x?}", expected);
            // Always collect the target response before interpreting the
            // master's short read. This preserves the target's actual error.
            match peer.next_event(&mut peer_rx) {
                Ok(HciTargetEvent::ReadComplete { len }) => {
                    bare_metal::info!("TARGET: read completion length={}", len);
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
            if result.rx_len != expected.len() || read != expected {
                return Err("DMA read payload/length mismatch");
            }
            bare_metal::info!("ROUND {} DMA READ PASS", round);

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
                            HciTargetEvent::Other(response)
                                if response.status == 0
                                    && response.tid == ast10x0_i3c_hci::TID_TARGET_IBI =>
                            {
                                bare_metal::info!("TARGET: IBI completed raw={:08x}", response.raw);
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
        }//rounds
        Ok(())
    })();
    let ring = master.dma_ring_mut().ok_or("DMA ring missing")?;
    if outcome.is_err() {
        ring.dump_ibi_state();
        bare_metal::info!(
            "Target HC={:08x} PIO={:08x} STATUS1={:08x}",
            peer_core.regs().hc_control(),
            peer_core.regs().pio_intr_status(),
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
        Err(error) => {
            bare_metal::error!("DMA loopback failed: {}", error);
            bare_metal::write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}
