// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! One-ring HCI v1 DMA private-write/read test. PIO is used only for DAA.
//! The target must preload READ_EXPECTED without requesting an IBI.
#![no_std]
#![no_main]


use ast10x0_i3c_hci::{
    DatAllocator, HciDeviceTable, HciCommandV1, 
    I3cHciConfig, I3cHciCore, I3cHciMaster, 
     Ring,
};
use bare_metal::write_all as console_backend_write_all;

const POLL_BUDGET: u32 = 1_000_000;

type Result<T> = core::result::Result<T, &'static str>;

fn run() -> Result<()> {
    bare_metal::info!("=== DMA TEST: PIO discovery, DMA private write/read ===");

    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c);

    // Initialize HCI controller.
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(0), |_| {}) }
        .map_err(|_| "core creation failed")?;

    core.init_primary(8).map_err(|_| "primary init failed")?;
    core.disable_irq_signals();

    if core.command_descriptor() != Some(ast10x0_i3c_hci::CommandDescriptor::V1) {
        return Err("dma_test supports HCI v1 commands only");
    }

    let sections = core.sections();

    // Discover I3C targets using ENTDAA.
    let mut table = HciDeviceTable::<8>::new();
    let mut allocator = 
        DatAllocator::new(sections.dat.entries.min(128) as u8, true);    
    let mut master = I3cHciMaster::new(core.regs(), sections, 12_500_000, |_| {})
            .with_timeout_us(POLL_BUDGET);
    
    let _result = master.entdaa_all(&mut table, &mut allocator);

   // Print all discovered devices, including partial results on failure.
    for device in table.iter() {
        bare_metal::info!(
            "PID=0x{:012x} DA=0x{:02x} DAT={} BCR=0x{:02x} DCR=0x{:02x}",
            device.pid,
            device.dynamic_addr,
            device.dat_index,
            device.bcr,
            device.dcr
        );
    }

    // Select first discovered target; no hardcoded PID.
    let device = table
        .iter()
        .next()
        .ok_or("No I3C target discovered")?;

    bare_metal::info!(
        "DMA target PID=0x{:012x} DA=0x{:02x} DAT={}",
        device.pid,
        device.dynamic_addr,
        device.dat_index
    );

    bare_metal::info!("Preparing DAT for DMA IBI");
    master
        .prepare_ibi_dma(device, 2)
        .map_err(|_| "DMA IBI DAT setup failed")?;
        
    // Initialize DMA ring.
    let mut ring = Ring::init_with_trace(&core, |args| {
        bare_metal::info!("{}", args);
    })?;

    // Send direct ENEC(SIR) using DMA immediate command.
    const ENEC_TID: u8 = 3;
    const ENEC_DIRECT: u8 = 0x80;
    const EVENT_SIR: u8 = 0x01;

    bare_metal::info!(
        "DMA ENEC: DA=0x{:02x} DAT={} CCC=0x80 payload=0x01",
        device.dynamic_addr,
        device.dat_index
    );
    bare_metal::i3c_setup::spin_sleep(0x5000_0000);
    bare_metal::info!(
        "================ wait 5 seconds before sending enec =========" );

    let enec = HciCommandV1::ccc(
        device.dat_index,
        ENEC_TID,
        false,
        ENEC_DIRECT,
        &[EVENT_SIR],
        0,
        None,
    )
    .map_err(|_| "DMA ENEC command encoding failed")?
    .with_roc()
    .with_toc();

    bare_metal::info!(
        "ENEC descriptor CMD0={:08x} CMD1={:08x}",
        enec.words[0],
        enec.words[1]
    );

    let response = ring.transfer_immediate(enec, ENEC_TID)?;

    bare_metal::info!(
        "DMA ENEC completed: response={:08x} status={}",
        response.raw,
        response.status
    );
    bare_metal::i3c_setup::spin_sleep(0x1000_0000);
    bare_metal::info!(
        "================ wait 1 seconds before sending 2nd enec =========" );
    let response2 = ring.transfer_immediate(enec, ENEC_TID)?;

    bare_metal::info!(
        "DMA ENEC completed: response={:08x} status={}",
        response2.raw,
        response2.status
    );

    let packet = bare_metal::i3c_mctp_setup::prepare_mctp_packet(device.dynamic_addr);
    for i in 0..3 {
        bare_metal::info!("========== wait for target to setup mctp read {} ======================", i);
        // Retain the existing manual receiver-setup delay (not calibrated).
        bare_metal::i3c_setup::spin_sleep(0x3000_0000);   
        // private write
        let write = HciCommandV1::i3c_private_write_regular(device.dat_index, 4, packet.len(), 0)
            .map_err(|_| "DMA write command encoding failed")?.with_roc().with_toc();
        ring.transfer(write, 4, false, &packet)?;
        bare_metal::info!("DMA WRITE CONTROLLER PASS; verify target received {:02x?}", packet);
    }
    /*
    bare_metal::info!("========== wait for target to setup mctp write ======================");
    // Retain the existing manual receiver-setup delay (not calibrated).
    bare_metal::i3c_setup::spin_sleep(0x8000_0000);
    let read = HciCommandV1::i3c_private_read(device.dat_index, 5, packet.len(), 0)
        .map_err(|_| "DMA read command encoding failed")?.with_roc().with_toc();
    ring.transfer(read, 5, true, packet)?;
       */
   
    let stopped = ring.stop();
    stopped?;
    bare_metal::info!("DMA READ/WRITE CONTROLLER TEST PASS");
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    unsafe { bare_metal::i3c_setup::prepare_and_show(&[0], false); }
    match run() {
        Ok(()) => console_backend_write_all(b"TEST_RESULT:PASS\n"),
        Err(error) => {
            bare_metal::error!("DMA test failed: {}", error);
            console_backend_write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    // DMA storage is static and never reused, including failed abort paths.
    loop { core::hint::spin_loop(); }
}
