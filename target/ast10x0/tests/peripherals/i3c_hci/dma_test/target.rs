// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! HCI v1 DMA discovery, MCTP IBI, and private-read test.
#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    CommandDescriptor, DatAllocator, HciDeviceTable, I3cHciConfig, I3cHciCore, I3cHciMaster, Ring,
};
use bare_metal::write_all as console_backend_write_all;

type Result<T> = core::result::Result<T, &'static str>;

fn run() -> Result<()> {
    bare_metal::info!("=== DMA TEST: DMA discovery, IBI, private read ===");

    bare_metal::i3c_setup::spin_sleep(0x5000_0000);

    cortex_m::peripheral::NVIC::mask(ast1080_pac::Interrupt::i3c);

    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::dma(0), |_| {}) }
        .map_err(|_| "core creation failed")?;
    core.init_primary(8).map_err(|_| "primary init failed")?;
    core.disable_irq_signals();
    if core.command_descriptor() != Some(CommandDescriptor::V1) {
        return Err("dma_test supports HCI v1 commands only");
    }

    // Rings exist before the first command, including ENTDAA. There is no
    // PIO command submission or PIO-to-DMA transition in this test.
    let ring = Ring::init_with_trace(&core, |args| bare_metal::info!("{}", args))?;
    let sections = core.sections();
    let mut master = I3cHciMaster::new(core.regs(), sections, 12_500_000, |_| {})
        .with_dma(ring)
        .map_err(|_| "DMA master setup failed")?;

    let outcome = (|| -> Result<()> {
        let mut table = HciDeviceTable::<8>::new();
        let mut allocator = DatAllocator::new(sections.dat.entries.min(128) as u8, true);
        bare_metal::info!("ENTDAA through DMA...");
        master
            .entdaa_all(&mut table, &mut allocator)
            .map_err(|error| {
                bare_metal::error!("DMA ENTDAA failed: {:?}", error);
                "DMA discovery failed"
            })?;
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
        let device = table.iter().next().ok_or("No I3C target discovered")?;

        // This configures DAT/payload termination and submits ENEC through
        // the master's selected DMA transport, requiring a successful response.
        bare_metal::info!("Enabling IBI through DMA (max payload=2)...");
        master
            .enable_ibi_with_max_payload(device, 2)
            .map_err(|error| {
                bare_metal::error!("DMA ENEC failed: {:?}", error);
                "DMA IBI enable failed"
            })?;
        bare_metal::info!("READY: IBI enabled; send aa,ee,33,44,55,66,77,88 from AST2700");
        bare_metal::info!("Waiting for DMA IBI (infinite wait)...");

        let ring = master.dma_ring_mut().ok_or("DMA ring missing")?;
        ring.dump_ibi_state();
        let mut ibi_data = [0u8; 128];
        let mut polls = 0u32;
        loop {
            match ring.poll_ibi(&mut ibi_data)? {
                Some(event) => {
                    bare_metal::info!(
                        "IBI received: DA=0x{:02x}, length={}, payload={:02x?}",
                        event.target_addr,
                        event.payload_len,
                        &ibi_data[..event.payload_len]
                    );
                    if event.target_addr != device.dynamic_addr {
                        bare_metal::error!("Unexpected IBI source address");
                        continue;
                    }
                    if event.payload_len != 2 || ibi_data[0] != 0xae {
                        return Err("IBI is not a two-byte MCTP pending-read notification");
                    }
                    break;
                }
                None => {
                    polls = polls.wrapping_add(1);
                    if polls % 1_000_000 == 0 {
                        ring.dump_ibi_state();
                    }
                    core::hint::spin_loop();
                }
            }
        }

        let expected = [0xaa, 0xee, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let mut received = [0u8; 8];
        let result = master
            .private_read(device.dat_index, &mut received)
            .map_err(|error| {
                bare_metal::error!("DMA private read failed: {:?}", error);
                "DMA private read failed"
            })?;
        bare_metal::info!("DMA read length={} data={:02x?}", result.rx_len, received);
        if result.rx_len != expected.len() || received != expected {
            return Err("DMA private read length or payload mismatch");
        }
        Ok(())
    })();

    // Stop the ring on success and failure; static DMA memory stays allocated.
    let ring = master.dma_ring_mut().ok_or("DMA ring missing")?;
    if outcome.is_err() {
        ring.dump_ibi_state();
    }
    let stopped = ring.stop();
    if let Err(error) = stopped {
        bare_metal::error!("DMA stop failed: {}", error);
    }
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
