// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
#![no_std]
#![no_main]

#[cfg(feature = "irq")]
use ast10x0_i3c_hci::dispatch_i3c_hci_irq;
use ast10x0_i3c_hci::{Controller, I3cController, I3cHciConfig, I3cHciCore};
use bare_metal::write_all as console_backend_write_all;
use cortex_m::peripheral::NVIC;

const I3C_BUS: u8 = 0;
const I3C_SCL_PRESET: u32 = 1_000_000;
type Result<T> = core::result::Result<T, &'static str>;

#[cfg(feature = "irq")]
#[unsafe(no_mangle)]
pub extern "C" fn interrupt_handler_96() {
    dispatch_i3c_hci_irq(I3C_BUS as usize);
}

fn write_packets(bus: &mut impl I3cController) -> Result<()> {
    let count = bus.discover().map_err(|error| {
        bare_metal::error!("Discovery failed: {:?}", error);
        "discovery failed"
    })?;
    bare_metal::info!("Discovered {} targets", count);
    let mut writes = 0;
    // Resolve through the public interface; applications do not own DAT slots.
    for address in 8..0x7f {
        let Some(device) = bus.device_by_address(address) else {
            continue;
        };
        if device.dcr != 0xcc {
            continue;
        }
        let packet = bare_metal::i3c_mctp_setup::prepare_mctp_packet(device.dynamic_addr);
        bare_metal::info!(
            "Target DA={:02x} PID={:012x}",
            device.dynamic_addr,
            device.pid
        );
        bare_metal::info!("Open the AST2700 target MCTP device and wait for RX; IBI is not needed");
        // Preserve the existing manual receiver-setup delay (not calibrated).
        bare_metal::i3c_setup::spin_sleep(0x8000_0000);
        bare_metal::info!(
            "MCTP WRITE da={:02x} bytes={:02x?}",
            device.dynamic_addr,
            packet
        );
        bus.write(device, &packet).map_err(|error| {
            bare_metal::error!("MCTP write failed: {:?}", error);
            "MCTP private write failed"
        })?;
        writes += 1;
        bare_metal::info!(
            "MCTP WRITE BUS PASS da={:02x}; verify bytes on AST2700",
            device.dynamic_addr
        );
    }
    if writes == 0 {
        return Err("no MCTP target found for write test");
    }
    Ok(())
}

fn run_mctp_write_test() -> Result<()> {
    bare_metal::info!("=== AST10x0 MCTP private-write test (PIO) ===");
    NVIC::mask(ast1080_pac::Interrupt::i3c);
    // SAFETY: this test owns HCI0 once at boot.
    let mut core = unsafe { I3cHciCore::new(I3cHciConfig::pio(I3C_BUS), |_| {}) }
        .map_err(|_| "core creation failed")?;
    core.init_primary(8).map_err(|_| "primary init failed")?;
    core.regs().aspeed_copy_sdr4_to_sdr0();
    core.disable_irq_signals();
    // Retain the existing optional IRQ configuration; no new IRQ test is added.
    #[cfg(feature = "irq")]
    {
        let ctx = core.isr_ctx().map_err(|_| "ISR context creation failed")?;
        if !ast10x0_i3c_hci::register_i3c_hci_irq_handler(I3C_BUS as usize, ctx) {
            return Err("IRQ registration failed");
        }
        NVIC::unpend(ast1080_pac::Interrupt::i3c);
        core.enable_irq_signals();
        unsafe {
            NVIC::unmask(ast1080_pac::Interrupt::i3c);
        }
    }
    let bus = Controller::<_, 8>::new(&mut core, I3C_SCL_PRESET, |_| {})
        .map_err(|_| "controller session creation failed")?
        .with_timeout_us(1_000_000);
    #[cfg(feature = "irq")]
    let bus = bus.with_irq_events(I3C_BUS as usize);
    let mut bus = bus;
    let outcome = write_packets(&mut bus);
    let stopped = bus.shutdown().map_err(|_| "controller shutdown failed");
    NVIC::mask(ast1080_pac::Interrupt::i3c);
    core.disable_irq_signals();
    #[cfg(feature = "irq")]
    ast10x0_i3c_hci::unregister_i3c_hci_irq_handler(I3C_BUS as usize);
    outcome?;
    stopped
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // SAFETY: Test startup owns SCU and the I3C controllers; no transfers are active.
    unsafe {
        bare_metal::i3c_setup::prepare_and_show(&[0], false);
    }
    let sentinel: &[u8] = match run_mctp_write_test() {
        Ok(()) => b"TEST_RESULT:PASS\n",
        Err(error) => {
            bare_metal::error!("I3C HCI MCTP private-write test failed: {}", error as &str);
            b"TEST_RESULT:FAIL\n"
        }
    };

    console_backend_write_all(sentinel);
    #[expect(clippy::empty_loop)]
    loop {}
}
