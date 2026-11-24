// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Hardware register/section smoke test; no target or bus transactions required.
#![no_std]
#![no_main]

use ast10x0_i3c_hci::{I3cHciCore, IoMode};

fn run() -> Result<(), &'static str> {
    let (bus, _) = bare_metal::i3c_setup::test_bus(true, IoMode::Pio)?;
    cortex_m::peripheral::NVIC::mask(bare_metal::i3c_setup::bus_interrupt(bus));
    // SAFETY: standalone startup exclusively owns SCU and all HCI instances.
    unsafe { bare_metal::i3c_setup::prepare_loopback(&[bus], false) };
    // Construction selects the configured port but does not enable bus traffic.
    let core = unsafe { I3cHciCore::from_board_with_io(bus, IoMode::Pio, |_| {}) }
        .map_err(|_| "configured core creation failed")?;
    bare_metal::info!("Register smoke: board controller HCI{}", bus);
    bare_metal::i3c_setup::show_registers(core.regs(), "register smoke")
        .map_err(|_| "invalid HCI section registers")?;
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    match run() {
        Ok(()) => bare_metal::write_all(b"TEST_RESULT:PASS\n"),
        Err(error) => {
            bare_metal::error!("Register smoke failed: {}", error);
            bare_metal::write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}
