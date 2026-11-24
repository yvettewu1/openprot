// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! AST1040 PIO target exercised by an external AST2700 MCTP controller.
#![no_std]
#![no_main]

use ast10x0_i3c_hci::IoMode;

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // SAFETY: standalone boot exclusively owns SCU, HCI ports and DMA storage.
    let result = unsafe { bare_metal::i3c_mctp_target::run(IoMode::Pio) };
    bare_metal::i3c_mctp_target::finish(result, IoMode::Pio)
}
