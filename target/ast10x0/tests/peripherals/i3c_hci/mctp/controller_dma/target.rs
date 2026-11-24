// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! AST1040 DMA controller echoes arbitrary data received from an AST2700 target.
#![no_std]
#![no_main]

use ast10x0_i3c_hci::IoMode;

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    // SAFETY: standalone boot exclusively owns SCU and all HCI instances.
    let result = unsafe { bare_metal::i3c_mctp_controller::run(IoMode::Dma, false) };
    bare_metal::i3c_mctp_controller::finish(result, IoMode::Dma)
}
