// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Board-configured DMA/DMA loopback using generic controller/target APIs.
#![no_std]
#![no_main]
use ast10x0_i3c_hci::{CommandDescriptor, IoMode};

fn run() -> bare_metal::i3c_loopback::Result<()> {
    // SAFETY: standalone startup owns SCU, all HCI instances and I3C DMA.
    unsafe {
        bare_metal::i3c_loopback::run(
            IoMode::Dma,
            IoMode::Dma,
            CommandDescriptor::V1,
            bare_metal::i3c_loopback::dma_rounds,
        )
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn test_main() -> ! {
    bare_metal::i3c_loopback::finish(run(), "DMA V1")
}
