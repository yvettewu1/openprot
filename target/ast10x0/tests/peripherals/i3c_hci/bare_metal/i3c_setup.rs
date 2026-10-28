// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Shared AST1080 I3C reset and clock setup for bare-metal tests.

use core::hint::spin_loop;

const I3C_MASK: u32 = 0x00ff_0000;
const I3C_DMA_MASK: u32 = 0x8000_0000;

/// Stop I3C0-7 clocks, reset I3C0-7 and I3C DMA, then restart the clocks.
///
/// Call during test bring-up, before using any of these controllers or DMA.
/// The caller must own SCU access and ensure SCU writes are enabled.
/// Spin counts preserve the existing test sequence; they are not microseconds.
pub fn reset_and_enable_i3cs(scu: &ast1080_pac::scu::RegisterBlock) {
    // Stop all I3C clocks.
    scu.scu240().write(|w| unsafe { w.bits(I3C_MASK) });
    spin_sleep(100_000);

    // Assert and release I3C reset.
    scu.scu200().write(|w| unsafe { w.bits(I3C_MASK) });
    spin_sleep(1_000_000);
    scu.scu204().write(|w| unsafe { w.bits(I3C_MASK) });
    spin_sleep(1_000_000);

    // Assert and release I3C DMA reset.
    scu.scu220().write(|w| unsafe { w.bits(I3C_DMA_MASK) });
    spin_sleep(100_000);
    scu.scu224().write(|w| unsafe { w.bits(I3C_DMA_MASK) });
    spin_sleep(100_000);

    // Restart I3C clocks.
    scu.scu244().write(|w| unsafe { w.bits(I3C_MASK) });
    spin_sleep(100_000);
}

pub fn spin_sleep(iterations: u32) {
    for _ in 0..iterations {
        spin_loop();
    }
}

/// Display SCU state without changing pinmux or reset/clock registers.
pub fn show_scu(scu: &ast1080_pac::scu::RegisterBlock) {
    let mux = scu.scu400().read().bits();
    crate::info!(
        "SCU400=0x{:08x} I3C0 SCL={} SDA={}",
        mux,
        mux & 7,
        (mux >> 4) & 7
    );
    crate::info!(
        "SCU200(reset)=0x{:08x} SCU220(DMA reset)=0x{:08x} SCU240(clock stop)=0x{:08x}",
        scu.scu200().read().bits(),
        scu.scu220().read().bits(),
        scu.scu240().read().bits()
    );
}

pub fn show_sections(source: &str, sections: &ast10x0_i3c_hci::HciSections) {
    crate::info!(
        "{} DAT offset=0x{:04x} entries={} entry_bytes={}",
        source,
        sections.dat.offset,
        sections.dat.entries,
        sections.dat.entry_size
    );
    crate::info!(
        "{} DCT offset=0x{:04x} entries={} entry_bytes={}",
        source,
        sections.dct.offset,
        sections.dct.entries,
        sections.dct.entry_size
    );
    crate::info!(
        "{} RHS offset=0x{:04x} PIO offset=0x{:04x} EXT offset=0x{:04x}",
        source,
        sections.ring_headers_offset,
        sections.pio_offset,
        sections.ext_caps_offset
    );
}

/// Non-destructive register snapshot; never reads RX/response/IBI FIFO ports.
/// Caller must serialize access and enable the controller clocks first.
pub fn show_registers(
    regs: &ast10x0_i3c_hci::I3cHciRegisters,
    stage: &str,
) -> Result<(), ast10x0_i3c_hci::I3cHciError> {
    use ast10x0_i3c_hci::{discover_sections_from_regs, parse_hci_version};
    let version = regs.hci_version();
    let caps = regs.hc_capabilities();
    let dat = regs.dat_section();
    let dct = regs.dct_section();
    let rhs = regs.ring_headers_section();
    let pio = regs.pio_section();
    let ext = regs.ext_caps_section();
    crate::info!("--- HCI{} {} ---", regs.bus(), stage);
    crate::info!(
        "VERSION=0x{:08x} CAPABILITIES=0x{:08x} CMD_FORMAT={:?}",
        version,
        caps,
        ast10x0_i3c_hci::select_command_descriptor(caps)
    );
    crate::info!(
        "HC_CONTROL=0x{:08x} DEVICE_ADDR=0x{:08x} RESET=0x{:08x} PRESENT=0x{:08x}",
        regs.hc_control(),
        regs.controller_device_addr(),
        regs.reset_control(),
        regs.present_state()
    );
    crate::info!(
        "DAT_RAW=0x{:08x} DCT_RAW=0x{:08x} RHS_RAW=0x{:08x} PIO_RAW=0x{:08x} EXT_RAW=0x{:08x}",
        dat,
        dct,
        rhs,
        pio,
        ext
    );
    crate::info!(
        "HC_INTR_STATUS=0x{:08x} PIO_INTR_STATUS=0x{:08x} PIO_QUEUE_SIZE=0x{:08x}",
        regs.intr_status(),
        regs.pio_intr_status(),
        regs.pio_queue_size()
    );
    crate::info!(
        "ASPEED_CTRL=0x{:08x} ASPEED_STATUS=0x{:08x} INTR_SUMMARY=0x{:08x}",
        regs.aspeed_ctrl(),
        regs.aspeed_status(),
        regs.aspeed_intr_summary()
    );
    let sections =
        discover_sections_from_regs(parse_hci_version(version)?, dat, dct, rhs, pio, ext)?;
    show_sections("hardware", &sections);
    show_table(regs, "DAT", sections.dat, 8);
    show_table(regs, "DCT", sections.dct, 16);
    Ok(())
}

fn show_table(
    regs: &ast10x0_i3c_hci::I3cHciRegisters,
    name: &str,
    section: ast10x0_i3c_hci::TableSection,
    expected_size: u8,
) {
    // Keep diagnostics within the 4 KiB controller window and reported table.
    let end =
        u32::from(section.offset) + u32::from(section.entries) * u32::from(section.entry_size);
    if section.offset == 0
        || section.offset % 4 != 0
        || section.entry_size != expected_size
        || end > 0x1000
    {
        crate::info!("{} contents skipped: absent or invalid table bounds", name);
        return;
    }
    let count = section.entries.min(16);
    crate::info!(
        "{} contents: first {} of {} entries",
        name,
        count,
        section.entries
    );
    for index in 0..count {
        let mut words = [0u32; 4];
        for word in 0..section.entry_size / 4 {
            let Some(value) =
                regs.table_word(section.offset, section.entry_size, index as u8, word)
            else {
                return;
            };
            words[word as usize] = value;
        }
        if expected_size == 8 {
            crate::info!(
                "DAT[{}] @+0x{:04x}: {:08x} {:08x}",
                index,
                u32::from(section.offset) + u32::from(index) * 8,
                words[0],
                words[1]
            );
        } else {
            let entry = ast10x0_i3c_hci::DctEntryV1::from_words(words);
            crate::info!(
                "DCT[{}] @+0x{:04x}: {:08x} {:08x} {:08x} {:08x} PID={:012x} BCR={:02x} DCR={:02x}",
                index,
                u32::from(section.offset) + u32::from(index) * 16,
                words[0],
                words[1],
                words[2],
                words[3],
                entry.pid,
                entry.bcr,
                entry.dcr
            );
        }
    }
}

/// Prepare clocks and display the initial state before any test transfers.
///
/// # Safety
/// Call once at startup with exclusive ownership of SCU, I3C0-3 and I3C DMA.
/// This resets all four controllers; no transfer may be active on any of them.
pub unsafe fn prepare_and_show(buses: &[u8], hv: bool) {
    let scu = unsafe { ast10x0_scu_pinctrl::ScuRegisters::new_global_unlocked() };
    // pin control
    if hv {
        crate::info!("I3C HV setup");
        scu.regs().scu400().write(|w| unsafe { w.bits(0x0) });
        scu.regs().scu408().write(|w| unsafe { w.bits(0x2222) });
    } else {
        crate::info!("I3C LV setup");
        // scu.apply_pinctrl_group(pinctrl::PINCTRL_I3C_HCI0);  
        scu.regs().scu400().write(|w| unsafe { w.bits(0x1111) }); 
        scu.regs().scu408().write(|w| unsafe { w.bits(0x22220000) }); 
    }
     crate::info!("SCU pinctrl applied for I3C HCI0");
    let scu400 = scu.regs().scu400().read().bits();
    let scu408 = scu.regs().scu408().read().bits();
    crate::info!(
        "SCU400 [0x74C02400] = 0x{:08x}, I3C0 SCL mux={} SDA mux={}, SCU400 = 0x{:08x}",
        scu400,
        scu400 & 0x7,
        (scu400 >> 4) & 0x7,
        scu408
    );


    reset_and_enable_i3cs(scu.regs());
    show_scu(scu.regs());
    for &bus in buses {
        let Some(regs) = (unsafe { ast10x0_i3c_hci::I3cHciRegisters::new(bus) }) else {
            crate::error!("Invalid diagnostic HCI bus {}", bus);
            continue;
        };
        if let Err(error) = show_registers(&regs, "startup, before core init") {
            crate::error!("HCI{} startup decode error: {:?}", bus, error);
        }
    }
}
