// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Phase 6: I3C HCI Target Mode Test
//!
//! Purpose:
//! Validate AST1080 MIPI I3C HCI target-mode operation using two HCI
//! controllers on the same device:
//!
//!     I3C0 = master/controller
//!     I3C1 = target
//!
//! The two I3C ports must be physically connected on the board.
//!
//! Test flow:
//!
//! 1. Configure pinmux for I3C0 and I3C1.
//! 2. Initialize I3C1 as an HCI target.
//! 3. Program the target identity:
//!      - PID
//!      - BCR
//!      - DCR
//! 4. Enable target mode and verify HCI1 reports target role.
//! 5. Initialize I3C0 as the HCI master.
//!
//! Dynamic address assignment:
//!
//! 6. I3C0 runs ENTDAA.
//! 7. I3C1 participates as an I3C target and receives a dynamic address.
//! 8. I3C0 reads the discovered device information from DCT and stores the
//!    target in the HCI device table.
//! 9. Find I3C1 by PID and verify I3C1 reports the same assigned dynamic
//!    address through the ASPEED target status register.
//!
//! Private write test:
//!
//! 10. I3C0 sends a private write to I3C1 using the assigned dynamic address.
//! 11. I3C1 reads its target response, drains the RX FIFO, and verifies the
//!     received payload.
//!
//! Private read test:
//!
//! 12. I3C1 queues known response data into the target TX path.
//! 13. I3C0 performs a private read from I3C1 using the assigned dynamic
//!     address.
//! 14. I3C0 verifies the returned data.
//! 15. I3C1 verifies the target read-completion response.
//!
//! Expected result:
//!
//!     I3C0 ENTDAA             -> I3C1 receives a dynamic address
//!     I3C0 private write      -> I3C1 receives and verifies data
//!     I3C1 queues read data   -> I3C0 private read receives and verifies data
//!
//! This test validates the Phase 6 target-mode foundation:
//!
//!     - target identity configuration
//!     - target enable/mode selection
//!     - dynamic-address assignment through ENTDAA
//!     - target RX/private-write path
//!     - target TX/private-read path
//!
//! IBI and Hot-Join are not part of this test; they belong to Phase 7.

#![no_std]
#![no_main]

use ast10x0_i3c_hci::{
    DatAllocator,
    HciDeviceTable,
    HciTargetConfig,
    HciTargetEvent,
    I3cHciConfig,
    I3cHciCore,
    I3cHciMaster,
    I3cHciTarget,
};
use ast10x0_scu_pinctrl::{pinctrl, ScuRegisters};

use codegen as _;
use entry as _;
use target_common::{declare_target, TargetInterface};

const UART5_BASE: *const ast1080_pac::uart::RegisterBlock =
    0x7e78_4000 as *const _;

/*
 * Test topology:
 *
 *     AST1080 HCI0                  AST1080 HCI1
 *     master/controller             target
 *
 *         SDA0 ----------------------- SDA1
 *         SCL0 ----------------------- SCL1
 *
 * The two ports must be physically connected.
 */
const MASTER_BUS: u8 = 0;
const TARGET_BUS: u8 = 1;

const I3C_SCL_HZ: u32 = 12_500_000;
const TIMEOUT_US: u32 = 1_000_000;

/*
 * HCI/ASPEED DAA currently expects address-indexed DAT.
 */
const ADDRESS_INDEXED_DAT: bool = true;

/*
 * This self-test expects only our HCI1 target, but leave a few entries
 * available so discovery remains easy to debug if another device is present.
 */
const MAX_I3C_DEVICES: usize = 4;

/*
 * Target identity.
 *
 * PID is 48 bits.
 *
 * These values are test values. Replace them with the desired AST1080
 * identity later if needed.
 */
const TARGET_PID: u64 = 0x1234_5678_9abc;
const TARGET_BCR: u8 = 0x00;
const TARGET_DCR: u8 = 0x00;

/*
 * Controller -> target test payload.
 */
const MASTER_WRITE_DATA: [u8; 4] = [
    0x11,
    0x22,
    0x33,
    0x44,
];

/*
 * Target -> controller test payload.
 */
const TARGET_READ_DATA: [u8; 4] = [
    0xaa,
    0xbb,
    0xcc,
    0xdd,
];

#[unsafe(no_mangle)]
pub fn console_backend_write_all(
    buf: &[u8],
) -> pw_status::Result<()> {
    let uart = unsafe { &*UART5_BASE };

    for byte in buf {
        while !uart.uartlsr().read().thre().bit() {}

        uart.uartthr()
            .write(|w| unsafe { w.bits(*byte as u32) });
    }

    Ok(())
}

pub struct Target {}

fn expect_true(
    condition: bool,
    message: &'static str,
) -> Result<(), &'static str> {
    if condition {
        Ok(())
    } else {
        Err(message)
    }
}

fn run_target_mode_test() -> Result<(), &'static str> {
    pw_log::info!(
        "=== AST1080 I3C HCI target-mode self-test ==="
    );

    /*
     * ===============================================================
     * 1. Configure I3C0 and I3C1 pinmux.
     * ===============================================================
     */
    let scu =
        unsafe { ScuRegisters::new_global_unlocked() };

    scu.apply_pinctrl_group(
        pinctrl::PINCTRL_I3C_HCI0,
    );

    scu.apply_pinctrl_group(
        pinctrl::PINCTRL_I3C_HCI1,
    );

    pw_log::info!(
        "pinmux configured: HCI0=master HCI1=target"
    );

    /*
     * ===============================================================
     * 2. Initialize HCI1 TARGET first.
     * ===============================================================
     *
     * The target must be ready before the master starts ENTDAA.
     */
    let mut target_core =
        unsafe {
            I3cHciCore::new(
                I3cHciConfig::pio(TARGET_BUS),
                |_us| {},
            )
        }
        .map_err(|_| {
            "target HCI core create failed"
        })?;

    target_core
        .init()
        .map_err(|_| {
            "target HCI core init failed"
        })?;

    pw_log::info!(
        "HCI{} target core initialized",
        TARGET_BUS as u32,
    );

    /*
     * No static address here.
     *
     * We want ENTDAA to assign the target's dynamic address.
     */
    let target_config = HciTargetConfig {
        pid: TARGET_PID,
        bcr: TARGET_BCR,
        dcr: TARGET_DCR,
        static_addr: None,
    };

    let mut target =
        I3cHciTarget::new(
            target_core.regs(),
            |_us| {},
        )
        .with_timeout_us(
            TIMEOUT_US,
        );

    target
        .configure(&target_config)
        .map_err(|_| {
            "target identity configure failed"
        })?;

    target
        .enable()
        .map_err(|_| {
            "target mode enable failed"
        })?;

    let role = target.role();

    pw_log::info!(
        "HCI{} role target={} secondary={}",
        TARGET_BUS as u32,
        role.target as u32,
        role.secondary as u32,
    );

    expect_true(
        role.target,
        "HCI1 did not enter target mode",
    )?;

    expect_true(
        target.dynamic_addr().is_none(),
        "target unexpectedly has dynamic address before ENTDAA",
    )?;

    pw_log::info!(
        "target identity PID={:04x}_{:08x} BCR={:02x} DCR={:02x}",
        ((TARGET_PID >> 32) & 0xffff) as u32,
        (TARGET_PID & 0xffff_ffff) as u32,
        TARGET_BCR as u32,
        TARGET_DCR as u32,
    );

    /*
     * ===============================================================
     * 3. Initialize HCI0 MASTER.
     * ===============================================================
     */
    let mut master_core =
        unsafe {
            I3cHciCore::new(
                I3cHciConfig::pio(MASTER_BUS),
                |_us| {},
            )
        }
        .map_err(|_| {
            "master HCI core create failed"
        })?;

    master_core
        .init()
        .map_err(|_| {
            "master HCI core init failed"
        })?;

    let sections =
        master_core.sections();

    expect_true(
        sections.dat.entries != 0,
        "master DAT table missing",
    )?;

    expect_true(
        sections.dct.entries != 0,
        "master DCT table missing",
    )?;

    pw_log::info!(
        "HCI{} master core initialized DAT={} DCT={}",
        MASTER_BUS as u32,
        sections.dat.entries as u32,
        sections.dct.entries as u32,
    );

    let dat_entries =
        if sections.dat.entries > 128 {
            128
        } else {
            sections.dat.entries as u8
        };

    let mut dat_allocator =
        DatAllocator::new(
            dat_entries,
            ADDRESS_INDEXED_DAT,
        );

    let mut device_table =
        HciDeviceTable::<MAX_I3C_DEVICES>::new();

    let mut master =
        I3cHciMaster::new(
            master_core.regs(),
            sections,
            I3C_SCL_HZ,
            |_us| {},
        )
        .with_timeout_us(
            TIMEOUT_US,
        );

    /*
     * ===============================================================
     * 4. ENTDAA
     * ===============================================================
     */
    pw_log::info!(
        "starting ENTDAA"
    );

    let count = master
        .entdaa_all(
            &mut device_table,
            &mut dat_allocator,
        )
        .map_err(|_| {
            "ENTDAA failed"
        })?;

    pw_log::info!(
        "ENTDAA discovered {} device(s)",
        count as u32,
    );

    expect_true(
        count != 0,
        "ENTDAA discovered no targets",
    )?;

    /*
     * Dump everything discovered. This helps if another I3C device is
     * physically present on the connected bus.
     */
    for device in device_table.iter() {
        pw_log::info!(
            "device DA={:02x} DAT={} PID={:04x}_{:08x} BCR={:02x} DCR={:02x}",
            device.dynamic_addr as u32,
            device.dat_index as u32,
            ((device.pid >> 32) & 0xffff) as u32,
            (device.pid & 0xffff_ffff) as u32,
            device.bcr as u32,
            device.dcr as u32,
        );
    }

    /*
     * Locate HCI1 by PID rather than assuming it was the first DAA device.
     */
    let target_device = device_table
        .by_pid(TARGET_PID)
        .ok_or(
            "target PID not found after ENTDAA",
        )?;

    let assigned_addr =
        target_device.dynamic_addr;

    pw_log::info!(
        "target discovered: DAT={} DA={:02x}",
        target_device.dat_index as u32,
        assigned_addr as u32,
    );

    /*
     * Confirm the target controller sees the same dynamic address.
     */
    let target_reported_addr =
        target
            .dynamic_addr()
            .ok_or(
                "target has no dynamic address after ENTDAA",
            )?;

    pw_log::info!(
        "target reports dynamic address {:02x}",
        target_reported_addr as u32,
    );

    expect_true(
        target_reported_addr == assigned_addr,
        "master/target dynamic address mismatch",
    )?;

    pw_log::info!(
        "ENTDAA PASS: target assigned DA={:02x}",
        assigned_addr as u32,
    );

    /*
     * ===============================================================
     * 5. PRIVATE WRITE
     *
     * HCI0 master -> HCI1 target
     * ===============================================================
     */
    pw_log::info!(
        "starting private write to DA={:02x}",
        assigned_addr as u32,
    );

    master
        .private_write_addr(
            &device_table,
            assigned_addr,
            &MASTER_WRITE_DATA,
        )
        .map_err(|_| {
            "master private write failed"
        })?;

    pw_log::info!(
        "master private write completed"
    );

    /*
     * The target should now have a target response plus RX FIFO data.
     */
    let mut target_rx = [0u8; 64];

    let event = target
        .next_event(
            &mut target_rx,
        )
        .map_err(|_| {
            "target failed to receive private write"
        })?;

    match event {
        HciTargetEvent::PrivateWrite {
            len,
        } => {
            pw_log::info!(
                "target private write event len={}",
                len as u32,
            );

            expect_true(
                len == MASTER_WRITE_DATA.len(),
                "private-write length mismatch",
            )?;

            expect_true(
                target_rx[..MASTER_WRITE_DATA.len()]
                    == MASTER_WRITE_DATA,
                "private-write data mismatch",
            )?;

            pw_log::info!(
                "private write data: {:02x} {:02x} {:02x} {:02x}",
                target_rx[0] as u32,
                target_rx[1] as u32,
                target_rx[2] as u32,
                target_rx[3] as u32,
            );
        }

        HciTargetEvent::Ccc {
            ccc,
            len,
        } => {
            pw_log::error!(
                "unexpected CCC ccc={:02x} len={}",
                ccc as u32,
                len as u32,
            );

            return Err(
                "unexpected CCC instead of private write",
            );
        }

        HciTargetEvent::ReadComplete {
            len,
        } => {
            pw_log::error!(
                "unexpected read completion len={}",
                len as u32,
            );

            return Err(
                "unexpected read completion",
            );
        }

        HciTargetEvent::Other(_) => {
            return Err(
                "unexpected target response",
            );
        }
    }

    pw_log::info!(
        "PRIVATE WRITE PASS"
    );

    /*
     * ===============================================================
     * 6. PRIVATE READ
     *
     * HCI1 target -> HCI0 master
     * ===============================================================
     *
     * Target read data must be queued BEFORE the controller issues
     * the private read.
     */
    pw_log::info!(
        "queueing target read data"
    );

    target
        .queue_read_data(
            &TARGET_READ_DATA,
        )
        .map_err(|_| {
            "target queue read data failed"
        })?;

    pw_log::info!(
        "target queued {:02x} {:02x} {:02x} {:02x}",
        TARGET_READ_DATA[0] as u32,
        TARGET_READ_DATA[1] as u32,
        TARGET_READ_DATA[2] as u32,
        TARGET_READ_DATA[3] as u32,
    );

    let mut master_rx = [0u8; 4];

    pw_log::info!(
        "master private read from DA={:02x}",
        assigned_addr as u32,
    );

    let result = master
        .private_read_addr(
            &device_table,
            assigned_addr,
            &mut master_rx,
        )
        .map_err(|_| {
            "master private read failed"
        })?;

    expect_true(
        result.rx_len == master_rx.len(),
        "master private-read short response",
    )?;

    pw_log::info!(
        "master received {:02x} {:02x} {:02x} {:02x}",
        master_rx[0] as u32,
        master_rx[1] as u32,
        master_rx[2] as u32,
        master_rx[3] as u32,
    );

    expect_true(
        master_rx == TARGET_READ_DATA,
        "private-read data mismatch",
    )?;

    /*
     * The target should also receive a completion response for the queued
     * read-data descriptor.
     */
    let event = target
        .next_event(
            &mut target_rx,
        )
        .map_err(|_| {
            "target read-completion event failed"
        })?;

    match event {
        HciTargetEvent::ReadComplete {
            len,
        } => {
            pw_log::info!(
                "target read completion len={}",
                len as u32,
            );

            expect_true(
                len == TARGET_READ_DATA.len(),
                "target read-completion length mismatch",
            )?;
        }

        HciTargetEvent::PrivateWrite {
            len,
        } => {
            pw_log::error!(
                "unexpected private-write len={}",
                len as u32,
            );

            return Err(
                "unexpected private write after read",
            );
        }

        HciTargetEvent::Ccc {
            ccc,
            len,
        } => {
            pw_log::error!(
                "unexpected CCC ccc={:02x} len={}",
                ccc as u32,
                len as u32,
            );

            return Err(
                "unexpected CCC after private read",
            );
        }

        HciTargetEvent::Other(_) => {
            return Err(
                "unexpected response after private read",
            );
        }
    }

    pw_log::info!(
        "PRIVATE READ PASS"
    );

    /*
     * ===============================================================
     * Phase 6 complete.
     * ===============================================================
     */
    pw_log::info!(
        "ENTDAA PASS: DA={:02x}",
        assigned_addr as u32,
    );

    pw_log::info!(
        "PRIVATE WRITE PASS"
    );

    pw_log::info!(
        "PRIVATE READ PASS"
    );

    pw_log::info!(
        "=== AST1080 I3C HCI target-mode self-test complete ==="
    );

    Ok(())
}

impl TargetInterface for Target {
    const NAME: &'static str =
        "AST1080 I3C HCI target-mode self-test";

    fn main() -> ! {
        let sentinel: &[u8] =
            match run_target_mode_test() {
                Ok(()) => {
                    b"TEST_RESULT:PASS\n"
                }
                Err(error) => {
                    pw_log::error!("I3C HCI target-mode test failed: {}", error as &str);
                    b"TEST_RESULT:FAIL\n"
                }
            };

        let _ =
            console_backend_write_all(
                sentinel,
            );

        #[expect(clippy::empty_loop)]
        loop {}
    }
}

declare_target!(Target);