// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST2700 DCSCM BMC boot handoff through SPIM1.
//!
//! This keeps the BMC in reset while the first external SPI monitor path is
//! configured, then switches the external mux back to the BMC and releases the
//! AST2700 reset lines.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;

#[path = "test_common.rs"]
mod test_common;

use ast10x0_board::{
    apply_spim_external_mux, delay_us, enable_flash_power, set_bmc_resets, spim_external_mux_state,
};
use ast10x0_peripherals::scu::{ScuExtMuxSelect, ScuRegisters, SpiMonitorInstance};
use ast10x0_peripherals::spimonitor::{
    PrivilegeDirection, PrivilegeOp, SpiMonitorController, SpiMonitorPolicy,
};
use console_backend::console_backend_write_all;
use target_common::{declare_target, TargetInterface};
use test_common::TestConfig;
use {console_backend as _, entry as _};

struct Spim1Config;

impl test_common::TestConfig for Spim1Config {
    const INSTANCE: SpiMonitorInstance = SpiMonitorInstance::Spim0;
    const CONTROLLER: SpiMonitorController = SpiMonitorController::Spim0;
}

#[repr(align(16))]
struct LogRam(UnsafeCell<[u32; test_common::LOG_RAM_WORDS]>);

// SAFETY: This target is single-threaded and assigns the buffer to SPIM1 only.
unsafe impl Sync for LogRam {}

static SPIM1_LOG: LogRam = LogRam(UnsafeCell::new([0; test_common::LOG_RAM_WORDS]));

const WRITE_PROTECTED_LENGTH: u32 = 0x0010_0000;
const ROUTE_SETTLE_US: u32 = 60_000;
const RESET_HOLD_US: u32 = 1_000_000;
const ALLOW_COMMANDS: [u8; 32] = [
    0x03, 0x13, 0x0b, 0x0c, 0x6b, 0x6c, 0x01, 0x05, 0x35, 0x06, 0x04, 0x20, 0x21, 0x9f, 0x5a, 0xb7,
    0xe9, 0x32, 0x34, 0xd8, 0xdc, 0x02, 0x12, 0x3b, 0x3c, 0x70, 0xbb, 0xbc, 0x50, 0xeb, 0xec, 0xc2,
];

fn log_buffer() -> &'static mut [u32] {
    // SAFETY: The test is single-threaded and calls this exactly once.
    unsafe { &mut *SPIM1_LOG.0.get() }
}

fn production_policy() -> SpiMonitorPolicy {
    let mut policy = SpiMonitorPolicy::empty();
    policy.allow_commands.copy_from_slice(&ALLOW_COMMANDS);
    policy.allow_command_count = ALLOW_COMMANDS.len();
    let _ = policy.add_region(
        0,
        WRITE_PROTECTED_LENGTH,
        PrivilegeDirection::Write,
        PrivilegeOp::Disable,
    );
    policy
}

fn run_ast2700_bmc_boot() -> Result<(), test_common::TestError> {
    pw_log::info!("=== AST2700 DCSCM BMC boot via SPIM1 ===");

    let scu = unsafe { ScuRegisters::new_global_unlocked() };
    let policy = production_policy();

    pw_log::info!("=== Hold BMC in reset ===");
    if !set_bmc_resets(true) {
        pw_log::info!("FAIL: BMC reset outputs did not assert");
        return Err(test_common::TestError::Check);
    }
    delay_us(RESET_HOLD_US);

    pw_log::info!("=== Enable flash power ===");
    if !enable_flash_power(&scu) {
        pw_log::info!("FAIL: flash power GPIO readback");
        return Err(test_common::TestError::Check);
    }

    pw_log::info!("=== Configure SPIM1 before BMC release ===");
    test_common::configure_wiring::<Spim1Config>(&scu)?;
    let spim1 = test_common::initialize_monitor_with_policy::<Spim1Config>(log_buffer(), &policy)?;
    test_common::dump_policy(&spim1, WRITE_PROTECTED_LENGTH)?;
    test_common::validate_filtering(&spim1)?;

    pw_log::info!("=== Hand BMC flash mux to external BMC master ===");
    spim1.reset_filter_state();
    apply_spim_external_mux(Spim1Config::INSTANCE, ScuExtMuxSelect::Mux0);
    if spim_external_mux_state(Spim1Config::INSTANCE) != Some(ScuExtMuxSelect::Mux0) {
        pw_log::info!("FAIL: SPIM1 external mux handoff");
        return Err(test_common::TestError::Check);
    }
    delay_us(ROUTE_SETTLE_US);

    pw_log::info!("=== Release BMC reset ===");
    if !set_bmc_resets(false) {
        pw_log::info!("FAIL: BMC reset outputs did not release");
        return Err(test_common::TestError::Check);
    }

    pw_log::info!("PASS: AST2700 BMC reset released with SPIM1 routed to BMC");
    Ok(())
}

struct Target;

impl TargetInterface for Target {
    const NAME: &'static str = "AST2700 DCSCM BMC Boot via SPIM1";

    fn main() -> ! {
        let sentinel = if run_ast2700_bmc_boot().is_ok() {
            b"TEST_RESULT:PASS\n"
        } else {
            b"TEST_RESULT:FAIL\n"
        };
        let _ = console_backend_write_all(sentinel);
        #[expect(clippy::empty_loop)]
        loop {}
    }
}

declare_target!(Target);
