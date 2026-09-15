// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use ast10x0_board::{
    apply_spim_external_mux, delay_us, enable_flash_power, set_bmc_resets, Ast10x0Board,
    Ast10x0BoardDescriptor, I2cBusCfg,
};
use ast10x0_peripherals::i2c::{ClockConfig, I2cConfig, I2cSpeed, I2cXferMode};
use ast10x0_peripherals::scu::{
    pinctrl, ScuExtMuxSelect, ScuRegisters, SpiMonitorInstance, SpiMonitorPassthrough,
    SpiMonitorSource,
};
use ast10x0_peripherals::spimonitor::{
    PassthroughMode, PrivilegeDirection, PrivilegeOp, SpiMonitor, SpiMonitorController,
    SpiMonitorPolicy, Uninitialized,
};
use console_backend::console_backend_write_all;
use entry as _;
use target_common::{declare_target, TargetInterface};

pub struct Target;

const I2C0_CFG: I2cConfig = I2cConfig {
    speed: I2cSpeed::Standard,
    xfer_mode: I2cXferMode::BufferMode,
    multi_master: true,
    smbus_timeout: false,
    smbus_alert: false,
    clock_config: ClockConfig::ast1060_default(),
};

static PINCTRL_GROUPS: [&[ast10x0_peripherals::scu::PinctrlPin]; 1] = [pinctrl::PINCTRL_I2C0];
static I2C_BUSES: [I2cBusCfg; 1] = [I2cBusCfg {
    bus: 0,
    config: I2C0_CFG,
}];

const WRITE_PROTECTED_LENGTH: u32 = 0x0010_0000;
const ROUTE_SETTLE_US: u32 = 60_000;
const RESET_HOLD_US: u32 = 1_000_000;
const ALLOW_COMMANDS: [u8; 32] = [
    0x03, 0x13, 0x0b, 0x0c, 0x6b, 0x6c, 0x01, 0x05, 0x35, 0x06, 0x04, 0x20, 0x21, 0x9f, 0x5a, 0xb7,
    0xe9, 0x32, 0x34, 0xd8, 0xdc, 0x02, 0x12, 0x3b, 0x3c, 0x70, 0xbb, 0xbc, 0x50, 0xeb, 0xec, 0xc2,
];

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

fn ast2700_bmc_boot_handoff() -> bool {
    let scu = unsafe { ScuRegisters::new_global_unlocked() };

    if !set_bmc_resets(true) {
        return false;
    }
    delay_us(RESET_HOLD_US);

    if !enable_flash_power(&scu) {
        return false;
    }

    ast10x0_board::apply_spim_pinctrl(&scu, SpiMonitorInstance::Spim0);
    scu.disable_spim_cs_internal_pull_down(SpiMonitorInstance::Spim0);
    if scu
        .set_spim_internal_mux(SpiMonitorSource::Spi1, 0)
        .is_err()
    {
        return false;
    }
    if scu.route_control_raw() & 0x0f != 0 {
        return false;
    }
    scu.set_spim_passthrough(SpiMonitorInstance::Spim0, SpiMonitorPassthrough::Enabled);
    scu.set_spim_miso_multi_func(SpiMonitorInstance::Spim0, true);
    scu.set_spim_filter(SpiMonitorInstance::Spim0, true);

    apply_spim_external_mux(SpiMonitorInstance::Spim0, ScuExtMuxSelect::Mux1);

    let monitor = unsafe { SpiMonitor::<Uninitialized>::new(SpiMonitorController::Spim0) };
    monitor.software_reset();
    let Ok(configured) = monitor.apply_policy(&production_policy()) else {
        return false;
    };
    configured.set_push_pull(true);
    configured.set_passthrough(PassthroughMode::Disabled);
    configured.enable();

    configured.reset_filter_state();
    apply_spim_external_mux(SpiMonitorInstance::Spim0, ScuExtMuxSelect::Mux0);
    delay_us(ROUTE_SETTLE_US);

    set_bmc_resets(false)
}

impl TargetInterface for Target {
    const NAME: &'static str = "AST10x0 MCTP Server";

    fn main() -> ! {
        // SAFETY: kernel main() runs once with exclusive hardware ownership.
        if unsafe {
            Ast10x0Board::new(Ast10x0BoardDescriptor {
                pinctrl_groups: &PINCTRL_GROUPS,
                i2c_buses: &I2C_BUSES,
            })
            .init()
        }
        .is_err()
        {
            loop {}
        }

        if ast2700_bmc_boot_handoff() {
            delay_us(200_000_000);
            codegen::start();
        }

        loop {}
    }
    fn shutdown(code: u32) -> ! {
        let sentinel: &[u8] = if code == 0 {
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
