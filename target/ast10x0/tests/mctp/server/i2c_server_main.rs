// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! I2C server app: board init + open Bus 0 + run the server-runtime loop.

#![no_main]
#![no_std]

use app_i2c_server::{handle, signals};
use ast10x0_peripherals::i2c::{ClockConfig, I2cConfig, I2cSpeed, I2cXferMode};
use i2c_server_runtime::{run, Bus};
use userspace::entry;

const SLAVE_CFG: I2cConfig = I2cConfig {
    speed: I2cSpeed::Standard,
    xfer_mode: I2cXferMode::BufferMode,
    multi_master: true,
    smbus_timeout: false,
    smbus_alert: false,
    clock_config: ClockConfig::ast1060_default(),
};

#[entry]
fn entry() {
    // SAFETY: board init ran init_bus(0) in the kernel; the server owns bus 0.
    let driver = match unsafe { i2c_backend::open_bus(0, &SLAVE_CFG) } {
        Ok(d) => d,
        Err(_) => {
            pw_log::error!("open_bus(0) failed");
            loop {}
        }
    };

    pw_log::info!("I2C server ready on Bus 0");

    let mut buses = [Bus::new(handle::I2C, handle::I2C0_IRQ, driver)];
    run(handle::WG, signals::I2C0, &mut buses);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
