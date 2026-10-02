// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

#![no_std]

//! Minimal AST10x0 SCU pinctrl crate for small firmware tests.

#[path = "scu/registers.rs"]
pub mod registers;

pub use registers::ScuRegisters;

#[path = "scu/pinctrl.rs"]
pub mod pinctrl;
