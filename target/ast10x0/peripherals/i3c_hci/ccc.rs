// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! I3C CCC constants used by the MIPI HCI driver.

pub const I3C_CCC_ENEC_BROADCAST: u8 = 0x00;
pub const I3C_CCC_DISEC_BROADCAST: u8 = 0x01;
pub const I3C_CCC_ENTDAA: u8 = 0x07;
pub const I3C_CCC_ENEC_DIRECT: u8 = 0x80;
pub const I3C_CCC_DISEC_DIRECT: u8 = 0x81;
pub const I3C_CCC_GETPID: u8 = 0x8d;

pub const I3C_CCC_EVENT_SIR: u8 = 1 << 0;
pub const I3C_CCC_EVENT_MR: u8 = 1 << 1;
pub const I3C_CCC_EVENT_HJ: u8 = 1 << 3;
