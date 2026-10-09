// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! I3C CCC constants used by the MIPI HCI driver.

pub const I3C_CCC_ENEC_BROADCAST: u8 = 0x00;
pub const I3C_CCC_DISEC_BROADCAST: u8 = 0x01;
pub const I3C_CCC_ENTAS0: u8 = 0x02;
pub const I3C_CCC_ENTAS1: u8 = 0x03;
pub const I3C_CCC_ENTAS2: u8 = 0x04;
pub const I3C_CCC_ENTAS3: u8 = 0x05;
pub const I3C_CCC_RSTDAA_BROADCAST: u8 = 0x06;
pub const I3C_CCC_ENTDAA: u8 = 0x07;
pub const I3C_CCC_SETMWL_BROADCAST: u8 = 0x09;
pub const I3C_CCC_SETMRL_BROADCAST: u8 = 0x0a;
pub const I3C_CCC_ENEC_DIRECT: u8 = 0x80;
pub const I3C_CCC_DISEC_DIRECT: u8 = 0x81;
pub const I3C_CCC_RSTDAA_DIRECT: u8 = 0x86;
pub const I3C_CCC_SETDASA: u8 = 0x87;
pub const I3C_CCC_SETNEWDA: u8 = 0x88;
pub const I3C_CCC_SETMWL_DIRECT: u8 = 0x89;
pub const I3C_CCC_SETMRL_DIRECT: u8 = 0x8a;
pub const I3C_CCC_GETMWL: u8 = 0x8b;
pub const I3C_CCC_GETMRL: u8 = 0x8c;
pub const I3C_CCC_GETPID: u8 = 0x8d;
pub const I3C_CCC_GETBCR: u8 = 0x8e;
pub const I3C_CCC_GETDCR: u8 = 0x8f;
pub const I3C_CCC_GETSTATUS: u8 = 0x90;
pub const I3C_CCC_GETMXDS: u8 = 0x94;

pub const I3C_CCC_EVENT_SIR: u8 = 1 << 0;
pub const I3C_CCC_EVENT_MR: u8 = 1 << 1;
pub const I3C_CCC_EVENT_HJ: u8 = 1 << 3;
