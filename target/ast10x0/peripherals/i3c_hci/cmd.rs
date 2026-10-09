// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI command descriptor helpers.
//!
//! The v1 descriptor layout follows Zephyr drivers/i3c/mipi_hci/cmd.h and
//! cmd_v1.c from AspeedTech-BMC/zephyr aspeed-main-v3.7.0.

use super::error::{I3cHciError, Result};

pub const CMD_0_TOC: u32 = 1 << 31;
pub const CMD_0_ROC: u32 = 1 << 30;
pub const CMD_0_ATTR_MASK: u32 = 0x7;
pub const CMD_0_ATTR_I: u32 = 0x1;
pub const CMD_0_ATTR_R: u32 = 0x0;
pub const CMD_0_ATTR_M: u32 = 0x7;

pub const RESP_STATUS_SHIFT: u32 = 28;
pub const RESP_STATUS_MASK: u32 = 0xf << RESP_STATUS_SHIFT;
pub const RESP_TID_SHIFT: u32 = 24;
pub const RESP_TID_MASK: u32 = 0xf << RESP_TID_SHIFT;
pub const RESP_DATA_LENGTH_MASK: u32 = 0x003f_ffff;

pub const RESP_SUCCESS: u8 = 0x0;
pub const RESP_ERR_CRC: u8 = 0x1;
pub const RESP_ERR_PARITY: u8 = 0x2;
pub const RESP_ERR_FRAME: u8 = 0x3;
pub const RESP_ERR_ADDR_HEADER: u8 = 0x4;
pub const RESP_ERR_NACK: u8 = 0x5;
pub const RESP_ERR_OVERFLOW: u8 = 0x6;
pub const RESP_ERR_I3C_SHORT_READ: u8 = 0x7;
pub const RESP_ERR_HC_TERMINATED: u8 = 0x8;
pub const RESP_ERR_I2C_WR_DATA_NACK: u8 = 0x9;
pub const RESP_ERR_NOT_SUPPORTED: u8 = 0x0a;
pub const RESP_ERR_ABORTED_WITH_CRC: u8 = 0x0b;

pub const MODE_I3C_SDR0: u8 = 0x0;
pub const MODE_I3C_SDR1: u8 = 0x1;
pub const MODE_I3C_SDR2: u8 = 0x2;
pub const MODE_I3C_SDR3: u8 = 0x3;
pub const MODE_I3C_SDR4: u8 = 0x4;
pub const MODE_I2C_FM: u8 = 0x0;
pub const MODE_I2C_FMP: u8 = 0x1;
pub const TID_TARGET_IBI: u8 = 0x1;
pub const TID_TARGET_RD_DATA: u8 = 0x2;

const CMD_T0_TID_SHIFT: u32 = 3;
const CMD_T0_TID_MASK: u32 = 0x3 << CMD_T0_TID_SHIFT;
const CMD_T0_DATA_LENGTH_SHIFT: u32 = 16;
const CMD_T0_DATA_LENGTH_MASK: u32 = 0xffff << CMD_T0_DATA_LENGTH_SHIFT;

/// One-word command descriptor used while operating as an I3C target.
///
/// Target descriptors are different from the normal two-word HCI v1
/// controller descriptors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciTargetCommand {
    pub word: u32,
    pub tid: u8,
    pub data_len: usize,
}

impl HciTargetCommand {
    /// Queue IBI payload bytes that will be sent when the ASPEED target IBI
    /// request bit is asserted.
    pub fn ibi_payload(len: usize) -> Result<Self> {
        Self::target_command(TID_TARGET_IBI, len, true)
    }

    /// Queue data that will be returned when the active controller performs
    /// a private read from this target.
    pub fn read_data(len: usize) -> Result<Self> {
        Self::target_command(TID_TARGET_RD_DATA, len, false)
    }

    fn target_command(tid: u8, len: usize, allow_zero: bool) -> Result<Self> {
        if (!allow_zero && len == 0) || len > u16::MAX as usize {
            return Err(I3cHciError::InvalidArgs);
        }

        let word = ((len as u32) << CMD_T0_DATA_LENGTH_SHIFT)
            | ((u32::from(tid) << CMD_T0_TID_SHIFT) & CMD_T0_TID_MASK);

        Ok(Self {
            word: word & (CMD_T0_DATA_LENGTH_MASK | CMD_T0_TID_MASK),
            tid,
            data_len: len,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HciCommandKind {
    AddressAssignment,
    Immediate,
    Regular,
    Internal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciCommandV1 {
    pub words: [u32; 2],
    pub tid: u8,
    pub kind: HciCommandKind,
    pub read: bool,
    pub data_len: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciResponse {
    pub raw: u32,
    pub status: u8,
    pub tid: u8,
    pub data_len: usize,
}

impl HciResponse {
    #[must_use]
    pub const fn parse(raw: u32) -> Self {
        Self {
            raw,
            status: ((raw & RESP_STATUS_MASK) >> RESP_STATUS_SHIFT) as u8,
            tid: ((raw & RESP_TID_MASK) >> RESP_TID_SHIFT) as u8,
            data_len: (raw & RESP_DATA_LENGTH_MASK) as usize,
        }
    }

    #[must_use]
    pub const fn success(self) -> bool {
        self.status == RESP_SUCCESS
    }
}

#[must_use]
pub const fn i3c_sdr_mode(i3c_scl_hz: u32) -> u8 {
    if i3c_scl_hz > 8_000_000 {
        MODE_I3C_SDR0
    } else if i3c_scl_hz > 6_000_000 {
        MODE_I3C_SDR1
    } else if i3c_scl_hz > 4_000_000 {
        MODE_I3C_SDR2
    } else if i3c_scl_hz > 2_000_000 {
        MODE_I3C_SDR3
    } else {
        MODE_I3C_SDR4
    }
}

#[must_use]
pub const fn i2c_mode(i2c_scl_hz: u32) -> u8 {
    if i2c_scl_hz >= 1_000_000 {
        MODE_I2C_FMP
    } else {
        MODE_I2C_FM
    }
}

impl HciCommandV1 {
    pub fn i3c_private(dat_index: u8, tid: u8, read: bool, data: &[u8], mode: u8) -> Result<Self> {
        Self::regular_or_immediate(dat_index, tid, read, data, mode, None, false, None)
    }

    pub fn i3c_private_write(dat_index: u8, tid: u8, data: &[u8], mode: u8) -> Result<Self> {
        Self::i3c_private(dat_index, tid, false, data, mode)
    }

    pub fn i3c_private_write_regular(
        dat_index: u8,
        tid: u8,
        data_len: usize,
        mode: u8,
    ) -> Result<Self> {
        Self::regular(dat_index, tid, false, data_len, mode, None, false, None)
    }

    pub fn i3c_private_read(dat_index: u8, tid: u8, len: usize, mode: u8) -> Result<Self> {
        Self::regular(dat_index, tid, true, len, mode, None, false, None)
    }

    pub fn i2c_write(dat_index: u8, tid: u8, data: &[u8], mode: u8) -> Result<Self> {
        Self::regular_or_immediate(dat_index, tid, false, data, mode, None, false, None)
    }

    pub fn i2c_write_regular(dat_index: u8, tid: u8, data_len: usize, mode: u8) -> Result<Self> {
        Self::regular(dat_index, tid, false, data_len, mode, None, false, None)
    }

    pub fn i2c_read(dat_index: u8, tid: u8, len: usize, mode: u8) -> Result<Self> {
        Self::regular(dat_index, tid, true, len, mode, None, false, None)
    }

    pub fn ccc(
        dat_index: u8,
        tid: u8,
        read: bool,
        ccc: u8,
        data: &[u8],
        mode: u8,
        defining_byte: Option<u8>,
    ) -> Result<Self> {
        Self::regular_or_immediate(
            dat_index,
            tid,
            read,
            data,
            mode,
            Some(ccc),
            true,
            defining_byte,
        )
    }

    pub fn ccc_regular(
        dat_index: u8,
        tid: u8,
        read: bool,
        ccc: u8,
        data_len: usize,
        mode: u8,
        defining_byte: Option<u8>,
    ) -> Result<Self> {
        Self::regular(
            dat_index,
            tid,
            read,
            data_len,
            mode,
            Some(ccc),
            true,
            defining_byte,
        )
    }

    fn regular(
        dat_index: u8,
        tid: u8,
        read: bool,
        data_len: usize,
        mode: u8,
        command_code: Option<u8>,
        command_present: bool,
        defining_byte: Option<u8>,
    ) -> Result<Self> {
        if dat_index > 0x7f || tid > 0x0f || mode > 0x07 || data_len > 0xffff {
            return Err(I3cHciError::InvalidArgs);
        }
        let cmd = command_code.unwrap_or(0) as u32;
        let dat = dat_index as u32;
        let tid_u32 = tid as u32;
        let mode = mode as u32;
        let cp = if command_present { 1 << 15 } else { 0 };
        let dbp = if command_present && defining_byte.is_some() {
            1 << 25
        } else {
            0
        };
        let rnw = if read { 1 << 29 } else { 0 };
        let word0 = CMD_0_ATTR_R
            | (tid_u32 << 3)
            | (cmd << 7)
            | cp
            | dbp
            | (dat << 16)
            | (mode << 26)
            | rnw;
        let def_byte = defining_byte.unwrap_or(0) as u32;
        let word1 = ((data_len as u32) << 16) | def_byte;
        Ok(Self {
            words: [word0, word1],
            tid,
            kind: HciCommandKind::Regular,
            read,
            data_len,
        })
    }

    fn regular_or_immediate(
        dat_index: u8,
        tid: u8,
        read: bool,
        data: &[u8],
        mode: u8,
        command_code: Option<u8>,
        command_present: bool,
        defining_byte: Option<u8>,
    ) -> Result<Self> {
        if read || data.len() > 4 {
            return Self::regular(
                dat_index,
                tid,
                read,
                data.len(),
                mode,
                command_code,
                command_present,
                defining_byte,
            );
        }
        if dat_index > 0x7f || tid > 0x0f || mode > 0x07 {
            return Err(I3cHciError::InvalidArgs);
        }

        let cmd = command_code.unwrap_or(0) as u32;
        let dat = dat_index as u32;
        let tid = tid as u32;
        let mode = mode as u32;
        let cp = if command_present { 1 << 15 } else { 0 };

        {
            let mut word0 = CMD_0_ATTR_I
                | (tid << 3)
                | (cmd << 7)
                | cp
                | (dat << 16)
                | ((data.len() as u32) << 23)
                | (mode << 26);
            let mut word1 = 0;
            for (idx, byte) in data.iter().enumerate() {
                word1 |= (*byte as u32) << (idx as u32 * 8);
            }
            word0 &= !CMD_0_ROC;
            Ok(Self {
                words: [word0, word1],
                tid: tid as u8,
                kind: HciCommandKind::Immediate,
                read,
                data_len: data.len(),
            })
        }
    }

    pub fn internal(tid: u8, sub_cmd: u8, param: u32) -> Result<Self> {
        if tid > 0x0f || sub_cmd > 0x0f || param > 0x000f_ffff {
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(Self {
            words: [
                CMD_0_ATTR_M | ((tid as u32) << 3) | ((sub_cmd as u32) << 8) | (param << 12),
                0,
            ],
            tid,
            kind: HciCommandKind::Internal,
            read: false,
            data_len: 0,
        })
    }

    #[must_use]
    pub const fn expects_response(self) -> bool {
        (self.words[0] & CMD_0_ROC) != 0 || (self.words[0] & CMD_0_ATTR_MASK) == CMD_0_ATTR_M
    }

    #[must_use]
    pub const fn with_toc(mut self) -> Self {
        self.words[0] |= CMD_0_TOC;
        self
    }

    #[must_use]
    pub const fn with_roc(mut self) -> Self {
        self.words[0] |= CMD_0_ROC;
        self
    }
}
