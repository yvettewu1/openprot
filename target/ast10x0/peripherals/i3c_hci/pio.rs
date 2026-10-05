// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Minimal polling PIO master path for MIPI I3C HCI.
//!
//! This follows the command/data/response ordering in Zephyr pio.c: queue data,
//! queue response expectation, write command descriptor words, then process the
//! response and trailing RX in polling mode.

use super::cmd::{HciCommandV1, HciResponse};
use super::error::{I3cHciError, Result};
use super::registers::I3cHciRegisters;

pub const STAT_TRANSFER_BLOCKED: u32 = 1 << 25;
pub const STAT_PERR_RESP_UFLOW: u32 = 1 << 24;
pub const STAT_PERR_CMD_OFLOW: u32 = 1 << 23;
pub const STAT_PERR_IBI_UFLOW: u32 = 1 << 22;
pub const STAT_PERR_RX_UFLOW: u32 = 1 << 21;
pub const STAT_PERR_TX_OFLOW: u32 = 1 << 20;
pub const STAT_ERR_RESP_QUEUE_FULL: u32 = 1 << 19;
pub const STAT_ERR_IBI_QUEUE_FULL: u32 = 1 << 17;
pub const STAT_ERR_RX_DATA_FULL: u32 = 1 << 15;
pub const STAT_ERR_TX_DATA_EMPTY: u32 = 1 << 13;
pub const STAT_TRANSFER_ERR: u32 = 1 << 9;
pub const STAT_TRANSFER_ABORT: u32 = 1 << 5;
pub const STAT_RESP_READY: u32 = 1 << 4;
pub const STAT_CMD_QUEUE_READY: u32 = 1 << 3;
pub const STAT_RX_THLD: u32 = 1 << 1;
pub const STAT_TX_THLD: u32 = 1 << 0;

pub const STAT_PROG_ERRORS: u32 = STAT_TRANSFER_BLOCKED
    | STAT_PERR_RESP_UFLOW
    | STAT_PERR_CMD_OFLOW
    | STAT_PERR_IBI_UFLOW
    | STAT_PERR_RX_UFLOW
    | STAT_PERR_TX_OFLOW;
pub const STAT_LATENCY_ERRORS: u32 = STAT_ERR_RESP_QUEUE_FULL
    | STAT_ERR_IBI_QUEUE_FULL
    | STAT_ERR_RX_DATA_FULL
    | STAT_ERR_TX_DATA_EMPTY;
pub const STAT_ALL_ERRORS: u32 =
    STAT_TRANSFER_ABORT | STAT_TRANSFER_ERR | STAT_LATENCY_ERRORS | STAT_PROG_ERRORS;

pub struct PioTransfer<'a> {
    pub command: HciCommandV1,
    pub tx: &'a [u8],
    pub rx: &'a mut [u8],
}

pub struct PioTransferResult {
    pub response: HciResponse,
    pub rx_len: usize,
}

pub struct I3cHciPio<'a, Y: FnMut(u32)> {
    regs: &'a I3cHciRegisters,
    yield_us: Y,
    timeout_us: u32,
}

impl<'a, Y: FnMut(u32)> I3cHciPio<'a, Y> {
    #[must_use]
    pub const fn new(regs: &'a I3cHciRegisters, yield_us: Y) -> Self {
        Self {
            regs,
            yield_us,
            timeout_us: 10_000,
        }
    }

    #[must_use]
    pub const fn with_timeout_us(mut self, timeout_us: u32) -> Self {
        self.timeout_us = timeout_us;
        self
    }

    pub fn submit_v1(&mut self, transfer: PioTransfer<'_>) -> Result<PioTransferResult> {
        let result = self.submit_v1_raw(transfer)?;
        if !result.response.success() {
            return Err(I3cHciError::Transfer(result.response.status));
        }
        Ok(result)
    }

    pub fn submit_v1_raw(&mut self, transfer: PioTransfer<'_>) -> Result<PioTransferResult> {
        validate_pio_transfer(&transfer)?;
        if !transfer.command.expects_response() {
            return Err(I3cHciError::Unsupported);
        }
        self.check_errors()?;
        self.write_tx_data(transfer.tx)?;
        self.wait_status(STAT_CMD_QUEUE_READY)?;
        self.regs
            .write_pio_command_queue_port(transfer.command.words[0]);
        self.regs
            .write_pio_command_queue_port(transfer.command.words[1]);
        let response = self.wait_response(transfer.command.tid)?;
        let rx_len = self.read_rx_data(transfer.rx, response.data_len)?;
        Ok(PioTransferResult { response, rx_len })
    }

    pub fn submit_v1_no_response(&mut self, transfer: PioTransfer<'_>) -> Result<()> {
        validate_pio_transfer(&transfer)?;
        if transfer.command.expects_response() {
            return Err(I3cHciError::InvalidArgs);
        }
        self.check_errors()?;
        self.write_tx_data(transfer.tx)?;
        self.wait_status(STAT_CMD_QUEUE_READY)?;
        self.regs
            .write_pio_command_queue_port(transfer.command.words[0]);
        self.regs
            .write_pio_command_queue_port(transfer.command.words[1]);
        Ok(())
    }

    fn write_tx_data(&mut self, tx: &[u8]) -> Result<()> {
        let mut offset = 0;
        while offset < tx.len() {
            self.wait_status(STAT_TX_THLD)?;
            let chunk = core::cmp::min(4, tx.len() - offset);
            self.regs
                .write_pio_tx_data_port(word_from_bytes(&tx[offset..offset + chunk]));
            offset += chunk;
        }
        Ok(())
    }

    fn read_rx_data(&mut self, rx: &mut [u8], expected: usize) -> Result<usize> {
        let wanted = core::cmp::min(rx.len(), expected);
        let mut offset = 0;
        while offset < wanted {
            self.wait_status(STAT_RX_THLD)?;
            let word = self.regs.pio_rx_data_port();
            let chunk = core::cmp::min(4, wanted - offset);
            bytes_from_word(word, &mut rx[offset..offset + chunk]);
            offset += chunk;
        }
        Ok(offset)
    }

    fn wait_response(&mut self, tid: u8) -> Result<HciResponse> {
        self.wait_status(STAT_RESP_READY)?;
        let response = HciResponse::parse(self.regs.pio_response_queue_port());
        if response.tid != tid {
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(response)
    }

    fn wait_status(&mut self, mask: u32) -> Result<()> {
        let mut remaining = self.timeout_us;
        loop {
            self.check_errors()?;
            if (self.regs.pio_intr_status() & mask) != 0 {
                return Ok(());
            }
            if remaining == 0 {
                return Err(I3cHciError::Timeout);
            }
            (self.yield_us)(1);
            remaining -= 1;
        }
    }

    fn check_errors(&self) -> Result<()> {
        let status = self.regs.pio_intr_status();
        if (status & STAT_ALL_ERRORS) != 0 {
            self.regs.write_pio_intr_status(status & STAT_ALL_ERRORS);
            return Err(I3cHciError::TransferStatus(status & STAT_ALL_ERRORS));
        }
        Ok(())
    }
}

#[must_use]
pub fn word_from_bytes(bytes: &[u8]) -> u32 {
    let mut word = 0;
    let mut i = 0;
    while i < bytes.len() {
        word |= (bytes[i] as u32) << (i as u32 * 8);
        i += 1;
    }
    word
}

pub fn bytes_from_word(word: u32, out: &mut [u8]) {
    let mut i = 0;
    while i < out.len() {
        out[i] = ((word >> (i as u32 * 8)) & 0xff) as u8;
        i += 1;
    }
}

pub fn validate_pio_transfer(transfer: &PioTransfer<'_>) -> Result<()> {
    if transfer.command.read {
        if !transfer.tx.is_empty() || transfer.rx.len() < transfer.command.data_len {
            return Err(I3cHciError::InvalidArgs);
        }
    } else if transfer.command.kind == super::cmd::HciCommandKind::Immediate {
        if !transfer.tx.is_empty() || !transfer.rx.is_empty() {
            return Err(I3cHciError::InvalidArgs);
        }
    } else if transfer.tx.len() != transfer.command.data_len || !transfer.rx.is_empty() {
        return Err(I3cHciError::InvalidArgs);
    }
    Ok(())
}
