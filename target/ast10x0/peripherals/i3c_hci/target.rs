// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 MIPI I3C HCI target-mode support.
//!
//! Covers target identity, target mode selection, private write/read handling,
//! target IBI requests, and Hot-Join requests.

use super::ccc::{
    I3C_CCC_DISEC_BROADCAST, I3C_CCC_DISEC_DIRECT, I3C_CCC_ENEC_BROADCAST, I3C_CCC_ENEC_DIRECT,
    I3C_CCC_GETBCR, I3C_CCC_GETDCR, I3C_CCC_GETMRL, I3C_CCC_GETMWL, I3C_CCC_GETMXDS,
    I3C_CCC_GETPID, I3C_CCC_GETSTATUS, I3C_CCC_RSTDAA_BROADCAST, I3C_CCC_RSTDAA_DIRECT,
    I3C_CCC_SETDASA, I3C_CCC_SETMRL_BROADCAST, I3C_CCC_SETMRL_DIRECT, I3C_CCC_SETMWL_BROADCAST,
    I3C_CCC_SETMWL_DIRECT, I3C_CCC_SETNEWDA,
};
use super::cmd::TID_TARGET_RD_DATA;
use super::constants::{
    ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR, ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ,
    ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT, ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ,
    ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT, ASPEED_I3C_SLV_CAP_CTRL_MR_REQ,
    ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT, ASPEED_I3C_SLV_STS1_CR_EN, ASPEED_I3C_SLV_STS1_HJ_EN,
    ASPEED_I3C_SLV_STS1_IBI_EN,
};
use super::daa::valid_i3c_address;
use super::error::{I3cHciError, Result};
use super::pio::{I3cHciPio, TargetResponse, STAT_ALL_ERRORS, STAT_RESP_READY};
use super::registers::I3cHciRegisters;
use super::types::InitMode;

const TARGET_PIO_IRQS: u32 = STAT_RESP_READY | STAT_ALL_ERRORS;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciTargetConfig {
    pub pid: u64,
    pub bcr: u8,
    pub dcr: u8,
    pub static_addr: Option<u8>,
}

impl HciTargetConfig {
    pub fn validate(&self) -> Result<()> {
        /*
         * I3C PID is 48 bits.
         */
        if self.pid >= (1u64 << 48) {
            return Err(I3cHciError::InvalidArgs);
        }

        if let Some(addr) = self.static_addr {
            if !valid_i3c_address(addr) {
                return Err(I3cHciError::InvalidArgs);
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HciTargetCcc {
    EnableEvents,
    DisableEvents,
    ResetDynamicAddress,
    SetDynamicAddressFromStatic,
    SetNewDynamicAddress,
    SetMaxWriteLength,
    SetMaxReadLength,
    GetMaxWriteLength,
    GetMaxReadLength,
    GetPid,
    GetBcr,
    GetDcr,
    GetStatus,
    GetMaxDataSpeed,
    Unknown(u8),
}

#[must_use]
pub const fn classify_target_ccc(ccc: u8) -> HciTargetCcc {
    match ccc {
        I3C_CCC_ENEC_BROADCAST | I3C_CCC_ENEC_DIRECT => HciTargetCcc::EnableEvents,
        I3C_CCC_DISEC_BROADCAST | I3C_CCC_DISEC_DIRECT => HciTargetCcc::DisableEvents,
        I3C_CCC_RSTDAA_BROADCAST | I3C_CCC_RSTDAA_DIRECT => HciTargetCcc::ResetDynamicAddress,
        I3C_CCC_SETDASA => HciTargetCcc::SetDynamicAddressFromStatic,
        I3C_CCC_SETNEWDA => HciTargetCcc::SetNewDynamicAddress,
        I3C_CCC_SETMWL_BROADCAST | I3C_CCC_SETMWL_DIRECT => HciTargetCcc::SetMaxWriteLength,
        I3C_CCC_SETMRL_BROADCAST | I3C_CCC_SETMRL_DIRECT => HciTargetCcc::SetMaxReadLength,
        I3C_CCC_GETMWL => HciTargetCcc::GetMaxWriteLength,
        I3C_CCC_GETMRL => HciTargetCcc::GetMaxReadLength,
        I3C_CCC_GETPID => HciTargetCcc::GetPid,
        I3C_CCC_GETBCR => HciTargetCcc::GetBcr,
        I3C_CCC_GETDCR => HciTargetCcc::GetDcr,
        I3C_CCC_GETSTATUS => HciTargetCcc::GetStatus,
        I3C_CCC_GETMXDS => HciTargetCcc::GetMaxDataSpeed,
        other => HciTargetCcc::Unknown(other),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HciTargetEvent {
    /// Active controller performed a private write to us.
    PrivateWrite { len: usize },

    /// A CCC reached this target.
    Ccc { ccc: u8, len: usize },

    /// Previously queued target read-data was consumed.
    ReadComplete { len: usize },

    /// Valid target response that does not map to a high-level target event.
    Other(TargetResponse),
}

impl HciTargetEvent {
    #[must_use]
    pub const fn ccc_kind(self) -> Option<HciTargetCcc> {
        match self {
            Self::Ccc { ccc, .. } => Some(classify_target_ccc(ccc)),
            _ => None,
        }
    }
}

pub struct I3cHciTarget<'a, Y: FnMut(u32)> {
    regs: &'a I3cHciRegisters,
    pio: I3cHciPio<'a, Y>,
    enabled: bool,
}

impl<'a, Y: FnMut(u32)> I3cHciTarget<'a, Y> {
    #[must_use]
    pub fn new(regs: &'a I3cHciRegisters, yield_us: Y) -> Self {
        Self {
            regs,
            pio: I3cHciPio::new(regs, yield_us),
            enabled: false,
        }
    }

    #[must_use]
    pub fn with_timeout_us(mut self, timeout_us: u32) -> Self {
        self.pio = self.pio.with_timeout_us(timeout_us);
        self
    }

    pub fn configure(&self, config: &HciTargetConfig) -> Result<()> {
        config.validate()?;

        self.regs.aspeed_set_slave_pid(config.pid);

        self.regs
            .aspeed_set_slave_char_ctrl(config.bcr, config.dcr, config.static_addr);

        Ok(())
    }

    /// Put AST1080 into pure target mode.
    pub fn enable(&mut self) -> Result<()> {
        /*
         * Matches ASPEED target_set_mode():
         *
         * ASPEED_I3C_CTRL_INIT | INIT_SLV_MODE
         */
        self.regs.aspeed_init(InitMode::Target);

        let cap_ctrl = self.regs.aspeed_slave_cap_ctrl();
        self.regs.write_aspeed_slave_cap_ctrl(
            cap_ctrl
                | ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT
                | ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT
                | ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT,
        );

        self.regs.write_pio_intr_status_enable(TARGET_PIO_IRQS);

        self.regs.write_pio_intr_signal_enable(TARGET_PIO_IRQS);

        let role = self.regs.aspeed_target_role();

        if !role.target {
            return Err(I3cHciError::Busy);
        }

        self.enabled = true;
        Ok(())
    }

    /// Stop delivering target interrupts.
    ///
    /// This intentionally does not switch AST1080 back into controller mode.
    /// Role switching should be explicit rather than hidden in `disable()`.
    pub fn disable(&mut self) {
        self.regs.write_pio_intr_signal_enable(0);
        self.enabled = false;
    }

    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    #[must_use]
    pub fn dynamic_addr(&self) -> Option<u8> {
        self.regs.aspeed_target_dynamic_addr()
    }

    #[must_use]
    pub fn role(&self) -> super::types::TargetRole {
        self.regs.aspeed_target_role()
    }

    /// Queue bytes that will be returned by the next controller private read.
    pub fn queue_read_data(&mut self, data: &[u8]) -> Result<()> {
        if !self.enabled {
            return Err(I3cHciError::Busy);
        }

        self.pio.target_queue_read_data(data)
    }

    /// Queue optional IBI payload bytes before requesting an IBI.
    pub fn queue_ibi_payload(&mut self, data: &[u8]) -> Result<()> {
        if !self.enabled {
            return Err(I3cHciError::Busy);
        }

        self.pio.target_queue_ibi_payload(data)
    }

    /// Raise a target interrupt IBI.
    ///
    /// ASPEED target mode follows Zephyr/AST2700 behavior: first make sure the
    /// active controller enabled SIR for this target, then set the private
    /// IBI request bit and wait for hardware to consume it.
    pub fn request_ibi(&mut self) -> Result<()> {
        if !self.enabled {
            return Err(I3cHciError::Busy);
        }

        if (self.regs.aspeed_slave_status1() & ASPEED_I3C_SLV_STS1_IBI_EN) == 0 {
            return Err(I3cHciError::Busy);
        }

        self.request_slave_capability(ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ)
    }

    pub fn accept_controller_role_requests(&self, enable: bool) {
        let mut cap_ctrl = self.regs.aspeed_slave_cap_ctrl();
        if enable {
            cap_ctrl |= ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR;
        } else {
            cap_ctrl &= !ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR;
        }
        self.regs.write_aspeed_slave_cap_ctrl(cap_ctrl);
    }

    /// Request controller-role ownership from the active controller.
    pub fn request_controller_role(&mut self) -> Result<()> {
        if !self.enabled {
            return Err(I3cHciError::Busy);
        }

        if (self.regs.aspeed_slave_status1() & ASPEED_I3C_SLV_STS1_CR_EN) == 0 {
            return Err(I3cHciError::Busy);
        }

        self.request_slave_capability(ASPEED_I3C_SLV_CAP_CTRL_MR_REQ)
    }

    /// Request Hot-Join while the target has no dynamic address.
    pub fn request_hotjoin(&mut self) -> Result<()> {
        if !self.enabled {
            return Err(I3cHciError::Busy);
        }

        if self.dynamic_addr().is_some() {
            return Err(I3cHciError::InvalidArgs);
        }

        if (self.regs.aspeed_slave_status1() & ASPEED_I3C_SLV_STS1_HJ_EN) == 0 {
            return Err(I3cHciError::Busy);
        }

        self.request_slave_capability(ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ)
    }

    /// Wait for and process the next target response.
    ///
    /// `rx` is used when the active controller writes data to this target.
    pub fn next_event(&mut self, rx: &mut [u8]) -> Result<HciTargetEvent> {
        if !self.enabled {
            return Err(I3cHciError::Busy);
        }

        let response = self.pio.target_next_response()?;

        let event = if response.target_received {
            /*
             * Controller wrote to target. The response tells us how many
             * bytes are waiting in RX FIFO.
             */
            let len = response.data_len;

            self.pio.target_read_rx(rx, len)?;

            if response.ccc {
                HciTargetEvent::Ccc {
                    ccc: response.ccc_hdr,
                    len,
                }
            } else {
                HciTargetEvent::PrivateWrite { len }
            }
        } else if response.tid == TID_TARGET_RD_DATA {
            /*
             * Target transmitted previously queued read data.
             */
            HciTargetEvent::ReadComplete {
                len: response.data_len,
            }
        } else {
            HciTargetEvent::Other(response)
        };

        /*
         * The ISR masks PIO signaling after latching a status to avoid
         * a level-triggered IRQ storm. Once normal context has drained the
         * response/FIFO we can safely re-arm it.
         */
        self.regs.write_pio_intr_signal_enable(TARGET_PIO_IRQS);

        Ok(event)
    }

    fn request_slave_capability(&mut self, request_bit: u32) -> Result<()> {
        let cap_ctrl = self.regs.aspeed_slave_cap_ctrl();
        self.regs
            .write_aspeed_slave_cap_ctrl(cap_ctrl | request_bit);
        self.pio.wait_aspeed_cap_ctrl_clear(request_bit)
    }
}
