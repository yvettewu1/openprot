// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! AST1080 MIPI I3C HCI target-mode support.
//!
//! Covers target identity, target mode selection, private write/read handling,
//! target IBI requests, and Hot-Join requests.

use super::api::{I3cTarget, TargetEvent};
use super::ccc::{
    I3C_CCC_DISEC_BROADCAST, I3C_CCC_DISEC_DIRECT, I3C_CCC_ENEC_BROADCAST, I3C_CCC_ENEC_DIRECT,
    I3C_CCC_GETBCR, I3C_CCC_GETDCR, I3C_CCC_GETMRL, I3C_CCC_GETMWL, I3C_CCC_GETMXDS,
    I3C_CCC_GETPID, I3C_CCC_GETSTATUS, I3C_CCC_RSTDAA_BROADCAST, I3C_CCC_RSTDAA_DIRECT,
    I3C_CCC_SETDASA, I3C_CCC_SETMRL_BROADCAST, I3C_CCC_SETMRL_DIRECT, I3C_CCC_SETMWL_BROADCAST,
    I3C_CCC_SETMWL_DIRECT, I3C_CCC_SETNEWDA,
};
use super::cmd::{HciTargetCommand, TID_TARGET_IBI, TID_TARGET_RD_DATA};
use super::constants::{
    ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR, ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ,
    ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT, ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ,
    ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT, ASPEED_I3C_SLV_CAP_CTRL_MR_REQ,
    ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT, ASPEED_I3C_SLV_STS1_CR_EN, ASPEED_I3C_SLV_STS1_HJ_EN,
    ASPEED_I3C_SLV_STS1_IBI_EN,
};
use super::daa::valid_i3c_address;
use super::dma::{DmaTargetRx, Ring};

use super::error::{I3cHciError, Result};
use super::pio::{STAT_ALL_ERRORS, STAT_RESP_READY, TargetResponse};
use super::registers::I3cHciRegisters;
use super::transport::Transport;
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

    /// A CCC reached this target. len counts received bytes; an automatic
    /// CCC read reply carries no RX payload and reports zero here.
    Ccc { ccc: u8, len: usize },

    /// Previously queued target read-data was consumed.
    ReadComplete { len: usize },

    /// Valid target response that does not map to a high-level target event.
    Other(TargetResponse),

    /// DMA target IBI completed.
    IbiComplete { status: u8 },

    /// DMA target response not otherwise classified.
    DmaComplete { tid: u8, status: u8, raw: u32 },
}

impl HciTargetEvent {
    fn from_dma_rx(record: DmaTargetRx) -> Self {
        let response = TargetResponse::parse(record.raw_status);
        if record.ccc {
            Self::Ccc {
                ccc: record.ccc_hdr,
                len: record.len,
            }
        } else if response.target_received {
            Self::PrivateWrite { len: record.len }
        } else {
            // Unsolicited status is not completion of a queued DMA command.
            // IBI/read completion ownership stays with the command/response ring.
            Self::DmaComplete {
                tid: response.tid,
                status: response.status,
                raw: record.raw_status,
            }
        }
    }

    #[must_use]
    pub const fn ccc_kind(self) -> Option<HciTargetCcc> {
        match self {
            Self::Ccc { ccc, .. } => Some(classify_target_ccc(ccc)),
            _ => None,
        }
    }
}

#[must_use = "target sessions require explicit shutdown"]
pub struct I3cHciTarget<'a, Y: FnMut(u32)> {
    regs: &'a I3cHciRegisters,
    transport: Transport<'a, Y>,
    dma_ibi_queued: bool,
    enabled: bool,
    trace: Option<fn(core::fmt::Arguments<'_>)>,
}

impl<'a, Y: FnMut(u32)> I3cHciTarget<'a, Y> {
    #[must_use]
    pub fn new(regs: &'a I3cHciRegisters, yield_us: Y) -> Self {
        Self {
            regs,
            transport: Transport::new(regs, yield_us),
            dma_ibi_queued: false,
            enabled: false,
            trace: None,
        }
    }

    /// Create and enable the target-side session from firmware configuration.
    /// Secondary and pure target roles retain their configured vendor mode.
    pub fn from_configured_core<F: FnMut(u32)>(
        core: &'a mut super::core::I3cHciCore<F>,
        yield_us: Y,
    ) -> Result<Self> {
        let config = core.board_config().ok_or(I3cHciError::InvalidArgs)?;
        let identity = config.role.identity().ok_or(I3cHciError::InvalidArgs)?;
        let mode = config.role.init_mode();
        if core.init_mode() != Some(mode) || core.selected_io() != Some(config.io_mode) {
            return Err(I3cHciError::InvalidArgs);
        }
        let mut target = Self::new(core.regs(), yield_us).with_poll_budget(config.poll_budget);
        if config.io_mode == super::core::IoMode::Dma {
            target = target.with_dma(Ring::init(core).map_err(I3cHciError::from)?)?;
        }
        if let Err(error) = target
            .configure(&identity)
            .and_then(|_| target.enable_with_mode(mode))
        {
            if target.shutdown().is_err() {
                return Err(I3cHciError::Dma(super::error::DmaError::Fault(
                    "target setup failed; DMA shutdown also failed and memory remains reserved",
                )));
            }
            return Err(error);
        }
        Ok(target)
    }

    #[must_use]
    pub fn with_poll_budget(mut self, poll_budget: u32) -> Self {
        self.transport = self.transport.with_poll_budget(poll_budget);
        self
    }

    /// Subscribe to the pre-registered event latch before unmasking the NVIC.
    /// Board/kernel integration retains ownership of interrupt routing.
    pub fn with_irq_events(mut self, bus: usize) -> Self {
        self.transport = self.transport.with_irq_events(bus);
        self
    }

    /// Trace target TX descriptors and raw completion responses.
    pub fn with_trace(mut self, trace: fn(core::fmt::Arguments<'_>)) -> Self {
        self.transport = self.transport.with_trace(trace);
        self.trace = Some(trace);
        self
    }

    /// Attach an initialized target DMA ring before enable().
    /// The caller initializes it using Ring::init(&peer_core).
    pub fn with_dma(mut self, ring: Ring) -> Result<Self> {
        self.transport =
            self.transport
                .attach_dma(ring, self.regs.bus(), super::dma::DmaMode::Target, None)?;
        Ok(self)
    }

    #[must_use]
    pub fn is_dma(&self) -> bool {
        self.transport.is_dma()
    }

    fn dma_error(message: super::error::DmaError) -> I3cHciError {
        I3cHciError::from(message)
    }

    pub fn configure(&self, config: &HciTargetConfig) -> Result<()> {
        config.validate()?;

        self.regs.aspeed_set_slave_pid(config.pid);

        self.regs
            .aspeed_set_slave_char_ctrl(config.bcr, config.dcr, config.static_addr);

        Ok(())
    }

    /// Legacy diagnostic setup uses secondary mode.
    pub fn enable(&mut self) -> Result<()> {
        self.enable_with_mode(InitMode::Secondary)
    }

    /// Enable the target using the same Secondary or Target role as core initialization.
    /// Primary mode is rejected.
    pub fn enable_with_mode(&mut self, mode: InitMode) -> Result<()> {
        if mode == InitMode::Primary {
            return Err(I3cHciError::InvalidArgs);
        }
        /*
         * Matches ASPEED target_set_mode():
         *
         * ASPEED_I3C_CTRL_INIT | INIT_SLV_MODE
         */
        self.regs.aspeed_init(mode);

        let cap_ctrl = self.regs.aspeed_slave_cap_ctrl();
        self.regs.write_aspeed_slave_cap_ctrl(
            cap_ctrl
                | ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT
                | ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT
                | ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT,
        );

        // Polling queue helpers need CMD_QUEUE_READY and TX/RX thresholds,
        // even though only completion/errors should signal a CPU interrupt.
        if !self.transport.is_dma() {
            self.regs.write_pio_intr_status_enable(u32::MAX);
            // Threshold encoding zero means one DWORD. The controller defaults
            // use half-FIFO thresholds, which strand short target CCC payloads.
            //self.regs.write_pio_data_buffer_thld_ctrl(0);
            self.regs.write_pio_intr_signal_enable(TARGET_PIO_IRQS);
        } else {
            self.regs.write_pio_intr_signal_enable(0);
        }

        let role = self.regs.aspeed_target_role();

        if !role.target {
            return Err(I3cHciError::Busy);
        }

        self.enabled = true;
        Ok(())
    }

    /// Stop DMA before releasing its storage. A failed stop retains the claim
    /// because hardware might still own descriptors or payload buffers.
    pub fn shutdown(mut self) -> Result<()> {
        let stopped = self.transport.shutdown();
        // Run target disable even if DMA could not stop. The ring retains its
        // memory claim on failure. Global/NVIC masking remains board-owned;
        // disabling signals alone does not imply that the DMA engine is idle.
        self.disable();
        stopped
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
        self.require_enabled()?;
        let command = HciTargetCommand::read_data(data.len())?;
        self.queue_command(command, data)
    }

    fn require_enabled(&self) -> Result<()> {
        if self.enabled {
            Ok(())
        } else {
            Err(I3cHciError::Busy)
        }
    }

    /// Both transports consume the same target command word and payload.
    fn queue_command(&mut self, command: HciTargetCommand, data: &[u8]) -> Result<()> {
        if let Some(dma) = self.transport.dma_mut() {
            dma.target_queue_command(command.word, data)
                .map_err(Self::dma_error)?;
            Ok(())
        } else {
            self.transport
                .pio_mut()?
                .target_queue_command(command, data)
        }
    }

    fn received_event(ccc: bool, ccc_hdr: u8, len: usize) -> HciTargetEvent {
        if ccc {
            HciTargetEvent::Ccc { ccc: ccc_hdr, len }
        } else {
            HciTargetEvent::PrivateWrite { len }
        }
    }

    /// Queue optional IBI payload bytes before requesting an IBI.
    pub fn queue_ibi_payload(&mut self, data: &[u8]) -> Result<()> {
        self.require_enabled()?;

        if self.transport.is_dma() {
            // The MDB is inline (enable bit 6); only the remaining bytes
            // belong in the DMA data buffer, exactly as for the PIO command.
            if data.is_empty() {
                return Err(I3cHciError::InvalidArgs);
            }
            if self.dma_ibi_queued {
                return Err(I3cHciError::Busy);
            }
            let payload = &data[1..];
            let command = HciTargetCommand::ibi_with_mdb(data[0], payload.len())?;
            self.queue_command(command, payload)?;
            self.dma_ibi_queued = true;
            Ok(())
        } else {
            self.transport.pio_mut()?.target_queue_ibi_payload(data)
        }
    }

    /// Raise a target interrupt IBI.
    ///
    /// ASPEED target mode follows Zephyr/AST2700 behavior: first make sure the
    /// active controller enabled SIR for this target, then set the private
    /// IBI request bit and wait for hardware to consume it.
    pub fn request_ibi(&mut self) -> Result<()> {
        self.start_ibi()?;
        self.transport
            .wait_capability_clear(self.regs, ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ)
    }

    /// Start an IBI without waiting, so a single CPU can service both ports.
    /// Queue the payload first and consume the target completion afterwards.
    pub fn start_ibi(&mut self) -> Result<()> {
        self.require_enabled()?;

        if (self.regs.aspeed_slave_status1() & ASPEED_I3C_SLV_STS1_IBI_EN) == 0 {
            return Err(I3cHciError::Busy);
        }

        if self.transport.is_dma() && !self.dma_ibi_queued {
            return Err(I3cHciError::InvalidArgs);
        }
        let cap_ctrl = self.regs.aspeed_slave_cap_ctrl();
        if cap_ctrl & ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ != 0 {
            return Err(I3cHciError::Busy);
        }
        self.regs
            .write_aspeed_slave_cap_ctrl(cap_ctrl | ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ);
        Ok(())
    }

    /// Hardware event-enable state updated by ENEC/DISEC.
    pub fn ibi_enabled(&self) -> bool {
        self.regs.aspeed_slave_status1() & ASPEED_I3C_SLV_STS1_IBI_EN != 0
    }

    /// Poll for a target response without waiting for a new bus transaction.
    pub fn poll_event(&mut self, rx: &mut [u8]) -> Result<Option<HciTargetEvent>> {
        self.require_enabled()?;
        if self.transport.is_dma() {
            return self.poll_dma_event(rx);
        }
        let Some(response) = self.transport.pio_mut()?.target_poll_response()? else {
            return Ok(None);
        };
        self.finish_pio_event(response, rx).map(Some)
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
        self.require_enabled()?;

        if (self.regs.aspeed_slave_status1() & ASPEED_I3C_SLV_STS1_CR_EN) == 0 {
            return Err(I3cHciError::Busy);
        }

        self.request_slave_capability(ASPEED_I3C_SLV_CAP_CTRL_MR_REQ)
    }

    /// Request Hot-Join while the target has no dynamic address.
    pub fn request_hotjoin(&mut self) -> Result<()> {
        self.require_enabled()?;

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
        self.require_enabled()?;

        if self.transport.is_dma() {
            return self.wait_dma_event(rx);
        }
        let response = self.transport.pio_mut()?.target_next_response()?;
        self.finish_pio_event(response, rx)
    }

    fn finish_pio_event(
        &mut self,
        response: TargetResponse,
        rx: &mut [u8],
    ) -> Result<HciTargetEvent> {
        let event = if response.target_received {
            /*
             * Controller wrote to target. The response tells us how many
             * bytes are waiting in RX FIFO.
             */
            let len = response.data_len;

            self.transport
                .pio_mut()?
                .target_read_rx(rx, len)
                .map(|_| Self::received_event(response.ccc, response.ccc_hdr, len))
        } else if response.ccc {
            // Hardware-serviced CCC reads have no RX FIFO bytes. Their TID
            // must not be mistaken for a queued private-read completion.
            Ok(Self::received_event(true, response.ccc_hdr, 0))
        } else if response.tid == TID_TARGET_RD_DATA {
            /*
             * Target transmitted previously queued read data.
             */
            Ok(HciTargetEvent::ReadComplete {
                len: response.data_len,
            })
        } else {
            Ok(HciTargetEvent::Other(response))
        };

        /*
         * The ISR masks PIO signaling after latching a status to avoid
         * a level-triggered IRQ storm. Once normal context has drained the
         * response/FIFO we can safely re-arm it, including a NoSpace result.
         */
        self.transport
            .pio_mut()?
            .enable_irq_signals(TARGET_PIO_IRQS);

        event
    }

    fn poll_dma_event(&mut self, rx: &mut [u8]) -> Result<Option<HciTargetEvent>> {
        let dma = self.transport.dma_mut().ok_or(I3cHciError::Unsupported)?;
        // Incoming private writes/CCCs are delivered through the target IBI ring.
        let record = dma.target_poll_rx(rx).map_err(|error| {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "HCI{} target_poll_rx failed: {}",
                    self.regs.bus(),
                    error
                ));
            }
            Self::dma_error(error)
        })?;

        if let Some(record) = record {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "HCI{} DMA RX: len={} ccc={} hdr=0x{:02x}",
                    self.regs.bus(),
                    record.len,
                    record.ccc,
                    record.ccc_hdr
                ));
            }
            return Ok(Some(HciTargetEvent::from_dma_rx(record)));
        }
        if let Some(response) = dma.target_poll_response().map_err(Self::dma_error)? {
            if response.tid == TID_TARGET_IBI {
                self.dma_ibi_queued = false;
                return Ok(Some(HciTargetEvent::IbiComplete {
                    status: response.status,
                }));
            }
            if response.tid == TID_TARGET_RD_DATA {
                // Match the PIO path: an unsuccessful target TX must not be
                // surfaced as ReadComplete, even when its length field is zero.
                if response.status != 0 {
                    return Err(I3cHciError::Transfer(response.status));
                }
                return Ok(Some(HciTargetEvent::ReadComplete {
                    len: response.data_len,
                }));
            }
            return Ok(Some(HciTargetEvent::DmaComplete {
                tid: response.tid,
                status: response.status,
                raw: response.raw,
            }));
        }
        Ok(None)
    }

    fn request_slave_capability(&mut self, request_bit: u32) -> Result<()> {
        let cap_ctrl = self.regs.aspeed_slave_cap_ctrl();
        self.regs
            .write_aspeed_slave_cap_ctrl(cap_ctrl | request_bit);
        self.transport.wait_capability_clear(self.regs, request_bit)
    }
}

impl<Y: FnMut(u32)> I3cTarget for I3cHciTarget<'_, Y> {
    fn address(&self) -> Option<u8> {
        self.dynamic_addr()
    }
    fn ibi_enabled(&self) -> bool {
        I3cHciTarget::ibi_enabled(self)
    }
    fn queue_read(&mut self, data: &[u8]) -> Result<()> {
        self.queue_read_data(data)
    }
    fn queue_ibi(&mut self, payload: &[u8]) -> Result<()> {
        self.queue_ibi_payload(payload)
    }
    fn start_ibi(&mut self) -> Result<()> {
        I3cHciTarget::start_ibi(self)
    }
    fn poll_event(&mut self, data: &mut [u8]) -> Result<Option<TargetEvent>> {
        let Some(event) = I3cHciTarget::poll_event(self, data)? else {
            return Ok(None);
        };
        Ok(Some(match event {
            HciTargetEvent::PrivateWrite { len } => TargetEvent::WriteReceived { len },
            HciTargetEvent::Ccc { ccc, len } => TargetEvent::CccReceived { ccc, len },
            HciTargetEvent::ReadComplete { len } => TargetEvent::ReadComplete { remaining: len },
            HciTargetEvent::IbiComplete { status } => {
                if status != 0 {
                    return Err(I3cHciError::Transfer(status));
                }
                TargetEvent::IbiComplete
            }
            HciTargetEvent::Other(response)
                if !response.ccc
                    && !response.target_received
                    && response.tid == super::cmd::TID_TARGET_IBI =>
            {
                if !response.success() {
                    return Err(I3cHciError::Transfer(response.status));
                }
                TargetEvent::IbiComplete
            }
            HciTargetEvent::DmaComplete { status, .. } if status != 0 => {
                return Err(I3cHciError::Transfer(status));
            }
            _ => TargetEvent::Other,
        }))
    }
}

impl<Y: FnMut(u32)> I3cHciTarget<'_, Y> {
    fn wait_dma_event(&mut self, rx: &mut [u8]) -> Result<HciTargetEvent> {
        super::completion::poll(
            self.transport.budget(),
            || self.poll_dma_event(rx),
            I3cHciError::Timeout,
        )
    }
}

#[cfg(test)]
mod event_tests {
    use super::*;

    fn status_event(raw: u32) -> HciTargetEvent {
        let response = TargetResponse::parse(raw);
        HciTargetEvent::from_dma_rx(DmaTargetRx {
            len: response.received_len(),
            ccc: response.ccc,
            ccc_hdr: response.ccc_hdr,
            raw_status: raw,
        })
    }

    #[test]
    fn automatic_ccc_reads_are_notifications_not_private_completions() {
        // CCC records with either reserved TX TID still belong to the status ring.
        for tid in [0, TID_TARGET_IBI, TID_TARGET_RD_DATA] {
            assert_eq!(
                status_event(0x048c_0003 | (u32::from(tid) << 24)),
                HciTargetEvent::Ccc { ccc: 0x8c, len: 0 },
            );
        }
        assert_eq!(
            status_event(0x0c80_0001),
            HciTargetEvent::Ccc { ccc: 0x80, len: 1 },
        );
        assert_eq!(
            status_event(0x0800_0008),
            HciTargetEvent::PrivateWrite { len: 8 }
        );
    }

    #[test]
    fn status_ring_cannot_complete_a_queued_ibi_or_private_read() {
        for tid in [TID_TARGET_IBI, TID_TARGET_RD_DATA] {
            let raw = u32::from(tid) << 24;
            assert_eq!(
                status_event(raw),
                HciTargetEvent::DmaComplete {
                    tid,
                    status: 0,
                    raw
                },
            );
        }
    }
}
