// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Opt-in non-destructive register dumps; never consume FIFO ports.
use super::cmd::{HciCommand, HciCommandV1, HciResponse};
use super::error::DmaError;
use super::registers::I3cHciRegisters;
use core::ptr::read_volatile;

impl I3cHciRegisters {
    /// Non-destructive snapshots; never read FIFO ports for diagnostics.
    pub fn trace_transfer_state(&self, stage: &str, trace: fn(core::fmt::Arguments<'_>)) {
        let queue = self.aspeed_queue_pointers();
        trace(format_args!(
            "HCI{} {} QUEUE_PTR0={:08x} CMD W/R={}/{} TX W/R={}/{} CTRL={:08x}",
            self.bus(),
            stage,
            queue,
            queue & 3,
            (queue >> 2) & 3,
            (queue >> 12) & 0x1f,
            (queue >> 20) & 0x1f,
            self.aspeed_ctrl()
        ));
        trace(format_args!(
            "HCI{} PHY SDR0 SCL={:08x} TBIT={:08x} HOLD={:08x} TARGET_HOLD={:08x} OPTION={:08x}",
            self.bus(),
            self.phy_038(),
            self.phy_03c(),
            self.phy_040(),
            self.phy_090(),
            self.phy_0cc()
        ));
        trace(format_args!(
            "HCI{} PHY SDR4 SCL={:08x} TBIT={:08x} HOLD={:08x}",
            self.bus(),
            self.phy_068(),
            self.phy_06c(),
            self.phy_070()
        ));
    }
}

impl<Y: FnMut(u32)> super::core::I3cHciCore<Y> {
    /// Print the successful initialization's calculated timing, once at bring-up.
    /// counter_scl is derived from SCL counts; it is not a wire measurement.
    pub fn trace_timing(&self, trace: fn(core::fmt::Arguments<'_>)) {
        let Some(timing) = self.programmed_timing() else {
            trace(format_args!(
                "HCI{} TIMING: no configured timing programmed",
                self.regs().bus()
            ));
            return;
        };
        let source = if self
            .board_config()
            .is_some_and(|b| b.timing.input_clock_hz.is_some())
        {
            "override"
        } else {
            "SCU"
        };
        let high = (timing.sdr[0] >> 16) & 0x7ff;
        let low = timing.sdr[0] & 0x7ff;
        trace(format_args!(
            "HCI{} TIMING input={}Hz ({}) requested_scl={}Hz counter_scl={}Hz bank=SDR{}",
            self.regs().bus(),
            timing.input_hz,
            source,
            timing.requested_scl_hz,
            timing.input_hz / (high + low + 2),
            timing.bank
        ));
        trace(format_args!(
            "HCI{} PHY calculated SCL={:08x} (H={} L={}) TBIT={:08x} HOLD={:08x} OD={:08x}",
            self.regs().bus(),
            timing.sdr[0],
            high,
            low,
            timing.sdr[1],
            timing.sdr[2],
            timing.od[1]
        ));
        let readback = [
            self.regs().phy_038(),
            self.regs().phy_03c(),
            self.regs().phy_040(),
        ];
        if readback != timing.sdr {
            trace(format_args!(
                "HCI{} PHY SDR0 readback mismatch: SCL={:08x} TBIT={:08x} HOLD={:08x}",
                self.regs().bus(),
                readback[0],
                readback[1],
                readback[2]
            ));
        }
    }
}

impl super::dma::Ring {
    pub(super) fn dump_dma_state(&self, stage: &str) {
        if let Some((regs, status, trace)) = self.diagnostic_context() {
            trace(format_args!(
                "{}: DMA:: RING_STATUS={:08x} INTR={:08x} OP1={:08x} OP2={:08x}",
                stage,
                regs.dma_ring_status(),
                status,
                regs.dma_ring_operation1(),
                regs.dma_ring_operation2(),
            ));
        }
    }
    pub fn dump_ibi_state(&self) {
        if let Some((regs, status, trace)) = self.diagnostic_context() {
            trace(format_args!(
                "IBI controller: HC_CONTROL={:08x} HC_INTR_STATUS={:08x} ASPEED_STATUS={:08x} IBI_TERMINATE={:08x}",
                regs.hc_control(),
                regs.intr_status(),
                regs.aspeed_status(),
                regs.aspeed_ibi_terminate_control(),
            ));
            trace(format_args!(
                "DMA: RING_STATUS={:08x} RING_CONTROL={:08x} CR_SETUP={:08x} IBI_SETUP={:08x}",
                regs.dma_ring_status(),
                regs.dma_ring_control(),
                regs.dma_ring_cr_setup(),
                regs.dma_ring_ibi_setup(),
            ));

            trace(format_args!(
                "DMA: INTR={:08x} INTR_EN={:08x} OP1={:08x} OP2={:08x} CHUNK={:08x}",
                status,
                regs.dma_ring_intr_status_enable(),
                regs.dma_ring_operation1(),
                regs.dma_ring_operation2(),
                regs.dma_ring_chunk_control(),
            ));

            trace(format_args!(
                "DMA: IBI_STATUS_BASE={:08x}:{:08x} IBI_DATA_BASE={:08x}:{:08x}",
                regs.dma_ring_ibi_status_base_hi(),
                regs.dma_ring_ibi_status_base_lo(),
                regs.dma_ring_ibi_data_base_hi(),
                regs.dma_ring_ibi_data_base_lo(),
            ));
        }
    }
    pub(super) fn snapshot(&self) {
        if let Some((regs, status, trace)) = self.diagnostic_context() {
            trace(format_args!(
                "DMA ring status={:08x} intr={:08x} op1={:08x} op2={:08x}",
                regs.dma_ring_status(),
                status,
                regs.dma_ring_operation1(),
                regs.dma_ring_operation2()
            ));
        }
    }
}

impl super::dma::Ring {
    /// Legacy diagnostic helper: compare RX with expected bytes.
    /// Applications use I3cController::read/write instead.
    pub fn transfer(
        &mut self,
        command: HciCommandV1,
        tid: u8,
        read: bool,
        xfer_data: &[u8],
    ) -> core::result::Result<HciResponse, DmaError> {
        let (result, buffer) =
            self.transfer_buffered(HciCommand::V1(command), tid, read, xfer_data)?;
        if !result.success() {
            return Err(DmaError::Fault("DMA response reports transfer failure"));
        }
        if read {
            if result.data_len != xfer_data.len() {
                return Err(DmaError::Fault("DMA read length mismatch"));
            }
            for (i, expected) in xfer_data.iter().enumerate() {
                let actual = unsafe { read_volatile(buffer.add(i)) };
                if actual != *expected {
                    return Err(DmaError::Fault("DMA read payload mismatch"));
                }
            }
        } else if result.data_len != 0 {
            return Err(DmaError::Fault("DMA write reports untransferred bytes"));
        }
        Ok(result)
    }
}
