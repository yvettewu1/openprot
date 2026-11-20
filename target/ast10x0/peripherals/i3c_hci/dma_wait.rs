// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Polling wait policy for DMA. Transfer construction and completion ownership
//! stay in dma.rs. Replace these waits with kernel event waits when IRQ support
//! is added; preserve probes, sampled-status acknowledgement and SRAM barriers.
use super::*;

impl Ring {
    pub(super) fn wait(&self, read: fn(&I3cHciRegisters) -> u32, mask: u32, expected: u32) -> DmaResult<()> {
        for _ in 0..POLL_BUDGET {
            if read(&self.regs) & mask == expected {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        self.snapshot();
        Err("DMA ring control timeout")
    }

    pub(super) fn wait_aspeed_dma_idle(&self) -> DmaResult<()> {
        for (name, read) in [
            (
                "WDMA",
                I3cHciRegisters::aspeed_wdma_debug_low as fn(&I3cHciRegisters) -> u32,
            ),
            (
                "RDMA",
                I3cHciRegisters::aspeed_rdma_debug_low as fn(&I3cHciRegisters) -> u32,
            ),
        ] {
            let mut idle = false;
            for _ in 0..POLL_BUDGET {
                if read(&self.regs) & 2 == 0 {
                    idle = true;
                    break;
                }
                core::hint::spin_loop();
            }
            if !idle {
                if let Some(trace) = self.trace {
                    trace(format_args!("ASPEED {} engine busy timeout", name));
                }
                return Err("ASPEED DMA engine busy timeout");
            }
        }
        Ok(())
    }

    pub(super) fn wait_command_event(&self, pending: &PendingCommand) -> DmaResult<u32> {
        let mut completion_status = None;
        for _ in 0..POLL_BUDGET {
            if let Some(status) = self.try_command_event(pending)? {
                completion_status = Some(status);
                break;
            }
            core::hint::spin_loop();
        }
        let Some(intr) = completion_status else {
            self.dump_wdma_debug("RESPONSE TIMEOUT");
            self.snapshot();
            return Err("DMA transfer completion timeout");
        };

        Ok(intr)
    }

    pub(super) fn wait_response_visible(&self, pending: &PendingCommand) -> DmaResult<u32> {
        let slot = pending.slot;
        let tid = pending.tid;
        let response = self.response_ptr(slot);
        // CR_DEQ advancement alone does not prove that the response is
        // visible in SRAM. Poll the poisoned slot with a bounded budget. This
        // diagnoses delayed writeback without accepting an unwritten response.
        // A barrier orders CPU accesses; it does not complete a DMA write.
        let mut raw_response = 0xffff_ffff;
        let mut response_polls = 0;
        for poll in 0..POLL_BUDGET {
            if let Some(raw) = self.try_response_visible(pending) {
                raw_response = raw;
                response_polls = poll;
                break;
            }
            core::hint::spin_loop();
        }
        if raw_response == 0xffff_ffff {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA response timeout: slot={} expected_tid={} address={:08x} stride={}",
                    slot, tid, response as usize, self.response_stride
                ));
                trace(format_args!(
                    "DMA RESP_BASE={:08x}:{:08x} CR_SETUP={:08x} WDMA_DBG={:08x} RDMA_DBG={:08x}",
                    self.regs.dma_ring_resp_base_hi(),
                    self.regs.dma_ring_resp_base_lo(),
                    self.regs.dma_ring_cr_setup(),
                    self.regs.aspeed_wdma_debug_low(),
                    self.regs.aspeed_rdma_debug_low()
                ));
                // Include nearby padding to expose writes at an unexpected
                // stride. Never consume a response from a different address.
                let base = unsafe { addr_of_mut!((*self.memory).responses).cast::<u32>() };
                for i in 0..16 {
                    let ptr = unsafe { base.add(i) };
                    trace(format_args!(
                        "DMA RESP word[{}] addr={:08x} value={:08x}",
                        i,
                        ptr as usize,
                        unsafe { read_volatile(ptr) }
                    ));
                }
            }
            self.snapshot();
            return Err("DMA response visibility timeout");
        }
        if response_polls != 0 {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "DMA response visible after {} extra polls",
                    response_polls
                ));
            }
        }

        Ok(raw_response)
    }
}
