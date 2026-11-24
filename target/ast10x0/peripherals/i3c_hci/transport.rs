// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Exactly one active transport. Role code does not own an inactive PIO engine.
use super::{
    cmd::IbiStatus,
    core::CommandDescriptor,
    dma::{DmaMode, Ring},
    error::{I3cHciError, Result},
    master::{HciTransfer, HciTransferResult},
    pio::I3cHciPio,
    registers::I3cHciRegisters,
};
pub(crate) enum Transport<'a, Y: FnMut(u32)> {
    Pio(I3cHciPio<'a, Y>),
    Dma {
        ring: Ring,
        yield_us: Y,
        budget: u32,
    },
}
impl<'a, Y: FnMut(u32)> Transport<'a, Y> {
    pub(crate) fn new(regs: &'a I3cHciRegisters, yield_us: Y) -> Self {
        Self::Pio(I3cHciPio::new(regs, yield_us))
    }
    pub(crate) fn attach_dma(
        self,
        ring: Ring,
        bus: u8,
        mode: DmaMode,
        format: Option<CommandDescriptor>,
    ) -> Result<Self> {
        if ring.bus() != bus
            || ring.mode() != mode
            || format.is_some_and(|f| f != ring.command_format())
        {
            return self.reject_dma(ring, I3cHciError::InvalidArgs);
        }
        match self {
            Self::Pio(pio) => {
                let mut ring = ring;
                if let Some(trace) = pio.trace_callback() {
                    ring = ring.with_trace(trace);
                }
                if pio.uses_irq_events() {
                    ring = ring.with_irq_events(bus as usize);
                }
                // Cached PIO status bits have different meanings in DMA mode.
                // Carry the subscription, not those old transport-specific bits.
                let (yield_us, budget) = pio.into_wait_context();
                ring.set_poll_budget(budget);
                Ok(Self::Dma {
                    ring,
                    yield_us,
                    budget,
                })
            }
            current @ Self::Dma { .. } => current.reject_dma(ring, I3cHciError::Busy),
        }
    }

    fn reject_dma(mut self, ring: Ring, reason: I3cHciError) -> Result<Self> {
        // attach_dma consumes both arguments, including an existing transport.
        // Stop all owned rings before either handle is dropped, even if the
        // first stop fails. Drop alone deliberately retains live DMA claims.
        let incoming_stopped = ring.stop();
        let current_stopped = self.shutdown();
        incoming_stopped.map_err(I3cHciError::Dma)?;
        current_stopped?;
        Err(reason)
    }

    /// Explicit and fallible: a failed stop retains the DMA memory claim.
    /// Board lifecycle code masks global/NVIC signals before calling this.
    pub(crate) fn shutdown(&mut self) -> Result<()> {
        match self {
            Self::Pio(_) => Ok(()),
            Self::Dma { ring, .. } => ring.stop().map_err(I3cHciError::from),
        }
    }

    pub(crate) fn is_dma(&self) -> bool {
        matches!(self, Self::Dma { .. })
    }
    pub(crate) fn dma_mut(&mut self) -> Option<&mut Ring> {
        match self {
            Self::Dma { ring, .. } => Some(ring),
            _ => None,
        }
    }
    pub(crate) fn pio_mut(&mut self) -> Result<&mut I3cHciPio<'a, Y>> {
        match self {
            Self::Pio(pio) => Ok(pio),
            _ => Err(I3cHciError::Unsupported),
        }
    }
    pub(crate) fn enable_pio_irq_signals(&self, mask: u32) {
        if let Self::Pio(pio) = self {
            pio.enable_irq_signals(mask);
        }
    }

    pub(crate) fn with_poll_budget(self, budget: u32) -> Self {
        match self {
            Self::Pio(pio) => Self::Pio(pio.with_poll_budget(budget)),
            Self::Dma {
                mut ring, yield_us, ..
            } => {
                ring.set_poll_budget(budget);
                Self::Dma {
                    ring,
                    yield_us,
                    budget,
                }
            }
        }
    }
    pub(crate) fn budget(&self) -> u32 {
        match self {
            Self::Pio(pio) => pio.poll_budget(),
            Self::Dma { budget, .. } => *budget,
        }
    }
    pub(crate) fn with_trace(self, trace: fn(core::fmt::Arguments<'_>)) -> Self {
        match self {
            Self::Pio(pio) => Self::Pio(pio.with_trace(trace)),
            Self::Dma {
                ring,
                yield_us,
                budget,
            } => Self::Dma {
                ring: ring.with_trace(trace),
                yield_us,
                budget,
            },
        }
    }
    pub(crate) fn with_irq_events(self, bus: usize) -> Self {
        match self {
            Self::Pio(pio) => Self::Pio(pio.with_irq_events(bus)),
            Self::Dma {
                ring,
                yield_us,
                budget,
            } => Self::Dma {
                ring: ring.with_irq_events(bus),
                yield_us,
                budget,
            },
        }
    }
    pub(crate) fn submit_raw(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        match self {
            Self::Pio(pio) => pio.submit_raw(transfer),
            Self::Dma { ring, .. } => ring.submit_raw(transfer),
        }
    }
    pub(crate) fn recover_after_daa_nack(&mut self, regs: &I3cHciRegisters) -> Result<()> {
        match self {
            Self::Pio(pio) => pio.recover_after_daa_nack(),
            Self::Dma { ring, .. } => ring.recover_after_daa_nack(regs),
        }
    }
    pub(crate) fn wait_capability_clear(
        &mut self,
        regs: &I3cHciRegisters,
        mask: u32,
    ) -> Result<()> {
        match self {
            Self::Pio(pio) => pio.wait_aspeed_cap_ctrl_clear(mask),
            Self::Dma {
                yield_us, budget, ..
            } => super::completion::poll(
                *budget,
                || {
                    if regs.aspeed_slave_cap_ctrl() & mask == 0 {
                        return Ok(Some(()));
                    }
                    yield_us(1);
                    Ok(None)
                },
                I3cHciError::Timeout,
            ),
        }
    }
    pub(crate) fn discard_partial_ibi(&mut self) -> Result<bool> {
        match self {
            Self::Pio(pio) => pio.discard_partial_ibi(),
            // DMA does not consume/copy a partial event; it remains in the ring.
            Self::Dma { .. } => Ok(true),
        }
    }
    pub(crate) fn poll_ibi(&mut self, payload: &mut [u8]) -> Result<Option<(IbiStatus, usize)>> {
        match self {
            Self::Pio(pio) => pio.poll_ibi(payload),
            Self::Dma { ring, .. } => {
                ring.poll_ibi(payload)
                    .map_err(I3cHciError::from)
                    .map(|event| {
                        event.map(|event| (IbiStatus::parse(event.status), event.payload_len))
                    })
            }
        }
    }
}
